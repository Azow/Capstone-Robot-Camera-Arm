"""A pretend Octopus board: speaks the serial protocol on a pseudo-terminal.

Lets arm_bridge (and RViz) run with no hardware. The simulation follows the
protocol spec (docs/control_system_decisions.md, section 4): the queue, limits,
GO/END, STOP with a ramp-down, the watchdog, and HOME (a slow move to step 0).
It does not simulate missed steps, endstops or the drivers.

Run it, then point arm_bridge's `port` parameter at the link it prints:

    ros2 run arm_bridge fake_board --link /tmp/fake_octopus

It can also be driven by hand: `picocom /tmp/fake_octopus`, then type `HELLO`.
"""

import argparse
import os
import select
import sys
import time
import tty
from collections import deque

from arm_bridge import protocol as p

FIRMWARE = 'fake-0.2'


class BoardSim:
    """The board's logic, with time passed in, so tests can run it without waiting."""

    def __init__(self, queue_size=64, max_rate=40000, watchdog=0.25, ramp_time=0.1,
                 home_time=2.0):
        self.queue_size = queue_size
        self.max_rate = max_rate
        self.watchdog = watchdog
        self.ramp_time = ramp_time
        self.home_time = home_time

        self.state = p.IDLE
        self.homed = False
        self.enabled = False
        self.limits = {}                     # joint index -> (min, max, max_rate)
        self.pos = [0.0] * len(p.JOINTS)     # steps; float while moving
        self.queue = deque()                 # (seq, dur_s, targets)
        self.ended = False
        self.segment = None                  # (seq, start, targets, t_start, dur) playing now
        self.ramp = None                     # (start, velocity, t_start, event) when ramping down
        self.home_move = None                # (start, t_start) while homing
        self.last_rx = 0.0
        self._seq = 0

    # ----- requests ----------------------------------------------------------

    def handle(self, line, now):
        """Run one request line. Returns the lines to send back."""
        self.last_rx = now
        parsed = p.parse_request(line)
        if parsed is None:
            return []
        seq, command, args = parsed
        self._seq = 0 if seq == '-' else int(seq)   # recorded with a SEG, for STATUS
        out = []
        try:
            values = getattr(self, f'_cmd_{command.lower()}', self._cmd_unknown)(args, now, out)
            reply = p.format_ok(seq, self._free(), **(values or {}))
        except _Refused as e:
            reply = p.format_err(seq, e.code, self._free(), e.message)
        return [reply] + out

    def _free(self):
        return self.queue_size - len(self.queue)

    def _cmd_unknown(self, args, now, out):
        raise _Refused(p.BAD_CMD, 'unknown command')

    def _cmd_ping(self, args, now, out):
        pass

    def _cmd_info(self, args, now, out):
        return {'firmware': FIRMWARE}

    def _cmd_hello(self, args, now, out):
        self._halt()
        self.limits = {}
        if self.state != p.FAULT:
            self.state = p.IDLE
        return {'proto': p.PROTOCOL_VERSION, 'fw': FIRMWARE, 'queue': self.queue_size,
                'max_rate': self.max_rate}

    def _cmd_limits(self, args, now, out):
        self._need_state(p.IDLE, p.FAULT)
        if len(args) != 4 or args[0].upper() not in p.JOINTS:
            raise _Refused(p.BAD_ARGS, 'LIMITS <joint> <min> <max> <max_rate>')
        lo, hi, rate = _ints(args[1:])
        if lo > hi or not 0 < rate <= self.max_rate:
            raise _Refused(p.BAD_ARGS, f'need min <= max and 0 < max_rate <= {self.max_rate}')
        self.limits[p.JOINTS.index(args[0].upper())] = (lo, hi, rate)

    def _cmd_enable(self, args, now, out):
        self._need_state(p.IDLE, p.FAULT)
        self.enabled = True

    def _cmd_disable(self, args, now, out):
        self._need_state(p.IDLE, p.FAULT)
        self.enabled = False
        self.homed = False

    def _cmd_home(self, args, now, out):
        self._need_state(p.IDLE)
        if not self.enabled:
            raise _Refused(p.DISABLED, 'drivers are off')
        self.homed = False
        self.state = p.HOMING
        self.home_move = (list(self.pos), now)

    def _cmd_setpos(self, args, now, out):
        self._need_state(p.IDLE)
        if len(args) != len(p.JOINTS):
            raise _Refused(p.BAD_ARGS, 'SETPOS <5 positions>')
        self.pos = [float(v) for v in _ints(args)]
        self.homed = True

    def _cmd_seg(self, args, now, out):
        self._need_state(p.IDLE, p.RUNNING)
        self._need_ready()
        if len(args) != 1 + len(p.JOINTS):
            raise _Refused(p.BAD_ARGS, 'SEG <dur_us> <5 targets>')
        dur_us, *targets = _ints(args)
        if not p.MIN_SEGMENT_US <= dur_us <= p.MAX_SEGMENT_US:
            raise _Refused(p.BAD_ARGS, f'dur_us must be {p.MIN_SEGMENT_US}..{p.MAX_SEGMENT_US}')
        if len(self.queue) >= self.queue_size:
            raise _Refused(p.QUEUE_FULL, '')
        prev = self._queue_end()
        for i, (a, b) in enumerate(zip(prev, targets)):
            lo, hi, rate = self.limits[i]
            if not lo <= b <= hi:
                raise _Refused(p.OUT_OF_LIMITS, f'{p.JOINTS[i]} {b} not in [{lo}, {hi}]')
            if abs(b - a) * 1e6 / dur_us > rate:
                raise _Refused(p.TOO_FAST, f'{p.JOINTS[i]} needs '
                               f'{abs(b - a) * 1e6 / dur_us:.0f} steps/s, max {rate}')
        self.queue.append((self._seq, dur_us / 1e6, targets))

    def _cmd_go(self, args, now, out):
        self._need_state(p.IDLE, p.RUNNING)
        self._need_ready()
        if self.state == p.IDLE:
            if not self.queue:
                raise _Refused(p.BAD_STATE, 'queue is empty')
            self.state = p.RUNNING
            self._next_segment(now)

    def _cmd_end(self, args, now, out):
        self._need_state(p.IDLE, p.RUNNING)
        self.ended = True

    def _cmd_stop(self, args, now, out):
        if self.state == p.RUNNING:
            self._start_ramp(now, p.STOPPED)
        elif self.state == p.HOMING:
            self.home_move = None
            self.state = p.IDLE
            out.append(p.format_event(p.STOPPED))
        elif self.state == p.IDLE:
            self._halt()
            out.append(p.format_event(p.STOPPED))
        # STOPPING: already ramping, it reports when still. FAULT: nothing moves.

    def _cmd_status(self, args, now, out):
        return {'state': self.state, 'homed': int(self.homed), 'enabled': int(self.enabled),
                'limits': int(self._has_limits()),
                'pos': p.format_positions(round(v) for v in self.pos),
                'seg': self.segment[0] if self.segment else 0}

    def _cmd_clear(self, args, now, out):
        if self.state == p.FAULT:
            self.state = p.IDLE

    # ----- checks ------------------------------------------------------------

    def _need_state(self, *states):
        if self.state not in states:
            raise _Refused(p.BAD_STATE, f'not allowed while {self.state}')

    def _has_limits(self):
        return len(self.limits) == len(p.JOINTS)

    def _need_ready(self):
        if not self._has_limits():
            raise _Refused(p.NO_LIMITS, 'send LIMITS for every joint first')
        if not self.homed:
            raise _Refused(p.NOT_HOMED, 'home first')
        if not self.enabled:
            raise _Refused(p.DISABLED, 'drivers are off')

    def _queue_end(self):
        if self.queue:
            return self.queue[-1][2]
        if self.segment:
            return self.segment[2]
        return [round(v) for v in self.pos]

    # ----- motion ------------------------------------------------------------

    def _halt(self):
        self.queue.clear()
        self.ended = False
        self.segment = None
        self.ramp = None
        self.home_move = None
        if self.state in (p.RUNNING, p.STOPPING, p.HOMING):
            self.state = p.IDLE

    def _next_segment(self, t_start):
        seq, dur, targets = self.queue.popleft()
        self.segment = (seq, list(self.pos), targets, t_start, dur)

    def _start_ramp(self, now, event):
        velocity = [0.0] * len(self.pos)
        if self.segment:
            _, start, targets, _, dur = self.segment
            velocity = [(b - a) / dur for a, b in zip(start, targets)]
        self.queue.clear()
        self.ended = False
        self.segment = None
        self.ramp = (list(self.pos), velocity, now, event)
        self.state = p.STOPPING

    def update(self, now):
        """Advance the simulation to `now`. Returns event lines to send."""
        out = []
        if self.state == p.RUNNING:
            if now - self.last_rx > self.watchdog:
                self._start_ramp(now, p.WATCHDOG)
            else:
                self._play(now, out)
        if self.state == p.STOPPING:
            start, velocity, t0, event = self.ramp
            t = min(now - t0, self.ramp_time)
            for i, (a, v) in enumerate(zip(start, velocity)):
                lo, hi, _ = self.limits.get(i, (-1e18, 1e18, 0))
                self.pos[i] = min(hi, max(lo, a + v * t - v * t * t / (2 * self.ramp_time)))
            if t >= self.ramp_time:
                self.ramp = None
                self.state = p.IDLE
                self.pos = [float(round(v)) for v in self.pos]
                out.append(p.format_event(event))
        if self.state == p.HOMING:
            start, t0 = self.home_move
            if now - self.last_rx > self.watchdog:
                self.home_move = None
                self.state = p.IDLE
                out.append(p.format_event(p.WATCHDOG))
            else:
                f = min(1.0, (now - t0) / self.home_time)
                self.pos = [a * (1 - f) for a in start]
                if f >= 1.0:
                    self.home_move = None
                    self.homed = True
                    self.state = p.IDLE
                    out.append(p.format_event(p.HOMED))
        return out

    def _play(self, now, out):
        while self.segment:
            seq, start, targets, t0, dur = self.segment
            elapsed = now - t0
            if elapsed < dur:
                f = elapsed / dur
                self.pos = [a + (b - a) * f for a, b in zip(start, targets)]
                return
            self.pos = [float(v) for v in targets]
            if self.queue:
                self._next_segment(t0 + dur)
            elif self.ended:
                self.segment = None
                self.ended = False
                self.state = p.IDLE
                out.append(p.format_event(p.DONE))
            else:
                # Ran dry: ramp down from the last segment's speed.
                self._start_ramp(now, p.UNDERRUN)
                return


class _Refused(Exception):

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code
        self.message = message


def _ints(args):
    try:
        return [int(a) for a in args]
    except ValueError:
        raise _Refused(p.BAD_ARGS, 'expected whole numbers') from None


# ----- pseudo-terminal server ------------------------------------------------

def serve(link, sim=None, period=0.002):
    sim = sim or BoardSim()
    master, slave = os.openpty()
    tty.setraw(slave)
    slave_name = os.ttyname(slave)
    if os.path.lexists(link):
        os.remove(link)
    os.symlink(slave_name, link)
    print(f'Fake board on {slave_name}, linked as {link}', flush=True)
    buffer = b''
    try:
        while True:
            ready, _, _ = select.select([master], [], [], period)
            now = time.monotonic()
            out = []
            if ready:
                buffer += os.read(master, 4096)
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    text = line.decode('ascii', 'replace').strip()
                    if text:
                        out += sim.handle(text, now)
            out += sim.update(now)
            if out:
                os.write(master, ''.join(out).encode('ascii'))
    finally:
        if os.path.islink(link):
            os.remove(link)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--link', default='/tmp/fake_octopus',
                        help='path of the symlink to the fake serial port')
    # Launched as a ROS node it also gets --ros-args ...; ignore those.
    args, _ = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    try:
        serve(args.link)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
