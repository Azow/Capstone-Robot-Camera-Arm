"""The bridge's logic: connect to the board, keep it configured, run paths, report.

No ROS here (bridge_node.py wraps it), so tests can drive it against the fake
board directly. Everything runs on one thread: call poll() regularly and the
command methods in between.
"""

import time
from dataclasses import dataclass, field

from arm_bridge import protocol as p
from arm_bridge.link import BoardLink, LinkError
from arm_bridge.planner import PlanError, plan_run

RECONNECT_PERIOD = 1.0   # s between connection attempts
MAX_SEGS_PER_POLL = 16   # so streaming never delays the STATUS poll (watchdog) for long


@dataclass
class Run:
    segments: list           # (dur_us, steps)
    duration: float          # s, including the move to the start
    sent: int = 0
    sent_time: float = 0.0   # s of run queued so far
    go_sent: bool = False
    end_sent: bool = False
    start_of: dict = field(default_factory=dict)   # SEG seq -> run time it starts, s


class ArmBridge:

    def __init__(self, config, port, log=print, clock=time.monotonic):
        self.config = config
        self.port = port
        self.log = log
        self._clock = clock
        self.link = None
        self._next_attempt = 0.0
        self.board = {}          # latest STATUS values
        self.positions = None    # latest positions, joint units, board order
        self.run = None
        self.progress = 0.0
        self.message = ''
        self._last_error = None

    # ----- connection --------------------------------------------------------

    @property
    def connected(self):
        return self.link is not None

    def _connect(self):
        link = BoardLink(self.port)
        try:
            hello = link.check('HELLO')
            proto = int(hello.values.get('proto', 0))
            if proto != p.PROTOCOL_VERSION:
                raise LinkError(f'board speaks protocol {proto}, '
                                f'arm_bridge speaks {p.PROTOCOL_VERSION}')
            board_max = int(hello.values.get('max_rate', 0))
            for j in self.config.joints:
                lo, hi = j.step_limits()
                rate = j.max_rate()
                if rate > board_max:
                    raise LinkError(f'{j.name} needs {rate} steps/s, the board tops out at '
                                    f'{board_max}; lower max_velocity in the config')
                link.check('LIMITS', j.name, lo, hi, rate)
        except LinkError:
            link.close()
            raise
        self.link = link
        self.log(f'Connected to the board on {self.port} '
                 f'(firmware {hello.values.get("fw", "?")})')
        self.message = 'Connected'

    def _disconnect(self, reason):
        if self.link:
            self.link.close()
        self.link = None
        self.board = {}
        if self.run:
            self.run = None
        self._say(f'Lost the board: {reason}')

    def _say(self, message):
        self.message = message
        self.log(message)

    # ----- the loop ----------------------------------------------------------

    def poll(self):
        """One cycle: (re)connect, stream segments, read the status and events."""
        if not self.link:
            now = self._clock()
            if now < self._next_attempt:
                return
            self._next_attempt = now + RECONNECT_PERIOD
            try:
                self._connect()
                self._last_error = None
            except LinkError as e:
                # Say it once, not every second.
                if str(e) != self._last_error:
                    self._last_error = str(e)
                    self._say(f'Waiting for the board on {self.port}: {e}')
                return
        try:
            self._stream()
            self._read_status()
            for event in self.link.take_events():
                self._on_event(event)
        except LinkError as e:
            self._disconnect(e)

    def _read_status(self):
        reply = self.link.check('STATUS')
        self.board = reply.values
        self.positions = self.config.to_units(p.parse_positions(reply.values['pos']))
        if self.run:
            seq = reply.values.get('seg', '0')
            if seq in self.run.start_of:
                self.progress = self.run.start_of[seq]

    def _stream(self):
        run = self.run
        if not run:
            return
        sent_now = 0
        while run.sent < len(run.segments) and self.link.free > 0 and sent_now < MAX_SEGS_PER_POLL:
            dur_us, steps = run.segments[run.sent]
            reply = self.link.request('SEG', dur_us, *steps)
            if not reply.ok:
                self._abort(f'The board refused segment {run.sent + 1}: '
                            f'{reply.code} {reply.message}'.strip())
                return
            # Run time at which this segment starts, for the progress readout.
            run.start_of[str(self.link.last_seq)] = run.sent_time
            run.sent_time += dur_us / 1e6
            run.sent += 1
            sent_now += 1
        # Start once the queue is full or the whole run is queued.
        if not run.go_sent and (self.link.free == 0 or run.sent == len(run.segments)):
            self.link.check('GO')
            run.go_sent = True
        if run.go_sent and not run.end_sent and run.sent == len(run.segments):
            self.link.check('END')
            run.end_sent = True

    def _abort(self, message):
        self.link.request('STOP')
        self.run = None
        self._say(message)

    def _on_event(self, event):
        if event.name == p.DONE:
            if self.run:
                self.progress = self.run.duration
            self.run = None
            self._say('Path finished')
        elif event.name in (p.STOPPED, p.UNDERRUN, p.WATCHDOG):
            was_running = self.run is not None
            self.run = None
            text = {p.STOPPED: 'Stopped',
                    p.UNDERRUN: 'Stopped: the PC fell behind (queue ran empty)',
                    p.WATCHDOG: 'Stopped: the board stopped hearing from the PC'}[event.name]
            if was_running or event.name != p.STOPPED:
                self._say(text)
        elif event.name == p.HOMED:
            self._say('Homed')
        elif event.name == p.EVENT_FAULT:
            self.run = None
            self._say(f'Board fault: {event.values.get("code", "?")}. Clear it and home again.')

    # ----- commands (return (ok, message)) ----------------------------------

    def _command(self, *request):
        if not self.link:
            return False, 'Not connected to the board'
        try:
            reply = self.link.request(*request)
        except LinkError as e:
            self._disconnect(e)
            return False, str(e)
        if not reply.ok:
            return False, f'{request[0]} refused: {reply.code} {reply.message}'.strip()
        return True, 'OK'

    def enable(self, on):
        return self._command('ENABLE' if on else 'DISABLE')

    def home(self):
        ok, message = self._command('HOME')
        if ok:
            self._say('Homing...')
        elif 'NOT_IMPLEMENTED' in message:
            message = 'This firmware cannot home yet (build step 5)'
        return ok, message

    def assume_homed(self):
        """Bench testing only: tell the board it's sitting at the home position."""
        ok, message = self._command('SETPOS', *([0] * len(p.JOINTS)))
        if ok:
            self._say('Told the board it is at home (bench testing)')
        return ok, message

    def clear(self):
        return self._command('CLEAR')

    def stop(self):
        was_running = self.run is not None
        self.run = None
        ok, message = self._command('STOP')
        if ok and was_running:
            self._say('Stopped')
        return ok, message

    def start_run(self, times, positions):
        """Run a path (times in s, positions in board order and joint units)."""
        if not self.link:
            return False, 'Not connected to the board'
        if self.run:
            return False, 'Already running a path'
        try:
            self._read_status()   # the last poll may predate a command just sent
        except LinkError as e:
            self._disconnect(e)
            return False, str(e)
        b = self.board
        if b.get('state') != p.IDLE:
            return False, f'The board is {b.get("state", "not reporting")}, not IDLE'
        if b.get('enabled') != '1':
            return False, 'Enable the motors first'
        if b.get('homed') != '1':
            return False, 'Home the arm first'
        start = p.parse_positions(b['pos'])
        try:
            segments, move, path = plan_run(self.config, start, times, positions)
        except PlanError as e:
            self._say(f'Cannot run: {e}')
            return False, str(e)
        self.run = Run(segments, move + path)
        self.progress = 0.0
        self._say(f'Running: {move:.1f} s to the start, then {path:.1f} s of path')
        return True, 'OK'
