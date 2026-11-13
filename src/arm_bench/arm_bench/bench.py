"""The motor test bench's logic, without the window: connect, enable, set ranges, jog, run
speed/acceleration tests, record results.

It drives the board through arm_bridge's ArmBridge (which streams the segments and keeps the
watchdog fed), with the board's limits opened up: the test range you set here is what keeps
a test in bounds, and speeds can go up to the board's cap. Positions are in motor steps,
counted from the lower end of the current joint's range (step 0 = the marker).
"""

import math
import os
import time
from dataclasses import replace

from arm_bridge import protocol as p
from arm_bridge.bridge import ArmBridge
from arm_bridge.link import LinkError

from arm_bench import profile, results, stall

BOARD_MAX_RATE = 40000      # steps/s, the firmware's kMaxRate
MAX_TEST_RATE = 38000       # steps/s; headroom for rounding to whole steps per segment
WIDE_STEPS = 200_000_000    # board soft limits, steps either side of 0: effectively none
STEPS_PER_MOTOR_REV = 200 * 16   # 1.8 deg motors at 16 microsteps (arm.yaml, firmware)
TEST_DONE = 'Test finished'
DRIVERS_TIMEOUT = 3.0
SEEK_SLACK = 5.0            # s past a stall seek's longest possible time before giving up


def is_linear(joint):
    return joint.name in ('Z', 'R')


def unit(joint):
    return 'mm' if is_linear(joint) else 'deg'


def steps_to_display(joint, steps):
    """Motor steps -> deg or mm of the joint (arm.yaml's steps_per_unit)."""
    value = steps / joint.steps_per_unit
    return value * 1000.0 if is_linear(joint) else math.degrees(value)


def display_to_steps(joint, value):
    value = value / 1000.0 if is_linear(joint) else math.radians(value)
    return value * joint.steps_per_unit


def bench_config(config):
    """arm.yaml with the board limits opened: positions effectively unlimited, and a top
    speed at the board's cap (ArmBridge sends these as LIMITS when it connects)."""
    joints = []
    for j in config.joints:
        spu = abs(j.steps_per_unit)
        joints.append(replace(
            j, min=j.home - WIDE_STEPS / spu, max=j.home + WIDE_STEPS / spu,
            # JointConfig.max_rate() = ceil(v * spu * 1.1 + 100): land just under the cap.
            max_velocity=(BOARD_MAX_RATE - 101) / (1.1 * spu)))
    return type(config)(joints)


class Bench:

    def __init__(self, config, port, ranges_path, results_path, log=print,
                 stall_path=None):
        self.arm_config = config                 # arm.yaml as written, for defaults
        self.config = bench_config(config)
        self.ranges_path = ranges_path
        self.results_path = results_path
        self.log = log
        self.bridge = ArmBridge(self.config, port, log=log)
        self.ranges = results.load_ranges(ranges_path)
        self.test = None          # the test running or waiting for a verdict
        self.test_running = False
        self.stall_path = stall_path or os.path.join(
            os.path.dirname(os.path.abspath(results_path)), 'stall_results.csv')
        self.seek = None          # stall seek in progress (dict)
        self.seek_samples = []    # (t_s, steps, sg) of the latest seek
        self.seek_sgthrs = None   # its threshold, for the graph
        self.seek_result = None   # the latest seek's row (stall.FIELDS)
        self._disable_pending = False
        self.stale = set()        # joints switched off on their own: position unknown

    # ----- connection --------------------------------------------------------

    @property
    def connected(self):
        return self.bridge.connected

    def connect(self, port):
        self.bridge.disconnect()
        self.bridge = ArmBridge(self.config, port, log=self.log)
        self.stale = set()
        self.bridge.poll()   # connects: HELLO, then LIMITS for every joint
        return self.connected

    def disconnect(self):
        self.stop()
        self.seek = None
        self.bridge.disconnect()

    def poll(self):
        """Call regularly while connected: streams segments, reads the status."""
        if self.seek is not None:
            # The board is busy with STALLSEEK, which any input aborts: no STATUS polls.
            self._poll_seek()
        elif self.connected:
            self.bridge.poll()
            if self._disable_pending and not self.moving:
                self._disable_pending = False
                if self.disable_all():
                    self.log('E-STOP: stopped, all drivers disabled')
                else:
                    self.log('E-STOP: stopped, but DISABLE failed: switch motor power off')
        if self.test_running and self.bridge.run is None:
            self.test_running = False
            if self.bridge.message != TEST_DONE:
                self.test['stopped_early'] = True
                self.test['why'] = self.bridge.message
            self.log('Test over: check the marker, then mark it Acceptable or Skipped')

    def _request(self, *request, timeout=None):
        """Send a request; returns the Reply, or None (and disconnects) if the link failed."""
        if not self.connected:
            self.log('Not connected')
            return None
        try:
            return self.bridge.link.request(*request, timeout=timeout)
        except LinkError as e:
            self.log(f'Link failed: {e}')
            self.bridge.disconnect()
            return None

    # ----- board state -------------------------------------------------------

    @property
    def board(self):
        return self.bridge.board if self.connected else {}

    @property
    def steps(self):
        pos = self.board.get('pos')
        return p.parse_positions(pos) if pos else None

    @property
    def enabled(self):
        return self.board.get('enabled') == '1'

    @property
    def homed(self):
        """The board knows where the motors are (Set lower here since the last power-off)."""
        return self.board.get('homed') == '1'

    @property
    def per_joint_power(self):
        """The firmware can power single joints (bench build: STATUS has power=)."""
        return 'power' in self.board

    @property
    def power(self):
        """Which joints are powered, board order."""
        if self.per_joint_power:
            return [v == 1 for v in p.parse_positions(self.board['power'])]
        return [self.enabled] * len(p.JOINTS)

    def joint_on(self, joint_index):
        return self.power[joint_index]

    def position_known(self, joint_index):
        """The board's count for this joint is good: it hasn't been switched off (on its own,
        or by Disable all) since it was last zeroed."""
        return self.homed and joint_index not in self.stale

    @property
    def moving(self):
        return (self.seek is not None or self.bridge.run is not None
                or self.board.get('state') in (p.RUNNING, p.STOPPING))

    def enable_all(self):
        """DRIVERS (configure + check every driver), then ENABLE. Returns (ok, report), where
        report has one line per driver."""
        reply = self._request('DRIVERS', timeout=DRIVERS_TIMEOUT)
        if reply is None:
            return False, ['No connection']
        if reply.ok or reply.code != p.BAD_CMD:
            report = [line[len('DRIVER '):] for line in self.bridge.link.text
                      if line.startswith('DRIVER ')]
        else:
            report = ['(this board has no drivers to check: fake board?)']
        drivers_ok = reply.ok or reply.code == p.BAD_CMD
        reply = self._request('ENABLE')
        if reply is None or not reply.ok:
            why = 'no connection' if reply is None else f'{reply.code} {reply.message}'
            return False, report + [f'ENABLE refused: {why}']
        self.bridge.poll()
        on = self.enabled
        report.append('All drivers enabled' if on else 'ENABLE sent but the board says disabled')
        return drivers_ok and on, report

    def disable_all(self):
        reply = self._request('DISABLE')
        if reply is None or not reply.ok:
            return False
        self.stale = set()   # the board clears homed: every joint needs zeroing anyway
        self.bridge.poll()
        return not self.enabled

    def set_power(self, joints_on):
        """Power exactly the joints in `joints_on` (board indices), the rest off. Needs the
        bench firmware. DRIVERS runs first whenever something is switched on (drivers forget
        their settings when motor power goes off). Returns (ok, message)."""
        if not self.per_joint_power:
            return False, 'Single-joint power needs the bench firmware (BENCH_TESTS 1)'
        if self.moving:
            return False, 'Not while moving'
        names = [j.name for j in self.config.joints]
        now = self.power
        turn_on = [i for i in range(len(names)) if i in joints_on and not now[i]]
        turn_off = [i for i in range(len(names)) if i not in joints_on and now[i]]
        notes = []
        if turn_on:
            reply = self._request('DRIVERS', timeout=DRIVERS_TIMEOUT)
            if reply is None:
                return False, 'No connection'
            silent = [line.split()[1] for line in self.bridge.link.text
                      if line.startswith('DRIVER ') and not line.endswith(' ok')]
            if silent:
                notes.append(f'not answering: {", ".join(silent)} (motor power off?)')
            reply = self._request('ENABLE', *(names[i] for i in turn_on))
            if reply is None or not reply.ok:
                return False, f'ENABLE refused: {reply.code} {reply.message}' if reply else \
                    'No connection'
        if turn_off:
            reply = self._request('DISABLE', *(names[i] for i in turn_off))
            if reply is None or not reply.ok:
                return False, f'DISABLE refused: {reply.code} {reply.message}' if reply else \
                    'No connection'
            self.stale.update(turn_off)
        self.bridge.poll()
        on = [names[i] for i, v in enumerate(self.power) if v]
        message = f'Powered: {", ".join(on) or "nothing"}'
        return True, '; '.join([message] + notes)

    def stop(self):
        if not self.connected:
            return
        if self.seek is not None:
            self._send_abort()
        else:
            self.bridge.stop()

    def emergency_stop(self):
        """Stop whatever is moving, then switch every driver off (Space in the window). The
        DISABLE goes out from poll() once the board is still (a STOP ramps down for 0.1 s)."""
        if not self.connected:
            return
        self._disable_pending = True
        self.stop()
        self.poll()

    def _send_abort(self):
        # STALLSEEK ends on any input; the board then runs the STOP too.
        try:
            self.bridge.link.send('STOP')
        except LinkError as e:
            self.log(f'Link failed: {e}')
            self.seek = None
            self.bridge.disconnect()

    # ----- range -------------------------------------------------------------

    def upper(self, joint_index):
        return self.ranges.get(self.config.joints[joint_index].name)

    def set_lower_here(self, joint_index):
        """Call this joint's current position step 0 (the lower end, where the marker goes).

        If the board still knew the position (it was jogged here), the upper end stays at the
        same physical place. If not (moved by hand with the motors off), the saved upper end
        is kept as a distance from here."""
        steps = self.steps
        if steps is None:
            return False, 'Not connected'
        name = self.config.joints[joint_index].name
        if self.position_known(joint_index) and name in self.ranges:
            self.ranges[name] -= steps[joint_index]
            if self.ranges[name] == 0:
                del self.ranges[name]
            results.save_ranges(self.ranges_path, self.ranges)
        steps[joint_index] = 0
        reply = self._request('SETPOS', *steps)
        if reply is None or not reply.ok:
            return False, 'SETPOS refused' if reply else 'No connection'
        self.stale.discard(joint_index)
        self.bridge.poll()
        return True, f'{name}: lower end set here'

    def set_upper_here(self, joint_index):
        steps = self.steps
        if steps is None or not self.position_known(joint_index):
            return False, 'Set the lower end first'
        if steps[joint_index] == 0:
            return False, 'The upper end can\'t be the lower end: jog away from it first'
        name = self.config.joints[joint_index].name
        self.ranges[name] = steps[joint_index]
        results.save_ranges(self.ranges_path, self.ranges)
        return True, f'{name}: upper end set {steps[joint_index]} steps from the lower end'

    # ----- moves -------------------------------------------------------------

    def jog_defaults(self, joint_index):
        """Jog speed and acceleration, display units: a quarter of arm.yaml's limits, capped
        well under the board's top speed."""
        j = self.arm_config.joints[joint_index]
        top = steps_to_display(j, 0.25 * MAX_TEST_RATE)
        speed = steps_to_display(j, j.max_velocity * abs(j.steps_per_unit))
        accel = steps_to_display(j, j.max_accel * abs(j.steps_per_unit))
        return min(0.25 * speed, top), 0.25 * accel

    def _move(self, builder, start_message, done_message):
        return self.bridge.run_segments(builder.segments, builder.duration,
                                        start_message, done_message)

    def _can_move(self, joint_index):
        """None if this joint can move, else why not."""
        name = self.config.joints[joint_index].name
        if not self.joint_on(joint_index):
            return f'{name} is not powered'
        if not self.position_known(joint_index):
            return f'{name}\'s position is unknown: Set lower here'
        return None

    def _rates(self, joint_index, speed, accel):
        j = self.config.joints[joint_index]
        return abs(display_to_steps(j, speed)), abs(display_to_steps(j, accel))

    def move_to(self, joint_index, target_steps, speed, accel):
        """Move one joint to target_steps at speed/accel (display units)."""
        steps = self.steps
        if steps is None:
            return False, 'Not connected'
        why = self._can_move(joint_index)
        if why:
            return False, why
        v, a = self._rates(joint_index, speed, accel)
        if not 0 < v <= MAX_TEST_RATE or a <= 0:
            return False, f'Speed must be above 0 and at most {MAX_TEST_RATE} steps/s'
        b = profile.SegmentBuilder(steps, joint_index)
        b.move_to(target_steps, v, a)
        if not b.segments:
            return False, 'Already there'
        name = self.config.joints[joint_index].urdf_name
        return self._move(b, f'Moving {name} ({b.duration:.1f} s)', f'{name} moved')

    def jog(self, joint_index, delta, speed, accel):
        """Move by delta (display units)."""
        steps = self.steps
        if steps is None:
            return False, 'Not connected'
        j = self.config.joints[joint_index]
        return self.move_to(joint_index, steps[joint_index] + round(display_to_steps(j, delta)),
                            speed, accel)

    def plan_test(self, joint_index, speed, accel, cycles, pause, jog_speed, jog_accel):
        """The test's segments (a SegmentBuilder) and its record, or raise ValueError."""
        steps = self.steps
        if steps is None:
            raise ValueError('Not connected')
        upper = self.upper(joint_index)
        if upper is None:
            raise ValueError('Set the lower and upper ends first')
        why = self._can_move(joint_index)
        if why:
            raise ValueError(why)
        v, a = self._rates(joint_index, speed, accel)
        if not 0 < v <= MAX_TEST_RATE:
            raise ValueError(f'Speed must be above 0 and at most {MAX_TEST_RATE} steps/s')
        if a <= 0 or cycles < 1:
            raise ValueError('Acceleration and cycles must be above 0')
        jv, ja = self._rates(joint_index, jog_speed, jog_accel)
        builder = profile.cycle_test(steps, joint_index, upper, v, a, cycles, pause,
                                     min(jv, MAX_TEST_RATE), ja)
        j = self.config.joints[joint_index]
        record = {
            'joint': j.name, 'speed': f'{speed:g}', 'accel': f'{accel:g}', 'unit': unit(j),
            'travel': f'{steps_to_display(j, upper):.2f}', 'cycles': cycles,
            'pause_s': f'{pause:g}', 'speed_steps_s': round(v), 'accel_steps_s2': round(a),
            'motor_rev_s': f'{v / STEPS_PER_MOTOR_REV:.2f}', 'travel_steps': upper,
            'steps_per_unit': f'{j.steps_per_unit:g}',
        }
        return builder, record

    def run_test(self, joint_index, speed, accel, cycles, pause, jog_speed, jog_accel):
        try:
            builder, record = self.plan_test(joint_index, speed, accel, cycles, pause,
                                             jog_speed, jog_accel)
        except ValueError as e:
            return False, str(e)
        ok, message = self._move(builder,
                                 f'Testing {record["joint"]}: {cycles} cycles, '
                                 f'{builder.duration:.1f} s', TEST_DONE)
        if ok:
            self.test = record
            self.test_running = True
        return ok, message

    @property
    def awaiting_verdict(self):
        return self.test is not None and not self.test_running

    def record_verdict(self, result, note=''):
        """Save the finished test with result ACCEPTABLE or SKIPPED."""
        if not self.awaiting_verdict:
            return False
        row = dict(self.test)
        if row.pop('stopped_early', False):
            note = f'stopped early ({row.pop("why", "")}). {note}'.strip()
        row.pop('why', None)
        row.update(time=time.strftime('%Y-%m-%d %H:%M:%S'), result=result, note=note)
        results.append_result(self.results_path, row)
        self.test = None
        return True

    def discard_test(self):
        if not self.test_running:
            self.test = None

    def set_zero_here(self, joint_index):
        """Call this joint's position step 0 (the stall test's zero: a hard stop)."""
        steps = self.steps
        if steps is None:
            return False, 'Not connected'
        steps[joint_index] = 0
        reply = self._request('SETPOS', *steps)
        if reply is None or not reply.ok:
            return False, 'SETPOS refused' if reply else 'No connection'
        self.stale.discard(joint_index)
        self.bridge.poll()
        return True, f'{self.config.joints[joint_index].name}: zero set here'

    # ----- focus stall test (bench firmware's STALLSEEK) --------------------

    def start_stall_seek(self, steps, rate, sgthrs, current_ma, record):
        """Turn FOCUS up to `steps` (signed) at `rate` steps/s until StallGuard trips.
        `record` is the start of the result row (speed etc., stall.FIELDS)."""
        if not self.connected:
            return False, 'Not connected'
        if self.moving:
            return False, 'Not while moving'
        joint = [j.name for j in self.config.joints].index(stall.JOINT)
        if not self.joint_on(joint):
            return False, f'{stall.JOINT} is not powered'
        if not self.position_known(joint):
            return False, 'Set zero first (put the ring on a stop by hand)'
        if steps == 0 or not 0 < rate <= stall.MAX_RATE:
            return False, f'Speed must be above 0 and at most {stall.MAX_RATE} steps/s'
        try:
            seq = self.bridge.link.send('STALLSEEK', stall.JOINT, steps, rate, sgthrs,
                                        current_ma)
        except LinkError as e:
            self.log(f'Link failed: {e}')
            self.bridge.disconnect()
            return False, 'Link failed'
        self.seek = {'seq': seq, 'record': dict(record), 'start': self.steps[joint],
                     'joint': joint, 'deadline': time.monotonic() + abs(steps) / rate
                     + SEEK_SLACK}
        self.seek_samples = []
        self.seek_sgthrs = sgthrs
        self.seek_result = None
        self.bridge.message = 'Stall seek running'
        return True, 'OK'

    def _poll_seek(self):
        try:
            lines = self.bridge.link.read_lines()
        except LinkError as e:
            self.log(f'Link failed: {e}')
            self.seek = None
            self.bridge.disconnect()
            return
        for line in lines:
            sample = stall.parse_sample(line)
            if sample:
                self.seek_samples.append(sample)
                continue
            try:
                message = p.parse_line(line)
            except p.ProtocolError:
                self.log(line)
                continue
            if isinstance(message, p.Reply) and message.seq == self.seek['seq']:
                self._finish_seek(message)
                return
        if time.monotonic() > self.seek['deadline']:
            self.log('Stall seek: no answer from the board in time; sending STOP')
            self.seek = None
            self._disable_pending = True
            self.bridge.stop()

    def _finish_seek(self, reply):
        seek, self.seek = self.seek, None
        self.bridge.poll()   # position and state again
        if not reply.ok:
            hint = ''
            if reply.code == p.BAD_CMD:
                hint = (' (no STALLSEEK: flash the bench build, BENCH_TESTS 1 in '
                        'arm_firmware.ino)')
            self.log(f'Stall seek refused: {reply.code} {reply.message}{hint}')
            self.bridge.message = 'Stall seek refused'
            return
        v = reply.values
        joint = self.arm_config.joints[seek['joint']]
        steps = int(v.get('steps', 0))
        row = dict(seek['record'])
        row.update(time=time.strftime('%Y-%m-%d %H:%M:%S'), result=v.get('result', '?'),
                   steps=steps, sg_min=v.get('sg_min', ''), t_ms=v.get('t_ms', ''),
                   steps_per_unit=f'{joint.steps_per_unit:g}')
        if row['result'] == stall.STALL_RESULT:
            # From zero (the stop it started from, by hand) to where it stalled.
            span = abs(seek['start'] + steps)
            c = stall.compare(joint, span)
            row.update(span_steps=span, predicted_steps=c['predicted_steps'],
                       error_pct=f'{c["error_pct"]:+.2f}',
                       implied_steps_per_unit=f'{c["implied_steps_per_unit"]:.2f}',
                       implied_lens_teeth=f'{c["implied_lens_teeth"]:.1f}')
        self.seek_result = row
        self.bridge.message = f'Stall seek: {row["result"]}'
        if row['result'] == stall.ABORTED_RESULT:
            self.log(f'Stall seek aborted after {steps} steps (not saved)')
        else:
            stall.append(self.stall_path, row)
            self.log(f'Stall seek: {row["result"]} after {steps} steps')

    # ----- console -----------------------------------------------------------

    def send_raw(self, line):
        """A hand-typed command (e.g. SPIN THETA 3200 800). Returns the lines to show.
        Blocks until the board answers: SPIN waits for the whole spin."""
        parsed = p.parse_request(line)
        if parsed is None or not parsed[1]:
            return []
        _, command, args = parsed
        if self.moving:
            return ['Not while moving']
        timeout = None
        if command == 'SPIN' and len(args) >= 2:
            try:
                rate = int(args[2]) if len(args) >= 3 else 400
                timeout = abs(int(args[1])) / max(rate, 1) + 2.0
            except ValueError:
                pass
        elif command == 'DRIVERS':
            timeout = DRIVERS_TIMEOUT
        reply = self._request(command, *args, timeout=timeout)
        if reply is None:
            return ['(no reply: disconnected)']
        out = list(self.bridge.link.text)
        if reply.ok:
            values = ' '.join(f'{k}={v}' for k, v in reply.values.items())
            out.append(' '.join(x for x in ('OK', reply.message, values) if x))
        else:
            out.append(f'ERR {reply.code} {reply.message}'.strip())
        self.bridge.poll()
        return out
