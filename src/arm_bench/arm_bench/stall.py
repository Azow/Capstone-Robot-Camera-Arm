"""The focus stall test: is arm.yaml's FOCUS steps_per_unit right?

Put the focus ring on one of its hard stops by hand and call that zero, then let the board
turn it toward the other stop until StallGuard trips (the firmware's bench-build STALLSEEK
command). The distance from zero to where it stalled is the ring's whole throw in motor
steps, measured. arm.yaml predicts it: (max - min) x steps_per_unit, 300 deg x 916.73 = 4800
steps. If they match, the gear ratio (and the guessed lens tooth count) is right.

This module is the arithmetic and the files; Bench runs the command and the window shows it.
"""

import csv
import math
import os

JOINT = 'FOCUS'
MOTOR_GEAR_TEETH = 60     # 0.8 module brass gear on the motor (arm.yaml FOCUS comment)
MOTOR_STEPS_PER_REV = 200 * 16   # FOCUS's 1.8 deg motor at 16 microsteps (the sketch)
STALL_RESULT = 'stall'
LIMIT_RESULT = 'limit'
ABORTED_RESULT = 'aborted'

# Defaults for the window. Current: about 40% of the 450 mA run current, so a stall can't
# hurt the lens's stop or the gears.
DEFAULT_SPEED = 30.0          # ring deg/s
DEFAULT_SGTHRS = 30
DEFAULT_CURRENT = 180         # mA
MIN_CURRENT = 50              # mA, the firmware's minimum
RUN_CURRENT = 450             # mA, FOCUS's run current in the sketch: the most a seek may use
MAX_RATE = 20000              # steps/s, the firmware's kMaxSpinRate
TRAVEL_MARGIN = 1.2           # default max travel: this times the predicted span

FIELDS = ('time', 'result', 'direction', 'speed_deg_s', 'rate_steps_s', 'sgthrs',
          'current_ma', 'steps', 'span_steps', 'predicted_steps', 'error_pct',
          'implied_steps_per_unit', 'implied_lens_teeth', 'sg_min', 't_ms', 'steps_per_unit')


def parse_sample(line):
    """'SG t=120 steps=58 sg=231' -> (t_s, steps, sg), or None for any other line."""
    tokens = line.split()
    if not tokens or tokens[0] != 'SG':
        return None
    try:
        values = dict(t.split('=', 1) for t in tokens[1:])
        return int(values['t']) / 1000.0, int(values['steps']), int(values['sg'])
    except (KeyError, ValueError):
        return None


def throw(joint):
    """The ring's whole throw, rad: arm.yaml's max - min (the lens's 300 deg)."""
    return joint.max - joint.min


def predicted_span(joint):
    """Motor steps from one hard stop to the other, by arm.yaml."""
    return throw(joint) * abs(joint.steps_per_unit)


def deg_to_steps(joint, deg):
    return math.radians(deg) * abs(joint.steps_per_unit)


def steps_to_deg(joint, steps):
    return math.degrees(steps / abs(joint.steps_per_unit))


def compare(joint, span_steps):
    """What a measured span says about arm.yaml: error and the implied numbers."""
    predicted = predicted_span(joint)
    ratio = span_steps / (MOTOR_STEPS_PER_REV * throw(joint) / (2 * math.pi))
    return {
        'span_steps': span_steps,
        'predicted_steps': round(predicted),
        'error_pct': 100.0 * (span_steps - predicted) / predicted,
        'implied_steps_per_unit': span_steps / throw(joint),
        'implied_lens_teeth': MOTOR_GEAR_TEETH * ratio,
    }


def load(path):
    if not os.path.exists(path):
        return []
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def append(path, row):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS, extrasaction='ignore')
        if new:
            writer.writeheader()
        writer.writerow(row)
