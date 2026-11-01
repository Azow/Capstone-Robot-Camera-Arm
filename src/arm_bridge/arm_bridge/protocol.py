"""The serial protocol between arm_bridge and the board, v1.

The spec is section 4 of docs/control_system_decisions.md; this module and the
firmware must both follow it. Only formatting and parsing live here: no serial
port, no timing, so the bridge and the fake board share it.
"""

from dataclasses import dataclass, field

PROTOCOL_VERSION = 1

# Board joint order. Every position list on the wire is in this order.
JOINTS = ('THETA', 'Z', 'R', 'YAW', 'PITCH')

MAX_LINE = 120
MIN_SEGMENT_US = 1000
MAX_SEGMENT_US = 1000000

# Board states.
IDLE = 'IDLE'
RUNNING = 'RUNNING'
STOPPING = 'STOPPING'
HOMING = 'HOMING'
FAULT = 'FAULT'

# Error codes.
BAD_CMD = 'BAD_CMD'
BAD_ARGS = 'BAD_ARGS'
BAD_STATE = 'BAD_STATE'
NO_LIMITS = 'NO_LIMITS'
NOT_HOMED = 'NOT_HOMED'
DISABLED = 'DISABLED'
QUEUE_FULL = 'QUEUE_FULL'
OUT_OF_LIMITS = 'OUT_OF_LIMITS'
TOO_FAST = 'TOO_FAST'
NOT_IMPLEMENTED = 'NOT_IMPLEMENTED'

# Events.
DONE = 'DONE'
STOPPED = 'STOPPED'
UNDERRUN = 'UNDERRUN'
WATCHDOG = 'WATCHDOG'
HOMED = 'HOMED'
EVENT_FAULT = 'FAULT'


class ProtocolError(Exception):
    """A line from the board that doesn't follow the protocol."""


@dataclass
class Reply:
    seq: str                  # the request's seq, or '-' for a hand-typed request
    ok: bool
    free: int                 # free queue slots (q=)
    code: str = ''            # error code, when not ok
    message: str = ''         # free text after an error code
    values: dict = field(default_factory=dict)   # key=value pairs of an OK reply


@dataclass
class Event:
    name: str
    values: dict = field(default_factory=dict)


def format_request(seq, command, *args):
    line = ' '.join([str(seq), command] + [str(a) for a in args])
    if len(line) > MAX_LINE:
        raise ValueError(f'request longer than {MAX_LINE} characters: {line!r}')
    return line + '\n'


def format_ok(seq, free, **values):
    return ' '.join([str(seq), 'OK', f'q={free}'] + [f'{k}={v}' for k, v in values.items()]) + '\n'


def format_err(seq, code, free, message=''):
    return ' '.join(p for p in (str(seq), 'ERR', code, f'q={free}', message) if p) + '\n'


def format_event(name, **values):
    return ' '.join(['!', name] + [f'{k}={v}' for k, v in values.items()]) + '\n'


def format_positions(positions):
    return ','.join(str(int(p)) for p in positions)


def parse_positions(text):
    values = [int(v) for v in text.split(',')]
    if len(values) != len(JOINTS):
        raise ProtocolError(f'expected {len(JOINTS)} positions, got {text!r}')
    return values


def _pairs(tokens):
    values = {}
    for token in tokens:
        key, sep, value = token.partition('=')
        if not sep:
            raise ProtocolError(f'expected key=value, got {token!r}')
        values[key] = value
    return values


def parse_line(line):
    """Parse a line from the board into a Reply or an Event."""
    tokens = line.strip().split()
    if not tokens:
        raise ProtocolError('empty line')
    if tokens[0] == '!':
        if len(tokens) < 2:
            raise ProtocolError(f'event without a name: {line!r}')
        return Event(tokens[1], _pairs(tokens[2:]))
    if len(tokens) < 3:
        raise ProtocolError(f'not a reply: {line!r}')
    seq, kind = tokens[0], tokens[1]
    if kind == 'OK':
        values = _pairs(tokens[2:])
        if 'q' not in values:
            raise ProtocolError(f'reply without q=: {line!r}')
        return Reply(seq, True, int(values.pop('q')), values=values)
    if kind == 'ERR':
        if len(tokens) < 4 or not tokens[3].startswith('q='):
            raise ProtocolError(f'error reply without q=: {line!r}')
        return Reply(seq, False, int(tokens[3][2:]), code=tokens[2],
                     message=' '.join(tokens[4:]))
    raise ProtocolError(f'not a reply: {line!r}')


def parse_request(line):
    """Split a request into (seq, command, args). seq is '-' when left out."""
    tokens = line.strip().split()
    if not tokens:
        return None
    if tokens[0].isdigit():
        seq, tokens = tokens[0], tokens[1:]
        if not tokens:
            return seq, '', []
    else:
        seq = '-'
    return seq, tokens[0].upper(), tokens[1:]
