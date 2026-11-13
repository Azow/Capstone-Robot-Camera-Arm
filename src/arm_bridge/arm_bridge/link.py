"""The serial link to the board: send a request, wait for its reply (stop-and-wait).

Events the board sends on its own are collected and handed out by take_events().
"""

import serial

from arm_bridge import protocol as p


class LinkError(Exception):
    """The port failed, or the board didn't answer properly. Reconnect."""


class BoardLink:

    def __init__(self, port, timeout=0.5, baud=115200):
        # The board is USB CDC, so the baud rate is ignored; the fake board is a pty.
        try:
            self._serial = serial.Serial(port, baud, timeout=timeout)
        except (serial.SerialException, OSError) as e:
            raise LinkError(str(e)) from e
        self._timeout = timeout
        self._seq = 0
        self._events = []
        self.free = 0           # free queue slots, from the latest reply
        self.text = []          # non-protocol lines (e.g. DRIVER reports) before the last reply
        self._partial = b''     # read_lines(): start of a line still arriving

    @property
    def last_seq(self):
        return self._seq

    def close(self):
        self._serial.close()

    def request(self, command, *args, timeout=None):
        """Send one request and return its Reply (ok or not). `timeout` overrides the
        per-line wait for slow commands."""
        self._seq += 1
        seq = str(self._seq)
        self.text = []
        wait = self._timeout if timeout is None else timeout
        try:
            if wait != self._timeout:
                self._serial.timeout = wait
            self._serial.write(p.format_request(seq, command, *args).encode('ascii'))
            while True:
                raw = self._serial.readline()
                if not raw.endswith(b'\n'):
                    raise LinkError(f'no reply to {command} within {wait} s')
                line = raw.decode('ascii', 'replace')
                try:
                    message = p.parse_line(line)
                except p.ProtocolError:
                    # A report line (DRIVER ...), or a partial line left over from
                    # before a reconnect.
                    if line.strip():
                        self.text.append(line.strip())
                    continue
                if isinstance(message, p.Event):
                    self._events.append(message)
                elif message.seq == seq:
                    self.free = message.free
                    return message
                # A reply to an older request (one that timed out): skip it.
        except (serial.SerialException, OSError) as e:
            raise LinkError(str(e)) from e
        finally:
            if wait != self._timeout:
                try:
                    self._serial.timeout = self._timeout
                except (serial.SerialException, OSError):
                    pass

    def send(self, command, *args):
        """Send a request without waiting for its reply; returns its seq. For long bring-up
        commands that print as they go (arm_bench's STALLSEEK): read with read_lines()."""
        self._seq += 1
        seq = str(self._seq)
        try:
            self._serial.write(p.format_request(seq, command, *args).encode('ascii'))
        except (serial.SerialException, OSError) as e:
            raise LinkError(str(e)) from e
        return seq

    def read_lines(self):
        """The complete lines that have arrived, without waiting."""
        try:
            waiting = self._serial.in_waiting
            if waiting:
                self._partial += self._serial.read(waiting)
        except (serial.SerialException, OSError) as e:
            raise LinkError(str(e)) from e
        *lines, self._partial = self._partial.split(b'\n')
        return [line.decode('ascii', 'replace').strip() for line in lines if line.strip()]

    def check(self, command, *args):
        """Like request(), but an ERR reply raises LinkError."""
        reply = self.request(command, *args)
        if not reply.ok:
            raise LinkError(f'{command} refused: {reply.code} {reply.message}'.strip())
        return reply

    def take_events(self):
        events, self._events = self._events, []
        return events
