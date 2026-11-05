"""Manual jogging window: move the real arm one joint at a time.

Opened from the editor panel's Arm row ("Manual jogging"); it is a test and planning tool,
not part of making paths. Use it to check that each motor turns the right way and that the
steps-per-unit numbers in arm_bridge's config are right (move 10 degrees, measure 10
degrees), to find where the soft limits should be, and to read off joint positions.

Each move goes through arm_bridge's /arm/move_joint: a smooth move of one joint, at the
chosen fraction of its speed and acceleration limits. Rotary joints are shown in degrees
and linear ones (Z, R) in millimetres; arm_bridge works in rad and m.
"""

import math

from camera_path_interfaces.srv import MoveJoint
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QColor, QFont, QKeySequence, QPalette
from python_qt_binding.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel, QPushButton,
    QStyle, QVBoxLayout, QWidget,
)
from std_srvs.srv import Trigger

MOVE_JOINT_SERVICE = '/arm/move_joint'
STOP_SERVICE = '/arm/stop'
LINEAR_JOINTS = ('Z', 'R')
# Speed dropdown: fraction of each joint's max speed and acceleration.
SPEEDS = (('10%', 0.1), ('25%', 0.25), ('50%', 0.5), ('100%', 1.0))
DEFAULT_SPEED = '25%'
# Step sizes for the -/+ buttons, in display units.
ROTARY_STEPS = (0.1, 1.0, 5.0, 15.0, 45.0)    # degrees
LINEAR_STEPS = (0.1, 1.0, 5.0, 20.0, 50.0)    # mm
DEFAULT_STEP = 5.0
STOP_COLOR = QColor(200, 30, 30)


def _is_linear(name):
    return name in LINEAR_JOINTS


def _to_display(name, value):
    """rad -> degrees, m -> mm."""
    return value * 1000.0 if _is_linear(name) else math.degrees(value)


def _from_display(name, value):
    return value / 1000.0 if _is_linear(name) else math.radians(value)


def _unit(name):
    return 'mm' if _is_linear(name) else '°'


class _JointRow:
    """The widgets for one joint."""

    def __init__(self, window, grid, row, name):
        self.name = name
        unit = _unit(name)
        mono = QFont('monospace')
        mono.setStyleHint(QFont.Monospace)

        self.position = QLabel('-')
        self.position.setFont(mono)
        self.position.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.steps = QLabel('-')
        self.steps.setFont(mono)
        self.steps.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.limits = QLabel('')
        self.step_box = QComboBox()
        for size in (LINEAR_STEPS if _is_linear(name) else ROTARY_STEPS):
            self.step_box.addItem(f'{size:g} {unit}', size)
        self.step_box.setCurrentIndex(self.step_box.findData(DEFAULT_STEP))
        self.minus = QPushButton('−')
        self.plus = QPushButton('+')
        for button in (self.minus, self.plus):
            button.setFixedWidth(36)
        self.minus.clicked.connect(lambda: window.nudge(self, -1))
        self.plus.clicked.connect(lambda: window.nudge(self, +1))
        self.target = QDoubleSpinBox()
        self.target.setDecimals(1 if _is_linear(name) else 2)
        self.target.setSuffix(f' {unit}')
        self.target.setMinimumWidth(110)
        self.go = QPushButton('Go')
        self.go.clicked.connect(lambda: window.go_to(self))
        self.buttons = (self.minus, self.plus, self.go)

        name_label = QLabel(f'<b>{name}</b>')
        for column, widget in enumerate((name_label, self.position, self.steps, self.limits,
                                         self.minus, self.step_box, self.plus, self.target,
                                         self.go)):
            grid.addWidget(widget, row, column)

    def show(self, position, steps, lower, upper):
        unit = _unit(self.name)
        lo, hi = _to_display(self.name, lower), _to_display(self.name, upper)
        self.position.setText(f'{_to_display(self.name, position):9.2f} {unit}')
        self.steps.setText(f'{steps:d}')
        self.limits.setText(f'[{lo:g}, {hi:g}] {unit}')
        if (self.target.minimum(), self.target.maximum()) != (lo, hi):
            self.target.setRange(lo, hi)


class JogWindow(QWidget):

    def __init__(self, node, parent=None):
        super().__init__(parent, Qt.Window)
        self.setWindowTitle('Manual Jogging')
        self._move_client = node.create_client(MoveJoint, MOVE_JOINT_SERVICE)
        self._stop_client = node.create_client(Trigger, STOP_SERVICE)
        self._status = None
        self._rows = {}

        self._state = QLabel('arm_bridge is not running')
        self._speed_box = QComboBox()
        for label, scale in SPEEDS:
            self._speed_box.addItem(label, scale)
        self._speed_box.setCurrentText(DEFAULT_SPEED)
        self._speed_box.setToolTip('Fraction of each joint\'s max speed and acceleration')
        self._stop = QPushButton(self.style().standardIcon(QStyle.SP_MediaStop), 'Stop')
        self._stop.setToolTip('Stop the arm now (Esc)')
        self._stop.setShortcut(QKeySequence(Qt.Key_Escape))
        palette = self._stop.palette()
        palette.setColor(QPalette.ButtonText, STOP_COLOR)
        self._stop.setPalette(palette)
        self._stop.clicked.connect(self._on_stop)
        top = QHBoxLayout()
        top.addWidget(QLabel('Speed'))
        top.addWidget(self._speed_box)
        top.addSpacing(12)
        top.addWidget(self._stop)
        top.addStretch(1)

        self._grid = QGridLayout()
        for column, title in enumerate(('Joint', 'Position', 'Steps', 'Limits', '',
                                        'Step', '', 'Go to', '')):
            self._grid.addWidget(QLabel(f'<i>{title}</i>'), 0, column)

        self._copy = QPushButton('Copy positions')
        self._copy.setToolTip('Copy every joint\'s position to the clipboard (rad and m)')
        self._copy.clicked.connect(self._on_copy)
        self._message = QLabel('')
        self._message.setWordWrap(True)
        bottom = QHBoxLayout()
        bottom.addWidget(self._copy)
        bottom.addWidget(self._message, 1)

        warning = QLabel('These buttons move the real arm. Keep clear of it. '
                         'Home (or Set home here) and turn the motors on in the editor first.')
        warning.setWordWrap(True)

        layout = QVBoxLayout(self)
        layout.addWidget(warning)
        layout.addWidget(self._state)
        layout.addLayout(top)
        layout.addLayout(self._grid)
        layout.addLayout(bottom)
        self.update_status(None)

    # ----- status from arm_bridge (forwarded by the editor panel) -----------

    def update_status(self, msg):
        """Show an ArmStatus, or None when arm_bridge has gone quiet."""
        self._status = msg
        if msg is not None and not self._rows and msg.joint_names:
            for i, name in enumerate(msg.joint_names):
                self._rows[name] = _JointRow(self, self._grid, i + 1, name)
        can_move = bool(msg and msg.connected and msg.state == 'IDLE' and msg.enabled
                        and msg.homed and not msg.running)
        for row in self._rows.values():
            for button in row.buttons:
                button.setEnabled(can_move)
        self._stop.setEnabled(bool(msg and msg.connected))
        self._copy.setEnabled(bool(msg and msg.positions))

        if msg is None:
            self._state.setText('arm_bridge is not running')
            return
        if not msg.connected:
            self._state.setText(f'Not connected.  {msg.message}')
            return
        for name, position, steps, lower, upper in zip(
                msg.joint_names, msg.positions, msg.steps, msg.lower, msg.upper):
            self._rows[name].show(position, steps, lower, upper)
        flags = [msg.state, 'homed' if msg.homed else 'not homed',
                 'motors on' if msg.enabled else 'motors off']
        self._state.setText(f'{", ".join(flags)}.  {msg.message}')

    # ----- moves -------------------------------------------------------------

    def nudge(self, row, sign):
        value = _from_display(row.name, sign * row.step_box.currentData())
        self._move(row.name, value, relative=True)

    def go_to(self, row):
        self._move(row.name, _from_display(row.name, row.target.value()), relative=False)

    def _move(self, name, value, relative):
        if not self._move_client.service_is_ready():
            self._message.setText('arm_bridge is not running.')
            return
        request = MoveJoint.Request(joint=name, value=float(value), relative=relative,
                                    speed_scale=float(self._speed_box.currentData()))
        future = self._move_client.call_async(request)
        future.add_done_callback(self._on_reply)

    def _on_stop(self):
        if self._stop_client.service_is_ready():
            self._stop_client.call_async(Trigger.Request()).add_done_callback(self._on_reply)

    def _on_reply(self, future):
        response = future.result()
        if response is None:
            self._message.setText('arm_bridge did not answer.')
        elif not response.success:
            self._message.setText(response.message)
        else:
            self._message.setText('')

    def _on_copy(self):
        msg = self._status
        text = '  '.join(f'{name}={value:.6f}'
                         for name, value in zip(msg.joint_names, msg.positions))
        QApplication.clipboard().setText(text)
        self._message.setText(f'Copied: {text}')
