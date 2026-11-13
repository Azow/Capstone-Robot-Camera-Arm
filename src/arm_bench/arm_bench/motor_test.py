"""Motor test bench window: find each joint's top speed and acceleration without encoders.

Two tests, one tab each. Speed / accel: put a marker at the joint's lower end, set the lower and upper ends, then
run out-and-back cycles at a speed and acceleration. Every test ends back at the lower end:
if the marker still lines up, no steps were skipped (a skip loses 4 full steps, 7.2 deg at
the motor, so it shows). Mark it Acceptable or Skipped and it goes into results.csv.
Focus stall test (stall_panel.py, needs the bench firmware build): measures the focus
ring's throw between its hard stops by StallGuard and compares it with arm.yaml.

Esc stops. Space stops and disables every driver (an emergency stop), except while typing
in the console box.

    ros2 run arm_bench motor_test --port /dev/ttyACM0
    ros2 run arm_bench motor_test --port /tmp/fake_octopus   (with: ros2 run arm_bridge fake_board)

Talks to the board over serial directly; no other ROS nodes are needed.
"""

import argparse
import glob
import os
import sys

from python_qt_binding.QtCore import QEvent, QObject, Qt, QTimer
from python_qt_binding.QtGui import QColor, QFont, QKeySequence, QPalette
from python_qt_binding.QtWidgets import (
    QAbstractItemView, QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout,
    QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMessageBox, QPlainTextEdit,
    QPushButton,
    QShortcut, QSizePolicy, QSpinBox, QTableWidget, QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget,
)

from arm_bridge.planner import ArmConfig

from arm_bench import results
from arm_bench.stall_panel import StallPanel
from arm_bench.bench import (
    MAX_TEST_RATE, STEPS_PER_MOTOR_REV, Bench, display_to_steps, steps_to_display, unit,
)

POLL_MS = 20
ROTARY_STEPS = (0.1, 1.0, 5.0, 15.0, 45.0, 90.0)   # deg
LINEAR_STEPS = (0.1, 1.0, 5.0, 20.0, 50.0, 100.0)  # mm
STOP_COLOR = QColor(200, 30, 30)
Z_INDEX = 1   # board order: THETA Z R YAW PITCH FOCUS
POWER_STYLE = ('QPushButton { min-width: 64px; } '
               'QPushButton:checked { background-color: rgb(60, 150, 70); color: white; '
               'font-weight: bold; }')
HALT_COLOR = QColor(215, 120, 20)
GOOD_COLOR = QColor(60, 150, 70)
TABLE_COLUMNS = ('time', 'joint', 'result', 'speed', 'accel', 'unit', 'travel', 'cycles',
                 'speed_steps_s', 'accel_steps_s2', 'motor_rev_s', 'note')
HELP = ('1. Pick a joint and power it (Power row; with "Only the selected joint" only it '
        'stays on).  2. Put it at its lower end (jog there, or switch it off, move it by hand '
        'and switch it on again) and line up your marker.  3. Set lower '
        'here, jog to the far end, Set upper here.  4. Run test, then check the marker: '
        'Acceptable or Skipped.  If it skipped, realign to the marker and Set lower here again.'
        '  Esc stops.  Space stops and disables all drivers.')


def _default_config():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('arm_bridge'), 'config', 'arm.yaml')
    except Exception:   # not installed through colcon: use the source tree
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, '..', '..', 'arm_bridge', 'config', 'arm.yaml')


def _ports():
    found = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/octopus')
                   + glob.glob('/tmp/fake_octopus'))
    return found or ['/dev/ttyACM0']


def _mono(widget):
    font = QFont('monospace')
    font.setStyleHint(QFont.Monospace)
    widget.setFont(font)
    return widget


def _spin(decimals, maximum, suffix, value):
    box = QDoubleSpinBox()
    box.setDecimals(decimals)
    box.setRange(0.0, maximum)
    box.setSuffix(suffix)
    box.setValue(value)
    box.setMinimumWidth(150)
    return box


class _SpaceStop(QObject):
    """Space anywhere in the app = emergency stop. Swallowed, so it never clicks the focused
    button. Not while typing a command in the console box (and the board is still)."""

    def __init__(self, window):
        super().__init__(window)
        self.window = window

    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress and event.key() == Qt.Key_Space:
            w = self.window
            if w.console_input.hasFocus() and not w.bench.moving:
                return False
            if not event.isAutoRepeat():
                w.emergency_stop()
            return True
        return False


def _color_button(button, color):
    palette = button.palette()
    palette.setColor(QPalette.Button, color)
    palette.setColor(QPalette.ButtonText, Qt.white)
    button.setPalette(palette)
    button.setAutoFillBackground(True)


class MotorTestWindow(QWidget):

    def __init__(self, bench, port):
        super().__init__()
        self.bench = bench
        self.setWindowTitle('Motor Test Bench')
        bench.log = self._log
        bench.bridge.log = self._log

        layout = QVBoxLayout(self)
        layout.addLayout(self._build_connection(port))
        layout.addWidget(self._build_power())
        self.drivers = QLabel('')
        self.drivers.setWordWrap(True)
        layout.addWidget(self.drivers)
        speed_page = QWidget()
        page = QVBoxLayout(speed_page)
        help_label = QLabel(HELP)
        help_label.setWordWrap(True)
        help_label.setStyleSheet('color: gray')
        page.addWidget(help_label)
        middle = QHBoxLayout()
        page.addLayout(middle)
        left = QVBoxLayout()
        left.addWidget(self._build_joint())
        left.addWidget(self._build_jog())
        left.addStretch()
        middle.addLayout(left)
        middle.addWidget(self._build_test(), 1)
        self.stall_panel = StallPanel(self)
        self.modes = QTabWidget()
        self.modes.addTab(speed_page, 'Speed / accel test')
        self.modes.addTab(self.stall_panel, 'Focus stall test')
        self.modes.currentChanged.connect(self._follow_selection)
        layout.addWidget(self.modes, 2)

        tabs = QTabWidget()
        tabs.addTab(self._build_results(), 'Results')
        tabs.addTab(self._build_console(), 'Console / log')
        layout.addWidget(tabs, 1)

        shortcut = QShortcut(QKeySequence(Qt.Key_Escape), self)
        shortcut.setContext(Qt.ApplicationShortcut)
        shortcut.activated.connect(self.stop)
        self._space = _SpaceStop(self)
        QApplication.instance().installEventFilter(self._space)

        self._joint_changed()
        self._load_results()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(POLL_MS)
        self.resize(1150, 900)

    # ----- layout ------------------------------------------------------------

    def _build_connection(self, port):
        row = QHBoxLayout()
        self.port_box = QComboBox()
        self.port_box.setEditable(True)
        self.port_box.addItems(_ports())
        self.port_box.setCurrentText(port)
        self.port_box.setMinimumWidth(170)
        self.connect_button = QPushButton('Connect')
        self.connect_button.clicked.connect(self._toggle_connection)
        self.state = QLabel('Not connected')
        self.state.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)  # never squeeze the stop buttons
        self.enable_button = QPushButton('Enable all')
        self.enable_button.clicked.connect(self._enable_all)
        self.disable_button = QPushButton('Disable all')
        self.disable_button.clicked.connect(self._disable_all)
        self.stop_button = QPushButton('STOP (Esc)')
        self.stop_button.setMinimumHeight(40)
        self.stop_button.setMinimumWidth(120)
        bold = self.stop_button.font()
        bold.setBold(True)
        self.stop_button.setFont(bold)
        _color_button(self.stop_button, HALT_COLOR)
        self.stop_button.clicked.connect(self.stop)
        self.estop_button = QPushButton('E-STOP + DISABLE (Space)')
        self.estop_button.setToolTip('Stop, then switch every driver off. Motors go limp: '
                                     'Z will drop!')
        self.estop_button.setMinimumHeight(40)
        self.estop_button.setMinimumWidth(200)
        self.estop_button.setFont(bold)
        _color_button(self.estop_button, STOP_COLOR)
        self.estop_button.clicked.connect(self.emergency_stop)
        for widget in (QLabel('Port'), self.port_box, self.connect_button):
            row.addWidget(widget)
        row.addWidget(self.state, 1)   # takes the free space; long messages get cut, not the buttons
        for widget in (self.enable_button, self.disable_button, self.stop_button,
                       self.estop_button):
            row.addWidget(widget)
        return row

    def _build_power(self):
        box = QWidget()
        box.setStyleSheet(POWER_STYLE)
        row = QHBoxLayout(box)
        row.setContentsMargins(0, 0, 0, 0)
        row.addWidget(QLabel('Power:'))
        self.power_buttons = []
        for i, j in enumerate(self.bench.config.joints):
            button = QPushButton(j.name)
            button.setCheckable(True)
            button.setToolTip(f'Switch {j.name} on or off on its own (bench firmware). A joint '
                              'switched off loses its position: Set lower here after.')
            button.clicked.connect(lambda _, i=i: self._toggle_power(i))
            row.addWidget(button)
            self.power_buttons.append(button)
        self.power_only = QPushButton('Power this joint only')
        self.power_only.clicked.connect(self._power_selected_only)
        self.only_selected = QCheckBox('Only the selected joint')
        self.only_selected.setChecked(True)
        self.only_selected.setToolTip('Picking a joint powers it and switches the others off '
                                      '(once anything is powered). Asks before Z.')
        self.only_selected.toggled.connect(self._follow_selection)
        self.power_note = QLabel('')
        self.power_note.setStyleSheet('color: gray')
        row.addWidget(self.power_only)
        row.addWidget(self.only_selected)
        row.addWidget(self.power_note)
        row.addStretch()
        return box

    def _build_joint(self):
        box = QGroupBox('Joint and range')
        grid = QGridLayout(box)
        self.joint_box = QComboBox()
        for j in self.bench.config.joints:
            self.joint_box.addItem(f'{j.name}  ({j.urdf_name})')
        self.joint_box.currentIndexChanged.connect(self._joint_changed)
        self.position = _mono(QLabel('-'))
        self.lower_label = _mono(QLabel('0  (marker)'))
        self.upper_label = _mono(QLabel('not set'))
        self.set_lower = QPushButton('Set lower here')
        self.set_lower.setToolTip('Call this position the lower end (step 0). Line the marker '
                                  'up here first.')
        self.set_lower.clicked.connect(self._set_lower)
        self.set_upper = QPushButton('Set upper here')
        self.set_upper.clicked.connect(self._set_upper)
        self.go_lower = QPushButton('Go to lower')
        self.go_lower.clicked.connect(lambda: self._go(0))
        self.go_upper = QPushButton('Go to upper')
        self.go_upper.clicked.connect(lambda: self._go(self.bench.upper(self._joint())))
        grid.addWidget(QLabel('Joint'), 0, 0)
        grid.addWidget(self.joint_box, 0, 1, 1, 2)
        grid.addWidget(QLabel('Position'), 1, 0)
        grid.addWidget(self.position, 1, 1, 1, 2)
        grid.addWidget(QLabel('Lower end'), 2, 0)
        grid.addWidget(self.lower_label, 2, 1)
        grid.addWidget(self.set_lower, 2, 2)
        grid.addWidget(QLabel('Upper end'), 3, 0)
        grid.addWidget(self.upper_label, 3, 1)
        grid.addWidget(self.set_upper, 3, 2)
        grid.addWidget(self.go_lower, 4, 1)
        grid.addWidget(self.go_upper, 4, 2)
        return box

    def _build_jog(self):
        box = QGroupBox('Jog')
        grid = QGridLayout(box)
        self.jog_step = QComboBox()
        self.minus = QPushButton('−')
        self.plus = QPushButton('+')
        for button in (self.minus, self.plus):
            button.setFixedWidth(44)
        self.minus.clicked.connect(lambda: self._jog(-1))
        self.plus.clicked.connect(lambda: self._jog(+1))
        self.jog_speed = _spin(2, 1e6, '', 1.0)
        self.jog_accel = _spin(2, 1e7, '', 1.0)
        grid.addWidget(self.minus, 0, 0)
        grid.addWidget(self.jog_step, 0, 1)
        grid.addWidget(self.plus, 0, 2)
        grid.addWidget(QLabel('Jog speed'), 1, 0)
        grid.addWidget(self.jog_speed, 1, 1, 1, 2)
        grid.addWidget(QLabel('Jog accel'), 2, 0)
        grid.addWidget(self.jog_accel, 2, 1, 1, 2)
        note = QLabel('Jog speed is also used to get back to the lower end before a test.')
        note.setWordWrap(True)
        note.setStyleSheet('color: gray')
        grid.addWidget(note, 3, 0, 1, 3)
        return box

    def _build_test(self):
        box = QGroupBox('Speed / acceleration test')
        grid = QGridLayout(box)
        self.speed = _spin(2, 1e6, '', 1.0)
        self.accel = _spin(2, 1e7, '', 1.0)
        self.speed_steps = _mono(QLabel(''))
        self.accel_steps = _mono(QLabel(''))
        self.cycles = QSpinBox()
        self.cycles.setRange(1, 1000)
        self.cycles.setValue(5)
        self.pause = _spin(2, 10.0, ' s', 0.5)
        self.estimate = QLabel('')
        self.run_button = QPushButton('Run test')
        self.run_button.setMinimumHeight(36)
        self.run_button.clicked.connect(self._run_test)
        for widget in (self.speed, self.accel, self.cycles, self.pause):
            widget.valueChanged.connect(self._update_test_labels)
        self.verdict = QLabel('')
        self.verdict.setWordWrap(True)
        self.acceptable = QPushButton('✓ Acceptable')
        _color_button(self.acceptable, GOOD_COLOR)
        self.acceptable.clicked.connect(lambda: self._verdict(results.ACCEPTABLE))
        self.skipped = QPushButton('✗ Skipped')
        _color_button(self.skipped, STOP_COLOR)
        self.skipped.clicked.connect(lambda: self._verdict(results.SKIPPED))
        self.discard = QPushButton('Discard')
        self.discard.clicked.connect(self._discard)
        self.note = QLineEdit()
        self.note.setPlaceholderText('Note (optional): noise, load, what you saw...')

        grid.addWidget(QLabel('Top speed'), 0, 0)
        grid.addWidget(self.speed, 0, 1)
        grid.addWidget(self.speed_steps, 0, 2)
        grid.addWidget(QLabel('Acceleration'), 1, 0)
        grid.addWidget(self.accel, 1, 1)
        grid.addWidget(self.accel_steps, 1, 2)
        grid.addWidget(QLabel('(same rate speeding up and slowing down)'), 2, 1, 1, 2)
        grid.addWidget(QLabel('Cycles'), 3, 0)
        grid.addWidget(self.cycles, 3, 1)
        grid.addWidget(QLabel('lower → upper → lower, each'), 3, 2)
        grid.addWidget(QLabel('Pause at ends'), 4, 0)
        grid.addWidget(self.pause, 4, 1)
        grid.addWidget(self.estimate, 5, 0, 1, 3)
        grid.addWidget(self.run_button, 6, 0, 1, 3)
        grid.addWidget(self.verdict, 7, 0, 1, 3)
        verdicts = QHBoxLayout()
        for widget in (self.acceptable, self.skipped, self.discard):
            verdicts.addWidget(widget)
        grid.addLayout(verdicts, 8, 0, 1, 3)
        grid.addWidget(self.note, 9, 0, 1, 3)
        grid.setColumnStretch(2, 1)
        return box

    def _build_results(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        top = QHBoxLayout()
        self.only_joint = QCheckBox('Only this joint')
        self.only_joint.setChecked(True)
        self.only_joint.toggled.connect(self._load_results)
        path = QLabel(f'Saved to {self.bench.results_path}')
        path.setTextInteractionFlags(Qt.TextSelectableByMouse)
        path.setStyleSheet('color: gray')
        top.addWidget(self.only_joint)
        top.addStretch()
        top.addWidget(path)
        layout.addLayout(top)
        self.table = QTableWidget(0, len(TABLE_COLUMNS))
        self.table.setHorizontalHeaderLabels(
            ['time', 'joint', 'result', 'speed', 'accel', 'unit', 'travel', 'cycles',
             'steps/s', 'steps/s²', 'motor rev/s', 'note'])
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSortingEnabled(True)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.table)
        return widget

    def _build_console(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self.console = _mono(QPlainTextEdit())
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(2000)
        self.console_input = _mono(QLineEdit())
        self.console_input.setPlaceholderText(
            'Board command, e.g. SPIN THETA 3200 800, ENDSTOPS, INFO, HELP  (Enter to send; '
            'SPIN blocks the window until it finishes and makes the board forget its position)')
        self.console_input.returnPressed.connect(self._send_console)
        layout.addWidget(self.console)
        layout.addWidget(self.console_input)
        return widget

    # ----- helpers -----------------------------------------------------------

    def _joint(self):
        return self.joint_box.currentIndex()

    def _joint_config(self):
        return self.bench.config.joints[self._joint()]

    def _power_target(self):
        """The joint the Power row's "only" acts on: FOCUS on the focus stall tab, else the
        joint picked in the speed test."""
        if self.modes.currentWidget() is self.stall_panel:
            return self.stall_panel.joint_index
        return self._joint()

    def _fmt(self, steps):
        j = self._joint_config()
        return f'{steps_to_display(j, steps):.2f} {unit(j)}  ({steps} steps)'

    def _log(self, message):
        self.console.appendPlainText(message)

    def _report(self, ok, message):
        if not ok or message != 'OK':
            self._log(message)

    # ----- actions -----------------------------------------------------------

    def _toggle_connection(self):
        if self.bench.connected:
            self.bench.disconnect()
            self._log('Disconnected')
            return
        port = self.port_box.currentText().strip()
        self._log(f'Connecting to {port}...')
        if not self.bench.connect(port):
            self._log('Could not connect (see above). Is the board plugged in / attached to '
                      'WSL with usbipd, or the fake board running?')

    def _enable_all(self):
        ok, report = self.bench.enable_all()
        for line in report:
            self._log(f'  {line}')
        bad = [line for line in report if 'NO ANSWER' in line or 'refused' in line]
        if ok and not bad:
            self.drivers.setText('<span style="color:green">All drivers answered and are '
                                 'enabled.</span>')
        else:
            self.drivers.setText('<span style="color:#c00">' + '<br>'.join(
                bad or report[-1:]) + '</span>')

    def _disable_all(self):
        if self.bench.moving:
            self._log('Stop first')
            return
        if self.bench.disable_all():
            self.drivers.setText('All drivers disabled: motors are free (Z will drop!). The '
                                 'board forgot where they are: Set lower here after Enable.')
        else:
            self._log('Disable failed')

    def stop(self):
        self.bench.stop()

    def _apply_power(self, joints_on):
        """Power exactly `joints_on`; asks first if that switches Z off."""
        b = self.bench
        joints_on = set(joints_on)
        if b.power[Z_INDEX] and Z_INDEX not in joints_on:
            box = QMessageBox(self)
            box.setIcon(QMessageBox.Warning)
            box.setWindowTitle('Switch Z off?')
            box.setText('This switches Z off. The arm drops unless Z is resting at the '
                        'bottom.')
            off = box.addButton('Switch Z off', QMessageBox.DestructiveRole)
            keep = box.addButton('Keep Z on', QMessageBox.AcceptRole)
            box.addButton(QMessageBox.Cancel)
            box.setDefaultButton(keep)
            box.exec_()
            if box.clickedButton() == keep:
                joints_on.add(Z_INDEX)
            elif box.clickedButton() != off:
                return
        ok, message = b.set_power(joints_on)
        self._log(message)
        color = 'green' if ok and 'not answering' not in message else '#c00'
        self.drivers.setText(f'<span style="color:{color}">{message}</span>')

    def _toggle_power(self, index):
        on = {i for i, v in enumerate(self.bench.power) if v}
        on ^= {index}
        self._apply_power(on)

    def _power_selected_only(self):
        self._apply_power({self._power_target()})

    def _follow_selection(self):
        """With "Only the selected joint": power follows the joint picked, once anything is
        on (so just browsing joints with everything off doesn't power anything up)."""
        b = self.bench
        if (self.only_selected.isChecked() and b.connected and b.per_joint_power
                and not b.moving and any(b.power)
                and [i for i, v in enumerate(b.power) if v] != [self._power_target()]):
            self._power_selected_only()

    def emergency_stop(self):
        if not self.bench.connected:
            return
        self._log('E-STOP (Space): stopping and disabling all drivers')
        self.bench.emergency_stop()
        self.drivers.setText('<span style="color:#c00"><b>E-STOP:</b> all drivers disabled. '
                             'Motors are free; the board forgot where they are.</span>')

    def _set_lower(self):
        self._report(*self.bench.set_lower_here(self._joint()))

    def _set_upper(self):
        self._report(*self.bench.set_upper_here(self._joint()))

    def _go(self, target):
        if target is None:
            return
        self._report(*self.bench.move_to(self._joint(), target, self.jog_speed.value(),
                                         self.jog_accel.value()))

    def _jog(self, direction):
        self._report(*self.bench.jog(self._joint(), direction * self.jog_step.currentData(),
                                     self.jog_speed.value(), self.jog_accel.value()))

    def _run_test(self):
        self.note.clear()
        self._report(*self.bench.run_test(
            self._joint(), self.speed.value(), self.accel.value(), self.cycles.value(),
            self.pause.value(), self.jog_speed.value(), self.jog_accel.value()))

    def _verdict(self, result):
        if self.bench.record_verdict(result, self.note.text().strip()):
            self._log(f'Recorded: {result}')
            self.note.clear()
            self._load_results()

    def _discard(self):
        self.bench.discard_test()

    def _send_console(self):
        line = self.console_input.text().strip()
        if not line:
            return
        self.console_input.clear()
        self._log(f'> {line}')
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            for out in self.bench.send_raw(line):
                self._log(out)
        finally:
            QApplication.restoreOverrideCursor()

    # ----- updates -----------------------------------------------------------

    def _joint_changed(self):
        j = self._joint_config()
        u = unit(j)
        self.jog_step.clear()
        for size in (LINEAR_STEPS if u == 'mm' else ROTARY_STEPS):
            self.jog_step.addItem(f'{size:g} {u}', size)
        self.jog_step.setCurrentIndex(2)
        top = steps_to_display(j, MAX_TEST_RATE)
        jog_speed, jog_accel = self.bench.jog_defaults(self._joint())
        for box, value, suffix in ((self.jog_speed, jog_speed, f' {u}/s'),
                                   (self.jog_accel, jog_accel, f' {u}/s²'),
                                   (self.speed, jog_speed, f' {u}/s'),
                                   (self.accel, jog_accel, f' {u}/s²')):
            box.setSuffix(suffix)
            if box in (self.jog_speed, self.speed):
                box.setMaximum(abs(top))
            box.setValue(abs(value))
        self._update_test_labels()
        self._load_results()
        self._follow_selection()

    def _update_test_labels(self):
        j = self._joint_config()
        v = abs(display_to_steps(j, self.speed.value()))
        a = abs(display_to_steps(j, self.accel.value()))
        self.speed_steps.setText(f'= {v:,.0f} steps/s = {v / STEPS_PER_MOTOR_REV:.2f} motor '
                                 f'rev/s  (max {MAX_TEST_RATE:,})')
        self.accel_steps.setText(f'= {a:,.0f} steps/s²')
        self.estimate.setText('')
        if self.bench.steps is not None and self.bench.upper(self._joint()) is not None:
            try:
                builder, _ = self.bench.plan_test(
                    self._joint(), self.speed.value(), self.accel.value(), self.cycles.value(),
                    self.pause.value(), self.jog_speed.value(), self.jog_accel.value())
                self.estimate.setText(f'Test takes about {builder.duration:.1f} s')
            except ValueError as e:
                self.estimate.setText(str(e))

    def _load_results(self):
        rows = results.load_results(self.bench.results_path)
        if self.only_joint.isChecked():
            name = self._joint_config().name
            rows = [r for r in rows if r.get('joint') == name]
        self.table.setSortingEnabled(False)
        self.table.setRowCount(len(rows))
        for i, row in enumerate(reversed(rows)):   # newest first
            for c, key in enumerate(TABLE_COLUMNS):
                item = QTableWidgetItem(str(row.get(key, '')))
                if key == 'result':
                    item.setForeground(GOOD_COLOR if row.get(key) == results.ACCEPTABLE
                                       else STOP_COLOR)
                self.table.setItem(i, c, item)
        self.table.setSortingEnabled(True)

    def _tick(self):
        b = self.bench
        b.poll()
        connected = b.connected
        self.connect_button.setText('Disconnect' if connected else 'Connect')
        self.port_box.setEnabled(not connected)
        if not connected:
            self.state.setText('Not connected')
        else:
            board = b.board
            power = 'enabled' if b.enabled else 'DISABLED'
            target = self._power_target()
            name = self.bench.config.joints[target].name
            where = (f'{name} position known' if b.position_known(target)
                     else f'{name} position unknown')
            self.state.setText(f'{board.get("state", "?")} · {power} · {where} · '
                               f'{b.bridge.message}')
            self.state.setToolTip(self.state.text())

        steps = b.steps
        joint = self._joint()
        upper = b.upper(joint)
        if steps is None:
            self.position.setText('-')
        elif not b.position_known(joint):
            self.position.setText('unknown: Set lower here')
        else:
            self.position.setText(self._fmt(steps[joint]))
        self.upper_label.setText('not set' if upper is None else self._fmt(upper))

        moving = b.moving
        idle = connected and not moving
        joint_on = connected and b.joint_on(joint)
        can_move = idle and joint_on and b.position_known(joint)
        self.enable_button.setEnabled(idle)
        self.disable_button.setEnabled(idle)
        per_joint = connected and b.per_joint_power
        power = b.power if connected else [False] * len(self.power_buttons)
        for button, on in zip(self.power_buttons, power):
            button.setChecked(on)
            button.setEnabled(idle and per_joint)
        self.power_only.setEnabled(idle and per_joint)
        self.power_only.setText(f'Power {self.bench.config.joints[self._power_target()].name} '
                                'only')
        self.only_selected.setEnabled(per_joint)
        self.power_note.setText('' if per_joint or not connected else
                                'single-joint power needs the bench firmware')
        self.joint_box.setEnabled(not moving and not b.awaiting_verdict)
        self.set_lower.setEnabled(idle and joint_on)
        self.set_upper.setEnabled(idle and b.position_known(joint))
        self.go_lower.setEnabled(can_move)
        self.go_upper.setEnabled(can_move and upper is not None)
        for widget in (self.minus, self.plus):
            widget.setEnabled(can_move)
        self.run_button.setEnabled(can_move and upper is not None and not b.awaiting_verdict)
        self.console_input.setEnabled(idle)

        waiting = b.awaiting_verdict
        for widget in (self.acceptable, self.skipped, self.discard, self.note):
            widget.setEnabled(waiting)
        if b.test_running:
            self.verdict.setText('Testing... watch and listen. Esc stops.')
        elif waiting:
            early = ' (stopped early)' if b.test.get('stopped_early') else ''
            self.verdict.setText(f'<b>Test over{early}.</b> Is the marker still lined up?')
        else:
            self.verdict.setText('')
        if not self.estimate.text() and steps is not None and upper is not None:
            self._update_test_labels()
        self.stall_panel.tick()

    def closeEvent(self, event):
        self.bench.disconnect()
        super().closeEvent(event)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Motor speed/acceleration test bench.')
    parser.add_argument('--port', default=_ports()[0], help='serial port of the board')
    parser.add_argument('--config', default=_default_config(), help="arm_bridge's arm.yaml")
    parser.add_argument('--data-dir', default=os.path.expanduser('~/arm_bench'),
                        help='where results.csv and ranges.yaml are kept')
    # Launched with ros2 run it may also get --ros-args ...; ignore those.
    args, _ = parser.parse_known_args(sys.argv[1:] if argv is None else argv)

    config = ArmConfig.load(args.config)
    bench = Bench(config, args.port, os.path.join(args.data_dir, 'ranges.yaml'),
                  os.path.join(args.data_dir, 'results.csv'))
    app = QApplication(sys.argv[:1])
    window = MotorTestWindow(bench, args.port)
    window.show()
    window._log(f'Config: {os.path.abspath(args.config)}')
    window._log(f'Results and ranges: {args.data_dir}')
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
