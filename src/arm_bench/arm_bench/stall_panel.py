"""The motor test window's "Focus stall test" tab: measured vs predicted focus throw.

See stall.py for what the test is. This is the page: settings, start, the live StallGuard
graph, and the table of past runs (~/arm_bench/stall_results.csv).
"""

from python_qt_binding.QtCore import QPointF, QRectF, Qt
from python_qt_binding.QtGui import QColor, QDoubleValidator, QIntValidator, QPainter, QPen
from python_qt_binding.QtWidgets import (
    QAbstractItemView, QGridLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
    QPushButton, QSizePolicy, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from arm_bench import stall

HELP = ('1. Focus off, turn the ring by hand onto one hard stop, then Power focus only '
        '(the other motors stay off) and Set zero here.  2. Point the direction at the other stop and Start seek: it '
        'stops when StallGuard trips, and the distance from zero is the measured throw.  '
        'Tip: threshold 0 never trips. Run one short seek mid-throw to see the free-running '
        'reading, then pick a threshold with 2 × threshold well below it.  Needs the bench '
        'firmware (BENCH_TESTS 1).')
TABLE_COLUMNS = ('time', 'result', 'direction', 'speed_deg_s', 'sgthrs', 'current_ma',
                 'span_steps', 'predicted_steps', 'error_pct', 'implied_lens_teeth',
                 'implied_steps_per_unit', 'sg_min', 'steps', 't_ms')
TABLE_HEADERS = ('time', 'result', 'dir', 'deg/s', 'SGTHRS', 'mA', 'measured', 'predicted',
                 'error %', 'lens teeth', 'steps/rad', 'SG min', 'steps', 'ms')
GOOD = QColor(60, 150, 70)
BAD = QColor(200, 30, 30)


class SgPlot(QWidget):
    """StallGuard reading against time for the latest seek, with the trip level (2 x SGTHRS)
    and the ignored start-up time shaded."""

    def __init__(self):
        super().__init__()
        self.samples = []
        self.sgthrs = None
        self.setMinimumHeight(220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_data(self, samples, sgthrs):
        self.samples = samples
        self.sgthrs = sgthrs
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        pal = self.palette()
        painter.fillRect(self.rect(), pal.base())
        text = pal.text().color()
        grid = QColor(text)
        grid.setAlpha(50)
        left, top, right, bottom = 46, 10, 10, 28
        area = QRectF(left, top, self.width() - left - right, self.height() - top - bottom)
        if area.width() < 20 or area.height() < 20:
            return
        t_max = max([s[0] for s in self.samples] + [1.0])
        sg_top = max([s[2] for s in self.samples] + [2 * (self.sgthrs or 0), 100])
        sg_top = 50 * (int(sg_top * 1.1) // 50 + 1)

        def point(t, sg):
            return QPointF(area.left() + area.width() * t / t_max,
                           area.bottom() - area.height() * sg / sg_top)

        # Ignored start-up (the firmware's kStallIgnoreMs).
        ignore = QColor(text)
        ignore.setAlpha(25)
        painter.fillRect(QRectF(point(0, sg_top), point(min(0.1, t_max), 0)), ignore)
        painter.setPen(QPen(grid, 1))
        for i in range(6):
            sg = sg_top * i / 5
            y = point(0, sg).y()
            painter.drawLine(QPointF(area.left(), y), QPointF(area.right(), y))
            painter.setPen(text)
            painter.drawText(QRectF(0, y - 8, left - 6, 16), Qt.AlignRight | Qt.AlignVCenter,
                             f'{sg:.0f}')
            painter.setPen(QPen(grid, 1))
        for i in range(6):
            t = t_max * i / 5
            x = point(t, 0).x()
            painter.setPen(text)
            painter.drawText(QRectF(x - 30, area.bottom() + 4, 60, 16), Qt.AlignCenter,
                             f'{t:.1f} s')
        painter.setPen(text)
        painter.drawText(QRectF(left + 6, top, 300, 18), Qt.AlignLeft, 'StallGuard (SG_RESULT)')
        if self.sgthrs is not None:
            level = 2 * self.sgthrs
            painter.setPen(QPen(BAD, 1.5, Qt.DashLine))
            y = point(0, level).y()
            painter.drawLine(QPointF(area.left(), y), QPointF(area.right(), y))
            painter.drawText(QRectF(area.right() - 200, y - 18, 196, 16), Qt.AlignRight,
                             f'trips at ≤ {level}')
        if len(self.samples) >= 2:
            painter.setPen(QPen(pal.highlight().color(), 2))
            pts = [point(t, sg) for t, _, sg in self.samples]
            for a, b in zip(pts, pts[1:]):
                painter.drawLine(a, b)
        elif not self.samples:
            painter.setPen(text)
            painter.drawText(area, Qt.AlignCenter, 'Start a seek to see StallGuard live')


def _edit(text, validator, width=90):
    box = QLineEdit(text)
    box.setValidator(validator)
    box.setMaximumWidth(width)
    return box


def _number(box):
    try:
        return float(box.text())
    except ValueError:
        return None


class StallPanel(QWidget):

    def __init__(self, window):
        super().__init__()
        self.window = window
        self.bench = window.bench
        names = [j.name for j in self.bench.config.joints]
        self.joint_index = names.index(stall.JOINT)
        self.joint = self.bench.arm_config.joints[self.joint_index]   # arm.yaml as written
        self._shown_result = None

        layout = QHBoxLayout(self)
        left = QVBoxLayout()
        help_label = QLabel(HELP)
        help_label.setWordWrap(True)
        help_label.setStyleSheet('color: gray')
        left.addWidget(help_label)
        left.addWidget(self._build_settings())
        left.addStretch()
        holder = QWidget()
        holder.setLayout(left)
        holder.setFixedWidth(490)
        layout.addWidget(holder)
        right = QVBoxLayout()
        self.summary = QLabel('')
        self.summary.setWordWrap(True)
        self.summary.setTextFormat(Qt.RichText)
        right.addWidget(self.summary)
        self.plot = SgPlot()
        right.addWidget(self.plot, 2)
        right.addWidget(self._build_table(), 1)
        layout.addLayout(right, 1)
        self._update_labels()
        self.load_results()

    # ----- layout ------------------------------------------------------------

    def _build_settings(self):
        box = QGroupBox('Settings')
        grid = QGridLayout(box)
        predicted = stall.predicted_span(self.joint)
        self.predicted = QLabel(
            f'Predicted throw: <b>{predicted:.0f} steps</b> = '
            f'{stall.steps_to_deg(self.joint, predicted):.1f}° × '
            f'{self.joint.steps_per_unit:g} steps/rad (arm.yaml)')
        self.predicted.setWordWrap(True)
        self.position = QLabel('-')
        self.power_on = QPushButton('Power focus only')
        self.power_on.setToolTip('Switch FOCUS on and every other motor off (bench firmware). '
                                 'Asks first if Z is on.')
        self.power_on.clicked.connect(lambda: self.window._apply_power({self.joint_index}))
        self.power_off = QPushButton('Focus off')
        self.power_off.setToolTip('Switch FOCUS off so the ring turns by hand. Its position is '
                                  'lost: Set zero here after powering it again.')
        self.power_off.clicked.connect(self._focus_off)
        self.power_state = QLabel('')
        self.set_zero = QPushButton('Set zero here')
        self.set_zero.setToolTip('This position (a hard stop, placed by hand) is step 0')
        self.set_zero.clicked.connect(self._set_zero)
        self.go_zero = QPushButton('Go to zero')
        self.go_zero.setToolTip('Turn back to zero at the speed below')
        self.go_zero.clicked.connect(self._go_zero)

        self.direction = QPushButton()
        self.direction.setCheckable(True)
        self.direction.setToolTip('Click to flip. The names assume the sketch\'s invert_dir '
                                  'for FOCUS is right: check which way the ring really turns.')
        self.direction.toggled.connect(self._update_labels)
        self.speed = _edit(f'{stall.DEFAULT_SPEED:g}', QDoubleValidator(0.01, 1e5, 2))
        self.speed_steps = QLabel('')
        self.sgthrs = _edit(str(stall.DEFAULT_SGTHRS), QIntValidator(0, 255))
        self.sgthrs_note = QLabel('')
        self.current = _edit(str(stall.DEFAULT_CURRENT),
                             QIntValidator(stall.MIN_CURRENT, stall.RUN_CURRENT))
        current_note = QLabel(f'{stall.MIN_CURRENT}..{stall.RUN_CURRENT} mA')
        travel = stall.TRAVEL_MARGIN * stall.steps_to_deg(self.joint, predicted)
        self.travel = _edit(f'{travel:.0f}', QDoubleValidator(0.1, 3600, 1))
        self.travel_note = QLabel('')
        for widget in (self.speed, self.sgthrs, self.current, self.travel):
            widget.textChanged.connect(self._update_labels)
        self.start = QPushButton('Start seek')
        self.start.setMinimumHeight(36)
        self.start.clicked.connect(self._start)

        row = 0
        grid.addWidget(self.predicted, row, 0, 1, 3)
        row += 1
        grid.addWidget(QLabel('Power'), row, 0)
        grid.addWidget(self.power_on, row, 1)
        grid.addWidget(self.power_off, row, 2)
        row += 1
        grid.addWidget(self.power_state, row, 1, 1, 2)
        row += 1
        grid.addWidget(QLabel('Position'), row, 0)
        grid.addWidget(self.position, row, 1, 1, 2)
        row += 1
        grid.addWidget(self.set_zero, row, 1)
        grid.addWidget(self.go_zero, row, 2)
        row += 1
        grid.addWidget(QLabel('Direction'), row, 0)
        grid.addWidget(self.direction, row, 1, 1, 2)
        row += 1
        grid.addWidget(QLabel('Speed (ring °/s)'), row, 0)
        grid.addWidget(self.speed, row, 1)
        grid.addWidget(self.speed_steps, row, 2)
        row += 1
        grid.addWidget(QLabel('Stall threshold'), row, 0)
        grid.addWidget(self.sgthrs, row, 1)
        grid.addWidget(self.sgthrs_note, row, 2)
        row += 1
        grid.addWidget(QLabel('Current (mA)'), row, 0)
        grid.addWidget(self.current, row, 1)
        grid.addWidget(current_note, row, 2)
        row += 1
        grid.addWidget(QLabel('Max travel (°)'), row, 0)
        grid.addWidget(self.travel, row, 1)
        grid.addWidget(self.travel_note, row, 2)
        row += 1
        grid.addWidget(self.start, row, 0, 1, 3)
        return box

    def _build_table(self):
        self.table = QTableWidget(0, len(TABLE_COLUMNS))
        self.table.setHorizontalHeaderLabels(TABLE_HEADERS)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setToolTip(f'Saved to {self.bench.stall_path}')
        return self.table

    # ----- values ------------------------------------------------------------

    def _sign(self):
        return -1 if self.direction.isChecked() else 1

    def _settings(self):
        """(steps, rate, sgthrs, current, record) from the boxes, or raise ValueError."""
        speed, sgthrs, current, travel = (_number(b) for b in
                                          (self.speed, self.sgthrs, self.current, self.travel))
        if None in (speed, sgthrs, current, travel):
            raise ValueError('Fill in every box')
        rate = round(stall.deg_to_steps(self.joint, speed))
        if not 0 < rate <= stall.MAX_RATE:
            raise ValueError(f'Speed must give 1..{stall.MAX_RATE} steps/s')
        if not 0 <= sgthrs <= 255:
            raise ValueError('Threshold is 0..255')
        if not stall.MIN_CURRENT <= current <= stall.RUN_CURRENT:
            raise ValueError(f'Current is {stall.MIN_CURRENT}..{stall.RUN_CURRENT} mA')
        steps = self._sign() * round(stall.deg_to_steps(self.joint, travel))
        if steps == 0:
            raise ValueError('Max travel must be above 0')
        record = {'direction': '+' if steps > 0 else '-', 'speed_deg_s': f'{speed:g}',
                  'rate_steps_s': rate, 'sgthrs': int(sgthrs), 'current_ma': int(current)}
        return steps, rate, int(sgthrs), int(current), record

    def _update_labels(self):
        self.direction.setText('− steps  (toward ∞)' if self.direction.isChecked() else
                               '+ steps  (toward close focus)')
        speed = _number(self.speed)
        rate = stall.deg_to_steps(self.joint, speed) if speed else 0
        self.speed_steps.setText(f'= {rate:,.0f} steps/s' +
                                 (f' (max {stall.MAX_RATE:,})' if rate > stall.MAX_RATE else ''))
        sgthrs = _number(self.sgthrs)
        self.sgthrs_note.setText(f'trips at SG ≤ {2 * sgthrs:.0f}' if sgthrs is not None
                                 else '0..255')
        travel = _number(self.travel)
        self.travel_note.setText(
            f'= {stall.deg_to_steps(self.joint, travel):,.0f} steps'
            if travel else '')

    # ----- actions -----------------------------------------------------------

    def _focus_off(self):
        on = {i for i, v in enumerate(self.bench.power) if v}
        self.window._apply_power(on - {self.joint_index})

    def _set_zero(self):
        self.window._report(*self.bench.set_zero_here(self.joint_index))

    def _go_zero(self):
        speed = _number(self.speed) or stall.DEFAULT_SPEED
        _, accel = self.bench.jog_defaults(self.joint_index)
        self.window._report(*self.bench.move_to(self.joint_index, 0, speed, accel))

    def _start(self):
        try:
            steps, rate, sgthrs, current, record = self._settings()
        except ValueError as e:
            self.window._log(str(e))
            return
        self.window._log(f'> STALLSEEK FOCUS {steps} {rate} {sgthrs} {current}')
        self.window._report(*self.bench.start_stall_seek(steps, rate, sgthrs, current, record))

    # ----- updates -----------------------------------------------------------

    def load_results(self):
        rows = list(reversed(stall.load(self.bench.stall_path)))   # newest first
        self.table.setRowCount(len(rows))
        for i, row in enumerate(rows):
            for c, key in enumerate(TABLE_COLUMNS):
                item = QTableWidgetItem(str(row.get(key, '')))
                if key == 'result':
                    item.setForeground(GOOD if row.get(key) == stall.STALL_RESULT else BAD)
                self.table.setItem(i, c, item)

    def _summary(self, row):
        if row['result'] != stall.STALL_RESULT:
            why = ('it went the whole max travel without a stall: threshold too low, or the '
                   'gear slipping?' if row['result'] == stall.LIMIT_RESULT else 'stopped by you')
            return f'<b>No stall</b> ({row["result"]}): {why}'
        error = float(row['error_pct'])
        color = '#2a7' if abs(error) <= 2 else '#c80' if abs(error) <= 5 else '#c00'
        return (f'Measured <b>{row["span_steps"]}</b> steps vs predicted '
                f'<b>{row["predicted_steps"]}</b>: <b style="color:{color}">{error:+.2f}%</b>. '
                f'Implies {row["implied_steps_per_unit"]} steps/rad, a '
                f'{row["implied_lens_teeth"]}-tooth lens gear (with the {stall.MOTOR_GEAR_TEETH}'
                f'-tooth motor gear). Lowest SG after start-up: {row["sg_min"]}. '
                'If it stalled far too early, the threshold is too high (look at the graph).')

    def tick(self):
        b = self.bench
        idle = b.connected and not b.moving
        steps = b.steps
        if steps is None:
            self.position.setText('-')
        elif not b.position_known(self.joint_index):
            self.position.setText('unknown: Set zero here')
        else:
            s = steps[self.joint_index]
            self.position.setText(f'{stall.steps_to_deg(self.joint, s):.1f}°  ({s} steps)')
        on = b.connected and b.joint_on(self.joint_index)
        known = b.position_known(self.joint_index)
        per_joint = b.connected and b.per_joint_power
        if not b.connected:
            self.power_state.setText('')
        elif not per_joint:
            self.power_state.setText('<span style="color:#c00">single-joint power needs the '
                                     'bench firmware</span>')
        else:
            others = [j.name for j, v in zip(b.config.joints, b.power) if v
                      and j.name != stall.JOINT]
            self.power_state.setText(
                ('<b style="color:#2a7">FOCUS on</b>' if on else '<b>FOCUS off</b> (turns by '
                 'hand)') + (f'; also on: {", ".join(others)}' if others else '; others off'))
        self.power_on.setEnabled(idle and per_joint and (not on or any(
            v for i, v in enumerate(b.power) if i != self.joint_index)))
        self.power_off.setEnabled(idle and per_joint and on)
        self.set_zero.setEnabled(idle and on)
        self.go_zero.setEnabled(idle and on and known)
        self.start.setEnabled(idle and on and known)
        for widget in (self.direction, self.speed, self.sgthrs, self.current, self.travel):
            widget.setEnabled(b.seek is None)
        self.start.setText('Seeking...  (Space = e-stop)' if b.seek is not None else 'Start seek')
        self.plot.set_data(b.seek_samples, b.seek_sgthrs)
        if b.seek_result is not self._shown_result:
            self._shown_result = b.seek_result
            if b.seek_result:
                self.summary.setText(self._summary(b.seek_result))
                self.load_results()
