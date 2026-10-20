"""Table window for typing camera/focus waypoint pairs.

Every change publishes the full PathTable on /camera_path/table, which
path_node turns into RViz markers.

Table layout: even table rows are waypoint rows (one PathRow each), odd table
rows sit between two waypoints and hold the segment-type dropdowns.
"""

import math
import os
import signal
import sys

import rclpy
from camera_path_interfaces.msg import PathRow, PathTable
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QAbstractItemView, QApplication, QComboBox, QFileDialog, QHBoxLayout, QHeaderView,
    QLabel, QMainWindow, QMessageBox, QPushButton, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from camera_path_editor.sampling import SUPPORTED_SEGMENT_TYPES
from camera_path_editor.table_file import load_table, save_table, TableFileError

TABLE_TOPIC = '/camera_path/table'

WINDOW_TITLE = 'Camera Path Editor'
FILE_FILTER = 'Path tables (*.json);;All files (*)'

HEADERS = ['#', 'Camera X', 'Camera Y', 'Camera Z', 'Focus X', 'Focus Y', 'Focus Z']
CAMERA_COLS = (1, 2, 3)
FOCUS_COLS = (4, 5, 6)

LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


def _fmt(value):
    return format(value, 'g')


class PathPanel(QMainWindow):

    def __init__(self, node):
        super().__init__()
        self._node = node
        self._pub = node.create_publisher(PathTable, TABLE_TOPIC, LATCHED_QOS)

        # The model. The table widget is rebuilt from these after every
        # structural change (add, insert, delete, move).
        self._rows = []           # each: [cx, cy, cz, fx, fy, fz]
        self._camera_segs = []    # len(rows) - 1
        self._focus_segs = []     # len(rows) - 1
        self._file_path = None    # last file saved or loaded

        self._update_title()
        self.resize(880, 480)

        self._table = QTableWidget(0, len(HEADERS))
        self._table.setHorizontalHeaderLabels(HEADERS)
        self._table.verticalHeader().setVisible(False)
        self._table.setSelectionMode(QAbstractItemView.SingleSelection)
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self._table.itemChanged.connect(self._on_item_changed)

        buttons = QHBoxLayout()
        for text, slot in (
            ('Add row', self.add_row),
            ('Insert row', self.insert_row),
            ('Delete row', self.delete_row),
            ('Move up', self.move_up),
            ('Move down', self.move_down),
        ):
            button = QPushButton(text)
            button.clicked.connect(slot)
            buttons.addWidget(button)
        buttons.addStretch()
        for text, slot in (
            ('Save...', self.save_file),
            ('Load...', self.load_file),
        ):
            button = QPushButton(text)
            button.clicked.connect(slot)
            buttons.addWidget(button)

        self._status = QLabel('')

        layout = QVBoxLayout()
        layout.addWidget(self._table)
        layout.addLayout(buttons)
        layout.addWidget(self._status)
        central = QWidget()
        central.setLayout(layout)
        self.setCentralWidget(central)

        self._rebuild_table()
        self._publish()

    # ----- selection helpers -------------------------------------------------

    def _selected_index(self):
        """Return the selected waypoint index, or None.

        Clicking a segment row counts as selecting the waypoint above it.
        """
        table_row = self._table.currentRow()
        if table_row < 0 or not self._rows:
            return None
        return min(table_row // 2, len(self._rows) - 1)

    def _select(self, index):
        if index is not None and 0 <= index < len(self._rows):
            self._table.setCurrentCell(index * 2, 1)

    # ----- button actions ----------------------------------------------------

    def add_row(self):
        """Append a row at the end, copying the last row."""
        new = list(self._rows[-1]) if self._rows else [0.0] * 6
        self._insert_at(len(self._rows), new)

    def insert_row(self):
        """Insert a row below the selected row, copying it."""
        index = self._selected_index()
        if index is None:
            self.add_row()
            return
        self._insert_at(index + 1, list(self._rows[index]))

    def delete_row(self):
        index = self._selected_index()
        if index is None:
            self._show_status('Select a row to delete.')
            return
        del self._rows[index]
        if self._camera_segs:
            # Drop the gap below the row, or the one above if it was the last row.
            gap = min(index, len(self._camera_segs) - 1)
            del self._camera_segs[gap]
            del self._focus_segs[gap]
        self._structure_changed(min(index, len(self._rows) - 1))

    def move_up(self):
        index = self._selected_index()
        if index is None or index == 0:
            return
        self._swap(index, index - 1)

    def move_down(self):
        index = self._selected_index()
        if index is None or index >= len(self._rows) - 1:
            return
        self._swap(index, index + 1)

    def _insert_at(self, index, values):
        self._rows.insert(index, values)
        if len(self._rows) >= 2:
            # A new row adds one gap; segment types stay tied to their position.
            gap = min(index, len(self._camera_segs))
            self._camera_segs.insert(gap, 'linear')
            self._focus_segs.insert(gap, 'linear')
        self._structure_changed(index)

    def _swap(self, a, b):
        # Waypoints move; segment types stay with the gap position.
        self._rows[a], self._rows[b] = self._rows[b], self._rows[a]
        self._structure_changed(b)

    def _structure_changed(self, select_index):
        self._rebuild_table()
        self._select(select_index)
        self._publish()

    # ----- save / load -------------------------------------------------------

    def save_file(self):
        path, _ = QFileDialog.getSaveFileName(
            self, 'Save path table', self._dialog_start(), FILE_FILTER)
        if not path:
            return
        if not os.path.splitext(path)[1]:
            path += '.json'
        self.save_to(path)

    def load_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self, 'Load path table', self._dialog_start(), FILE_FILTER)
        if path:
            self.load_from(path)

    def save_to(self, path):
        try:
            save_table(path, self._rows, self._camera_segs, self._focus_segs)
        except TableFileError as e:
            QMessageBox.warning(self, 'Save failed', str(e))
            return
        self._file_path = path
        self._update_title()
        self._show_status(f'Saved {len(self._rows)} rows to {os.path.basename(path)}.')

    def load_from(self, path):
        try:
            rows, camera_segs, focus_segs, notes = load_table(path)
        except TableFileError as e:
            QMessageBox.warning(self, 'Load failed', f'{os.path.basename(path)}: {e}')
            return
        self._rows, self._camera_segs, self._focus_segs = rows, camera_segs, focus_segs
        self._file_path = path
        self._update_title()
        self._structure_changed(None)
        status = f'Loaded {len(rows)} rows from {os.path.basename(path)}.'
        if notes:
            status += ' Note: ' + '; '.join(notes) + '.'
        self._show_status(status)

    def _dialog_start(self):
        if self._file_path:
            return os.path.dirname(self._file_path)
        return os.path.expanduser('~')

    def _update_title(self):
        if self._file_path:
            self.setWindowTitle(f'{WINDOW_TITLE} - {os.path.basename(self._file_path)}')
        else:
            self.setWindowTitle(WINDOW_TITLE)

    # ----- table building ----------------------------------------------------

    def _rebuild_table(self):
        self._table.blockSignals(True)
        self._table.clearSpans()
        self._table.setRowCount(max(0, 2 * len(self._rows) - 1))

        for i, values in enumerate(self._rows):
            table_row = 2 * i
            number = QTableWidgetItem(str(i + 1))
            number.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
            number.setTextAlignment(Qt.AlignCenter)
            self._table.setItem(table_row, 0, number)
            for col, value in zip(CAMERA_COLS + FOCUS_COLS, values):
                item = QTableWidgetItem(_fmt(value))
                item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self._table.setItem(table_row, col, item)

            if i < len(self._rows) - 1:
                self._add_segment_row(table_row + 1, i)

        self._table.blockSignals(False)

    def _add_segment_row(self, table_row, gap):
        blank = QTableWidgetItem('')
        blank.setFlags(Qt.NoItemFlags)
        self._table.setItem(table_row, 0, blank)
        for column, cols, segs in (
            ('camera', CAMERA_COLS, self._camera_segs),
            ('focus', FOCUS_COLS, self._focus_segs),
        ):
            self._table.setSpan(table_row, cols[0], 1, len(cols))
            self._table.setCellWidget(
                table_row, cols[0], self._segment_widget(column, gap, segs))

    def _segment_widget(self, column, gap, segs):
        combo = QComboBox()
        combo.addItems(SUPPORTED_SEGMENT_TYPES)
        combo.setCurrentText(segs[gap])
        combo.currentTextChanged.connect(
            lambda text, segs=segs, gap=gap: self._on_segment_changed(segs, gap, text))

        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(6, 0, 6, 0)
        layout.addWidget(QLabel(f'{column} segment {gap + 1}→{gap + 2}:'))
        layout.addWidget(combo, 1)
        return widget

    # ----- edits -------------------------------------------------------------

    def _on_item_changed(self, item):
        table_row, col = item.row(), item.column()
        if table_row % 2 or col == 0:
            return
        index = table_row // 2
        field = col - 1
        old = self._rows[index][field]
        typed = item.text()
        try:
            value = float(typed)
            if not math.isfinite(value):
                raise ValueError
        except ValueError:
            self._table.blockSignals(True)
            item.setText(_fmt(old))
            self._table.blockSignals(False)
            self._show_status(f'"{typed}" is not a number, kept {_fmt(old)}.')
            return

        self._table.blockSignals(True)
        item.setText(_fmt(value))
        self._table.blockSignals(False)
        if value != old:
            self._rows[index][field] = value
            self._publish()

    def _on_segment_changed(self, segs, gap, text):
        segs[gap] = text
        self._publish()

    # ----- publishing --------------------------------------------------------

    def build_message(self):
        msg = PathTable()
        for cx, cy, cz, fx, fy, fz in self._rows:
            row = PathRow()
            row.camera.x, row.camera.y, row.camera.z = cx, cy, cz
            row.focus.x, row.focus.y, row.focus.z = fx, fy, fz
            msg.rows.append(row)
        msg.camera_segment_types = list(self._camera_segs)
        msg.focus_segment_types = list(self._focus_segs)
        return msg

    def _publish(self):
        self._pub.publish(self.build_message())
        self._show_status(f'{len(self._rows)} rows published.')

    def _show_status(self, text):
        self._status.setText(text)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('camera_path_panel')

    app = QApplication(sys.argv)
    panel = PathPanel(node)
    panel.show()

    # Let Ctrl+C / launch shutdown close the window, and keep rclpy serviced.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(50)

    exit_code = app.exec_()
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
