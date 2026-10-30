"""Receive a PathTable, sample the camera and focus paths, and draw them in RViz."""

import math

import rclpy
from camera_path_interfaces.msg import PathTable
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from camera_path_editor.sampling import sample_path

TABLE_TOPIC = '/camera_path/table'
MARKER_TOPIC = '/camera_path/markers'

SPHERE_DIAMETER = 0.03
LABEL_HEIGHT = 0.03
LABEL_OFFSET = 0.04
LINE_WIDTH = 0.005
ARROW_SHAFT = 0.003
ARROW_HEAD = 0.01
ARROW_HEAD_LENGTH = 0.02
MIN_ARROW_LENGTH = 1e-4   # skip arrows where the camera sits on its focus point

BLUE = (0.1, 0.4, 1.0)
RED = (1.0, 0.15, 0.15)
GREEN = (0.2, 0.8, 0.3)
ARROW_ALPHA = 0.6   # see-through so many arrows converging on one point stay readable

LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class PathNode(Node):

    def __init__(self):
        super().__init__('camera_path_node')
        self.declare_parameter('points_per_segment', 10)
        self.declare_parameter('frame_id', 'base_link')
        # The panel's "Show aim arrows" checkbox sets this.
        self.declare_parameter('show_aim_arrows', True)

        self._last_table = None
        self._pub = self.create_publisher(MarkerArray, MARKER_TOPIC, LATCHED_QOS)
        self.create_subscription(PathTable, TABLE_TOPIC, self._on_table, LATCHED_QOS)
        # Redraw with the new settings when a parameter changes at runtime.
        self.add_post_set_parameters_callback(self._on_params_changed)

        self.get_logger().info(f'Waiting for tables on {TABLE_TOPIC}')

    def _on_params_changed(self, _params):
        if self._last_table is not None:
            self._render(self._last_table)

    def _on_table(self, msg):
        self._last_table = msg
        self._render(msg)

    def _render(self, table):
        n = self.get_parameter('points_per_segment').value
        if n < 1:
            self.get_logger().warn(f'points_per_segment={n} is invalid, using 1')
            n = 1

        cameras = [(r.camera.x, r.camera.y, r.camera.z) for r in table.rows]
        focuses = [(r.focus.x, r.focus.y, r.focus.z) for r in table.rows]

        camera_dense = sample_path(
            cameras, list(table.camera_segment_types), n,
            lambda i, t: self._warn_unknown('camera', i, t))
        focus_dense = sample_path(
            focuses, list(table.focus_segment_types), n,
            lambda i, t: self._warn_unknown('focus', i, t))

        array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self._frame_id()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        for i, (cam, foc) in enumerate(zip(cameras, focuses)):
            array.markers.append(self._sphere('camera_waypoints', i, cam, BLUE))
            array.markers.append(self._label('camera_labels', i, cam, BLUE))
            array.markers.append(self._sphere('focus_waypoints', i, foc, RED))
            array.markers.append(self._label('focus_labels', i, foc, RED))

        # A line needs at least 2 points, so skip the paths for 0 or 1 rows.
        if len(table.rows) >= 2:
            array.markers.append(self._line('camera_path', camera_dense, BLUE))
            array.markers.append(self._line('focus_path', focus_dense, RED))

        # Aim arrows: camera sample k points at focus sample k. Both paths use
        # the same N and u values, so the two dense lists line up one to one.
        arrows = 0
        if self.get_parameter('show_aim_arrows').value:
            for k, (cam, foc) in enumerate(zip(camera_dense, focus_dense)):
                if math.dist(cam, foc) > MIN_ARROW_LENGTH:
                    array.markers.append(self._arrow('aim_arrows', k, cam, foc))
                    arrows += 1

        self._pub.publish(array)
        self.get_logger().info(
            f'Drew {len(table.rows)} rows (N={n}): camera path {len(camera_dense)} points, '
            f'focus path {len(focus_dense)} points, {arrows} aim arrows')

    def _warn_unknown(self, column, index, seg_type):
        self.get_logger().warn(
            f'Unknown {column} segment type "{seg_type}" at segment {index + 1}, '
            'drawing it as linear')

    def _frame_id(self):
        return self.get_parameter('frame_id').value

    def _base_marker(self, ns, marker_id, marker_type, color):
        m = Marker()
        m.header.frame_id = self._frame_id()
        m.ns = ns
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        return m

    def _sphere(self, ns, index, xyz, color):
        m = self._base_marker(ns, index, Marker.SPHERE, color)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        m.scale.x = m.scale.y = m.scale.z = SPHERE_DIAMETER
        return m

    def _label(self, ns, index, xyz, color):
        m = self._base_marker(ns, index, Marker.TEXT_VIEW_FACING, color)
        m.pose.position.x, m.pose.position.y = xyz[0], xyz[1]
        m.pose.position.z = xyz[2] + LABEL_OFFSET
        m.scale.z = LABEL_HEIGHT
        m.text = str(index + 1)
        return m

    def _line(self, ns, points, color):
        m = self._base_marker(ns, 0, Marker.LINE_STRIP, color)
        m.scale.x = LINE_WIDTH
        m.points = [Point(x=p[0], y=p[1], z=p[2]) for p in points]
        return m

    def _arrow(self, ns, index, start, end):
        m = self._base_marker(ns, index, Marker.ARROW, GREEN)
        m.color.a = ARROW_ALPHA
        # With two points, scale is shaft diameter, head diameter, head length.
        m.scale.x, m.scale.y, m.scale.z = ARROW_SHAFT, ARROW_HEAD, ARROW_HEAD_LENGTH
        m.points = [Point(x=start[0], y=start[1], z=start[2]),
                    Point(x=end[0], y=end[1], z=end[2])]
        return m


def main(args=None):
    rclpy.init(args=args)
    node = PathNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launch can send a second Ctrl+C while we clean up; ignore it.
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
