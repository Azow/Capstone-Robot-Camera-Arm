"""Solve IK for every point of the camera path and play the arm along it.

Listens to the same PathTable the editor publishes, samples it exactly like
path_node does (same points_per_segment gives the same points), solves each
point in order, and publishes JointState so robot_state_publisher moves the
arm in RViz. Playback starts paused; the editor's Play/Pause button (or the
/camera_path/set_playing service) plays from the first point to the last
reachable one and stops there. Play at the end starts over. The editor's
slider jumps to a point through /camera_path/seek, and its speed dropdown sets
the `speed` parameter. A PlaybackState on /camera_path/playback keeps the
editor's controls in step.

If a point can't be reached, solving stops there: the arm plays up to the
point before it, and a PathIkStatus says which point and why. The editor
panel shows that in its IK column; RViz only shows the arm and the paths.
"""

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from camera_path_editor.sampling import sample_path
from camera_path_interfaces.msg import PathIkStatus, PathTable, PlaybackState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt32
from std_srvs.srv import SetBool

from camera_path_ik.arm_model import ArmModel, THETA, YAW
from camera_path_ik.ik import IkSolver

TABLE_TOPIC = '/camera_path/table'
STATUS_TOPIC = '/camera_path/ik_status'
JOINT_TOPIC = '/joint_states'
PLAYBACK_TOPIC = '/camera_path/playback'
SEEK_TOPIC = '/camera_path/seek'
SET_PLAYING_SERVICE = '/camera_path/set_playing'

LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


def _default_urdf_path():
    return os.path.join(get_package_share_directory('arm_description_v2'),
                        'urdf', 'camera_arm_v2.urdf')


class IkNode(Node):

    def __init__(self):
        super().__init__('camera_path_ik')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('points_per_segment', 10)
        self.declare_parameter('playback_rate', 10.0)    # path points per second at 1x
        self.declare_parameter('speed', 1.0)             # multiplier; the editor's dropdown sets it
        # Temporary: ignore the Theta and Yaw limits so they can spin freely.
        # Read once at startup.
        self.declare_parameter('free_spin', False)

        urdf_path = self.get_parameter('urdf_path').value or _default_urdf_path()
        with open(urdf_path) as f:
            model = ArmModel.from_urdf(f.read())
        if self.get_parameter('free_spin').value:
            model.remove_limits([THETA, YAW])
        self._solver = IkSolver(model)
        self.get_logger().info(f'Arm measured from {urdf_path}:\n{model.describe()}')
        if self.get_parameter('free_spin').value:
            self.get_logger().warn('free_spin is on: Theta and Yaw limits are ignored')

        self._last_table = None
        self._poses = [model.q_ref]   # joint positions to play, in order
        self._index = 0
        self._count = 0               # reachable points; 0 when only showing the reference pose
        self._playing = False
        self._last_playback = None    # last PlaybackState sent, as (playing, index, count)

        self._joint_pub = self.create_publisher(JointState, JOINT_TOPIC, 10)
        self._status_pub = self.create_publisher(PathIkStatus, STATUS_TOPIC, LATCHED_QOS)
        self._playback_pub = self.create_publisher(PlaybackState, PLAYBACK_TOPIC, LATCHED_QOS)
        self.create_service(SetBool, SET_PLAYING_SERVICE, self._on_set_playing)
        self.create_subscription(UInt32, SEEK_TOPIC, self._on_seek, 10)
        self._publish_playback()
        self.create_subscription(PathTable, TABLE_TOPIC, self._on_table, LATCHED_QOS)
        self._timer = None
        self._restart_timer()
        # Pick up parameter changes at runtime: re-time and/or re-solve.
        self.add_post_set_parameters_callback(self._on_params_changed)

        self.get_logger().info(f'Waiting for tables on {TABLE_TOPIC}')

    # ----- solving -----------------------------------------------------------

    def _on_params_changed(self, params):
        names = {p.name for p in params}
        # Speed changes keep the current point and play state; only N re-solves.
        if names & {'playback_rate', 'speed'}:
            self._restart_timer()
        if 'points_per_segment' in names and self._last_table is not None:
            self._solve(self._last_table)

    def _on_table(self, msg):
        self._last_table = msg
        self._solve(msg)

    def _solve(self, table):
        n = self.get_parameter('points_per_segment').value
        if n < 1:
            self.get_logger().warn(f'points_per_segment={n} is invalid, using 1')
            n = 1

        cameras = [(r.camera.x, r.camera.y, r.camera.z) for r in table.rows]
        focuses = [(r.focus.x, r.focus.y, r.focus.z) for r in table.rows]
        # Unknown types fall back to linear; path_node already warns about them.
        camera_dense = sample_path(cameras, list(table.camera_segment_types), n)
        focus_dense = sample_path(focuses, list(table.focus_segment_types), n)

        results = self._solver.solve_path(camera_dense, focus_dense)
        reached = [r.q for r in results if r.reached]
        failed = None if not results or results[-1].reached else len(results) - 1

        # Nothing reachable (empty table, or the first point fails): show the reference pose.
        self._poses = reached or [self._solver.model.q_ref]
        self._count = len(reached)
        # A new path starts paused at its first point.
        self._index = 0
        self._playing = False
        self._publish_pose()

        total = len(camera_dense)
        held = sum(1 for r in results if r.held)
        note = f', held a joint at {held} singular points' if held else ''
        if failed is None:
            self.get_logger().info(f'Solved all {total} points (N={n}){note}')
        else:
            reason = results[failed].reason
            playing = (f'Playing the {failed} points before it.' if failed
                       else 'Nothing to play; showing the reference pose.')
            self.get_logger().warn(
                f'Point {failed + 1} of {total} ({self._where(failed, n, len(cameras))}) is '
                f'unreachable: {reason}. {playing}')
        self._publish_status(total, failed, n, len(cameras),
                             None if failed is None else results[failed].reason)

    @staticmethod
    def _where(index, n, rows):
        """Describe a dense point index in terms of the editor's rows."""
        segment, step = divmod(index, n)
        if segment >= rows - 1:
            return f'row {rows}'
        if step == 0:
            return f'row {segment + 1}'
        return f'segment {segment + 1}→{segment + 2}, {step}/{n} of the way'

    # ----- playback ----------------------------------------------------------

    def _restart_timer(self):
        rate = self.get_parameter('playback_rate').value * self.get_parameter('speed').value
        if rate <= 0:
            self.get_logger().warn(f'playback_rate x speed = {rate} is invalid, using 1')
            rate = 1.0
        if self._timer is not None:
            self.destroy_timer(self._timer)
        self._timer = self.create_timer(1.0 / rate, self._tick)

    def _tick(self):
        # Ctrl+C / launch shutdown can stop ROS just before a tick; publishing then fails.
        if not self.context.ok():
            return
        if self._playing:
            if self._index < len(self._poses) - 1:
                self._index += 1
            else:
                # At the end: stop and hold the last pose.
                self._playing = False
        # Publish every tick, even when paused, so the arm stays drawn in RViz.
        self._publish_pose()

    def _on_set_playing(self, request, response):
        # Play from the end starts over from the first point.
        if request.data and self._index >= len(self._poses) - 1:
            self._index = 0
        self._playing = request.data
        self._publish_pose()
        response.success = True
        response.message = 'playing' if request.data else 'paused'
        return response

    def _on_seek(self, msg):
        # Scrubbing pauses, and shows the point right away instead of on the next tick.
        self._playing = False
        self._index = min(msg.data, len(self._poses) - 1)
        self._publish_pose()

    def _publish_pose(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._solver.model.names)
        msg.position = [float(v) for v in self._poses[self._index]]
        self._joint_pub.publish(msg)
        self._publish_playback()

    def _publish_playback(self):
        # Only on change: paused ticks would otherwise resend the same state.
        state = (self._playing, self._index, self._count)
        if state == self._last_playback:
            return
        self._last_playback = state
        self._playback_pub.publish(
            PlaybackState(playing=self._playing, index=self._index, count=self._count))

    # ----- status for the editor panel --------------------------------------

    def _publish_status(self, total, failed, n, rows, reason):
        msg = PathIkStatus(points_total=total, row=-1, segment=-1)
        if failed is None:
            msg.points_reached = total
        else:
            msg.points_reached = failed
            msg.reason = reason
            segment, step = divmod(failed, n)
            if segment >= rows - 1:
                msg.row = rows - 1
            elif step == 0:
                msg.row = segment
            else:
                msg.segment = segment
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IkNode()
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
