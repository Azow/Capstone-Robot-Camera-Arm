"""ROS node for arm_bridge: the link between ROS and the Octopus board.

Keeps the latest timed path from camera_path_ik (/camera_path/trajectory) and,
on /arm/run, streams it to the board (D-09). Publishes the board's position on
/joint_states (so RViz shows the real arm) and an ArmStatus on /arm/status for
the editor panel.

Services (std_srvs): /arm/run, /arm/stop, /arm/home, /arm/clear (Trigger),
/arm/enable (SetBool), and /arm/assume_homed (Trigger, bench testing only:
tells the board it is sitting at the home position).

The serial port is handled on its own thread, so a slow or missing board never
blocks ROS callbacks.
"""

import os
import queue
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from camera_path_interfaces.msg import ArmStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

from arm_bridge.bridge import ArmBridge
from arm_bridge.planner import ArmConfig, PlanError

TRAJECTORY_TOPIC = '/camera_path/trajectory'
JOINT_TOPIC = '/joint_states'
STATUS_TOPIC = '/arm/status'

COMMAND_TIMEOUT = 2.0   # s a service waits for the serial thread
STATUS_EVERY = 5        # polls per ArmStatus (10 Hz at the default poll_period); the panel
                        # doesn't need more

LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class BridgeNode(Node):

    def __init__(self):
        super().__init__('arm_bridge')
        self.declare_parameter('port', '/dev/octopus')
        self.declare_parameter('config', os.path.join(
            get_package_share_directory('arm_bridge'), 'config', 'arm.yaml'))
        self.declare_parameter('poll_period', 0.02)

        config_path = self.get_parameter('config').value
        self._config = ArmConfig.load(config_path)
        self._bridge = ArmBridge(self._config, self.get_parameter('port').value,
                                 log=self.get_logger().info)
        self._period = self.get_parameter('poll_period').value
        self._trajectory = None
        self._commands = queue.Queue()   # jobs for the serial thread
        self._polls = 0

        self._joint_pub = self.create_publisher(JointState, JOINT_TOPIC, 10)
        self._status_pub = self.create_publisher(ArmStatus, STATUS_TOPIC, 10)
        self.create_subscription(JointTrajectory, TRAJECTORY_TOPIC, self._on_trajectory,
                                 LATCHED_QOS)
        b = self._bridge
        for name, function in (('run', self._run), ('stop', b.stop), ('home', b.home),
                               ('clear', b.clear), ('assume_homed', b.assume_homed)):
            self.create_service(Trigger, f'/arm/{name}', self._trigger_handler(function))
        self.create_service(SetBool, '/arm/enable', self._on_enable)

        self.get_logger().info(f'Joint config from {config_path}')
        self._running = True
        self._thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._thread.start()

    def shutdown(self):
        self._running = False
        self._thread.join(timeout=1.0)
        if self._bridge.link:
            # Leave the board stopped, not playing a queue nobody is feeding.
            self._bridge.stop()
            self._bridge.link.close()

    # ----- ROS side ----------------------------------------------------------

    def _on_trajectory(self, msg):
        self._trajectory = msg

    def _call(self, function, *args):
        """Run function on the serial thread and wait for its (ok, message)."""
        done = threading.Event()
        result = [(False, 'arm_bridge is busy, try again')]

        def job():
            result[0] = function(*args)
            done.set()
        self._commands.put(job)
        done.wait(COMMAND_TIMEOUT)
        return result[0]

    def _trigger_handler(self, function):
        def handler(request, response):
            response.success, response.message = self._call(function)
            return response
        return handler

    def _on_enable(self, request, response):
        response.success, response.message = self._call(self._bridge.enable, request.data)
        return response

    def _run(self):
        msg = self._trajectory
        if msg is None or not msg.points:
            return False, 'No path from camera_path_ik yet'
        try:
            positions = [self._config.reorder(msg.joint_names, list(pt.positions))
                         for pt in msg.points]
        except PlanError as e:
            return False, str(e)
        times = [pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9 for pt in msg.points]
        return self._bridge.start_run(times, positions)

    # ----- serial thread -----------------------------------------------------

    def _serial_loop(self):
        while self._running and rclpy.ok():
            started = time.monotonic()
            while not self._commands.empty():
                self._commands.get()()
            self._bridge.poll()
            self._publish()
            time.sleep(max(0.0, self._period - (time.monotonic() - started)))

    def _publish(self):
        b = self._bridge
        if b.connected and b.positions is not None:
            joints = JointState()
            joints.header.stamp = self.get_clock().now().to_msg()
            joints.name = self._config.urdf_names
            joints.position = [float(v) for v in b.positions]
            self._joint_pub.publish(joints)
        self._polls += 1
        if self._polls % STATUS_EVERY:
            return
        board = b.board
        self._status_pub.publish(ArmStatus(
            connected=b.connected,
            state=board.get('state', ''),
            homed=board.get('homed') == '1',
            enabled=board.get('enabled') == '1',
            running=b.run is not None,
            progress=float(b.progress),
            duration=float(b.run.duration if b.run else 0.0),
            message=b.message))


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.shutdown()
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
