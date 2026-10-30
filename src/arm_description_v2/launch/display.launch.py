from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('arm_description_v2')
    urdf_path = os.path.join(pkg_share, 'urdf', 'camera_arm_v2.urdf')

    with open(urdf_path) as f:
        robot_description = f.read()

    return LaunchDescription([
        # robot_state_publisher
        Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          parameters=[{'robot_description': robot_description}],
      ),   
        # Publish joint states (with sliders) so robot_state_publisher has something to work with
        Node(
          package='joint_state_publisher_gui',
          executable='joint_state_publisher_gui',
          parameters=[{'robot_description': robot_description}],
      ),
        Node(
          package='rviz2',
          executable='rviz2',
          arguments=['-d', os.path.join(pkg_share, 'launch', 'display.rviz')],
      ),
    ])