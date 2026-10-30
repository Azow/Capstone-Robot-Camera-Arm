"""Preview IK: the editor, the arm model, the IK node, a camera view, and RViz.

Type a path in the editor panel and the arm follows it in RViz. The Camera
panel shows the view from the arm's camera, so the focus point (red) should
stay in the middle of it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory('arm_description'),
                             'urdf', 'camera_arm.urdf')
    with open(urdf_path) as f:
        robot_description = f.read()
    rviz_config = os.path.join(get_package_share_directory('camera_path_ik'),
                               'rviz', 'preview.rviz')

    points_per_segment = ParameterValue(LaunchConfiguration('points_per_segment'),
                                        value_type=int)
    playback_rate = ParameterValue(LaunchConfiguration('playback_rate'), value_type=float)
    free_spin = ParameterValue(LaunchConfiguration('free_spin'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument(
            'points_per_segment', default_value='10',
            description='Dense points per segment (editor drawing and IK use the same)'),
        DeclareLaunchArgument(
            'playback_rate', default_value='20.0',
            description='Path points played per second'),
        # Temporary: on by default for now. Use free_spin:=false for the URDF limits.
        DeclareLaunchArgument(
            'free_spin', default_value='true',
            description='Ignore the Theta and Yaw limits so they can spin freely'),

        # The editor: table panel and path drawing.
        Node(
            package='camera_path_editor',
            executable='path_node',
            name='camera_path_node',
            output='screen',
            parameters=[{'points_per_segment': points_per_segment}],
        ),
        Node(
            package='camera_path_editor',
            executable='path_panel',
            name='camera_path_panel',
            output='screen',
        ),

        # The arm: IK publishes joint states, robot_state_publisher turns them into TF.
        Node(
            package='camera_path_ik',
            executable='ik_node',
            name='camera_path_ik',
            output='screen',
            parameters=[{
                'urdf_path': urdf_path,
                'points_per_segment': points_per_segment,
                'playback_rate': playback_rate,
                'free_spin': free_spin,
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        # Blank image + camera info, so RViz can draw the scene from the camera.
        Node(package='arm_description', executable='fake_camera'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
