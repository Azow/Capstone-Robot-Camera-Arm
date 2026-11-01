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
    urdf_dir = os.path.join(get_package_share_directory('arm_description_v2'), 'urdf')
    urdf_path = os.path.join(urdf_dir, 'camera_arm_v2.urdf')
    with open(urdf_path) as f:
        robot_description = f.read()
    with open(os.path.join(urdf_dir, 'scene.urdf')) as f:
        scene_description = f.read()
    rviz_config = os.path.join(get_package_share_directory('camera_path_ik'),
                               'rviz', 'preview.rviz')

    sample_period = ParameterValue(LaunchConfiguration('sample_period'), value_type=float)
    arrow_every = ParameterValue(LaunchConfiguration('arrow_every'), value_type=int)
    free_spin = ParameterValue(LaunchConfiguration('free_spin'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument(
            'sample_period', default_value='0.02',
            description='Seconds between dense path points (editor drawing and IK use the same)'),
        DeclareLaunchArgument(
            'arrow_every', default_value='5',
            description='Draw an aim arrow on every Nth dense point'),
        # Off: the arm keeps every URDF joint limit. free_spin:=true is a testing aid.
        DeclareLaunchArgument(
            'free_spin', default_value='false',
            description='Ignore the Theta and Yaw limits so they can spin freely'),

        # The editor: table panel and path drawing.
        Node(
            package='camera_path_editor',
            executable='path_node',
            name='camera_path_node',
            output='screen',
            parameters=[{'sample_period': sample_period,
                         'arrow_every': arrow_every}],
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
                'sample_period': sample_period,
                'free_spin': free_spin,
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        # The pillar, on its own so the Camera View can hide the arm but still show it.
        # Its description goes to /scene/robot_description; TF still goes to /tf_static.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='scene_state_publisher',
            namespace='scene',
            parameters=[{'robot_description': scene_description}],
        ),
        # Blank image + camera info, so RViz can draw the scene from the camera.
        Node(package='arm_description_v2', executable='fake_camera'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
