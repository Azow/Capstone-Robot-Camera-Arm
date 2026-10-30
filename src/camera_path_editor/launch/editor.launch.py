"""Start the camera path editor: table panel, path node, and RViz (no arm model)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('camera_path_editor')
    rviz_config = os.path.join(share, 'rviz', 'editor.rviz')

    points_per_segment = LaunchConfiguration('points_per_segment')
    arrow_every = LaunchConfiguration('arrow_every')

    return LaunchDescription([
        DeclareLaunchArgument(
            'points_per_segment', default_value='20',
            description='Dense points generated per segment (same for both paths)'),
        DeclareLaunchArgument(
            'arrow_every', default_value='2',
            description='Draw an aim arrow on every Nth point of each segment'),

        Node(
            package='camera_path_editor',
            executable='path_node',
            name='camera_path_node',
            output='screen',
            parameters=[{
                'points_per_segment': ParameterValue(points_per_segment, value_type=int),
                'arrow_every': ParameterValue(arrow_every, value_type=int),
            }],
        ),
        Node(
            package='camera_path_editor',
            executable='path_panel',
            name='camera_path_panel',
            output='screen',
        ),
        # No arm here, so nothing else publishes TF. RViz's fixed frame is
        # base_link, which only exists once something puts it in the TF tree.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_tf',
            arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
