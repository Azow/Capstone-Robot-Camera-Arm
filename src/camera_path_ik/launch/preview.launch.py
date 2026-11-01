"""Preview IK: the editor, the arm model, the IK node, a camera view, and RViz.

Type a path in the editor panel and the arm follows it in RViz. The Camera
panel shows the view from the arm's camera, so the focus point (red) should
stay in the middle of it.

arm:=fake or arm:=serial also runs arm_bridge, so the editor's Arm controls
can run the path on the fake board or the real one (port:=..., default
/dev/octopus). RViz then shows the board's position as the solid arm and the
preview as a see-through ghost.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


FAKE_PORT = '/tmp/fake_octopus'


def _arm_dependent_nodes(context, urdf_path, robot_description, sample_period, free_spin):
    """ik_node and RViz, plus arm_bridge, the fake board and the ghost arm for arm:=fake/serial."""
    arm = LaunchConfiguration('arm').perform(context)
    rviz_dir = os.path.join(get_package_share_directory('camera_path_ik'), 'rviz')
    ik_node = Node(
        package='camera_path_ik',
        executable='ik_node',
        name='camera_path_ik',
        output='screen',
        parameters=[{
            'urdf_path': urdf_path,
            'sample_period': sample_period,
            'free_spin': free_spin,
        }],
        # With an arm attached, /joint_states is the board's position; the preview
        # moves the ghost instead.
        remappings=[] if arm == 'none' else [('/joint_states', '/preview/joint_states')],
    )
    rviz_config = os.path.join(rviz_dir, 'preview.rviz' if arm == 'none' else 'preview_arm.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )
    if arm == 'none':
        return [ik_node, rviz]

    port = FAKE_PORT if arm == 'fake' else LaunchConfiguration('port').perform(context)
    actions = [ik_node, rviz]
    if arm == 'fake':
        actions.append(ExecuteProcess(
            cmd=['ros2', 'run', 'arm_bridge', 'fake_board', '--link', FAKE_PORT],
            output='screen'))
    actions += [
        Node(
            package='arm_bridge',
            executable='arm_bridge',
            name='arm_bridge',
            output='screen',
            parameters=[{'port': port}],
        ),
        # The ghost: the IK preview on its own model, with every frame prefixed
        # "preview/" and attached to base_link.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='preview_state_publisher',
            namespace='preview',
            parameters=[{'robot_description': robot_description,
                         'frame_prefix': 'preview/'}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='preview_tf',
            arguments=['--frame-id', 'base_link', '--child-frame-id', 'preview/base_link'],
        ),
    ]
    return actions


def generate_launch_description():
    urdf_dir = os.path.join(get_package_share_directory('arm_description_v2'), 'urdf')
    urdf_path = os.path.join(urdf_dir, 'camera_arm_v2.urdf')
    with open(urdf_path) as f:
        robot_description = f.read()
    with open(os.path.join(urdf_dir, 'scene.urdf')) as f:
        scene_description = f.read()

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
        DeclareLaunchArgument(
            'arm', default_value='none', choices=['none', 'fake', 'serial'],
            description='Run paths on an arm: none (preview only), fake board, or serial'),
        DeclareLaunchArgument(
            'port', default_value='/dev/octopus',
            description='Serial port of the board, for arm:=serial'),

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

        # The arm: robot_state_publisher turns /joint_states into TF.
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

        # IK, RViz, and (with arm:=fake/serial) the bridge and the ghost arm.
        OpaqueFunction(function=_arm_dependent_nodes,
                       args=[urdf_path, robot_description, sample_period, free_spin]),
    ])
