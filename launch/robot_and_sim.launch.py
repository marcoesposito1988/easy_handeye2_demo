# launch the robot and tracking simulators

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    arg_name = DeclareLaunchArgument('name')
    arg_eye_in_hand = DeclareLaunchArgument('eye_in_hand', default_value='True')
    arg_tracking_base_frame = DeclareLaunchArgument('tracking_base_frame', default_value='tracking_origin')
    arg_tracking_marker_frame = DeclareLaunchArgument('tracking_marker_frame', default_value='tracking_marker')
    arg_robot_base_frame = DeclareLaunchArgument('robot_base_frame', default_value='panda_link0')
    arg_robot_effector_frame = DeclareLaunchArgument('robot_effector_frame', default_value='panda_link8')

    include_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('easy_handeye2_demo'), 'launch/run_move_group.launch.py')))

    node_tracking = Node(
        package='easy_handeye2_demo',
        executable='tracking_simulator',
        name='tracking_simulator',
        parameters=[{
            # simulator arguments
            'frequency': 10,
            'translation_noise_stdev': 0.001,
            'rotation_noise_stdev': 0.0001,
            'hand_to_tracking': '0.12 0.21 0.137 0 0 0 1',
            'base_to_tracking': '1 0 0.5 0 0 0 1',
            # calibration arguments
            'name': LaunchConfiguration('name'),
            'eye_in_hand': LaunchConfiguration('eye_in_hand'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
        }]
    )

    return LaunchDescription([
        arg_name,
        arg_eye_in_hand,
        arg_tracking_base_frame,
        arg_tracking_marker_frame,
        arg_robot_base_frame,
        arg_robot_effector_frame,
        include_robot,
        node_tracking
    ])
