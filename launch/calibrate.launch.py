import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    arg_eye_in_hand = DeclareLaunchArgument('eye_in_hand')
    arg_tracking_base_frame = DeclareLaunchArgument('tracking_base_frame', default_value='tracking_origin')
    arg_tracking_marker_frame = DeclareLaunchArgument('tracking_marker_frame', default_value='tracking_marker')
    arg_robot_base_frame = DeclareLaunchArgument('robot_base_frame', default_value='panda_link0')
    arg_robot_effector_frame = DeclareLaunchArgument('robot_effector_frame', default_value='panda_link8')

    name = PythonExpression(['"easy_handeye2_demo_eih" if "true" == "',
                             LaunchConfiguration('eye_in_hand'),
                             '" else "easy_handeye2_demo_eob"']),

    incl_simulators = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('easy_handeye2_demo'), 'launch', 'robot_and_sim.launch.py')),
        launch_arguments={
            'name': name,
            'eye_in_hand': LaunchConfiguration('eye_in_hand'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
        }.items())

    incl_easy_handeye_calibrate = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('easy_handeye2'), 'launch', 'calibrate.launch.py')),
        condition=IfCondition(LaunchConfiguration('eye_in_hand')),
        launch_arguments={
            'name': name,
            'eye_in_hand': LaunchConfiguration('eye_in_hand'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
        }.items())

    return LaunchDescription([
        arg_eye_in_hand,
        arg_tracking_base_frame,
        arg_tracking_marker_frame,
        arg_robot_base_frame,
        arg_robot_effector_frame,
        incl_simulators,
        incl_easy_handeye_calibrate,
    ])
