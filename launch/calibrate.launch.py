import os

from ament_index_python import get_package_share_directory
from easy_handeye2.common_launch import arg_calibration_type, arg_automatic_robot_movement, arg_translation_delta_meters, \
    arg_rotation_delta_degrees, arg_max_velocity_scaling, arg_max_acceleration_scaling
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    name = PythonExpression(['"easy_handeye2_demo_eih" if "eye_in_hand" == "',
                             LaunchConfiguration('calibration_type'),
                             '" else "easy_handeye2_demo_eob"'])

    tracking_base_frame = 'tracking_base'
    tracking_marker_frame = 'tracking_marker'
    robot_base_frame = 'panda_link0'
    robot_effector_frame = 'panda_link8'

    incl_simulators = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('easy_handeye2_demo'), 'launch', 'robot_and_sim.launch.py')),
        launch_arguments={
            'name': name,
            'calibration_type': LaunchConfiguration('calibration_type'),
            'tracking_base_frame': tracking_base_frame,
            'tracking_marker_frame': tracking_marker_frame,
            'robot_base_frame': robot_base_frame,
            'robot_effector_frame': robot_effector_frame,
        }.items())

    incl_easy_handeye_calibrate = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('easy_handeye2'), 'launch', 'calibrate.launch.py')),
        launch_arguments={
            'name': name,
            'calibration_type': LaunchConfiguration('calibration_type'),
            'tracking_base_frame': tracking_base_frame,
            'tracking_marker_frame': tracking_marker_frame,
            'robot_base_frame': robot_base_frame,
            'robot_effector_frame': robot_effector_frame,
            'automatic_robot_movement': LaunchConfiguration('automatic_robot_movement'),
            'move_group': 'panda_arm',
            'move_group_namespace': '',
            'translation_delta_meters': LaunchConfiguration('translation_delta_meters'),
            'rotation_delta_degrees': LaunchConfiguration('rotation_delta_degrees'),
            'max_velocity_scaling': LaunchConfiguration('max_velocity_scaling'),
            'max_acceleration_scaling': LaunchConfiguration('max_acceleration_scaling'),
        }.items())

    return LaunchDescription([
        arg_calibration_type,
        arg_automatic_robot_movement,
        arg_translation_delta_meters,
        arg_rotation_delta_degrees,
        arg_max_velocity_scaling,
        arg_max_acceleration_scaling,
        incl_simulators,
        incl_easy_handeye_calibrate,
    ])
