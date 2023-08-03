import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from easy_handeye2.common_launch import arg_calibration_type, arg_tracking_base_frame, arg_tracking_marker_frame, arg_robot_base_frame, \
    arg_robot_effector_frame


def generate_launch_description():

    name = PythonExpression(['"easy_handeye2_demo_eih" if "eye_in_hand" == "',
                             LaunchConfiguration('calibration_type'),
                             '" else "easy_handeye2_demo_eob"']),

    incl_simulators = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('easy_handeye2_demo'), 'launch', 'robot_and_sim.launch.py')),
        launch_arguments={
            'name': name,
            'calibration_type': LaunchConfiguration('calibration_type'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
        }.items())

    incl_easy_handeye_publish = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('easy_handeye2'), 'launch', 'publish.launch.py')),
        launch_arguments={
            'name': name,
        }.items())

    incl_easy_handeye_evaluate = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('easy_handeye2'), 'launch', 'evaluate.launch.py')),
        launch_arguments={
            'name': name,
        }.items())

    return LaunchDescription([
        arg_calibration_type,
        arg_tracking_base_frame,
        arg_tracking_marker_frame,
        arg_robot_base_frame,
        arg_robot_effector_frame,
        incl_simulators,
        incl_easy_handeye_publish,
        incl_easy_handeye_evaluate,
    ])
