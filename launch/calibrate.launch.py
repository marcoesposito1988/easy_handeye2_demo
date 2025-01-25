import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from easy_handeye2.common_launch import arg_calibration_type

# hardcode demo data

tracking_base_frame='tracking_base_frame'
tracking_marker_frame='tracking_marker_frame'
robot_base_frame='panda_link0'
robot_effector_frame='panda_link8'

def generate_launch_description():

    name = PythonExpression(['"easy_handeye2_demo_eih" if "eye_in_hand" == "',
                             LaunchConfiguration('calibration_type'),
                             '" else "easy_handeye2_demo_eob"']),

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
        }.items())

    return LaunchDescription([
        arg_calibration_type,
        incl_simulators,
        incl_easy_handeye_calibrate,
    ])
