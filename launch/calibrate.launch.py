import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    arg_eye_in_hand = DeclareLaunchArgument('eye_in_hand', default_value='True')

    include_robot_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('easy_handeye2_demo'), 'launch', 'robot_and_sim.launch.py')),
        launch_arguments={'eye_in_hand': 'eye_in_hand'}.items())

    # TODO: push to easy_handeye2/calibrate.launch.py
    node_dummy_calib = Node(package='tf2_ros', executable='static_transform_publisher', name='dummy_publisher', arguments='1 1 1 0 1.5 0 $(arg robot_base_frame) $(arg tracking_base_frame) 10')

    return LaunchDescription([
        arg_eye_in_hand,
        include_robot_sim
    ])
