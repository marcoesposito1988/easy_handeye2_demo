import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


import launch

import launch_pytest
from launch_pytest.tools import process as process_tools

import pytest


# @pytest.fixture
# def hello_world_proc():
#     # Launch a process to test
#     return launch.actions.ExecuteProcess(
#         cmd=['echo', 'hello_world'],
#         shell=True,
#         cached_output=True,
#     )


def generate_launch_description(calibration_type):
    # launch this packages' calibrate.launch.py 

    incl_easy_handeye_demo_calibrate = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('easy_handeye2_demo'), 'launch', 'calibrate.launch.py')),
        launch_arguments={
            'calibration_type': calibration_type,
        }.items())

    return LaunchDescription([
        incl_easy_handeye_demo_calibrate,
    ])


# This function specifies the processes to be run for our test.
@launch_pytest.fixture
def launch_description_eih():
    return generate_launch_description('eye_in_hand')

@launch_pytest.fixture
def launch_description_eob():
    return generate_launch_description('eye_on_base')

@pytest.mark.launch(fixture=launch_description)
def test_read_stdout(hello_world_proc, launch_context):
    """Check if 'hello_world' was found in the stdout."""
    def validate_output(output):
        # this function can use assertions to validate the output or return a boolean.
        # pytest generates easier to understand failures when assertions are used.
        assert output.splitlines() == ['hello_world'], 'process never printed hello_world'
    process_tools.assert_output_sync(
        launch_context, hello_world_proc, validate_output, timeout=5)

    def validate_output(output):
        return output == 'this will never happen'
    assert not process_tools.wait_for_output_sync(
        launch_context, hello_world_proc, validate_output, timeout=0.1)
    yield
    # this is executed after launch service shutdown
    assert hello_world_proc.return_code == 0