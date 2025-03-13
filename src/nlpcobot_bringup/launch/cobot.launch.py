# filepath manipulation
import os
import sys

# Basic Launch file requirements
from launch import LaunchDescription
from launch_ros.actions import Node

# Other package Launch files
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # tmr_ros2 driver
    tm5_700_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('tm5-700_moveit_config')), 
             '/launch/tm5-700_run_move_group.launch.py']
        )
    )
    
    ld = LaunchDescription()
    ld.add_action(tm5_700_launch_file)
    return ld
