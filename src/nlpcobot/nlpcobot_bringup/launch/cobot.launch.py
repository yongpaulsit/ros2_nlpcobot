# filepath manipulation
import os

# Basic Launch file requirements
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    publish_srdf = DeclareLaunchArgument(
        "publish_robot_description_semantic",
        default_value='true',
    )
    
    use_gazebo = DeclareLaunchArgument(
        "use_gazebo",
        default_value='false',
    )
    
    # tmr_ros2 driver
    tm5_700_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tm5-700_moveit_config'), 
                'launch',
                'tm5-700_moveit.launch.py'
            )
        ),
        launch_arguments=[(
            'use_gazebo', LaunchConfiguration('use_gazebo'),
        )],
    )
    
    ld = LaunchDescription()
    ld.add_action(publish_srdf)
    ld.add_action(use_gazebo)
    ld.add_action(tm5_700_launch_file)
    return ld
