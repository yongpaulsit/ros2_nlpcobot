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
    ld = LaunchDescription()
    
    # Speech to Text Node
    speech_to_text_node = Node(
        package='nlpcobot_cpp_py',
        executable='speech_to_text_node.py',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
    )
    
    # Detect Object Node
    detect_object_node = Node(
        package='nlpcobot_cpp_py',
        executable='detect_object_node.py',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
    )
    
    # Image Publisher Node
    image_publisher_node = Node(
        package='nlpcobot_cpp_py',
        executable='image_publisher_node.py',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
    )
    
    # Parse Command Node
    parse_command_node = Node(
        package='nlpcobot_cpp_py',
        executable='parse_command_node.py',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
    )
    
    # Parse Command Node
    nlpcobot_node = Node(
        package='nlpcobot_cpp_py',
        executable='nlpcobot_node.py',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
    )
    
    # tmr_ros2 driver
    tm5_700_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('tm5-700_moveit_config')), 
             '/launch/tm5-700_run_move_group.launch.py']
        )
    )
    
    ld.add_action(tm5_700_launch_file)
    ld.add_action(nlpcobot_node)
    ld.add_action(speech_to_text_node)
    ld.add_action(parse_command_node)
    ld.add_action(image_publisher_node)
    ld.add_action(detect_object_node)
    return ld
