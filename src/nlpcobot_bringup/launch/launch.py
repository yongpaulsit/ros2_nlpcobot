# filepath manipulation
import os
import sys

# Basic Launch file requirements
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

# Other package Launch files
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    print('1')
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    print('2')
    # Default Launch Arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", 
        default_value="", 
        description="Top level namespace. Not Implemented",
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="true", 
        description="Use simulation time if true",
    )
    
    print('3')
    # Speech to Text Node
    speech_to_text_node = Node(
        package='nlpcobot_cpp_py',
        executable='speech_to_text_node.py',
        output='screen',
        parameters= [{"use_sim_time": use_sim_time}],
    )
    
    print('4')
    # Detect Object Node
    detect_object_node = Node(
        package='nlpcobot_cpp_py',
        executable='detect_object_node.py',
        output='screen',
        parameters= [{"use_sim_time": use_sim_time}],
    )
    
    print('5')
    # Image Publisher Node
    image_publisher_node = Node(
        package='nlpcobot_cpp_py',
        executable='image_publisher_node.py',
        output='screen',
        parameters= [{"use_sim_time": use_sim_time}],
    )
    
    print('6')
    # Parse Command Node
    parse_command_node = Node(
        package='nlpcobot_cpp_py',
        executable='parse_command_node.py',
        output='screen',
        parameters= [{"use_sim_time": use_sim_time}],
    )
    
    print('7')
    # Move Group Interface Node
    move_group_interface_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nlpcobot_bringup'), 
                'launch',
                'move_group_interface.launch.py'
            )
        )
    )
    
    print('8')
    # NLP Cobot Node
    nlpcobot_node = Node(
        package='nlpcobot_cpp_py',
        executable='nlpcobot_node.py',
        output='screen',
        parameters= [{"use_sim_time": use_sim_time}],
    )
    
    print('9')
    # tmr_ros2 driver
    tm5_700_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tm5-700_moveit_config'), 
                'launch',
                'tm5-700_run_move_group.launch.py'
            )
        )
    )
    
    # create launch description object
    ld = LaunchDescription()
    
    # declare launnch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(detect_object_node)
    ld.add_action(tm5_700_launch_file)
    ld.add_action(speech_to_text_node)
    ld.add_action(parse_command_node)
    ld.add_action(image_publisher_node)
    ld.add_action(move_group_interface_node)
    ld.add_action(nlpcobot_node)
    
    # good luck
    return ld
