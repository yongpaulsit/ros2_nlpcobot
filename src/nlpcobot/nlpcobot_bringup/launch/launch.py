# filepath manipulation
import os

# Basic Launch file requirements
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():    
    # Launch Arguments
    namespace = LaunchConfiguration("namespace")
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", 
        default_value="", 
        description="Top level namespace. Not Implemented",
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true",
    )
    
    # Speech to Text Node
    speech_to_text_node = Node(
        package='nlpcobot_cpp_py',
        executable='speech_to_text_node.py',
        output='screen',
        parameters= [{"use_sim_time": use_sim_time}],
    )
    
    # Detect Object Node
    detect_object_node = Node(
        package='nlpcobot_cpp_py',
        executable='detect_object_node.py',
        output='screen',
        parameters= [{"use_sim_time": use_sim_time}],
    )
    
    # Image Publisher Node
    image_publisher_node = Node(
        package='nlpcobot_cpp_py',
        executable='image_publisher_node.py',
        output='screen',
        parameters= [{"use_sim_time": use_sim_time}],
    )
    
    # Image Service Node
    image_service_node = Node(
        package='nlpcobot_cpp_py',
        executable='image_service_node.py',
        output='screen',
        parameters= [{"use_sim_time": use_sim_time}],
    )
    
    # Parse Command Node
    parse_command_node = Node(
        package='nlpcobot_cpp_py',
        executable='parse_command_node.py',
        output='screen',
        parameters= [{"use_sim_time": use_sim_time}],
    )
    
    # NLP Cobot Node
    nlpcobot_node = Node(
        package='nlpcobot_cpp_py',
        executable='nlpcobot_node.py',
        output='screen',
        parameters= [{"use_sim_time": use_sim_time}],
    )
    
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
    
    # tmr_ros2 driver
    cobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nlpcobot_bringup'), 
                'launch',
                'cobot.launch.py'
            )
        )
    )
    
    # create launch description object
    ld = LaunchDescription()
    
    # declare launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(cobot_launch)
    
    # my stuff
    ld.add_action(detect_object_node)
    ld.add_action(image_publisher_node)
    ld.add_action(image_service_node)
    ld.add_action(move_group_interface_node)
    ld.add_action(nlpcobot_node)
    ld.add_action(parse_command_node)
    ld.add_action(speech_to_text_node)
    
    # good luck
    return ld
