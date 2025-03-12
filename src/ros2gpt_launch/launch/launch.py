import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation time'),

        # Launch ros2gpt_node
        Node(
            package='ros2gpt_main',
            executable='ros2gpt',
            name='ros2gpt',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Launch whisper_node
        Node(
            package='ros2gpt_main',
            executable='whisper',
            name='whisper',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
    ])
