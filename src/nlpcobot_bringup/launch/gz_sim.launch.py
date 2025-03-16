# filepath manipulation
import os

# Basic Launch file requirements
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

import xacro


def generate_launch_description():
    use_gazebo = DeclareLaunchArgument(
        "use_gazebo",
        default_value='true',
    )
    
    # Configure robot_description
    # pkg_project_description = get_package_share_directory('tm_description')
    # pkg_moveit_config = get_package_share_directory('tm5-700_moveit_config')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_project_bringup = get_package_share_directory('nlpcobot_bringup')
    # xacro_path = 'tm5-700.urdf.xacro'

    # robot_description_path = os.path.join(
    #     pkg_project_description,
    #     'xacro',
    #     xacro_path,
    # )

    # robot_description_config = xacro.process_file(
    #     robot_description_path,
    #     mappings={'use_gazebo': 'true'}
    # )
    # robot_description = {'robot_description': robot_description_config.toxml()}

    # Launch an empty Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tm5_700',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(
                pkg_project_bringup,
                'config',
                'manipulator_bridge.yaml'
            ),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
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
    
    # launch arguments
    ld.add_action(use_gazebo)

    # robot
    ld.add_action(cobot_launch)

    # gazebo
    ld.add_action(gz_sim)
    ld.add_action(bridge)
    ld.add_action(spawn_entity)

    # good luck
    return ld
