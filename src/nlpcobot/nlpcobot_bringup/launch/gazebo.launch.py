import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # Configure robot_description
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nlpcobot_gazebo = get_package_share_directory('nlpcobot_gazebo')
    world_path = os.path.join(pkg_nlpcobot_gazebo, 'worlds', 'shapes.sdf')

    # Launch an empty Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + world_path}.items(),
    )
    
    # ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@ignition.msgs.Image
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@ignition.msgs.Image'
        ],
        output='screen'
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

    ld = LaunchDescription()
    ld.add_action(gz_sim)
    ld.add_action(bridge)
    ld.add_action(spawn_entity)
    return ld
