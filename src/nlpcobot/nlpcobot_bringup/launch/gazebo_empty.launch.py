# this cli creates an empty world
# ign gazebo empty.sdf
# this cli spawns the tm5-700 arm in the world
# ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/yongp/workspaces/nlpcobot_ws/src/nlpcobot/nlpcobot_description/urdf/tm5-700.urdf", name: "tm5-700"'

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # Configure robot_description
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

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

    ld = LaunchDescription()
    ld.add_action(gz_sim)
    ld.add_action(spawn_entity)
    return ld
