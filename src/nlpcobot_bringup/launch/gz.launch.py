import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    launch_arguments = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="true", 
        description="Use sim time if true"
    ),

    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            os.path.join(package_name, "worlds"),
            ":" + str(Path(desc_package_name).parent.resolve()),
        ],
    )

    xacro_file = os. path.join("desc_pkg", "urdf", "robot.urdf.xacro")

    robot_description_config = Command(
        ["xacro ", xacro_file, " sim_mode:=", use_sim_time]
    )

    # Create a robot_state_publisher node
    params = {
        "robot_description": launch_ros.descriptions.ParameterValue(
            robot_description_config, value_type=str
        ),
        "use_sim_time": use_sim_time,
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    world = os.path.join(
        get_package_share_directory(package_name), "worlds", "empty_world.sdf"
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", world]}.items(),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "groundhog",
            "-topic",
            "/robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "1.4",
        ],
        output="screen",
    )

    # Launch!
    return LaunchDescription([
        launch_arguments,
        ign_resource_path,
        robot_state_publisher_node,
        gz_sim,
        spawn_entity,
    ])
