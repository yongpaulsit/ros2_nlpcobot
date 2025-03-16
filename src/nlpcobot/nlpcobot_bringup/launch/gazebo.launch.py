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
    # Configure robot_description
    pkg_project_bringup = get_package_share_directory('nlpcobot_bringup')
    # pkg_project_gazebo = get_package_share_directory('nlpcobot_gazebo')
    pkg_project_description = get_package_share_directory('tm_description')
    pkg_moveit_config = get_package_share_directory('tm5-700_moveit_config')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    xacro_path = 'tm5-700.urdf.xacro'
    srdf_path = 'config/tm5-700.srdf'
    
    robot_description_path = os.path.join(
        pkg_project_description,
        'xacro',
        xacro_path,
    )
    
    # SRDF Configuration
    robot_description_semantic_path = os.path.join(
        pkg_moveit_config,
        srdf_path,
    )
    
    # Controllers
    controllers_path = os.path.join(
        pkg_moveit_config,
        'config/controllers.yaml',
    )
    
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
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
    
    # Setup to launch the simulator and Gazebo world
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
    
    # Everything Else
    nlpcobot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_project_bringup, 
                'launch',
                'launch.py'
            )
        )
    )
    
    # create launch description object
    ld = LaunchDescription()
    
    # declare launnch options
    ld.add_action(declare_use_sim_time_cmd)
    
    # gazebo
    ld.add_action(gz_sim)
    # ld.add_action(bridge)
    ld.add_action(spawn_entity)
    
    # my stuff
    ld.add_action(nlpcobot)
    
    # good luck
    return ld
