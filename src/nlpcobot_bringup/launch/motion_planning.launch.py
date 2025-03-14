import os

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Configure robot_description
    description_path = 'tm_description'
    xacro_path = 'tm5-700.urdf.xacro'
    moveit_config_path = 'tm5-700_moveit_config'    
    srdf_path = 'config/tm5-700.srdf'
    
    robot_description_path = os.path.join(
        get_package_share_directory(description_path),
        'xacro',
        xacro_path,
    )
    
    # SRDF Configuration
    robot_description_semantic_path = os.path.join(
        get_package_share_directory(moveit_config_path),
        srdf_path,
    )
    
    # Controllers
    controllers_path = os.path.join(
        get_package_share_directory(moveit_config_path),
        'config/controllers.yaml',
    )
    print('hi')
    moveit_config = (
        MoveItConfigsBuilder('tm5-700')
        .robot_description(file_path=robot_description_path)
        .robot_description_semantic(file_path=robot_description_semantic_path)
        .trajectory_execution(file_path=controllers_path)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )  

    print('hi')
    move_group_interface_node = Node(
        package="nlpcobot_cpp_py",
        executable="move_group_interface_node",
        name="move_group_interface",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    
    return LaunchDescription(
        [
            move_group_interface_node,
        ]
    )