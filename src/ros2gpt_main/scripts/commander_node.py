#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros2gpt_main.movegroup_client import MoveGroupClient  # Import the MoveItClient class

def main(args=None):
    rclpy.init(args=args)

    # Create the MoveGroup client
    moveit_client = MoveGroupClient(group_name='tmr_arm')

    # Define the target pose (position + orientation)
    target_pose = Pose()
    target_pose.position.x = 0.3
    target_pose.position.y = -0.15
    target_pose.position.z = 0.25
    target_pose.orientation.w = 1.0  # Neutral orientation

    # Send the goal to the MoveGroup action server
    moveit_client.send_goal(target_pose, end_effector_link='flange')

    rclpy.spin_once(moveit_client)

    moveit_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()