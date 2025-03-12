#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tm_msgs.srv import SetPositions

import time

class TMRobotMoveClient(Node):
    def __init__(self):
        super().__init__('tm_robot_move_client')
        self.client = self.create_client(SetPositions, 'set_positions')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_positions service...')

        self.get_logger().info('TM Robot Move Client Ready!')
        
    def send_position_joint(self):
        req = SetPositions.Request()
        req.motion_type = SetPositions.Request.PTP_J
        req.positions = [0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0]  # Assuming the robot accepts XYZ format
        req.velocity = 0.4 # m/s
        req.acc_time = 0.2 # ms
        req.blend_percentage = 10 # %
        req.fine_goal = False

        self.get_logger().info(f'Sending position: {req.positions}, Motion Type: {req.motion_type}')
        future = self.client.call_async(req)
        future.add_done_callback(self.response_callback)

    def send_position_cartesian(self, x, y, z):
        """
        Send a position command to the TM Cobot.

        :param x: float, X position in meters.
        :param y: float, Y position in meters.
        :param z: float, Z position in meters.
        """
        req = SetPositions.Request()
        req.motion_type = SetPositions.Request.PTP_T
        req.positions = [x, y, z, 0.0, 1.5708, 0.0]  # Assuming the robot accepts XYZ format
        req.velocity = 0.200 # m/s
        req.acc_time = 500.0 # ms
        req.blend_percentage = 0 # %
        req.fine_goal = False

        self.get_logger().info(f'Sending position: {req.positions}, Motion Type: {req.motion_type}')
        future = self.client.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info('Motion command executed successfully!')
            else:
                self.get_logger().warn('Failed to execute motion command.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TMRobotMoveClient()

    node.send_position_joint()
    # node.send_position_cartesian(0.03, 0.02, 0.20)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
