#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nlpcobot_interfaces.srv import ParseCommand, DetectObject
from nlpcobot_interfaces.action import MoveRobot
from nlpcobot_cpp_py.image_conversion import ImageConverter
from geometry_msgs.msg import PointStamped, Point
from tf2_ros import Buffer, TransformListener

# testing
from sensor_msgs.msg import Image

class NLPCobotNode(Node):
    def __init__(self):
        super().__init__('nlpcobot_node')

        self.get_logger().info("Initializing NLPCobotNode...")

        # Initialize Variables
        self._action = ''
        self._labels = ['']
        self._img_msg = Image()
        self._position = Point()

        # Static Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Transcribed Audio Listener
        self.transcribed_audio_subscriber = self.create_subscription(
            String,
            'transcribed_audio',
            self.transcribed_audio_listener_callback,
            10
        )
        self.transcribed_audio_subscriber

        # Sensor Image Listener
        self.sensor_image_subscriber = self.create_subscription(
            Image,
            'sensor_image',
            self.sensor_image_listener_callback,
            10
        )
        self.sensor_image_subscriber

        # Parse Command
        self.parse_command_client = self.create_client(
            ParseCommand,
            'parse_command'
        )
        
        # Detect Object
        self.detect_object_client = self.create_client(
            DetectObject,
            'detect_object'
        )

        # Move Robot Action
        self.move_robot_action_client = ActionClient(
            self, MoveRobot, 'move_robot')
        
        self.get_logger().info("NLPCobotNode Ready!")

        # Testing
        self.get_logger().info("Starting Test Case!")
        text = "Pick up the Cat for me, please"
        action, labels = self.parse_command_service_request(text)
        image = self._img_msg
        position = self.detect_object_service_request(image, labels)
        self.send_goal_move_robot([0.3, -0.2, 0.4])

        self.get_logger().info("Test Case Completed!")

    def transcribed_audio_listener_callback(self, msg):
        if msg.data:
            self.get_logger().info('I heard: "%s"' % msg.data)
            try:
                self._action, self._labels = self.parse_command_service_request(msg.data)
                self.check_command_action_callback()
            except Exception as e:
                self.get_logger().error(e)
                self.get_logger().warn("Please give a command with an action")
        else:
            return

    def sensor_image_listener_callback(self, msg):
        self._img_msg = msg
        self.get_logger().info('I got an Image!')

    def parse_command_service_request(self, text):
        request = ParseCommand.Request()
        request.text = text
        
        while not self.parse_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ParseCommandNode')
        self.get_logger().info("Parse Command Service Ready!")

        future = self.parse_command_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(
            f'Parsing Result: Action: {future.result().action}, Labels: {future.result().labels}')
        return future.result().action, future.result().labels

    def detect_object_service_request(self, image, labels):
        request = DetectObject.Request()
        request.image = image
        request.labels = labels
        
        while not self.detect_object_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for DetectObjectNode')
        self.get_logger().info("Detect Object Service Ready!")
        
        future = self.detect_object_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(
            f'Parsing Result: Point: {future.result().position}')
        return future.result().position

    def send_goal_move_robot(self, position):
        goal_msg = MoveRobot.Goal()
        goal_msg.x, goal_msg.y, goal_msg.z = position

        self.move_robot_action_client.wait_for_server()

        return self.move_robot_action_client.send_goal_async(goal_msg)

    # Function to transform from camera frame to world frame
    def transform_point(self, x, y, z):
        try:
            point_camera = PointStamped()
            point_camera.header.frame_id = "camera"
            point_camera.header.stamp = self.get_clock().now().to_msg()
            point_camera.point.x = x
            point_camera.point.y = y
            point_camera.point.z = z

            # Transform point to world frame
            point_world = self.tf_buffer.transform(
                point_camera, "world", rclpy.duration.Duration(seconds=1.0))
            return [point_world.point.x, point_world.point.y, point_world.point.z]

        except Exception as e:
            return None

    def check_command_action_callback(self):
        if self._action == "Place":
            self.get_logger().info('I am a Move command!')
        elif self._action == "Pick":
            self.get_logger().info('I am a Pick command!')
        elif self._action == "Move":
            self.get_logger().info('I am a Place command!')
        else:
            self.get_logger().info('Command not recognised, please try again!')

def main(args=None):
    rclpy.init(args=args)

    node = NLPCobotNode()

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
