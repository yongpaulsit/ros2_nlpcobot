#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from nlpcobot_interfaces.srv import ParseCommand, DetectObject, CaptureImage
from nlpcobot_interfaces.action import MoveRobot
from nlpcobot_cpp_py.image_conversion import ImageConverter
from nlpcobot_cpp_py.tf_camera_to_world import TfCamera2World
from geometry_msgs.msg import PointStamped, Point
from tf2_ros import Buffer, TransformListener

# testing
from sensor_msgs.msg import Image


class NLPCobotNode(Node):
    def __init__(self):
        super().__init__('nlpcobot_node')

        self.get_logger().info("Initializing NLPCobotNode...")

        # Initialize Variables
        self._text = None
        self._action = None
        self._labels = [[]]
        self._image_msg = Image()
        self._pixel_position = Point()

        # Static Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize Camera to World transformer
        self.tf_camera_to_world = TfCamera2World()
        self.image_converter = ImageConverter()

        # Transcribed Audio Listener
        self.transcribed_audio_subscriber = self.create_subscription(
            String,
            'transcribed_audio',
            self.transcribed_audio_listener_callback,
            10
        )
        self.transcribed_audio_subscriber

        # Camera Listener
        self.camera_subscriber = self.create_subscription(
            Image,
            'camera',
            self.camera_listener_callback,
            10
        )
        self.camera_subscriber

        # Sensor Image Listener
        self.sensor_image_subscriber = self.create_subscription(
            Image,
            'sensor_image',
            self.sensor_image_listener_callback,
            10
        )
        self.sensor_image_subscriber

        # Sensor Image Service Client
        self.capture_sensor_image_client = self.create_client(
            CaptureImage,
            'capture_sensor_image'
        )

        # Parse Command Service Client
        self.parse_command_client = self.create_client(
            ParseCommand,
            'parse_command'
        )

        # Detect Object Service Client
        self.detect_object_client = self.create_client(
            DetectObject,
            'detect_object'
        )

        # Move Robot Action Client
        self.move_robot_action_client = ActionClient(
            self, MoveRobot, 'move_robot')

        # Run
        self.create_timer(1.0, self.run)
        
        self.get_logger().info("NLPCobotNode Ready!")

    def transcribed_audio_listener_callback(self, msg):
        if msg.data:
            self.get_logger().info('I heard: "%s"' % msg.data)
            self._text = msg.data
        else:
            return

    def camera_listener_callback(self, msg):
        self._image_msg = msg

    def sensor_image_listener_callback(self, msg):
        self._image_msg = msg

    def capture_sensor_image_service_request(self):
        request = CaptureImage.Request()
        request.data = True

        while not self.capture_sensor_image_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SensorImageServiceNode')
        self.get_logger().info("Capture Sensor Image Service Ready!")

        future = self.capture_sensor_image_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Sensor Image Received!')
        return future.result().image

    def parse_command_service_request(self, text):
        request = ParseCommand.Request()
        request.text = text

        while not self.parse_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ParseCommandNode')
        self.get_logger().info("Parse Command Service Ready!")

        future = self.parse_command_client.call_async(request)
        future.add_done_callback(self.handle_parse_command_service_response)
    
    def handle_parse_command_service_response(self, future):
        try:
            self.get_logger().info(
                f'Parsing Result: Action: {future.result().action}, Labels: {future.result().labels}')
            self._action = future.result().action
            self._labels = future.result().labels
        except Exception as e:
            self.get_logger().error(e)

    def detect_object_service_request(self, image_msg, labels):
        if not image_msg.data:
            self.get_logger().info('Missing: Image...')
            return

        if not labels:
            self.get_logger().info('Missing: Labels...')
            return

        request = DetectObject.Request()
        request.image = image_msg
        request.labels = labels

        while not self.detect_object_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for DetectObjectNode')
        self.get_logger().info("Detect Object Service Ready!")

        future = self.detect_object_client.call_async(request)
        future.add_done_callback(self.handle_detect_object_service_response)
        
    def handle_detect_object_service_response(self, future):
        self.get_logger().info(
            f'Parsing Result: Point: {future.result().position}')
        self._pixel_position = future.result().position

    def send_goal_move_robot(self, position):
        goal_msg = MoveRobot.Goal()
        goal_msg.x, goal_msg.y, goal_msg.z = position

        self.move_robot_action_client.wait_for_server()

        return self.move_robot_action_client.send_goal_async(goal_msg)

    # Function to transform from camera frame to world frame
    def transform_pixel(self):
        u = self._pixel_position.x
        v = self._pixel_position.y
        camera_x, camera_y, camera_z = 0.4, 0, 1.0
        position = self.tf_camera_to_world.pixel_to_world(
            u, v, camera_x, camera_y, camera_z)
        return position

    def run(self):
        if self._text:
            self.parse_command_service_request(self._text)
            self._text = None
            
        if self._image_msg.data and self._action:
            self.image_converter.show(self._image_msg)
            if (self._action == "pick" or self._action == "place") and self._labels:
                self.detect_object_service_request(self._image_msg, self._labels)
            elif self.action == "move":
                self.get_logger().info("Not Implemented!")
            self._image_msg = Image()
            self._action = None
            self._labels = [[]]
            
        if self._pixel_position.x:
            position = self.transform_pixel()
            self.send_goal_move_robot(position)
            self._pixel_position = Point()

def main(args=None):
    rclpy.init(args=args)

    node = NLPCobotNode()
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        rclpy.spin(node)
        # executor.spin()

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
        pass

    finally:
        # executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
