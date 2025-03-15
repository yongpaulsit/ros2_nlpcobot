#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nlpcobot_interfaces.srv import ParseCommand, DetectObject
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
        self._action = None
        self._labels = [[]]
        self._image_msg = Image()
        self._position = Point()

        # Static Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize Camera to World transformer
        self.tf_camera_to_world = TfCamera2World()

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
        self._image_msg = msg
        # self.get_logger().info('I got an Image!')

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

    def detect_object_service_request(self, labels):
        if not self._image_msg.data:
            self.get_logger().info('Missing: Image...')
            return
        
        if not labels:
            self.get_logger().info('Missing: Labels...')
            return
        print(labels)
        
        request = DetectObject.Request()
        request.image = self._image_msg
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
    def transform_point(self, position):
        try:
            x, y, z = position
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
        if self._action == "place" or self._action == "pick":
            self.get_logger().info('I am a Pick/Place command!')
            if not self._labels:
                self.get_logger().info('Labels is not defined!')
                return
            if self._action == "place":
                self.get_logger().info('I am a Place command!')
            elif self._action == "pick":
                self.get_logger().info('I am a Place command!')
        elif self._action == "move":
            self.get_logger().info('I am a Move command!')
        else:
            self.get_logger().info('Command not recognised, please try again!')
            
    def test_case(self):
        # Testing
        self.get_logger().info("Starting Test Case!")
        text = "Pick up the Cat for me, please"
        action, labels = self.parse_command_service_request(text)
        self.check_command_action_callback()
        pixel_position = self.detect_object_service_request(labels)
        u = pixel_position.x
        v = pixel_position.y
        camera_x, camera_y, camera_z = 0.4, 0, 1.0
        position = self.tf_camera_to_world.pixel_to_world(u, v, camera_x, camera_y, camera_z)
        # position = self.transform_point(position)
        self.send_goal_move_robot(position)

        self.get_logger().info("Test Case Completed!")

def main(args=None):
    rclpy.init(args=args)

    node = NLPCobotNode()

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        #TESTING
        rclpy.spin_once(node)
        node.test_case()
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
