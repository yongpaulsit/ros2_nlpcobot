#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nlpcobot_interfaces.srv import ParseCommand, DetectObject
from nlpcobot_cpp_py.image_conversion import ImageConverter
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener

# testing
from sensor_msgs.msg import Image

# Test Cases
sentences = [
    "Pick up the Blue Block for me, please",
    "Move left",
    "Place the red ball on the table",
    "Pick the small green cube",
    "Move forward",
    "Grab the yellow cylinder"
]

class NLPCobotNode(Node):
    def __init__(self):
        super().__init__('nlpcobot_node')
        
        self.get_logger().info("Initializing NLPCobotNode...")
        
        # Initialize Variables
        self._action = ''
        self._labels = ['']
        self._img_msg = Image()
        
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
        self.get_logger().info("Transcribed Audio Listenener Ready!")
        
        # Sensor Image Listener
        self.sensor_image_subscriber = self.create_subscription(
            Image,
            'sensor_image',
            self.sensor_image_listener_callback,
            10
        )
        self.sensor_image_subscriber
        self.get_logger().info("Sensor Image Listener Ready!")
        
        # Parse Command
        self.parse_command_client = self.create_client(
            ParseCommand,
            'parse_command'
        )
        while not self.parse_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ParseCommandNode')
        self.get_logger().info("Parse Command Service Ready!")
        
        # Detect Object
        self.detect_object_client = self.create_client(
            DetectObject,
            'detect_object'
        )
        while not self.detect_object_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for DetectObjectNode')
        self.get_logger().info("Detect Object Service Ready!")
        
        # Testing
        self.get_logger().info("Starting Test Case!")
        text = "Pick up the Cat for me, please"
        action, labels = self.parse_command_request(text)
        image = self._img_msg
        position = self.detect_object_request(image, labels)
        
        self.get_logger().info("Test Case Completed!")
        
    def transcribed_audio_listener_callback(self, msg):
        if msg.data:
            self.get_logger().info('I heard: "%s"' % msg.data)
            self._action, self._labels = self.parse_command_request(msg.data)
        else:
            return
        
    def sensor_image_listener_callback(self, msg):
        self._img_msg = msg
        self.get_logger().info('I got an Image!')
        
    def parse_command_request(self, text):
        request = ParseCommand.Request()
        request.text = text
        future = self.parse_command_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'Parsing Result: Action: {future.result().action}, Labels: {future.result().labels}')
        return future.result().action, future.result().labels
        
    def detect_object_request(self, image, labels):
        request = DetectObject.Request()
        request.image = image
        request.labels = labels
        future = self.detect_object_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'Parsing Result: Point: {future.result().position}')
        return future.result().position
    
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
            point_world = self.tf_buffer.transform(point_camera, "world", rclpy.duration.Duration(seconds=1.0))
            return [point_world.point.x, point_world.point.y, point_world.point.z]

        except Exception as e:
            return None
            
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
