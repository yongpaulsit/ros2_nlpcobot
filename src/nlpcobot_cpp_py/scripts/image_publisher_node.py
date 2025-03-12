#!/usr/bin/env python3
import PIL.Image
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# for testing
import requests
from PIL import Image as pImage
from nlpcobot_cpp_py.image_conversion import ImageConverter

class SensorImagePublisherNode(Node):
    def __init__(self):
        super().__init__('sensor_image_publisher_node')
        self.get_logger().info("Initializing SensorImageStreamerNode...")
        
        self.publisher_ = self.create_publisher(
            Image, 
            'sensor_image',
            10
        )
        
        self.image_converter = ImageConverter()
        
        # for testing
        image_url = "http://images.cocodataset.org/val2017/000000039769.jpg"
        self.image = pImage.open(requests.get(image_url, stream=True).raw)

        # Create Self Fulfilling Prophecy
        timer_period = 2.0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        msg = Image()
        
        # for testing, pretend an image was capture from a camera
        msg = self.image_converter.pil_to_ros(self.image)
        
        self.publisher_.publish(msg)
        self.get_logger().info('Streaming Sensor Image!')            
            
def main(args=None):
    rclpy.init(args=args)
    
    node = SensorImagePublisherNode()
    
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
        