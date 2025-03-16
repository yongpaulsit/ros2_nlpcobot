#!/usr/bin/env python3
import PIL.Image
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# for testing
import requests
from PIL import Image as pImage
from nlpcobot_cpp_py.image_conversion import ImageConverter
from nlpcobot_interfaces.srv import CaptureImage

class SensorImageServiceNode(Node):
    def __init__(self):
        super().__init__('sensor_image_service_node')
        self.get_logger().info("Initializing SensorImageServiceNode...")
        
        self.service_ = self.create_service(
            CaptureImage,
            'capture_sensor_image',
            self.service_callback,
        )
        
        self.image_converter = ImageConverter()
        
        # for testing
        image_url = "http://images.cocodataset.org/val2017/000000039769.jpg"
        self.image = pImage.open(requests.get(image_url, stream=True).raw)
        
    def service_callback(self, request, response):
        self.get_logger().info(f'Capture Image Service Request Received!')
        try:
            # for testing, pretend an image was capture from a camera
            response.image = self.image_converter.pil_to_ros(self.image)
        except Exception as e:
            self.get_logger().warn(e)
            response = CaptureImage.Response()
        return response
            
def main(args=None):
    rclpy.init(args=args)
    
    node = SensorImageServiceNode()
    
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
        