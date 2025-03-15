#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nlpcobot_interfaces.srv import DetectObject

from PIL import Image
from nlpcobot_cpp_py.image_conversion import ImageConverter

import torch
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

class DetectObjectNode(Node):
    def __init__(self):
        super().__init__('detect_object_node')
        self.get_logger().info("Initializing DetectObjectNode...")
        
        # Image Processing
        self.image_converter = ImageConverter()
        
        # Create Service
        self.service_ = self.create_service(
            DetectObject,
            'detect_object',
            self.service_callback,
        )

        # Object Detection
        self.model_id = "IDEA-Research/grounding-dino-tiny"
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        self.processor = AutoProcessor.from_pretrained(self.model_id)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(self.model_id).to(self.device)
        
    def service_callback(self, request, response):
        self.get_logger().info('Received request!')
        try:
            image = self.image_converter.ros_to_pil(request.image)
        except Exception as e:
            self.get_logger().error(e)
            return
        
        labels = request.labels
        
        self.get_logger().info(f'Looking for "{labels}" in image...')
        results = self.detect(image, labels)
        
        if results is None:
            self.get_logger().info('Nothing Found!')
        self.get_logger().info(results)
            
        self.parse_results(results[0])
        
        response = DetectObject.Response()
        response.position.x = 0.5
        response.position.y = 1.5
        response.position.z = -0.5
        self.get_logger().info(response.position)
        return response
        
    def detect(self, image: Image, labels):
        inputs = self.processor(images=image, text=labels, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            box_threshold=0.4,
            text_threshold=0.3,
            target_sizes=[image.size[::-1]]
        )
        
        return results
        
    def parse_results(self, result):
        # Retrieve the first image result
        for box, score, labels in zip(result["boxes"], result["scores"], result["labels"]):
            box = [round(x, 2) for x in box.tolist()]
            self.get_logger().info(f"Detected {labels} with confidence {round(score.item(), 3)} at location {box}")
            
def main(args=None):
    rclpy.init(args=args)
    
    node = DetectObjectNode()
    
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
