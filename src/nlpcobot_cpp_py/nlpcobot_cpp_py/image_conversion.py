#!/usr/bin/env python3
import numpy as np
import cv2
from cv_bridge import CvBridge
from PIL import Image
from sensor_msgs.msg import Image as SensorImage

class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()

    def pil_to_ros(self, image: Image) -> SensorImage:      return self.cv2_to_ros(self.pil_to_cv2(image))
    def ros_to_pil(self, image: SensorImage) -> Image:      return self.cv2_to_pil(self.ros_to_cv2(image))
    def pil_to_cv2(self, image: Image) -> np.ndarray:       return cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
    def cv2_to_pil(self, image: np.ndarray) -> Image:       return Image.fromarray(cv2.cvtColor(np.array(image), cv2.COLOR_BGR2RGB))
    def cv2_to_ros(self, image: np.ndarray) -> SensorImage: return self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
    def ros_to_cv2(self, image: SensorImage) -> np.ndarray: return self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        
    def image_ros_to_pil(self, im: SensorImage) -> Image:
        try:
            img = Image.frombytes("RGB", (im.width, im.height), im.data)
        except:
            print("Failed to convert Image from sensor_msgs to PIL")
            return None
        return img
    
# abandoned stuff, can be ignored
if False:
    import rclpy.time

    def pillow_check_mode(self, im: Image):
        if im.mode == 'RGB':
            return "rgb8"
        if im.mode == 'RGBA':
            return "rgba8"
        return None

    def image_pil_to_ros(self, im: Image) -> SensorImage:
        try:
            img = SensorImage()
            img.header.stamp = rclpy.time.Time().to_msg()
            img.header.frame_id = "camera_frame"
            img.width, img.height = im.size
            img.encoding = self.pillow_check_mode(im)
            img.is_bigendian = 0 # Assume all PIL Images are little endian
            img.step = img.width * (len(img.encoding) - 1) * 1 # Full row length in bytes
            img.data = im.tobytes()
        except Exception as e:
            print(f"Failed to convert Image from PIL to sensor_msgs: {e}")
            return None
        return img