#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread

import torch
from RealtimeSTT import AudioToTextRecorder

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('realtimestt_node')
        self.get_logger().info("Initializing RealtimeSTTNode...")
        self.get_logger().info("Wait until it says Speak Now...")
        
        self.publisher_ = self.create_publisher(
            String, 
            'transcribed_audio',
            10
        )
        
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.recorder = AudioToTextRecorder(
            language='en', 
            device=device, 
            # wake_words="jarvis"
        )
        
        self.thread = Thread(target=self.main, daemon=True)
        self.thread.start()
        
    def process_text(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info("I said: '%s'" % msg.data)
        
    def main(self):
        while True:
            self.recorder.text(self.process_text)
            
def main(args=None):
    rclpy.init(args=args)
    
    node = SpeechToTextNode()
    
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
        