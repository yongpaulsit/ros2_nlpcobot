#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2gpt_interfaces.srv import TranscribeAudio
from ros2gpt_main.whisper_client_node import WhisperClientNode
# from ros2gpt_main.commander_node import CommanderNode

class Ros2GPTNode(Node):
    def __init__(self):
        super().__init__('ros2gpt_node')
        self.get_logger().info("Initializing Ros2GPTNode...")

        # Instantiate the WhisperClient to handle the Whisper interactions
        self.whisper_client_node = WhisperClientNode(self)

        # Example: Start recording and transcribing audio
        self.whisper_client_node.record_and_transcribe()

    def text_callback(self, msg):
        """
        Callback to process transcribed text (if necessary).
        """
        self.get_logger().info(f"Received transcribed text: {msg.data}")
        # Here, handle the command or pass it to another system
        self.process_command(msg.data)

    def process_command(self, text):
        """
        Example command processing logic.
        """
        self.get_logger().info(f"Processing command: {text}")
        if "pick up the pikachu" in text.lower():
            self.get_logger().info("Command: Picking up Pikachu!")
        elif "place the pikachu" in text.lower():
            self.get_logger().info("Command: Placing Pikachu next to Charmander.")
        else:
            self.get_logger().info("Command not recognized.")

def main(args=None):
    rclpy.init(args=args)
    node = Ros2GPTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
