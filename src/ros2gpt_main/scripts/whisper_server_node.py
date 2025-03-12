#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2gpt_interfaces.srv import TranscribeAudio
import numpy as np
import whisper

class WhisperServerNode(Node):
    def __init__(self):
        super().__init__('whisper_server_node')

        # Initialize Whisper model
        self.get_logger().info("Loading Whisper model...")
        self.model = whisper.load_model("base")  # Use "base", "small", or others

        # Create the service
        self.transcribe_audio_service = self.create_service(
            TranscribeAudio,
            '/transcribe_audio',
            self.transcribe_audio_callback
        )
        
        self.get_logger().info("Whisper Node is ready.")
        
    def transcribe_audio_callback(self, request, response):
        """
        Callback function to handle the incoming audio data for transcription.
        """
        try:
            # Extract audio data from the request
            audio_data = np.array(request.audio_data, dtype=np.float32)

            # Whisper expects audio in float32 format, scaled to [-1, 1]
            audio_data = np.clip(audio_data, -1.0, 1.0)  # Ensure values are in range

            # Perform transcription
            self.get_logger().info("Processing audio with Whisper...")
            result = self.model.transcribe(audio_data, fp16=False)  # Disable FP16 for CPU-only systems

            # Set the transcribed text as the response
            response.transcribed_text = result["text"]
            self.get_logger().info(f"Transcribed Text: {result['text']}")
        except Exception as e:
            self.get_logger().error(f"Failed to transcribe audio: {e}")
            response.transcribed_text = "Error: Could not transcribe the audio."
        
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = WhisperServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
