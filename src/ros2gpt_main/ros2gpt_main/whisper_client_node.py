#!/usr/bin/env python3
import rclpy
from ros2gpt_interfaces.srv import TranscribeAudio
import numpy as np
import sounddevice as sd
import whisper

SAMPLE_RATE = 16000  # Whisper model expects 16kHz input
DURATION = 10  # Recording duration in seconds

class WhisperClientNode:
    def __init__(self, node):
        self.node = node
        # Create the client to call the TranscribeAudio service
        self.client = self.node.create_client(TranscribeAudio, '/transcribe_audio')

        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting again...')

        self.model = whisper.load_model("base")  # Initialize Whisper model

    def record_audio(self, duration, sample_rate):
        """
        Capture audio from the microphone.
        """
        self.node.get_logger().info(f"Recording for {duration} seconds...")
        audio = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype=np.float32)
        sd.wait()  # Wait until recording is complete
        return np.squeeze(audio)  # Remove single-dimensional entries

    def transcribe_audio(self, audio_data):
        """
        Send the audio data to the TranscribeAudio service and return the transcribed text.
        """
        try:
            request = TranscribeAudio.Request()
            request.audio_data = audio_data.tolist()  # Convert numpy array to list

            # Call the service and wait for the result
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            # Get the transcription result
            response = future.result()
            self.node.get_logger().info(f"Transcribed Text: {response.transcribed_text}")
            return response.transcribed_text
        except Exception as e:
            self.node.get_logger().error(f"Failed to transcribe audio: {e}")
            return None

    def record_and_transcribe(self):
        """
        Continuously records and sends audio data for transcription.
        """
        while rclpy.ok():
            # Record audio from the microphone
            audio_data = self.record_audio(DURATION, SAMPLE_RATE)

            # Transcribe the audio and log the result
            transcribed_text = self.transcribe_audio(audio_data)
            if transcribed_text:
                self.node.get_logger().info(f"Received transcription: {transcribed_text}")
