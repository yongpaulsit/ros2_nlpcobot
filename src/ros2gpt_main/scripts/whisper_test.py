#!/usr/bin/env python3
import whisper
import sounddevice as sd
import numpy as np
from concurrent.futures import ThreadPoolExecutor

# Load Whisper model
model = whisper.load_model("base")

# Define audio capture parameters
SAMPLE_RATE = 16000  # Whisper model expects 16kHz input
DURATION = 10  # Recording duration in seconds

def record_audio(duration, sample_rate):
    """
    Capture audio from the microphone.
    """
    print(f"Recording for {duration} seconds...")
    audio = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype=np.float32)
    sd.wait()  # Wait until recording is complete
    return np.squeeze(audio)  # Remove single-dimensional entries

def transcribe_audio(audio_data):
    """
    Transcribe the given audio data using Whisper.
    """
    print("Transcribing...")
    result = model.transcribe(audio_data, fp16=False)  # Disable FP16 for CPU-only systems
    print("Transcription: ", result['text'])

def main():
    with ThreadPoolExecutor() as executor:
        try:
            while True:
                # Record audio
                audio_data = record_audio(DURATION, SAMPLE_RATE)
                
                # Whisper expects 16-bit PCM audio scaled to [-1, 1]
                audio_data = audio_data.astype(np.float32)

                # Submit transcription to a separate thread
                executor.submit(transcribe_audio, audio_data)

        except KeyboardInterrupt:
            print("\nExiting program.")

if __name__ == "__main__":
    main()
