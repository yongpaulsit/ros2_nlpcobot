#!/usr/bin/env python3
from RealtimeSTT import AudioToTextRecorder

def process_text(text):
    print(text)
    
if __name__ == '__main__':
    recorder = AudioToTextRecorder(language='en', device='cpu')

    while True:
        recorder.text(process_text)
        
    recorder = AudioToTextRecorder(language='en', device='cpu', wake_words="jarvis")
    
    print('Say "jarvis" to start recording.')
    print(recorder.text())