---
sidebar_position: 3
title: Whisper for Voice Commands
---

# Whisper for Voice Commands: The Robot's Ears

The most natural way for a human to interact with a humanoid robot is through speech. To enable this, our robot needs a robust **Automatic Speech Recognition (ASR)** system to act as its ears. OpenAI's **Whisper** is a state-of-the-art, open-source ASR model that is perfect for this task.

## Why Whisper?

-   **High Accuracy:** Whisper has been trained on a massive and diverse dataset of audio from the web, making it incredibly accurate at transcribing English and many other languages, even in noisy environments.
-   **Robustness:** It handles a wide variety of accents, background noise, and technical language exceptionally well.
-   **Open Source:** You can run Whisper models locally on your own hardware, which is critical for a robotics application where you may not have a constant internet connection and want to minimize latency.

## Integrating Whisper into ROS 2

The goal is to create a ROS 2 node that listens to a microphone's audio stream and publishes the transcribed text to a topic.

```mermaid
graph TD
    A[Microphone Hardware] --> B(Audio Driver Node);
    B -- /audio_stream (AudioDataMsg) --> C{Whisper ASR Node};
    C -- "Bring me the water bottle" --> D[/transcribed_text (StringMsg)];
    D --> E[LLM Cognitive Planner];
```

### The `whisper_ros` Node

Let's outline the structure of a Python `rclpy` node that accomplishes this. This node will:
1.  Subscribe to an audio stream topic. For simplicity, we'll assume a node is already publishing microphone data to `/audio_stream`.
2.  Collect incoming audio chunks into a buffer.
3.  When a pause is detected (silence), send the complete audio buffer to the Whisper model for transcription.
4.  Publish the resulting text.

:::info
Running Whisper requires a powerful GPU for real-time performance. This fits perfectly into our Isaac ROS ecosystem. You'll need to install the `openai-whisper` package and `PyAudio` or a similar library for audio input.
:::

### Example: A Whisper ROS 2 Node

```python
# whisper_ros_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import whisper
import numpy as np
import torch

class WhisperASRNode(Node):
    def __init__(self):
        super().__init__('whisper_asr_node')
        self.subscription = self.create_subscription(
            AudioData,
            '/audio_stream', # Assuming audio is published here
            self.audio_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/transcribed_text', 10)
        
        # Load the Whisper model
        # 'base', 'small', 'medium', 'large' offer different trade-offs
        # in speed and accuracy.
        self.get_logger().info("Loading Whisper model...")
        self.model = whisper.load_model("base.en") # Using the small English model
        self.get_logger().info("Whisper model loaded.")

        # Audio buffer
        self.audio_buffer = []

        # Simple silence detection state
        self.silence_frames = 0
        self.is_speaking = False

    def audio_callback(self, msg):
        # Convert audio data to a numpy array that Whisper can process
        # The exact conversion depends on your audio source format
        audio_float = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

        # Simple energy-based silence detection
        is_silent = np.mean(np.abs(audio_float)) < 0.01

        if self.is_speaking and is_silent:
            self.silence_frames += 1
            if self.silence_frames > 20: # If silent for ~1 second
                self.get_logger().info("Detected end of speech, transcribing...")
                
                # Process the buffered audio
                full_audio = np.concatenate(self.audio_buffer)
                result = self.model.transcribe(full_audio)
                
                # Publish the transcription
                text_msg = String()
                text_msg.data = result['text']
                self.publisher_.publish(text_msg)
                self.get_logger().info(f"Published text: {result['text']}")

                # Reset buffer and state
                self.audio_buffer = []
                self.is_speaking = False
                self.silence_frames = 0
        elif not is_silent:
            self.is_speaking = True
            self.silence_frames = 0
            self.audio_buffer.append(audio_float)

def main(args=None):
    rclpy.init(args=args)
    whisper_node = WhisperASRNode()
    rclpy.spin(whisper_node)
    whisper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
This node provides the first and most crucial step in the VLA chain: converting the user's spoken command into a format that the robot's cognitive brain can begin to understand and process.
