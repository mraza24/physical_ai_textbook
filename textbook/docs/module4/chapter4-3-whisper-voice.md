---
sidebar_position: 4
title: Chapter 4.3 - Whisper Voice Commands
---

# Chapter 4.3: Whisper Voice Commands

**Module**: 4 - Vision-Language-Action
**Week**: 12
**Estimated Reading Time**: 30 minutes

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. Install and configure OpenAI Whisper for speech-to-text
2. Create ROS 2 node for voice input processing
3. Integrate Whisper with LLM planning pipeline
4. Handle audio capture and noise filtering
5. Build voice-controlled robot interface

---

## Prerequisites

- Completed Chapter 4.2 (LLM Integration)
- Microphone or audio input device
- Understanding of audio processing basics

---

## Introduction

**OpenAI Whisper** is a state-of-the-art speech-to-text (STT) model that enables **hands-free robot control** through voice commands. Instead of typing "pick up the red cup", you can simply say it.

**Why Voice Control?**
- **Natural Interface**: Speak to robots like humans
- **Hands-Free**: Control robots while carrying objects
- **Accessibility**: Easier for users with mobility limitations
- **Multilingual**: Whisper supports 99+ languages

**Example Workflow**:
```
User: "Robot, move to the kitchen and grab a water bottle"
           ↓
Whisper: Transcribe audio → "robot move to the kitchen and grab a water bottle"
           ↓
LLM: Generate plan → [navigate(kitchen), pick_object("water_bottle")]
           ↓
Robot: Execute actions
```

---

## Key Terms

:::info Glossary Terms
- **Whisper**: OpenAI's robust speech-to-text model (99 languages, trained on 680k hours)
- **STT**: Speech-to-Text conversion
- **VAD**: Voice Activity Detection (detect when user is speaking)
- **Wake Word**: Trigger phrase to activate voice control (e.g., "Hey robot")
- **Inference Time**: Time to transcribe audio (50ms-2s depending on model size)
- **Real-Time Factor**: Ratio of processing time to audio duration (<1.0 for real-time)
:::

---

## Core Concepts

### 1. Whisper Architecture

**Whisper** is a transformer-based encoder-decoder model trained on 680,000 hours of multilingual web data.

#### Model Variants

| Model | Parameters | Speed (RTF) | Accuracy (WER) | Use Case |
|-------|------------|-------------|----------------|----------|
| **tiny** | 39M | 0.05 | 15% | Real-time edge (Jetson) |
| **base** | 74M | 0.10 | 12% | Fast desktop |
| **small** | 244M | 0.25 | 9% | Balanced (Recommended) |
| **medium** | 769M | 0.50 | 7% | High accuracy |
| **large** | 1550M | 1.00 | 5% | Maximum accuracy (cloud) |

**RTF (Real-Time Factor)**: 0.10 = 10× faster than real-time (1 sec audio → 0.1 sec processing)

**WER (Word Error Rate)**: Lower is better (5% = 95% accuracy)

#### Installation

```bash
# Install Whisper
pip install openai-whisper

# For faster inference (optional, CUDA required)
pip install openai-whisper[gpu]

# Test installation
whisper --help
```

#### Basic Usage

**Command Line**:
```bash
# Transcribe audio file
whisper audio.mp3 --model small --language en

# Output: audio.txt, audio.srt, audio.vtt
```

**Python API**:
```python
import whisper

# Load model (first time downloads ~500MB for 'small')
model = whisper.load_model("small")

# Transcribe audio
result = model.transcribe("audio.mp3")

print(result["text"])
# Output: "Robot, pick up the red cup and place it on the table."
```

**Real-Time Streaming**:
```python
import whisper
import pyaudio
import numpy as np

model = whisper.load_model("base")

# Audio capture
p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)

print("Listening...")

try:
    while True:
        # Capture 3 seconds of audio
        frames = []
        for _ in range(0, int(16000 / 1024 * 3)):
            data = stream.read(1024)
            frames.append(np.frombuffer(data, dtype=np.int16))

        # Convert to float32 audio
        audio = np.concatenate(frames).astype(np.float32) / 32768.0

        # Transcribe
        result = model.transcribe(audio, fp16=False, language='en')
        text = result["text"].strip()

        if text:
            print(f"You said: {text}")

except KeyboardInterrupt:
    stream.stop_stream()
    stream.close()
    p.terminate()
```

### 2. Audio Capture and Processing

#### PyAudio Setup

**Installation**:
```bash
# Ubuntu/Debian
sudo apt-get install portaudio19-dev
pip install pyaudio

# macOS
brew install portaudio
pip install pyaudio

# Windows
pip install pyaudio
```

**List Audio Devices**:
```python
import pyaudio

p = pyaudio.PyAudio()
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    print(f"{i}: {info['name']} (channels: {info['maxInputChannels']})")

p.terminate()
```

#### Voice Activity Detection (VAD)

**Problem**: Don't want to transcribe silence or background noise.
**Solution**: Use VAD to detect when user is speaking.

**Simple Energy-Based VAD**:
```python
import numpy as np

def detect_voice(audio, threshold=500):
    """Detect if audio contains speech"""
    energy = np.sqrt(np.mean(audio**2))
    return energy > threshold

# Usage
audio_chunk = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
if detect_voice(audio_chunk):
    print("Voice detected!")
```

**Advanced: Silero VAD (More Robust)**:
```bash
pip install silero-vad
```

```python
import torch
from silero_vad import load_silero_vad, get_speech_timestamps

model = load_silero_vad()

# Detect speech segments
speech_timestamps = get_speech_timestamps(audio, model, sampling_rate=16000)

for segment in speech_timestamps:
    start = segment['start']
    end = segment['end']
    print(f"Speech from {start}ms to {end}ms")
```

#### Noise Filtering

**Problem**: Background noise reduces transcription accuracy.
**Solution**: Apply noise reduction filter.

```bash
pip install noisereduce
```

```python
import noisereduce as nr

# Reduce noise
reduced_audio = nr.reduce_noise(y=audio, sr=16000)

# Now transcribe cleaned audio
result = model.transcribe(reduced_audio)
```

### 3. ROS 2 Voice Node

#### Custom Message

**File**: `voice_interfaces/msg/VoiceCommand.msg`
```
string text
float32 confidence
std_msgs/Header header
```

**Build**:
```bash
cd ~/ros2_ws
colcon build --packages-select voice_interfaces
source install/setup.bash
```

#### Whisper Voice Node

**File**: `whisper_voice_node.py`
```python
import rclpy
from rclpy.node import Node
from voice_interfaces.msg import VoiceCommand
import whisper
import pyaudio
import numpy as np
from threading import Thread
import queue

class WhisperVoiceNode(Node):
    def __init__(self):
        super().__init__('whisper_voice_node')

        # Publisher
        self.pub = self.create_publisher(VoiceCommand, 'voice_commands', 10)

        # Load Whisper model
        model_size = self.declare_parameter('model_size', 'base').value
        self.model = whisper.load_model(model_size)
        self.get_logger().info(f'Loaded Whisper model: {model_size}')

        # Audio queue
        self.audio_queue = queue.Queue()

        # Start audio capture thread
        self.capture_thread = Thread(target=self.capture_audio, daemon=True)
        self.capture_thread.start()

        # Start processing thread
        self.process_thread = Thread(target=self.process_audio, daemon=True)
        self.process_thread.start()

        self.get_logger().info('Voice node ready. Listening...')

    def capture_audio(self):
        """Capture audio in background thread"""
        p = pyaudio.PyAudio()
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024
        )

        try:
            while rclpy.ok():
                # Capture 3 seconds
                frames = []
                for _ in range(0, int(16000 / 1024 * 3)):
                    data = stream.read(1024)
                    frames.append(np.frombuffer(data, dtype=np.int16))

                audio = np.concatenate(frames).astype(np.float32) / 32768.0

                # Only queue if voice detected
                if self.detect_voice(audio):
                    self.audio_queue.put(audio)

        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()

    def detect_voice(self, audio, threshold=500):
        """Simple energy-based VAD"""
        energy = np.sqrt(np.mean(audio**2))
        return energy > threshold

    def process_audio(self):
        """Process audio from queue"""
        while rclpy.ok():
            try:
                audio = self.audio_queue.get(timeout=1)

                # Transcribe
                result = self.model.transcribe(audio, fp16=False, language='en')
                text = result["text"].strip()

                if text:
                    # Publish voice command
                    msg = VoiceCommand()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.text = text
                    msg.confidence = 1.0  # Whisper doesn't provide confidence

                    self.pub.publish(msg)
                    self.get_logger().info(f'Command: "{text}"')

            except queue.Empty:
                continue

def main():
    rclpy.init()
    node = WhisperVoiceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run**:
```bash
ros2 run voice_control whisper_voice_node --ros-args -p model_size:=small
```

### 4. Voice Control Pipeline

#### Complete System: Voice → Whisper → LLM → Robot

**Architecture**:
```
Microphone → Whisper Node → LLM Planner Node → Robot Executor
                ↓                  ↓                    ↓
        voice_commands      action_plan         /cmd_vel, /gripper
```

#### Robot Executor Node

**File**: `robot_executor_node.py`
```python
import rclpy
from rclpy.node import Node
from voice_interfaces.msg import VoiceCommand
from llm_planner_interfaces.srv import GetPlan
from geometry_msgs.msg import Twist

class RobotExecutorNode(Node):
    def __init__(self):
        super().__init__('robot_executor_node')

        # Subscribe to voice commands
        self.sub = self.create_subscription(
            VoiceCommand,
            'voice_commands',
            self.voice_callback,
            10
        )

        # Client for LLM planner
        self.planner_client = self.create_client(GetPlan, 'get_plan')

        # Publisher for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Robot executor ready')

    def voice_callback(self, msg):
        """Handle voice command"""
        command = msg.text
        self.get_logger().info(f'Received voice command: "{command}"')

        # Get plan from LLM
        plan_request = GetPlan.Request()
        plan_request.command = command

        future = self.planner_client.call_async(plan_request)
        future.add_done_callback(lambda f: self.execute_plan(f.result()))

    def execute_plan(self, response):
        """Execute action plan"""
        if not response.success:
            self.get_logger().error(f'Planning failed: {response.message}')
            return

        actions = response.actions
        self.get_logger().info(f'Executing {len(actions)} actions')

        for action in actions:
            self.execute_action(action)

    def execute_action(self, action):
        """Execute single action"""
        # Parse action (simplified)
        if 'navigate' in action:
            self.get_logger().info(f'Navigating: {action}')
            # Send navigation command
            twist = Twist()
            twist.linear.x = 0.5
            self.cmd_vel_pub.publish(twist)

        elif 'pick_object' in action:
            self.get_logger().info(f'Picking object: {action}')
            # Send gripper command

        # Add more action handlers...

def main():
    rclpy.init()
    node = RobotExecutorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Launch Complete System

**File**: `voice_control.launch.py`
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Whisper voice node
        Node(
            package='voice_control',
            executable='whisper_voice_node',
            name='whisper',
            parameters=[{'model_size': 'small'}]
        ),

        # LLM planner node
        Node(
            package='llm_planner',
            executable='llm_planner_node',
            name='llm_planner',
            parameters=[{'api_key': 'sk-...'}]
        ),

        # Robot executor
        Node(
            package='voice_control',
            executable='robot_executor_node',
            name='executor'
        )
    ])
```

**Run**:
```bash
ros2 launch voice_control voice_control.launch.py
```

**Test**:
```bash
# Speak into microphone:
"Robot, move forward"
# Output:
# [whisper]: Command: "robot move forward"
# [llm_planner]: Plan: ['navigate(1.0, 0, 0)']
# [executor]: Navigating: navigate(1.0, 0, 0)
```

---

## Practical Example: Voice-Controlled Coffee Robot

**Scenario**: User controls coffee-making robot with voice.

### Step 1: Add Wake Word Detection

**Install**:
```bash
pip install pvporcupine  # Picovoice wake word engine
```

**Usage**:
```python
import pvporcupine

porcupine = pvporcupine.create(keywords=['computer'])  # Wake word: "computer"

while True:
    pcm = stream.read(porcupine.frame_length)
    keyword_index = porcupine.process(pcm)

    if keyword_index >= 0:
        print("Wake word detected! Listening for command...")
        # Now capture and transcribe command
```

### Step 2: Full Voice Control Loop

```python
import whisper
import pyaudio
import pvporcupine
import numpy as np

model = whisper.load_model("base")
porcupine = pvporcupine.create(keywords=['computer'])

p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=512)

print("Say 'Computer' to activate...")

while True:
    # Listen for wake word
    pcm = stream.read(porcupine.frame_length)
    keyword_index = porcupine.process(np.frombuffer(pcm, dtype=np.int16))

    if keyword_index >= 0:
        print("Listening...")

        # Capture 5 seconds of command
        frames = []
        for _ in range(0, int(16000 / 512 * 5)):
            data = stream.read(512)
            frames.append(np.frombuffer(data, dtype=np.int16))

        audio = np.concatenate(frames).astype(np.float32) / 32768.0

        # Transcribe
        result = model.transcribe(audio, fp16=False)
        command = result["text"].strip()

        if command:
            print(f"Command: {command}")

            # Send to robot
            # ros2_send_command(command)
```

### Step 3: Test Commands

**Test Script**:
```bash
# Say: "Computer"
# Robot: "Listening..."
# Say: "Make me a cup of coffee"
# Robot: "Understood. Making coffee..."

# Expected output:
# [whisper]: "make me a cup of coffee"
# [llm_planner]: Plan: ['navigate(0.5, 0.5, 0)', 'pick_object("cup")', ...]
# [executor]: Executing 9 actions...
```

---

## Advanced Topics

### Multi-Speaker Recognition

**Use Case**: Identify who is speaking (for personalized commands).

```bash
pip install speechbrain
```

```python
from speechbrain.pretrained import SpeakerRecognition

# Enroll speakers
speaker_model = SpeakerRecognition.from_hparams(source="speechbrain/spkrec-ecapa-voxceleb")

# Compare audio to enrolled speakers
score, prediction = speaker_model.verify_batch(enrollment_audio, test_audio)

if prediction:
    print("Authorized user detected")
```

### Multilingual Commands

**Whisper supports 99 languages**. Auto-detect or specify language:

```python
# Auto-detect language
result = model.transcribe(audio)
print(f"Detected language: {result['language']}")

# Force specific language
result = model.transcribe(audio, language='es')  # Spanish
```

**Example**:
```
User (Spanish): "Robot, tráeme una botella de agua"
Whisper: Transcribe → "robot traeme una botella de agua"
LLM: Understand Spanish → Generate plan
```

### Continuous Listening with Interruption

**Use Case**: User can interrupt robot mid-task.

```python
import threading

is_listening = threading.Event()
is_listening.set()

def continuous_listen():
    while is_listening.is_set():
        audio = capture_audio()
        text = model.transcribe(audio)["text"]

        if "stop" in text.lower():
            print("Stopping current task")
            robot.stop()

        elif text:
            robot.execute_command(text)

thread = threading.Thread(target=continuous_listen, daemon=True)
thread.start()
```

---

## Performance Optimization

### GPU Acceleration

**Use FP16 for 2× speedup on CUDA GPUs**:
```python
model = whisper.load_model("small").cuda()
result = model.transcribe(audio, fp16=True)  # FP16 inference
```

### Faster-Whisper (C++ Backend)

**10× faster than Python Whisper**:
```bash
pip install faster-whisper
```

```python
from faster_whisper import WhisperModel

model = WhisperModel("small", device="cuda", compute_type="float16")

segments, info = model.transcribe(audio, beam_size=5)

for segment in segments:
    print(f"[{segment.start:.2f}s -> {segment.end:.2f}s] {segment.text}")
```

**Benchmark**:
- Python Whisper (small): 0.25 RTF (250ms for 1s audio)
- Faster-Whisper (small): 0.03 RTF (30ms for 1s audio)

---

## Summary

This chapter covered **Whisper Voice Commands**:

1. **Whisper Models**: Tiny (39M) to Large (1.5B), 99 languages
2. **Audio Capture**: PyAudio, VAD, noise filtering
3. **ROS 2 Integration**: Voice node publishing commands
4. **Complete Pipeline**: Voice → Whisper → LLM → Robot
5. **Optimizations**: GPU, faster-whisper, wake word detection

**Key Takeaways**:
- Whisper enables natural voice control for robots
- "small" model (244M) provides best speed/accuracy tradeoff
- VAD reduces false positives from background noise
- Wake words (e.g., "Computer") prevent accidental activation
- Faster-whisper achieves 10× speedup over Python Whisper

**Next Chapter**: End-to-end VLA system integration combining vision, language, and action.

---

## End-of-Chapter Exercises

### Exercise 1: Voice-Controlled Robot (Difficulty: Medium)

**Tasks**:
1. Set up Whisper voice node with "base" model
2. Integrate with LLM planner from Chapter 4.2
3. Test 10 voice commands (navigate, pick, place)
4. Measure transcription accuracy (WER)
5. Measure end-to-end latency (voice to action)

**Success Criteria**: <5% WER, <2s latency

### Exercise 2: Wake Word Activation (Difficulty: Hard)

**Tasks**:
1. Install Picovoice Porcupine
2. Add wake word detection ("Computer" or "Robot")
3. Only transcribe after wake word
4. Add confirmation beep when wake word detected
5. Test false positive rate (should activate <1% of time)

**Success Criteria**: Wake word detection >95% accurate

---

## Further Reading

1. **Whisper Paper**: Radford et al., "Robust Speech Recognition via Large-Scale Weak Supervision" (2022)
   - https://arxiv.org/abs/2212.04356
2. **Whisper GitHub**: https://github.com/openai/whisper
3. **Faster-Whisper**: https://github.com/guillaumekln/faster-whisper
4. **Picovoice Porcupine (Wake Words)**: https://picovoice.ai/platform/porcupine/
5. **Silero VAD**: https://github.com/snakers4/silero-vad

---

## Next Chapter

Continue to **[Chapter 4.4: End-to-End VLA System](./chapter4-4-vla-system)** to integrate vision, language, and action into a complete embodied AI system.

**Module 4 Complete!** You can now build voice-controlled robots with VLA models.
