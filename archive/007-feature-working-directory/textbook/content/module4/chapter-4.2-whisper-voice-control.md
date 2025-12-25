# Chapter 4.2: Voice Control with Whisper

> **Module**: 4 - Voice-Language-Action Brain
> **Week**: 11
> **Estimated Reading Time**: 26 minutes

---

## Summary

Whisper (OpenAI, 2022) is a robust speech-to-text model enabling voice-controlled robotics. This chapter covers Whisper architecture, real-time audio processing with Voice Activity Detection (VAD), ROS 2 integration for voice commands, and multi-language support. Combined with VLA models (Chapter 4.1), Whisper enables hands-free robot control: "pick up the cup" (audio) → text → robot action.

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Explain** Whisper architecture (encoder-decoder transformer with mel spectrogram input)
2. **Implement** real-time speech-to-text with Voice Activity Detection (VAD) for robotics
3. **Integrate** Whisper with ROS 2 audio topics for voice-controlled robot systems
4. **Deploy** Whisper on edge devices (Jetson Orin) with <300ms latency
5. **Leverage** multi-language support for international robot deployment (98 languages)

**Prerequisite Knowledge**: Chapter 4.1 (VLA concepts), Module 1 (ROS 2), basic audio processing (sampling rates, spectrograms)

---

## Key Terms

- **Whisper**: OpenAI's open-source speech-to-text model trained on 680k hours of multilingual audio
- **Mel Spectrogram**: Time-frequency representation of audio using mel scale (mimics human hearing)
- **VAD (Voice Activity Detection)**: Algorithm detecting speech vs. silence/noise in audio stream
- **ASR (Automatic Speech Recognition)**: Converting spoken language to text (synonymous with speech-to-text)
- **Encoder-Decoder**: Two-part transformer—encoder processes mel spectrogram, decoder generates text tokens
- **Beam Search**: Decoding algorithm exploring multiple candidate transcripts, selecting highest-probability sequence
- **CTC (Connectionist Temporal Classification)**: Alternative ASR approach aligning audio frames to text (not used in Whisper)
- **Wake Word**: Trigger phrase activating speech recognition (e.g., "Hey robot")

---

## Core Concepts

### 1. Why Whisper for Robotics?

**Challenges with Traditional ASR** (Google Speech API, Azure STT):
- ❌ **Latency**: 500-2000ms (cloud round-trip)
- ❌ **Cost**: $0.006-0.024 per 15 seconds (expensive for 24/7 robots)
- ❌ **Privacy**: Audio sent to cloud servers
- ❌ **Internet Dependency**: Fails without WiFi/4G

**Whisper Advantages**:
- ✅ **On-Device**: Runs locally (no internet required)
- ✅ **Low Latency**: 100-300ms on GPU, 500-1000ms on Jetson
- ✅ **Zero Cost**: Open-source (MIT license)
- ✅ **Robust**: Handles noise, accents, overlapping speech
- ✅ **Multilingual**: 98 languages (English, Spanish, Chinese, Arabic, etc.)

**Model Sizes** (accuracy-latency trade-off):
| Model | Parameters | English WER | Latency (RTX 4090) | Latency (Jetson Orin) | Use Case |
|-------|------------|-------------|--------------------|-----------------------|----------|
| Tiny | 39M | 5.0% | 20ms | 100ms | Real-time, low-power |
| Base | 74M | 3.4% | 35ms | 180ms | Balanced |
| Small | 244M | 2.5% | 80ms | 400ms | High accuracy |
| Medium | 769M | 2.1% | 180ms | 1200ms | Offline transcription |
| Large-v3 | 1.5B | 1.4% | 350ms | 3000ms | Maximum accuracy |

**WER (Word Error Rate)**: Lower = better. 2.5% WER = 97.5% words correct.

**Robotics Recommendation**: **Base model** (74M params)—best balance of 180ms latency (Jetson) and 3.4% WER.

---

### 2. Whisper Architecture

**High-Level Pipeline**:
```
Audio (16kHz PCM) → Mel Spectrogram (80 bins × T frames)
                            ↓
              Encoder (Transformer, 6 layers)
                            ↓
              Audio Embeddings (384D × T)
                            ↓
              Decoder (Transformer, 6 layers)
                            ↓
              Text Tokens → "pick up the cup"
```

**Component Breakdown**:

**A. Preprocessing (Audio → Mel Spectrogram)**
- **Input**: Raw audio waveform (16kHz sampling rate, mono channel)
- **STFT (Short-Time Fourier Transform)**: Split audio into 25ms windows, compute frequencies
- **Mel Filter Bank**: Apply 80 triangular filters on mel scale (0-8000 Hz)
- **Log Scale**: `log(mel_spectrogram + 1e-10)` for numerical stability
- **Output**: 80 × T matrix (80 mel bins, T time frames where T = audio_duration / 0.02s)

**Example**: 3-second audio → T = 3 / 0.02 = 150 frames → 80 × 150 spectrogram

**B. Encoder (Spectrogram → Audio Embeddings)**
- **Architecture**: Transformer encoder (Base: 6 layers, 512 hidden dim, 8 attention heads)
- **Positional Encoding**: Sinusoidal encoding for temporal information
- **Output**: 384D embeddings per time frame (T × 384D)
- **Purpose**: Extract acoustic features (phonemes, prosody, speaker characteristics)

**C. Decoder (Embeddings → Text)**
- **Architecture**: Transformer decoder (Base: 6 layers, 512 hidden dim, cross-attention to encoder)
- **Autoregressive**: Generates one token at a time (BPE vocabulary 50k tokens)
- **Special Tokens**:
  - `<|startoftranscript|>`: Begin decoding
  - `<|en|>`: Language (English), `<|es|>` (Spanish), etc.
  - `<|notimestamps|>`: No word-level timing
  - `<|endoftext|>`: Stop decoding
- **Beam Search**: Explore top-5 candidate sequences, select highest probability

**Decoding Example**:
```
Step 1: <|startoftranscript|> <|en|> <|notimestamps|>
Step 2: ... "pick"
Step 3: ... "pick" "up"
Step 4: ... "pick" "up" "the"
Step 5: ... "pick" "up" "the" "cup" <|endoftext|>
```

---

### 3. Real-Time Processing with VAD

**Problem**: Continuous audio stream—when to transcribe?

**Solution**: Voice Activity Detection (VAD)
- **Purpose**: Detect speech segments in audio, ignore silence/noise
- **Algorithm**: Silero VAD (PyTorch, 1.5M params, 2ms latency)
- **Output**: Probability [0, 1] for each 30ms audio chunk (>0.5 = speech)

**Real-Time Pipeline**:
```
1. Microphone captures 30ms audio chunks (480 samples @ 16kHz)
   ↓
2. VAD: Is this chunk speech? (Yes → buffer, No → discard)
   ↓
3. Accumulate speech chunks until 1-2 seconds of silence
   ↓
4. Whisper transcription on buffered audio
   ↓
5. Publish text to ROS 2 topic /voice/transcript
```

**Tuning Parameters**:
- `speech_threshold`: 0.5 (default), lower = more sensitive (but false positives)
- `silence_duration`: 1.5 seconds (how long to wait before finalizing transcription)
- `min_speech_duration`: 0.3 seconds (reject very short utterances)

---

### 4. ROS 2 Integration

**Node Architecture**:
```
┌─────────────────────────────────────┐
│   Microphone (USB or built-in)      │
└──────────────┬──────────────────────┘
               ↓
┌──────────────────────────────────────┐
│   audio_capture_node                 │
│   Publishes: /audio/raw (Audio msg)  │
└──────────────┬───────────────────────┘
               ↓
┌──────────────────────────────────────┐
│   whisper_node                       │
│   Subscribes: /audio/raw             │
│   Publishes: /voice/transcript (String) │
└──────────────┬───────────────────────┘
               ↓
┌──────────────────────────────────────┐
│   vla_executor_node                  │
│   Subscribes: /voice/transcript      │
│   Publishes: /joint_commands         │
└──────────────────────────────────────┘
```

**Implementation**:

**A. Audio Capture Node**:
```python
# audio_capture_node.py
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
import pyaudio

class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture_node')
        self.publisher = self.create_publisher(AudioData, '/audio/raw', 10)

        # PyAudio setup
        self.pa = pyaudio.PyAudio()
        self.stream = self.pa.open(
            format=pyaudio.paInt16,  # 16-bit PCM
            channels=1,  # Mono
            rate=16000,  # 16kHz (Whisper requirement)
            input=True,
            frames_per_buffer=480,  # 30ms chunks
            stream_callback=self.audio_callback
        )

    def audio_callback(self, in_data, frame_count, time_info, status):
        msg = AudioData()
        msg.data = list(in_data)
        self.publisher.publish(msg)
        return (in_data, pyaudio.paContinue)

def main():
    rclpy.init()
    node = AudioCaptureNode()
    rclpy.spin(node)
```

**B. Whisper Node with VAD**:
```python
# whisper_node.py
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import whisper
import torch
import numpy as np
from silero_vad import get_speech_timestamps, load_silero_vad

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        # Load models
        self.whisper_model = whisper.load_model("base", device="cuda")  # 74M params
        self.vad_model = load_silero_vad()

        # Subscribers/Publishers
        self.create_subscription(AudioData, '/audio/raw', self.audio_callback, 10)
        self.transcript_pub = self.create_publisher(String, '/voice/transcript', 10)

        # Audio buffer
        self.audio_buffer = []
        self.sample_rate = 16000

    def audio_callback(self, msg):
        # Convert to numpy array
        audio_chunk = np.frombuffer(bytes(msg.data), dtype=np.int16).astype(np.float32) / 32768.0
        self.audio_buffer.append(audio_chunk)

        # Keep last 5 seconds (5 * 16000 / 480 = 167 chunks)
        if len(self.audio_buffer) > 167:
            self.audio_buffer.pop(0)

        # Check if speech present in last 2 seconds
        if len(self.audio_buffer) < 67:  # Need 2 seconds minimum
            return

        recent_audio = np.concatenate(self.audio_buffer[-67:])  # Last 2 seconds

        # VAD
        speech_timestamps = get_speech_timestamps(
            recent_audio,
            self.vad_model,
            sampling_rate=self.sample_rate,
            return_seconds=False
        )

        # If speech detected and followed by 1.5s silence
        if len(speech_timestamps) > 0:
            last_speech_end = speech_timestamps[-1]['end']
            current_sample = len(recent_audio)
            silence_duration = (current_sample - last_speech_end) / self.sample_rate

            if silence_duration > 1.5:  # 1.5 seconds of silence
                # Transcribe entire buffer
                full_audio = np.concatenate(self.audio_buffer)
                transcript = self.transcribe(full_audio)

                # Publish
                msg = String()
                msg.data = transcript
                self.transcript_pub.publish(msg)
                self.get_logger().info(f"Transcribed: {transcript}")

                # Clear buffer
                self.audio_buffer = []

    def transcribe(self, audio):
        audio_tensor = torch.from_numpy(audio).cuda()
        result = self.whisper_model.transcribe(audio_tensor, language='en')
        return result['text'].strip()

def main():
    rclpy.init()
    node = WhisperNode()
    rclpy.spin(node)
```

---

### 5. Wake Word Detection (Optional)

**Problem**: Robot transcribes all speech (including background conversations)

**Solution**: Wake word ("Hey robot") activates Whisper

**Implementation** (using Porcupine wake word engine):
```python
import pvporcupine

# Initialize Porcupine (free tier: 3 built-in wake words)
porcupine = pvporcupine.create(keywords=["computer"])  # or "jarvis", "alexa"

def audio_callback_with_wake_word(self, msg):
    audio_chunk = np.frombuffer(bytes(msg.data), dtype=np.int16)

    # Check for wake word
    keyword_index = porcupine.process(audio_chunk)

    if keyword_index >= 0:
        self.get_logger().info("Wake word detected! Listening...")
        self.listening_active = True
        self.listening_start_time = time.time()

    # Auto-deactivate after 10 seconds
    if self.listening_active and (time.time() - self.listening_start_time > 10):
        self.listening_active = False

    # Only transcribe if active
    if self.listening_active:
        # ... (existing transcription code)
```

---

### 6. Multi-Language Support

Whisper supports **98 languages** with single model—no retraining required.

**Language Detection** (automatic):
```python
result = model.transcribe(audio, language=None)  # Auto-detect
print(result['language'])  # Prints 'en', 'es', 'zh', etc.
```

**Force Specific Language**:
```python
result = model.transcribe(audio, language='es')  # Force Spanish
# Input: "recoge la taza roja"
# Output: "pick up the red cup" (if translate=True)
```

**Translation to English** (built-in):
```python
result = model.transcribe(audio, task='translate')
# Input (Spanish audio): "recoge la taza roja"
# Output (English text): "pick up the red cup"
```

**Performance by Language** (WER on Whisper Base):
| Language | WER | Notes |
|----------|-----|-------|
| English | 3.4% | Best (680k hours training data) |
| Spanish | 4.2% | High-resource language |
| Chinese | 5.8% | Mandarin, logographic script |
| Arabic | 7.1% | Dialectal variation |
| Hindi | 8.5% | Medium-resource |
| Swahili | 12.3% | Low-resource |

---

### 7. Edge Deployment (Jetson Orin)

**Challenges**:
- **Limited VRAM**: Jetson Orin Nano (8GB) vs RTX 4090 (24GB)
- **Slower Inference**: ARM CPU + Maxwell/Volta GPU
- **Power Constraints**: 10-15W budget for mobile robots

**Optimization Strategies**:

**A. Model Selection**: Use **Whisper Tiny** (39M params)
- VRAM: 300MB (fits easily on Jetson)
- Latency: 100ms on Jetson Orin Nano
- WER: 5.0% (acceptable for robotics commands)

**B. Quantization** (FP16 → INT8):
```bash
# Convert Whisper to TensorRT (INT8)
python -m whisper.export --model tiny --output whisper_tiny_int8.engine --int8
```
Result: 100ms → 60ms (1.7× speedup), 300MB → 150MB VRAM

**C. Faster-Whisper** (CTranslate2 backend):
- **Library**: https://github.com/guillaumekln/faster-whisper
- **Speed**: 4× faster than vanilla Whisper (optimized kernels)
- **Accuracy**: Identical to original Whisper

```python
from faster_whisper import WhisperModel

model = WhisperModel("base", device="cuda", compute_type="int8")
segments, info = model.transcribe(audio)
transcript = " ".join([seg.text for seg in segments])
```

**Jetson Orin Nano Performance**:
| Model | Backend | Latency | VRAM | Accuracy |
|-------|---------|---------|------|----------|
| Tiny (vanilla) | PyTorch FP32 | 200ms | 600MB | 5.0% WER |
| Tiny (faster-whisper) | CTranslate2 FP16 | 50ms | 300MB | 5.0% WER |
| Tiny (faster-whisper) | CTranslate2 INT8 | 35ms | 150MB | 5.3% WER |

**Recommendation**: **Faster-Whisper Tiny INT8** → 35ms latency, 150MB VRAM (optimal for Jetson)

---

## Practical Example: Voice-Controlled Pick-and-Place

### Overview

Build complete voice-controlled robot system: "Hey robot, pick up the red cup" → Whisper transcription → VLA execution.

### Implementation

**Step 1: Install Dependencies**

```bash
# Install Whisper + Faster-Whisper
pip install openai-whisper faster-whisper

# Install VAD
pip install silero-vad

# Install audio capture
sudo apt install portaudio19-dev
pip install pyaudio

# Install ROS 2 audio messages
sudo apt install ros-humble-audio-common
```

**Step 2: Launch Full System**

```bash
# Terminal 1: Audio capture
ros2 run voice_control audio_capture_node

# Terminal 2: Whisper transcription
ros2 run voice_control whisper_node

# Terminal 3: VLA executor (from Chapter 4.1)
ros2 run voice_control vla_executor_node

# Terminal 4: Robot hardware
ros2 launch ur_robot_driver ur5_bringup.launch.py
```

**Step 3: Test Voice Commands**

```bash
# Monitor transcriptions
ros2 topic echo /voice/transcript

# Speak into microphone:
User: "Pick up the red cup"
# System output:
[whisper_node]: Transcribed: "pick up the red cup"
[vla_executor_node]: Executing VLA with instruction: "pick up the red cup"
[ur5_driver]: Received joint commands, executing trajectory...
```

### Expected Outcome

- **Latency**: 180ms transcription + 80ms VLA + 40ms ROS 2 = **300ms total** (acceptable for manipulation)
- **Success Rate**: 85% on clear speech (3m distance, quiet room), 70% with background noise
- **Failure Modes**:
  - **Homophones**: "red cup" vs. "read cup" (context helps: VLA trained on manipulation, not literacy)
  - **Noisy environments**: WER increases 3.4% → 8.5% at 70dB background noise
  - **Accented speech**: Non-native English speakers may see 5-10% higher WER

### Troubleshooting

- **Issue**: High latency (500ms+ transcription)
  - **Solution**: Use Faster-Whisper backend (4× speedup) or switch to Tiny model

- **Issue**: VAD too sensitive (transcribes random noise)
  - **Solution**: Increase `speech_threshold` (0.5 → 0.7), add wake word detection

- **Issue**: Robot executes incorrect action (e.g., "pick blue bottle" → picks red cup)
  - **Solution**: VLA problem (not Whisper)—fine-tune VLA with 500 demos, verify transcription accuracy first

---

## Exercises

### Exercise 1: Whisper Model Comparison (Difficulty: Easy)

**Objective**: Measure accuracy-latency trade-off across Whisper models

**Task**: Transcribe same 10-second audio with Tiny/Base/Small, measure WER and latency

**Requirements**:
- Use LibriSpeech test set (100 utterances)
- Report: WER (%), latency (ms), VRAM (MB)
- Hardware: RTX 4090 or equivalent

**Expected Outcome**:
| Model | WER | Latency | VRAM |
|-------|-----|---------|------|
| Tiny | 5.0% | 20ms | 300MB |
| Base | 3.4% | 35ms | 600MB |
| Small | 2.5% | 80ms | 1.2GB |

**Estimated Time**: 60 minutes

---

### Exercise 2: Voice-Controlled Navigation (Difficulty: Medium)

**Objective**: Extend Whisper to mobile robot navigation

**Task**: Implement voice commands ("go forward 2 meters", "turn left 90 degrees")

**Requirements**:
- Parse Whisper transcript to extract navigation parameters
- Publish to /cmd_vel topic
- Test: 10 voice commands, measure success rate

**Expected Outcome**: 90% command recognition, 80% successful navigation

**Estimated Time**: 3 hours

---

### Exercise 3: Multi-Language Robot Interface (Difficulty: Hard)

**Objective**: Support English + Spanish voice commands

**Task**: Detect language, translate to English, execute VLA

**Requirements**:
- Auto-detect language (Whisper `language=None`)
- If Spanish: use `task='translate'` → English
- VLA executes English instruction
- Test: 20 commands (10 English, 10 Spanish)

**Expected Outcome**: 85% Spanish→English→Action success rate

**Estimated Time**: 4 hours (data collection + testing)

---

## Summary & Key Takeaways

- **Whisper Architecture**: Encoder-decoder transformer processing mel spectrograms (80 bins) → text tokens
- **Real-Time Processing**: VAD (Silero) detects speech segments, buffers 1-2 seconds → Whisper transcription
- **ROS 2 Integration**: Audio capture node → Whisper node → VLA executor (300ms end-to-end latency)
- **Edge Deployment**: Faster-Whisper Tiny INT8 achieves 35ms on Jetson Orin Nano (4× speedup vs vanilla)
- **Multi-Language**: 98 languages supported, auto-detect or translate to English for VLA
- **Robotics Recommendation**: **Base model** (74M params, 180ms Jetson, 3.4% WER) best balance for voice-controlled manipulation

**Connection to Chapter 4.3**: Whisper provides text instructions to VLAs, but for complex multi-step tasks, we need task planning—Chapter 4.3 covers LLMs (Claude, GPT-4) decomposing high-level goals ("make breakfast") into robot-executable steps.

---

## Additional Resources

- Whisper GitHub: https://github.com/openai/whisper
- Faster-Whisper: https://github.com/guillaumekln/faster-whisper
- Silero VAD: https://github.com/snakers4/silero-vad
- Whisper Paper: https://arxiv.org/abs/2212.04356 (Radford et al., 2022)

---

## Notes for Instructors

**Teaching Tips**:
- Live demo: Speak command → show mel spectrogram → Whisper output (students see audio→text pipeline)
- Compare cloud ASR (Google, 1500ms) vs. Whisper (100ms)—emphasize latency advantage

**Lab Ideas**:
- Lab 1: Transcribe audio samples with Tiny/Base/Small, plot WER vs. latency
- Lab 2: Implement voice-controlled robot (pick/place, navigation)
- Lab 3: Multi-language voice interface (English + student's native language)

**Assessment**:
- Quiz: Explain why mel scale used (Answer: Mimics human hearing, logarithmic frequency perception)
- Project: Voice-controlled robot completing 10 tasks (80% success target)

**Common Student Struggles**:
- Confusing sampling rate (16kHz required for Whisper, not 44.1kHz CD quality)
- Not understanding VAD purpose (continuous transcription wastes compute)
- Forgetting audio normalization (int16 → float32 range [-1, 1])

---

**Chapter Metadata**:
- **Word Count**: 3,100 words
- **Code Examples**: 7
- **Exercises**: 3
- **Glossary Terms**: 8
