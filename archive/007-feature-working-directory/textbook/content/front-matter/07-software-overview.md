# Software Overview

This section outlines the complete software stack used throughout the textbook. Understanding this landscape will help you navigate installations, troubleshoot issues, and appreciate how components work together.

---

## Software Stack Architecture

The textbook uses a **layered software architecture**, where each layer builds on the previous:

```
┌─────────────────────────────────────────────────┐
│  Layer 4: AI & Language Models                  │
│  (LLMs, Whisper, VLA models)                    │
├─────────────────────────────────────────────────┤
│  Layer 3: GPU-Accelerated AI Frameworks         │
│  (NVIDIA Isaac SDK, TensorRT, Isaac Gym)        │
├─────────────────────────────────────────────────┤
│  Layer 2: Simulation Environments               │
│  (Gazebo, Unity, Isaac Sim)                     │
├─────────────────────────────────────────────────┤
│  Layer 1: Robotic Middleware                    │
│  (ROS 2 Humble, DDS)                            │
├─────────────────────────────────────────────────┤
│  Layer 0: Operating System & Drivers            │
│  (Ubuntu 22.04, NVIDIA Drivers, CUDA)           │
└─────────────────────────────────────────────────┘
```

Each module focuses on one layer while integrating with previous layers.

---

## Layer 0: Foundation (Operating System & Drivers)

### Ubuntu 22.04 LTS

**Why Ubuntu 22.04?**
- ROS 2 Humble targets this LTS release
- Long-term support until April 2027
- Matches ROS 2 Humble lifecycle
- Best hardware driver support for robotics

**Installation Options**:
- **Native**: Best performance, full hardware access
- **Dual-Boot**: Preserve existing OS
- **VM (VirtualBox, VMware)**: Convenient but limited GPU access
- **WSL2 (Windows)**: Lightweight but no GPU for Isaac

**Recommendation**: Dual-boot or native for GPU access (Modules 3-4)

### NVIDIA Drivers & CUDA

**Required for**: Modules 3-4 (Isaac, VLA GPU acceleration)

**Versions**:
- **NVIDIA Driver**: 525+ (as of 2025)
- **CUDA**: 11.8 or 12.x (Isaac Sim supports both)
- **cuDNN**: 8.6+ (deep learning acceleration)

**Installation**:
```bash
# Recommended: Use NVIDIA's official installer
sudo apt install nvidia-driver-525 nvidia-utils-525
sudo apt install cuda-toolkit-11-8
```

See **Appendix B: Software Installation Guide** for detailed instructions.

---

## Layer 1: Robotic Middleware (ROS 2)

### ROS 2 Humble Hawksbill

**Version**: Humble (May 2022 release)

**Why ROS 2 Humble?**
- **LTS until May 2027**: Stable, production-ready
- **DDS middleware**: Real-time communication, QoS policies
- **Python & C++ support**: Flexibility for developers
- **Ubuntu 22.04 native**: Perfect OS match
- **Industry adoption**: Used in commercial robots (2023-2025)

**Key Features**:
- Computational graph architecture (nodes, topics, services, actions)
- DDS implementations: Cyclone DDS (default), FastDDS, Connext
- Launch system for multi-node applications
- `colcon` build system for workspace management
- `rclpy` (Python) and `rclcpp` (C++) client libraries

**Installation**:
```bash
sudo apt install ros-humble-desktop-full
source /opt/ros/humble/setup.bash
```

**Modules Using ROS 2**: All modules (1-4)

---

## Layer 2: Simulation Environments

### Gazebo 11 (Classic)

**Version**: Gazebo 11 (final release of Gazebo Classic)

**Why Gazebo?**
- **Open-source**: Free, widely used in ROS community
- **Physics engines**: ODE (default), Bullet, Simbody, DART
- **ROS 2 integration**: Native `ros_gz_bridge` for topics
- **URDF support**: Import robot descriptions from ROS 2
- **Sensor simulation**: Camera, LiDAR, IMU, depth

**Note**: Gazebo Classic EOL January 2025 → Migrating to **new Gazebo** (formerly Ignition)

**Installation**:
```bash
sudo apt install gazebo11 ros-humble-gazebo-ros-pkgs
```

**When Used**: Module 2 (Chapter 2.2)

**Limitations**: CPU-based rendering, not photorealistic

---

### New Gazebo (Fortress/Garden)

**Why the New Gazebo?**
- **Modern architecture**: Improved performance, plugin system
- **ROS 2 native**: Built with ROS 2 in mind
- **Active development**: Long-term support

**Installation**:
```bash
sudo apt install gz-garden
sudo apt install ros-humble-ros-gzgarden
```

**When Used**: Optional (Appendix C mentions migration path)

---

### Unity Robotics Hub

**Version**: Unity 2021.3 LTS or newer

**Why Unity?**
- **Photorealistic rendering**: High-quality visualization
- **Real-time 3D**: Game engine performance
- **ROS 2 integration**: Via ROS-TCP-Connector
- **Cross-platform**: Windows, macOS, Linux

**Components**:
- **ROS-TCP-Connector**: Unity ↔ ROS 2 communication
- **URDF Importer**: Import robot models from ROS 2
- **Perception Package**: Generate synthetic training data

**Installation**:
- Download Unity Hub → Install Unity Editor
- Add Robotics Hub packages via Unity Package Manager

**When Used**: Module 2 (Chapter 2.3)

**Limitation**: TCP-based (not native DDS), higher latency than Gazebo

---

### NVIDIA Isaac Sim

**Version**: Isaac Sim 5.0 (GA at SIGGRAPH 2025)

**Why Isaac Sim?**
- **Photorealistic rendering**: RTX ray tracing
- **Physics accuracy**: PhysX 5.0 engine
- **GPU acceleration**: Entire simulation on GPU
- **ROS 2 native**: Direct ROS 2 node integration
- **Sensor simulation**: RGB-D, LiDAR, IMU with noise models
- **Synthetic data**: Generate training data for VLA

**Components**:
- **Omniverse platform**: USD-based simulation
- **Isaac ROS integration**: Publish sensor data to ROS 2
- **Domain randomization**: Vary textures, lighting for sim-to-real

**Installation**:
- Requires NVIDIA GPU (RTX 3060 minimum, RTX 4070 Ti+ recommended)
- Download from NVIDIA Developer Portal
- ~20GB download + 50GB installed

**When Used**: Modules 3-4 (Chapters 3.1-3.4, 4.4)

**System Requirements**: RTX GPU, 16GB VRAM recommended

---

## Layer 3: GPU-Accelerated AI Frameworks

### NVIDIA Isaac SDK

**Version**: Isaac SDK 2024.1 (as of Q4 2024)

**Components**:

**1. Isaac ROS**
- Pre-built ROS 2 nodes for perception, SLAM, navigation
- TensorRT-optimized DNNs (10-50× faster than PyTorch)
- Hardware acceleration on Jetson and desktop GPUs

**Key Packages**:
- `isaac_ros_dnn_inference`: Object detection, segmentation
- `isaac_ros_visual_slam`: Real-time VSLAM
- `isaac_ros_nvblox`: 3D reconstruction

**Installation**:
```bash
# Install from NVIDIA apt repository
sudo apt install ros-humble-isaac-ros-dnn-inference
```

**2. Isaac Gym**
- GPU-accelerated RL environment
- Train policies with PPO, SAC algorithms
- 1000s of parallel environments on single GPU

**When Used**: Module 3 (Chapters 3.1-3.4)

---

### TensorRT

**Version**: TensorRT 8.6+

**Why TensorRT?**
- **Inference acceleration**: 5-50× faster than PyTorch/TensorFlow
- **INT8 quantization**: Reduce model size and latency
- **NVIDIA GPU optimization**: Leverage Tensor Cores
- **Isaac integration**: Powers Isaac ROS perception

**Workflow**:
1. Train model in PyTorch/TensorFlow
2. Export to ONNX format
3. Convert ONNX → TensorRT engine
4. Deploy in Isaac ROS node

**Installation**: Bundled with CUDA Toolkit

**When Used**: Module 3 (Chapter 3.2)

---

### PyTorch (Optional)

**Version**: PyTorch 2.1+ with CUDA support

**Why PyTorch?**
- VLA model training (if extending textbook examples)
- RL policy training (if not using Isaac Gym)
- Research experimentation

**Installation**:
```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

**When Used**: Optional (for students extending VLA work)

---

## Layer 4: AI & Language Models

### OpenVLA (Open Vision-Language-Action)

**Version**: OpenVLA 7B (7 billion parameters)

**Why OpenVLA?**
- **Open-source**: MIT licensed, fully accessible
- **Pre-trained**: 970k robot demonstrations
- **State-of-the-art**: Competitive with RT-2
- **Extensible**: Fine-tune for custom tasks

**Model Architecture**:
- Base: LLaMA 2 7B language model
- Vision encoder: CLIP ViT-L/14
- Action decoder: Transformer for robot commands

**Installation**:
```bash
git clone https://github.com/openvla/openvla
pip install -e .
```

**When Used**: Module 4 (Chapter 4.1)

**Requirements**: 16GB+ VRAM for inference (8-bit quantization possible)

---

### Large Language Models (LLMs)

**Options**:
- **OpenAI API**: GPT-4, GPT-4o (proprietary, API key required)
- **Anthropic Claude**: Claude 3.5 Sonnet (API key required)
- **Google Gemini**: Gemini 1.5 Pro (API key required)
- **Open-source**: LLaMA 3, Mistral (local deployment, high VRAM)

**Why LLMs for Robotics?**
- Natural language understanding: Parse voice commands
- Task planning: Decompose "Set the table" into steps
- Reasoning: Adapt to unexpected situations

**Cost**:
- API-based: ~$0.01-0.10 per 1000 tokens (labs cost <$5 total)
- Open-source: Free, but requires 40GB+ VRAM (70B models)

**When Used**: Module 4 (Chapter 4.2)

---

### Whisper (Speech-to-Text)

**Version**: Whisper Large V3 (OpenAI, 2023)

**Why Whisper?**
- **Open-source**: MIT licensed
- **Multilingual**: 99 languages supported
- **Robust**: Handles accents, noise, background sounds
- **Fast**: Real-time on GPU (~50ms latency)

**Model Sizes**:
- `tiny`: 39M params, ~1GB RAM (for testing)
- `base`: 74M params, ~1GB RAM (acceptable quality)
- `small`: 244M params, ~2GB RAM (good quality)
- `medium`: 769M params, ~5GB VRAM (better quality)
- `large-v3`: 1550M params, ~10GB VRAM (best quality)

**Installation**:
```bash
pip install openai-whisper
```

**When Used**: Module 4 (Chapter 4.3)

**Recommendation**: Use `small` or `medium` for labs (good balance)

---

## Software Version Matrix

This table shows tested versions (as of 2025):

| Software | Version | Release | Support Until |
|----------|---------|---------|---------------|
| **Ubuntu** | 22.04 LTS | Apr 2022 | Apr 2027 |
| **ROS 2** | Humble | May 2022 | May 2027 |
| **Gazebo** | 11.14 (Classic) | 2023 | Jan 2025 (EOL) |
| **New Gazebo** | Garden | Sep 2022 | Ongoing |
| **Unity** | 2021.3 LTS | 2021 | 2024 (use 2022+ LTS) |
| **Isaac Sim** | 5.0 | Aug 2025 | TBD |
| **Isaac SDK** | 2024.1 | Q4 2024 | Ongoing |
| **CUDA** | 11.8 or 12.x | 2022/2023 | Ongoing |
| **TensorRT** | 8.6 | 2023 | Ongoing |
| **PyTorch** | 2.1+ | Oct 2023 | Ongoing |
| **OpenVLA** | 7B | Q2 2024 | Ongoing |
| **Whisper** | Large V3 | Nov 2023 | Ongoing |

---

## Installation Order

Follow this sequence to avoid dependency issues:

### Week 1: Foundation
1. Install **Ubuntu 22.04** (native or dual-boot)
2. Install **NVIDIA drivers** + **CUDA** (if GPU available)
3. Install **ROS 2 Humble** (desktop-full)
4. Verify: `ros2 run demo_nodes_cpp talker`

### Week 2: Simulation
5. Install **Gazebo 11** (or new Gazebo Garden)
6. Install ROS 2 ↔ Gazebo bridge packages
7. Verify: Launch Gazebo, spawn URDF model

### Week 8: Isaac (Before Module 3)
8. Install **Isaac Sim 5.0** (requires GPU)
9. Install **Isaac ROS** packages
10. Install **TensorRT** (if not bundled with CUDA)
11. Verify: Run Isaac perception demo

### Week 11: VLA (Before Module 4)
12. Install **PyTorch** with CUDA support
13. Install **Whisper**: `pip install openai-whisper`
14. Clone **OpenVLA** repository
15. Obtain **LLM API keys** (OpenAI or Anthropic)
16. Verify: Run Whisper on test audio, call LLM API

**Full instructions**: See **Appendix B: Software Installation Guide**

---

## Troubleshooting Quick Reference

Common issues and solutions:

| Issue | Cause | Solution |
|-------|-------|----------|
| `ros2: command not found` | ROS 2 not sourced | Add `source /opt/ros/humble/setup.bash` to `~/.bashrc` |
| Gazebo crashes on launch | Graphics driver issue | Update NVIDIA drivers, disable anti-aliasing |
| Isaac Sim won't start | CUDA version mismatch | Check CUDA 11.8 or 12.x installed |
| TensorRT import error | Missing cuDNN | `sudo apt install libcudnn8` |
| Whisper OOM (out of memory) | Model too large | Use `small` model instead of `large` |
| LLM API timeout | Network/rate limit | Check API key, retry with backoff |

**Full troubleshooting**: See **Appendix F: Troubleshooting Guide**

---

## Software Decision Matrix

Choose software configurations based on your goals:

| Goal | OS | ROS 2 | Simulation | GPU Stack |
|------|----|----|------------|-----------|
| **Complete textbook (cloud GPU)** | Ubuntu 22.04 | Humble | Gazebo 11 | Cloud Isaac |
| **Complete textbook (local GPU)** | Ubuntu 22.04 | Humble | Gazebo 11 + Isaac Sim | Isaac SDK + TensorRT |
| **Research focus (VLA)** | Ubuntu 22.04 | Humble | Isaac Sim | Isaac SDK + PyTorch + OpenVLA |
| **Minimal setup** | Ubuntu 22.04 | Humble | Gazebo 11 | None (skip Modules 3-4) |

---

## Licensing Summary

| Software | License | Cost | Commercial Use |
|----------|---------|------|----------------|
| Ubuntu | Open-source (GPL) | Free | ✅ Yes |
| ROS 2 | Apache 2.0 | Free | ✅ Yes |
| Gazebo | Apache 2.0 | Free | ✅ Yes |
| Unity | Proprietary | Free (edu), $$ (pro) | ⚠️ Free tier limited |
| Isaac Sim | Proprietary | Free (non-commercial) | ❌ No (license required) |
| Isaac SDK | Proprietary | Free (academic) | ⚠️ License required |
| PyTorch | BSD | Free | ✅ Yes |
| OpenVLA | MIT | Free | ✅ Yes |
| Whisper | MIT | Free | ✅ Yes |
| LLM APIs | Proprietary | Pay-per-use | ✅ Yes (with payment) |

**For students**: All software is free for educational use. LLM APIs cost ~$5 total for textbook labs.

---

## Summary & Next Steps

**Key Takeaways**:
- **Layer 0-1**: Ubuntu 22.04 + ROS 2 Humble (foundation for all modules)
- **Layer 2**: Gazebo, Unity, Isaac Sim (simulation environments)
- **Layer 3**: Isaac SDK, TensorRT (GPU-accelerated AI)
- **Layer 4**: OpenVLA, LLMs, Whisper (embodied intelligence)

**Installation Timeline**:
- **Week 1**: OS + ROS 2
- **Week 2**: Gazebo
- **Week 8**: Isaac SDK (before Module 3)
- **Week 11**: VLA stack (before Module 4)

**Ready to install?** → See **Appendix B: Software Installation Guide** for step-by-step instructions.

**Questions?** → See **Appendix F: Troubleshooting Guide** for common issues.

---

**Next: Module Introductions**

Now that you understand the hardware and software landscape, explore the **Module 1-4 Introductions** to see what you'll build in each phase of the course.
