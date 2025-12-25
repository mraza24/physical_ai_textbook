# Appendix C: Software Stack Reference

> **Quick reference for the complete software stack, layer-by-layer architecture, and version compatibility matrix.**

## Software Architecture (5 Layers)

### Layer 0: Operating System & Drivers
- **Ubuntu 22.04 LTS** (Jammy Jellyfish) - Host OS
- **NVIDIA Driver 535+** - GPU support
- **CUDA 12.2+** - Parallel computing
- **cuDNN 8.9+** - Deep learning primitives

### Layer 1: ROS 2 Middleware
- **ROS 2 Humble Hawksbill** (LTS until 2027-05)
- **DDS**: CycloneDDS (default), FastDDS, Connext
- **rmw_cyclonedds_cpp** - Recommended middleware implementation

### Layer 2: Simulation & Digital Twins
- **Gazebo Harmonic** (gz-sim 8.x) - Modern simulator
- **Isaac Sim 4.5+** - GPU-accelerated, RTX ray tracing
- **Unity 2022.3 LTS** + Unity Robotics Hub 0.7+

### Layer 3: Perception & AI
- **Isaac ROS 3.0+** - GPU perception (Nvblox, VSLAM, DNN inference)
- **TensorRT 8.6+** - Inference optimization
- **PyTorch 2.1+** with CUDA 12.1 - Model training
- **OpenVLA 7B** - Vision-language-action model

### Layer 4: Navigation & Manipulation
- **Nav2 (Humble)** - Mobile robot navigation
- **MoveIt2 (Humble)** - Manipulator motion planning
- **Isaac Lab 1.0+** - Reinforcement learning

### Layer 5: Voice & Language
- **Whisper (base/small/medium)** - Speech recognition
- **Claude 3.5 Sonnet API** - Task planning LLM
- **GPT-4 Turbo API** - Alternative LLM

---

## Version Compatibility Matrix

| Component | Minimum | Recommended | Latest Tested | Notes |
|-----------|---------|-------------|---------------|-------|
| **Ubuntu** | 22.04.0 | 22.04.3 | 22.04.4 | LTS only |
| **ROS 2** | Humble 0.25.0 | Humble 0.25.5 | Humble 0.26.1 | No Iron/Jazzy yet |
| **Python** | 3.10.0 | 3.10.12 | 3.10.14 | 3.11+ breaks some pkgs |
| **NVIDIA Driver** | 535.54 | 535.129 | 545.29 | ≥535 for CUDA 12.2 |
| **CUDA** | 12.0 | 12.2 | 12.3 | Match driver version |
| **Gazebo** | gz-harmonic (8.0) | gz-harmonic (8.5) | gz-harmonic (8.6) | Classic EOL Jan 2025 |
| **Isaac Sim** | 4.0.0 | 4.5.0 | 4.5.1 | Requires RTX GPU |
| **Unity** | 2021.3 LTS | 2022.3 LTS | 2023.2 LTS | Stick to LTS |
| **TensorRT** | 8.5.0 | 8.6.1 | 8.6.3 | Bundled with JetPack |
| **PyTorch** | 2.0.0 | 2.1.2 | 2.2.1 | cu121 index for CUDA 12.1 |
| **Whisper** | N/A | base.en / small | large-v3 | Larger = slower but accurate |

---

## Package Dependencies (ROS 2 Humble)

**Core Development**:
```bash
sudo apt install -y \
  ros-humble-desktop-full \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep
```

**Simulation**:
```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros-gz \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2
```

**Navigation**:
```bash
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-turtlebot4-navigation
```

**Manipulation**:
```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-moveit-visual-tools \
  ros-humble-warehouse-ros-mongo
```

**Isaac ROS** (from source, see Appendix A):
```bash
# Requires building from GitHub repositories
# isaac_ros_common
# isaac_ros_dnn_inference
# isaac_ros_visual_slam
# isaac_ros_nvblox
```

---

## Python Environment

**System Python (Ubuntu 22.04 default)**:
```bash
python3 --version  # Should show Python 3.10.12

# Essential packages
pip install \
  numpy \
  matplotlib \
  opencv-python \
  pandas \
  scipy \
  scikit-learn
```

**PyTorch with CUDA**:
```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
```

**Robotics-Specific**:
```bash
pip install \
  open3d \
  transforms3d \
  roboticstoolbox-python \
  spatialmath-python
```

**VLA & LLM**:
```bash
pip install \
  transformers \
  accelerate \
  datasets \
  openai-whisper \
  faster-whisper \
  anthropic \
  openai
```

---

## Docker Images (Optional Alternative)

**Official ROS 2 Humble**:
```bash
docker pull osrf/ros:humble-desktop-full
docker run -it --rm osrf/ros:humble-desktop-full
```

**NVIDIA Isaac ROS**:
```bash
docker pull nvcr.io/nvidia/isaac-ros-dev-aarch64:release_3.0-humble
docker run -it --gpus all nvcr.io/nvidia/isaac-ros-dev-aarch64:release_3.0-humble
```

**Advantages**: Isolated environment, reproducible, no system pollution
**Disadvantages**: GPU pass through complexity, file system mapping, debugging harder

---

## Common Version Conflicts & Solutions

### Issue 1: ROS 2 + Isaac Sim Python mismatch
**Problem**: Isaac Sim uses its own Python 3.10.13, ROS 2 uses system Python 3.10.12
**Solution**: Use Isaac Sim's Python for Isaac code, system Python for ROS 2 code. Bridge via ROS 2 topics, not direct imports.

### Issue 2: PyTorch CUDA version mismatch
**Problem**: `pip install torch` installs CPU-only version by default
**Solution**: Always use index-url: `--index-url https://download.pytorch.org/whl/cu121`

### Issue 3: Gazebo Classic vs New naming
**Problem**: `gazebo` command launches Classic (deprecated), not Harmonic
**Solution**: Use `gz sim` for new Gazebo, `gazebo11` for Classic

### Issue 4: Unity Robotics Hub ROS 2 branch
**Problem**: Default branch is main (ROS 1), not main-ros2
**Solution**: Always specify branch: `git clone <url> -b main-ros2`

---

## Development Tools

**Code Editors**:
- **VS Code** + ROS extension (recommended)
- **CLion** + ROS plugin (JetBrains, paid)
- **Vim/Neovim** + coc-ros (advanced users)

**Debugging**:
- **GDB** - C++ debugging
- **rqt_console** - ROS 2 log viewer
- **rqt_graph** - Node/topic visualization
- **rviz2** - 3D visualization
- **Foxglove Studio** - Modern ROS 2 UI

**Profiling**:
- **ros2 run performance_test** - DDS throughput/latency
- **nvidia-smi** - GPU monitoring
- **nvtop** - Interactive GPU monitor
- **Nsight Systems** - CUDA profiling

**Version Control**:
- **Git** + **GitHub/GitLab**
- **vcs** (vcstool) - Multi-repo management for ROS 2 workspaces

---

**Software stack complete. See Appendix A for installation steps, Appendix D for troubleshooting.**
