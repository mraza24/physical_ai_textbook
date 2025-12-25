# Appendix A: Installation & Setup Guide

> **Comprehensive installation instructions for all software and tools used in this textbook.**

This appendix provides step-by-step installation guides for the complete Physical AI development environment, from basic ROS 2 setup through GPU-accelerated simulation and VLA deployment.

**Estimated Setup Time**: 4-8 hours for full stack (depends on download speeds and hardware)

**System Requirements**:
- **Workstation**: Ubuntu 22.04 LTS, 32GB RAM, RTX 4070 Ti or better (12GB+ VRAM), 500GB+ SSD
- **Edge Device** (optional): Jetson Orin Nano 8GB or AGX Orin 32GB/64GB
- **Internet**: Stable connection (downloads: ~50GB total)

---

## Table of Contents

1. [System Preparation](#1-system-preparation)
2. [ROS 2 Humble Installation](#2-ros-2-humble-installation)
3. [Gazebo Installation](#3-gazebo-installation)
4. [Unity Robotics Hub Setup](#4-unity-robotics-hub-setup)
5. [NVIDIA Isaac Sim Installation](#5-nvidia-isaac-sim-installation)
6. [Isaac ROS Installation](#6-isaac-ros-installation)
7. [Jetson Orin Setup](#7-jetson-orin-setup)
8. [Python Environment Setup](#8-python-environment-setup)
9. [Voice & LLM API Setup](#9-voice--llm-api-setup)
10. [Verification & Testing](#10-verification--testing)

---

## 1. System Preparation

### 1.1 Ubuntu 22.04 LTS Installation

**If starting from scratch**:

```bash
# Download Ubuntu 22.04 LTS Desktop
# https://ubuntu.com/download/desktop

# Create bootable USB (on Linux)
sudo dd if=ubuntu-22.04.3-desktop-amd64.iso of=/dev/sdX bs=4M status=progress

# Install Ubuntu with:
# - Minimum 100GB partition for /
# - Separate /home partition recommended
# - Enable "Install third-party software" for GPU drivers
```

### 1.2 NVIDIA Driver Installation

**For RTX 4070 Ti / 4080 / 4090**:

```bash
# Add NVIDIA driver PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install recommended driver (check compatibility: nvidia-smi should show CUDA 12.x)
sudo ubuntu-drivers autoinstall

# Reboot and verify
sudo reboot
nvidia-smi  # Should show driver version 535+ and CUDA 12.2+
```

**Expected output**:
```
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 535.129.03             Driver Version: 535.129.03   CUDA Version: 12.2      |
|-----------------------------------------+------------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id        Disp.A   | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |         Memory-Usage   | GPU-Util  Compute M. |
|=========================================+========================+======================|
|   0  NVIDIA GeForce RTX 4070 Ti     Off | 00000000:01:00.0  On   |                  N/A |
| 30%   45C    P8              15W / 285W |    800MiB / 12282MiB   |      2%      Default |
+-----------------------------------------+------------------------+----------------------+
```

### 1.3 Essential Build Tools

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-pip \
    python3-dev \
    software-properties-common \
    ca-certificates \
    gnupg \
    lsb-release

# Install development libraries
sudo apt install -y \
    libeigen3-dev \
    libboost-all-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    libgtest-dev
```

---

## 2. ROS 2 Humble Installation

### 2.1 Install ROS 2 Humble Desktop

**Official Debian packages (recommended)**:

```bash
# Set locale
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install
sudo apt update
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y \
    ros-dev-tools \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    python3-colcon-common-extensions
```

**Installation size**: ~2.5GB

### 2.2 Environment Setup

```bash
# Source ROS 2 (add to ~/.bashrc for persistence)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version  # Should show: ros2 cli version: 0.25.5

# Test with demo nodes (in separate terminals)
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

### 2.3 Create Workspace

```bash
# Create workspace structure
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build empty workspace
colcon build

# Source workspace overlay (add to ~/.bashrc after ROS 2 source)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source install/setup.bash
```

---

## 3. Gazebo Installation

### 3.1 Gazebo (New) Installation

**Gazebo Harmonic (latest stable, recommended)**:

```bash
# Add Gazebo GPG key
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Add Gazebo repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic
sudo apt update
sudo apt install -y gz-harmonic

# Verify installation
gz sim --version  # Should show: Gazebo Sim, version 8.x.x
```

**Installation size**: ~1.2GB

### 3.2 ROS 2 - Gazebo Bridge

```bash
# Install ros_gz bridge for Humble-Harmonic integration
sudo apt install -y ros-humble-ros-gz

# Test integration
gz sim shapes.sdf &  # Launch Gazebo world
ros2 topic list      # Should show Gazebo topics if bridge active
```

### 3.3 Gazebo Classic (Legacy, for compatibility)

**Only install if working with old projects**:

```bash
sudo apt install -y gazebo11 libgazebo11-dev
```

---

## 4. Unity Robotics Hub Setup

### 4.1 Unity Hub Installation

```bash
# Download Unity Hub (2024.1.2+)
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHubSetup.AppImage

# Make executable and run
chmod +x UnityHubSetup.AppImage
./UnityHubSetup.AppImage
```

### 4.2 Unity Editor Installation

**Via Unity Hub**:
1. Open Unity Hub
2. Navigate to **Installs** → **Install Editor**
3. Select **Unity 2022.3 LTS** (Long Term Support)
4. Add modules:
   - Linux Build Support
   - Documentation
   - Language packs (optional)

**Installation size**: ~8GB

### 4.3 Unity Robotics Hub

**In Unity Editor (create new project first)**:

```
1. Window → Package Manager
2. Click "+" → Add package from git URL
3. Enter: https://github.com/Unity-Technologies/Unity-Robotics-Hub.git?path=/com.unity.robotics.ros-tcp-connector
4. Import sample scenes from package
```

### 4.4 ROS-TCP-Endpoint (ROS 2 side)

```bash
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git -b main-ros2
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash

# Launch endpoint server (default port 10000)
ros2 run ros_tcp_endpoint default_server_endpoint
```

---

## 5. NVIDIA Isaac Sim Installation

### 5.1 Prerequisites

**NVIDIA Omniverse Launcher**:

```bash
# Download from NVIDIA website (requires free account)
# https://www.nvidia.com/en-us/omniverse/download/

wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable and run
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

### 5.2 Isaac Sim Installation

**Via Omniverse Launcher**:
1. Open Omniverse Launcher
2. Navigate to **Exchange** → Search "Isaac Sim"
3. Install **Isaac Sim 4.5.0** (or latest)
4. Wait for download (~20GB) and installation

**Installation size**: ~25GB

**First launch** (verify GPU compatibility):
```bash
# Launch from Omniverse Launcher or via command line
~/.local/share/ov/pkg/isaac-sim-4.5.0/isaac-sim.sh

# Should open Isaac Sim GUI with sample scene
```

### 5.3 Isaac Sim Python Environment

```bash
# Activate Isaac Sim Python environment
cd ~/.local/share/ov/pkg/isaac-sim-4.5.0
source setup_conda_env.sh

# Verify Isaac Python
python -c "import isaacsim; print('Isaac Sim Python OK')"

# Install additional packages in Isaac Python
./python.sh -m pip install pandas matplotlib seaborn
```

---

## 6. Isaac ROS Installation

### 6.1 Install Isaac ROS Common

```bash
cd ~/ros2_ws/src

# Clone Isaac ROS meta-repository
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Install dependencies
cd isaac_ros_common/scripts
./install_dependencies.sh

# Build
cd ~/ros2_ws
colcon build --packages-select isaac_ros_common
source install/setup.bash
```

### 6.2 Install Isaac ROS Perception Packages

```bash
cd ~/ros2_ws/src

# DNN Inference (TensorRT)
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git

# Visual SLAM
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# Nvblox (3D reconstruction)
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git

# Install rosdep dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build all Isaac ROS packages
colcon build --symlink-install
source install/setup.bash
```

**Note**: Isaac ROS requires CUDA-enabled build. Ensure `nvcc` is in PATH:
```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

---

## 7. Jetson Orin Setup

### 7.1 Flash JetPack 6.0

**Prerequisites**:
- Jetson Orin Nano/AGX Developer Kit
- Ubuntu 20.04/22.04 host PC
- USB-C cable

**Steps**:

```bash
# Install NVIDIA SDK Manager on host PC
wget https://developer.nvidia.com/downloads/sdkmanager_2.1.0-11682_amd64.deb
sudo dpkg -i sdkmanager_2.1.0-11682_amd64.deb

# Launch SDK Manager
sdkmanager

# Follow GUI:
# 1. Select Jetson Orin (detected via USB)
# 2. Select JetPack 6.0 (includes Ubuntu 22.04, CUDA 12.2, TensorRT 8.6)
# 3. Flash and install (30-60 minutes)
```

### 7.2 ROS 2 on Jetson

**On Jetson (after JetPack installation)**:

```bash
# Install ROS 2 Humble (same commands as Section 2.1)
sudo apt install -y ros-humble-desktop-full

# Install Isaac ROS packages (optimized for Jetson)
# Follow Section 6 instructions
```

### 7.3 Power Mode Configuration

```bash
# Check available power modes
sudo nvpmodel -q

# Set to maximum performance (AGX Orin: 50W, Nano: 15W)
sudo nvpmodel -m 0

# Enable jetson_clocks for maximum frequency
sudo jetson_clocks
```

---

## 8. Python Environment Setup

### 8.1 Install Miniconda (optional, for isolated environments)

```bash
# Download Miniconda
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh

# Install
bash Miniconda3-latest-Linux-x86_64.sh

# Create environment for robotics
conda create -n robotics python=3.10
conda activate robotics

# Install common packages
pip install numpy pandas matplotlib opencv-python torch torchvision
```

### 8.2 PyTorch with CUDA Support

```bash
# Install PyTorch 2.1 with CUDA 12.1
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# Verify GPU access
python -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
python -c "import torch; print(f'GPU: {torch.cuda.get_device_name(0)}')"
```

**Expected output**:
```
CUDA available: True
GPU: NVIDIA GeForce RTX 4070 Ti
```

### 8.3 OpenVLA Dependencies

```bash
# Install Hugging Face Transformers
pip install transformers accelerate datasets

# Install OpenVLA (from source)
git clone https://github.com/openvla/openvla.git
cd openvla
pip install -e .

# Download pre-trained weights (~14GB)
python scripts/download_model.py --model openvla-7b
```

---

## 9. Voice & LLM API Setup

### 9.1 Whisper Installation

```bash
# Install OpenAI Whisper
pip install openai-whisper

# Download models (auto-downloaded on first use, or manually)
whisper --model base.en --help  # Downloads base.en model

# Install Faster-Whisper (optimized)
pip install faster-whisper

# Test transcription
whisper audio.wav --model base --language English
```

### 9.2 LLM API Keys

**Claude API (Anthropic)**:
```bash
# Sign up: https://console.anthropic.com/
# Create API key → Copy to environment variable

echo 'export ANTHROPIC_API_KEY="sk-ant-..."' >> ~/.bashrc
source ~/.bashrc
```

**GPT-4 API (OpenAI)**:
```bash
# Sign up: https://platform.openai.com/
# Create API key → Copy to environment variable

echo 'export OPENAI_API_KEY="sk-..."' >> ~/.bashrc
source ~/.bashrc
```

**Test API access**:
```python
import anthropic

client = anthropic.Anthropic(api_key=os.environ["ANTHROPIC_API_KEY"])
message = client.messages.create(
    model="claude-3-5-sonnet-20241022",
    max_tokens=100,
    messages=[{"role": "user", "content": "Hello, Claude!"}]
)
print(message.content[0].text)
```

### 9.3 Silero VAD

```bash
# Install dependencies
pip install torch torchaudio

# Download Silero VAD model (auto-downloaded on first use)
python -c "import torch; torch.hub.load('snakers4/silero-vad', 'silero_vad', force_reload=False)"
```

---

## 10. Verification & Testing

### 10.1 ROS 2 System Check

```bash
# Check ROS 2 version
ros2 doctor --report

# List available packages
ros2 pkg list | grep -E "(gazebo|isaac|demo)"

# Run turtlesim test
ros2 run turtlesim turtlesim_node &
ros2 run turtlesim turtle_teleop_key
```

### 10.2 Gazebo + ROS 2 Integration Test

```bash
# Launch Gazebo with ROS 2 bridge
ros2 launch gazebo_ros gazebo.launch.py

# In another terminal, spawn a simple model
ros2 run gazebo_ros spawn_entity.py -entity box -database box

# List Gazebo topics
ros2 topic list | grep gazebo
```

### 10.3 Isaac Sim + ROS 2 Bridge Test

```bash
# Launch Isaac Sim with ROS 2 bridge
cd ~/.local/share/ov/pkg/isaac-sim-4.5.0
./isaac-sim.sh --ros2

# In Isaac Sim GUI: Load ROS2 sample scene (Carter robot)
# File → Open → omniverse://localhost/NVIDIA/Samples/Isaac/4.5/ROS2/carter_warehouse_navigation.usd

# In terminal, check ROS 2 topics
ros2 topic list | grep carter
ros2 topic echo /cmd_vel
```

### 10.4 GPU Performance Check

```bash
# Monitor GPU during Isaac Sim operation
watch -n 1 nvidia-smi

# Expected: 4-8GB VRAM usage, 30-60% GPU util for complex scenes
```

### 10.5 End-to-End VLA Test

```bash
# Activate robotics environment
conda activate robotics

# Run minimal VLA inference test (from textbook code examples)
cd ~/ros2_ws/src/textbook_examples/module4
python test_openvla_inference.py --model openvla-7b --prompt "pick red cube"

# Expected output:
# Loaded OpenVLA-7B (7.2B params)
# Input: RGB image (224x224), text "pick red cube"
# Output: Action tokens [12, 45, 78, ...] → Joint angles [0.1, -0.2, 0.5, ...]
# Inference time: 75ms (RTX 4070 Ti)
```

---

## Common Installation Issues

### Issue 1: ROS 2 packages not found after installation

**Solution**:
```bash
# Re-source setup files
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Rebuild with correct sourcing
cd ~/ros2_ws
colcon build --symlink-install
```

### Issue 2: NVIDIA driver conflicts (black screen after driver install)

**Solution**:
```bash
# Boot to recovery mode (hold Shift at boot)
# Drop to root shell

# Purge all NVIDIA packages
apt purge nvidia-* libnvidia-*

# Reinstall specific driver version
ubuntu-drivers install nvidia-driver-535

# Update initramfs
update-initramfs -u

# Reboot
reboot
```

### Issue 3: Isaac Sim crashes on launch (CUDA/Vulkan errors)

**Solution**:
```bash
# Check Vulkan support
vulkaninfo | grep deviceName

# Install Vulkan drivers
sudo apt install mesa-vulkan-drivers vulkan-tools

# Check CUDA toolkit version (must match driver)
nvcc --version  # Should show CUDA 12.2+

# If version mismatch, install CUDA toolkit:
wget https://developer.download.nvidia.com/compute/cuda/12.2.0/local_installers/cuda_12.2.0_535.54.03_linux.run
sudo sh cuda_12.2.0_535.54.03_linux.run
```

### Issue 4: colcon build fails with "package not found"

**Solution**:
```bash
# Update rosdep database
rosdep update

# Install all dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Clean build artifacts and rebuild
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Issue 5: Jetson runs out of memory during build

**Solution**:
```bash
# Increase swap space on Jetson
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make swap permanent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Build with reduced parallelism
colcon build --symlink-install --parallel-workers 2
```

---

## Next Steps

After completing installation:

1. **Chapter 1.1**: Start with ROS 2 fundamentals - create first node
2. **Chapter 2.2**: Test Gazebo simulation - spawn robot, control via ROS 2
3. **Chapter 3.1**: Launch Isaac Sim - run GPU-accelerated perception demo
4. **Chapter 4.1**: Run OpenVLA inference - test vision-language-action pipeline

**Additional resources**:
- ROS 2 Humble documentation: https://docs.ros.org/en/humble/
- Gazebo tutorials: https://gazebosim.org/docs
- Isaac Sim manual: https://docs.omniverse.nvidia.com/isaacsim/latest/
- Textbook code repository: [GitHub URL to be added]

---

**Installation complete! Ready to build physical AI systems.**
