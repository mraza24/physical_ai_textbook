---
sidebar_position: 2
title: Appendix B - Software Installation Guide
---

# Appendix B: Software Installation Guide

Complete step-by-step installation instructions for all software dependencies.

---

## Prerequisites

- Ubuntu 22.04 LTS installed (native, not VM)
- Internet connection
- Administrator (sudo) access
- 50GB+ free disk space

---

## Part 1: ROS 2 Humble Installation

### Step 1: Set Locale
```bash
locale  # Check UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 2: Add ROS 2 Repository
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2 Humble
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

### Step 4: Install Development Tools
```bash
sudo apt install ros-dev-tools
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
```

### Step 5: Initialize rosdep
```bash
sudo rosdep init
rosdep update
```

### Step 6: Source Setup Script
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 7: Verify Installation
```bash
ros2 --help
ros2 run demo_nodes_cpp talker
```

**Expected**: Talker node publishes "Hello World" messages.

---

## Part 2: Gazebo Harmonic Installation

### Option A: Binary Installation (Recommended)
```bash
sudo apt-get update
sudo apt-get install lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install gz-harmonic
```

### Option B: Source Build (Advanced)
See: https://gazebosim.org/docs/harmonic/install_ubuntu_src

### Install ROS 2 - Gazebo Bridge
```bash
sudo apt install ros-humble-ros-gz
```

### Verify Installation
```bash
gz sim -v4 shapes.sdf
```

**Expected**: Gazebo GUI opens with shapes world.

---

## Part 3: NVIDIA Isaac Installation

### Prerequisites
- NVIDIA GPU with driver 525.60.13+
- CUDA 12.x installed (see Appendix A)

### Install Isaac ROS
```bash
# Install Docker
sudo apt-get update
sudo apt-get install docker.io
sudo usermod -aG docker $USER
newgrp docker

# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# Clone Isaac ROS Common
cd ~/workspaces
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
cd isaac_ros_common
./scripts/run_dev.sh
```

### Install Isaac Sim (Optional)
1. Download from: https://developer.nvidia.com/isaac-sim
2. Extract to `~/isaac-sim`
3. Run: `./isaac-sim.sh`

**Requires**: NVIDIA GPU with 12GB+ VRAM

---

## Part 4: Unity Installation

### Install Unity Hub
```bash
sudo sh -c 'echo "deb https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unityhub.list'
wget -qO - https://hub.unity3d.com/linux/keys/public | sudo apt-key add -
sudo apt update
sudo apt-get install unityhub
```

### Install Unity Editor
1. Open Unity Hub
2. Install Unity 2022.3 LTS
3. Add Linux Build Support module

### Install Unity Robotics Hub
1. Create new Unity project (3D template)
2. Window → Package Manager
3. Add package from git URL:
   - `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
   - `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`

---

## Part 5: Python Dependencies

### Install Python 3.10+
```bash
python3 --version  # Should be 3.10+
sudo apt install python3-pip
```

### Install Key Libraries
```bash
pip3 install --upgrade pip

# Deep Learning
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# Computer Vision
pip3 install opencv-python opencv-contrib-python

# Isaac / TensorRT
pip3 install tensorrt

# Robotics Utilities
pip3 install numpy scipy matplotlib
pip3 install transforms3d

# VLA / LLM
pip3 install openai anthropic
pip3 install openai-whisper

# Development
pip3 install jupyter ipython
```

---

## Part 6: Additional Tools

### Install Git
```bash
sudo apt install git
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### Install VS Code
```bash
sudo snap install code --classic
```

**Extensions to install**:
- ROS (Microsoft)
- Python (Microsoft)
- C/C++ (Microsoft)
- CMake Tools

### Install Terminator (Terminal Multiplexer)
```bash
sudo apt install terminator
```

---

## Verification Checklist

Run these commands to verify installations:

```bash
# ROS 2
ros2 --version
# Expected: ros2 cli version 0.25.x

# Gazebo
gz sim --version
# Expected: Gazebo Harmonic

# CUDA
nvcc --version
nvidia-smi
# Expected: CUDA 12.x, GPU listed

# Python
python3 --version
pip3 list | grep torch
pip3 list | grep opencv
# Expected: torch, opencv-python

# Git
git --version
```

---

## Troubleshooting Common Issues

### ROS 2: "Package not found"
```bash
source /opt/ros/humble/setup.bash
```

### Gazebo: "gz command not found"
```bash
export PATH=$PATH:/usr/local/bin
source ~/.bashrc
```

### CUDA: "CUDA driver version is insufficient"
- Update NVIDIA drivers
- Reboot

### Docker Permission Denied
```bash
sudo usermod -aG docker $USER
newgrp docker
```

### Python Import Errors
```bash
pip3 install --upgrade <package-name>
```

---

## Workspace Setup

### Create ROS 2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Clone Textbook Examples
```bash
cd ~/ros2_ws/src
git clone https://github.com/physical-ai-textbook/examples.git
cd ~/ros2_ws
colcon build
```

---

**Installation Complete!** Proceed to Chapter 1.1 to start learning.

**See Also**:
- Appendix A: Hardware Setup Guide
- Appendix F: Troubleshooting Guide
