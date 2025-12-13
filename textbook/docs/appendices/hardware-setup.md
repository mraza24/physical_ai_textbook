---
sidebar_position: 1
title: Appendix A - Hardware Setup Guide
---

# Appendix A: Hardware Setup Guide

Complete hardware specifications and setup instructions for Physical AI development.

---

## Overview

This appendix provides detailed hardware recommendations for each module, purchase links, and setup instructions.

---

## Minimum Requirements (Modules 1-2)

### Development Workstation
- **CPU**: Intel i5-12400 / AMD Ryzen 5 5600 (6 cores, 12 threads)
- **RAM**: 16GB DDR4-3200
- **Storage**: 256GB NVMe SSD (for OS + ROS 2 + Gazebo)
- **GPU**: Integrated graphics (sufficient for Gazebo)
- **OS**: Ubuntu 22.04 LTS (native installation, not VM)

**Estimated Cost**: $600-800

---

## Recommended for Modules 3-4 (GPU-Accelerated AI)

### High-Performance Workstation
- **CPU**: Intel i7-13700K / AMD Ryzen 7 7700X (8+ cores)
- **RAM**: 32GB DDR5-5200
- **Storage**: 1TB NVMe Gen4 SSD
- **GPU**: NVIDIA RTX 4070 Ti (12GB VRAM) or RTX 4080 (16GB VRAM)
- **PSU**: 750W 80+ Gold
- **Cooling**: Adequate for GPU thermal management

**Estimated Cost**: $2,000-2,500

### GPU Comparison

| GPU Model | VRAM | CUDA Cores | TOPS (INT8) | Price | Use Case |
|-----------|------|------------|-------------|-------|----------|
| RTX 4060 Ti | 8GB | 4,352 | ~22 | $400 | Entry-level (Module 3 only) |
| RTX 4070 | 12GB | 5,888 | ~29 | $600 | Good for learning |
| RTX 4070 Ti | 12GB | 7,680 | ~40 | $800 | **Recommended** |
| RTX 4080 | 16GB | 9,728 | ~49 | $1,200 | High-end development |
| RTX 4090 | 24GB | 16,384 | ~82 | $1,600 | Professional / research |

**Recommendation**: RTX 4070 Ti offers best price/performance for this course.

---

## Edge Deployment Hardware (Optional)

### NVIDIA Jetson Orin Series

**Jetson Orin Nano** (Entry-level)
- **Performance**: 40 TOPS
- **RAM**: 8GB
- **Power**: 7-15W
- **Price**: ~$499
- **Use Case**: Lightweight perception, testing

**Jetson Orin NX** (Mid-range)
- **Performance**: 100 TOPS
- **RAM**: 16GB
- **Power**: 10-25W
- **Price**: ~$899
- **Use Case**: Autonomous navigation, moderate perception

**Jetson Orin AGX** (High-end)
- **Performance**: 275 TOPS
- **RAM**: 64GB
- **Power**: 15-60W
- **Price**: ~$1,999
- **Use Case**: Complex VLA systems, production deployment

---

## Sensors (Optional for Hardware Testing)

### Intel RealSense Depth Cameras

**RealSense D435** (General-purpose)
- **Depth Range**: 0.3m - 3m
- **RGB**: 1920x1080 @ 30 FPS
- **Depth**: 1280x720 @ 30 FPS
- **Price**: ~$200
- **Use Case**: Indoor VSLAM, object detection

**RealSense D455** (Longer range)
- **Depth Range**: 0.4m - 6m
- **RGB**: 1920x1080 @ 30 FPS
- **Depth**: 1280x720 @ 30 FPS
- **Price**: ~$350
- **Use Case**: Outdoor navigation, larger spaces

### LiDAR (Advanced)

**SLAMTEC RPLIDAR A1** (2D)
- **Range**: 12m
- **Scan Rate**: 5.5 Hz
- **Price**: ~$99
- **Use Case**: 2D mapping, obstacle avoidance

**Ouster OS0** (3D, professional)
- **Range**: 50m
- **Resolution**: 64/128 beams
- **Price**: ~$5,000+
- **Use Case**: Autonomous vehicles, outdoor mapping

---

## Robot Platforms (Optional)

### Mobile Robots

**TurtleBot 4** (Recommended for learning)
- **Base**: iRobot Create 3
- **Sensors**: RealSense D435, IMU, RPLIDAR
- **Price**: ~$1,500
- **Use Case**: ROS 2 development, navigation

**NVIDIA Carter** (Advanced)
- **Base**: Segway
- **Sensors**: Stereo cameras, LiDAR
- **Compute**: Jetson AGX
- **Price**: ~$10,000+
- **Use Case**: Isaac ROS development

### Humanoid Robots

**Unitree G1** (Research-grade)
- **DOF**: 23 joints
- **Height**: 1.3m
- **Weight**: 35 kg
- **Price**: ~$16,000
- **Use Case**: Humanoid manipulation research

**ROBOTIS OP3** (Educational)
- **DOF**: 20 joints
- **Height**: 0.5m
- **Price**: ~$10,000
- **Use Case**: Education, competition

---

## Setup Instructions

### Desktop Workstation Setup

1. **Install Ubuntu 22.04 LTS**
   - Download: https://ubuntu.com/download/desktop
   - Create bootable USB with Rufus (Windows) or Etcher
   - **Important**: Use native installation, not VM

2. **Install NVIDIA Drivers**
   ```bash
   sudo ubuntu-drivers autoinstall
   sudo reboot
   ```

3. **Verify GPU**
   ```bash
   nvidia-smi
   ```
   Should show GPU model and driver version.

4. **Install CUDA 12.x**
   ```bash
   wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
   sudo dpkg -i cuda-keyring_1.1-1_all.deb
   sudo apt update
   sudo apt install cuda-toolkit-12-4
   ```

5. **Install cuDNN**
   - Download from NVIDIA: https://developer.nvidia.com/cudnn
   - Requires NVIDIA Developer account (free)

6. **Continue to Appendix B for ROS 2 installation**

---

## Jetson Setup

### Flashing Jetson Orin

1. Download JetPack 6.x from NVIDIA SDK Manager
2. Connect Jetson via USB in recovery mode
3. Flash with SDK Manager (Ubuntu host required)
4. Follow on-screen instructions

**Detailed guide**: https://developer.nvidia.com/embedded/jetpack

---

## Purchasing Recommendations

### Budget Tiers

**Tier 1: Minimum ($800-1,000)**
- Desktop CPU + 16GB RAM + 256GB SSD
- Integrated GPU (Modules 1-2 only)
- No sensors or robots

**Tier 2: Recommended ($2,500-3,000)**
- High-performance desktop with RTX 4070 Ti
- Complete Modules 1-4 with GPU acceleration
- Optional: RealSense D435 for testing

**Tier 3: Professional ($5,000-10,000)**
- Workstation with RTX 4080/4090
- Jetson Orin AGX for edge deployment
- TurtleBot 4 for physical testing
- Multiple sensors (RealSense, LiDAR)

---

## Cloud Alternatives

If you don't have local GPU:
- **AWS EC2 G5 instances**: RTX A10G (24GB VRAM), ~$1-2/hour
- **Google Cloud GPU VMs**: A100 (40GB VRAM), ~$2-4/hour
- **Paperspace**: RTX 4000/5000 series, ~$0.50-1.50/hour

**Cost estimate**: $100-200 for entire course if using cloud GPUs.

---

## Troubleshooting Hardware

### GPU Not Detected
1. Check NVIDIA drivers: `nvidia-smi`
2. Verify CUDA: `nvcc --version`
3. Reinstall drivers if needed

### Insufficient VRAM
- Reduce batch size in DNN inference
- Use INT8 quantization
- Consider cloud GPU upgrade

### Overheating
- Check thermal paste on CPU/GPU
- Improve case airflow
- Consider aftermarket cooling

---

**See Also**:
- Appendix B: Software Installation Guide
- Appendix E: Cloud vs. On-Premise GPU Deployment
