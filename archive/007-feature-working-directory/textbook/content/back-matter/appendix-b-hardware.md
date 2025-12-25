# Appendix B: Hardware Reference Guide

> **Comprehensive hardware specifications, three-tier architecture, and component selection guide for physical AI development.**

This appendix provides detailed hardware requirements, specifications, and recommendations for building physical AI systems following the textbook's modular approach.

---

## Table of Contents

1. [Three-Tier Hardware Architecture](#1-three-tier-hardware-architecture)
2. [Workstation GPUs (Development)](#2-workstation-gpus-development)
3. [Embedded Platforms (Deployment)](#3-embedded-platforms-deployment)
4. [Sensors & Perception Hardware](#4-sensors--perception-hardware)
5. [Robot Platforms](#5-robot-platforms)
6. [Networking & Connectivity](#6-networking--connectivity)
7. [Hardware Selection Matrix](#7-hardware-selection-matrix)
8. [Power & Thermal Management](#8-power--thermal-management)

---

## 1. Three-Tier Hardware Architecture

The textbook follows a **three-tier approach** balancing performance, cost, and accessibility:

### Tier 1: Professional Development Workstation
**Target Users**: Research labs, industry practitioners, graduate courses with funding

**Configuration**:
- **GPU**: NVIDIA RTX 4090 (24GB VRAM) or A6000 (48GB VRAM)
- **CPU**: AMD Ryzen 9 7950X or Intel i9-14900K (16+ cores)
- **RAM**: 64-128GB DDR5 (6000MHz)
- **Storage**: 2TB NVMe SSD (Gen 4, 7000MB/s read)
- **Budget**: $4,000 - $8,000

**Capabilities**:
- Full Isaac Sim 4.5 with RTX ray tracing (60 FPS at 1080p)
- 4096-8192 parallel RL environments (Isaac Gym)
- Multiple robots in Gazebo (10+ robots, real-time physics)
- OpenVLA 7B inference (35ms latency with TensorRT)
- Simultaneous Unity Editor + ROS 2 + LLM API calls

**Use Cases**: Chapters 3.1-3.4 (Isaac ecosystem), Chapter 4.4 (VLA training)

---

### Tier 2: Student/Hobbyist Workstation
**Target Users**: Undergraduate courses, self-learners, hackathons (this textbook's primary target)

**Configuration**:
- **GPU**: NVIDIA RTX 4070 Ti (12GB VRAM) or RTX 4060 Ti (16GB)
- **CPU**: AMD Ryzen 7 7700X or Intel i7-13700K (8-12 cores)
- **RAM**: 32GB DDR5 (5600MHz)
- **Storage**: 1TB NVMe SSD (Gen 3, 3500MB/s)
- **Budget**: $1,800 - $2,500

**Capabilities**:
- Isaac Sim 4.5 (30-45 FPS, reduced ray tracing quality)
- 1024-2048 parallel RL environments
- Gazebo with 3-5 robots simultaneously
- OpenVLA 7B inference (75-120ms latency)
- All textbook examples run smoothly

**Use Cases**: All chapters, optimal for learning and prototyping

---

### Tier 3: Cloud GPU / Laptop with eGPU
**Target Users**: Budget-conscious students, remote learners, initial exploration

**Configuration A (Cloud GPU)**:
- **Platform**: AWS g5.xlarge (NVIDIA A10G, 24GB VRAM)
- **CPU**: 4 vCPUs (AMD EPYC 7R32)
- **RAM**: 16GB
- **Cost**: $1.00-$1.50/hour (~$50-75/month for 50 hours)

**Configuration B (Laptop + eGPU)**:
- **Laptop**: Any modern laptop (USB-C Thunderbolt 3/4)
- **eGPU Enclosure**: Razer Core X ($300) + RTX 4060 (8GB, $300)
- **Total**: $600 + laptop cost

**Capabilities**:
- ROS 2 + Gazebo (2-3 robots max)
- Isaac Sim (cloud only, 20-30 FPS with latency)
- Unity Robotics Hub (local or cloud)
- OpenVLA inference (cloud: 60ms, local eGPU: 150-200ms)

**Limitations**:
- Cloud: Latency for interactive work, ongoing costs
- eGPU: Thunderbolt bandwidth bottleneck (20-30% performance loss)

**Use Cases**: Chapters 1-2 fully supported, Chapters 3-4 with reduced parallelism

---

## 2. Workstation GPUs (Development)

### 2.1 NVIDIA RTX 40-Series Comparison

| Model | VRAM | CUDA Cores | Tensor Cores (4th Gen) | TDP | FP32 TFLOPS | INT8 TOPs | Price (2025) | Tier |
|-------|------|------------|------------------------|-----|-------------|-----------|--------------|------|
| **RTX 4090** | 24GB | 16,384 | 512 | 450W | 82.6 | 1,321 | $1,600 | 1 |
| **RTX 4080 Super** | 16GB | 10,240 | 320 | 320W | 52.2 | 836 | $1,000 | 1/2 |
| **RTX 4070 Ti Super** | 16GB | 8,448 | 264 | 285W | 44.1 | 706 | $800 | 2 |
| **RTX 4070 Ti** | 12GB | 7,680 | 240 | 285W | 40.1 | 642 | $700 | 2 |
| **RTX 4060 Ti (16GB)** | 16GB | 4,352 | 136 | 165W | 22.1 | 353 | $500 | 2/3 |
| **RTX 4060 Ti (8GB)** | 8GB | 4,352 | 136 | 165W | 22.1 | 353 | $400 | 3 |

**Key Observations**:
- **VRAM** is critical for Isaac Sim (8GB minimum, 12GB+ recommended)
- **Tensor Cores** accelerate TensorRT inference (4th gen = 2× faster than 3rd gen)
- **INT8 TOPs** indicate quantized model performance (OpenVLA, YOLOv8)
- **RTX 4070 Ti** offers best value for textbook workloads (Tier 2 sweet spot)

### 2.2 Professional Workstation GPUs

| Model | VRAM | ECC | Multi-GPU | FP64 | Price (2025) | Use Case |
|-------|------|-----|-----------|------|--------------|----------|
| **NVIDIA A6000** | 48GB | Yes | NVLink | 19.5 TFLOPS | $4,500 | Large-scale RL training, multi-robot sims |
| **NVIDIA A5000** | 24GB | Yes | NVLink | 8.0 TFLOPS | $2,500 | Production VLA training |
| **NVIDIA RTX A4000** | 16GB | Yes | No | 4.7 TFLOPS | $1,000 | Budget workstation option |

**When to choose professional GPUs**:
- Need >24GB VRAM for large VLA models (e.g., RT-2 562B)
- Multi-GPU setups (NVLink for model parallelism)
- ECC memory for critical applications (rare in robotics research)
- Note: Consumer RTX 40-series often faster for inference due to higher clock speeds

---

## 3. Embedded Platforms (Deployment)

### 3.1 NVIDIA Jetson Family

| Model | GPU | CUDA Cores | Tensor Cores | VRAM | CPU | RAM | Power | Price | Release |
|-------|-----|------------|--------------|------|-----|-----|-------|-------|---------|
| **Jetson AGX Orin 64GB** | 2048-core Ampere | 2048 | 64 | Shared | 12-core Arm Cortex-A78AE | 64GB LPDDR5 | 15-60W | $2,000 | 2023 |
| **Jetson AGX Orin 32GB** | 1792-core Ampere | 1792 | 56 | Shared | 8-core Arm Cortex-A78AE | 32GB LPDDR5 | 15-40W | $1,000 | 2022 |
| **Jetson Orin NX 16GB** | 1024-core Ampere | 1024 | 32 | Shared | 8-core Arm Cortex-A78AE | 16GB LPDDR5 | 10-25W | $600 | 2023 |
| **Jetson Orin Nano 8GB** | 1024-core Ampere | 1024 | 32 | Shared | 6-core Arm Cortex-A78AE | 8GB LPDDR5 | 7-15W | $500 | 2023 |
| **Jetson Orin Nano 4GB** | 512-core Ampere | 512 | 16 | Shared | 6-core Arm Cortex-A78AE | 4GB LPDDR5 | 5-10W | $250 | 2024 |

**Performance Estimates** (compared to RTX 4070 Ti desktop):
- **AGX Orin 64GB**: ~20% desktop performance, best mobile option
- **AGX Orin 32GB**: ~15% desktop performance, sweet spot for prototypes
- **Orin Nano 8GB**: ~8% desktop performance, sufficient for inference-only

**Textbook Recommendations**:
- **Chapter 3.3 (Navigation)**: Orin Nano 8GB minimum (Nvblox + VSLAM at 30 Hz)
- **Chapter 3.4 (RL Deployment)**: Orin NX 16GB (policy inference 50 Hz + safety checks)
- **Chapter 4.4 (VLA Integration)**: AGX Orin 32GB (Whisper + LLM + VLA pipeline <2s latency)

### 3.2 Alternative Embedded Platforms

| Platform | Compute | Strengths | Limitations | Price |
|----------|---------|-----------|-------------|-------|
| **Raspberry Pi 5** | Quad-core Cortex-A76, VideoCore VII | Low cost, broad community, GPIO | No CUDA (CPU-only TensorFlow Lite) | $80 |
| **Intel NUC 13 Extreme** | Core i9, RTX 4070 mobile | Desktop-class GPU in compact form | High power (180W), limited mobility | $2,500 |
| **Apple Mac Mini M2 Pro** | 16-core Neural Engine | Excellent power efficiency (20W) | No CUDA, limited ROS 2 support | $1,300 |
| **Khadas VIM4** | Amlogic A311D2, 6 TOPS NPU | Cost-effective NPU acceleration | Limited ML framework support | $200 |

**Use Case Comparison**:
- **Raspberry Pi 5**: Sensor interfacing, low-level control (no GPU perception)
- **Intel NUC**: Desktop replacement, stationary robots (power-hungry)
- **Mac Mini M2 Pro**: Development only (no deployment due to CUDA dependency)
- **Jetson**: Best balance for embedded AI (CUDA + TensorRT + power efficiency)

---

## 4. Sensors & Perception Hardware

### 4.1 RGB Cameras

| Camera | Resolution | FPS | FOV | Interface | Price | Use Case |
|--------|------------|-----|-----|-----------|-------|----------|
| **Intel RealSense D435i** | 1920×1080 | 90 | 87° H × 58° V | USB 3.1 | $350 | Depth + RGB + IMU, Chapter 2.4 VSLAM |
| **Intel RealSense D455** | 1280×720 | 90 | 90° H × 65° V | USB 3.1 | $500 | Extended depth range (6m vs 3m) |
| **Luxonis OAK-D** | 3840×2160 (4K) | 60 | 120° (fisheye) | USB 3.2 | $300 | On-device AI (Myriad X VPU), stereo |
| **Logitech C920 HD Pro** | 1920×1080 | 30 | 78° diagonal | USB 2.0 | $70 | Budget option, basic vision tasks |

**Selection Criteria**:
- **Depth sensing**: RealSense D435i/D455 for 3D reconstruction (Nvblox, Chapter 3.3)
- **Wide FOV**: OAK-D for navigation (obstacle avoidance, Chapter 3.3)
- **High FPS**: D435i at 90 FPS for fast motion tracking

### 4.2 LiDAR Sensors

| LiDAR | Range | FOV | Points/sec | Accuracy | Interface | Price | Use Case |
|-------|-------|-----|------------|----------|-----------|-------|----------|
| **Slamtec RPLIDAR A1M8** | 12m | 360° | 8,000 | ±5cm | USB-to-Serial | $100 | Indoor 2D mapping |
| **Hokuyo UST-10LX** | 10m | 270° | 43,200 | ±4cm | Ethernet | $1,500 | Precision 2D SLAM |
| **Velodyne VLP-16 (Puck)** | 100m | 360° H × 30° V | 300,000 | ±3cm | Ethernet | $4,000 | 3D outdoor mapping |
| **Livox Mid-360** | 40m | 360° H × 59° V | 200,000 | ±2cm | Ethernet | $500 | Budget 3D LiDAR |

**Textbook Integration**:
- **Chapter 2.4**: Simulate LiDAR in Gazebo/Isaac Sim (sensor plugins)
- **Chapter 3.3**: Fuse LiDAR + camera for costmap generation (Nav2)
- **Real robot**: RPLIDAR A1M8 sufficient for indoor mobile base

### 4.3 IMU & GPS

| Sensor | Type | Axes | Update Rate | Interface | Price | Use Case |
|--------|------|------|-------------|-----------|-------|----------|
| **VectorNav VN-100** | IMU + AHRS | 9-axis (3 gyro, 3 accel, 3 mag) | 800 Hz | UART/SPI | $600 | High-precision orientation |
| **Xsens MTi-3** | IMU + GNSS | 9-axis + GPS | 400 Hz | USB/UART | $2,500 | Outdoor navigation |
| **MPU-9250** | IMU | 9-axis | 1000 Hz | I2C/SPI | $10 | Budget prototyping |
| **u-blox ZED-F9P** | RTK GPS | N/A (position only) | 20 Hz | UART/I2C | $250 | Centimeter-level GPS |

**Allan Variance** (IMU noise characterization, Chapter 2.4):
- **VectorNav VN-100**: Gyro bias instability 10°/hr, accel 0.08 mg
- **MPU-9250**: Gyro bias instability 3000°/hr, accel 5 mg (100× worse)

### 4.4 Depth Sensors (Structured Light / ToF)

| Sensor | Technology | Range | FPS | Resolution | Interface | Price |
|--------|------------|-------|-----|------------|-----------|-------|
| **Intel RealSense D435i** | Stereo IR | 0.3-3m | 90 | 1280×720 | USB 3.1 | $350 |
| **Microsoft Azure Kinect** | ToF | 0.5-5.5m | 30 | 640×576 | USB 3.0 | $400 |
| **Apple iPhone 15 Pro LiDAR** | dToF | 0-5m | 30 | Proprietary | Lightning | Included |

**Trade-offs**:
- **Stereo IR (RealSense)**: Poor in sunlight (IR washout), excellent indoors
- **ToF (Azure Kinect)**: Better outdoor performance, lower resolution
- **iPhone LiDAR**: Convenient for prototyping (ARKit integration), limited ROS 2 support

---

## 5. Robot Platforms

### 5.1 Mobile Bases

| Platform | Drive | Payload | Speed | Battery | ROS 2 | Price | Use Case |
|----------|-------|---------|-------|---------|-------|-------|----------|
| **TurtleBot 4** | Differential | 20kg | 0.3 m/s | 3 hours | Native | $2,500 | Education, Chapter 1-2 |
| **Clearpath Jackal** | 4-wheel skid | 20kg | 2 m/s | 4 hours | Native | $15,000 | Outdoor research |
| **Unitree Go2** | Quadruped | 5kg | 1.5 m/s | 2 hours | Community | $3,500 | Legged locomotion, Chapter 3.4 |
| **Custom 3D-Printed** | Differential | 5kg | 0.5 m/s | Varies | DIY | $300 | Budget learning |

**TurtleBot 4 Specifications** (textbook recommended platform):
- **Compute**: Raspberry Pi 4 (4GB) or upgrade to Jetson Orin Nano
- **Sensors**: OAK-D Lite camera, 2D LiDAR (RPLIDAR A1 or Hokuyo)
- **Actuators**: 2× Create3 base motors, cliff sensors, IR bumpers
- **Software**: ROS 2 Humble, Nav2, SLAM Toolbox pre-configured

### 5.2 Manipulators

| Arm | DOF | Reach | Payload | Repeatability | ROS 2 | Price | Use Case |
|-----|-----|-------|---------|---------------|-------|-------|----------|
| **Kinova Gen3** | 7 | 902mm | 2kg | ±0.1mm | Native (MoveIt2) | $35,000 | Research manipulation |
| **UR5e (Universal Robots)** | 6 | 850mm | 5kg | ±0.03mm | Native | $25,000 | Industrial automation |
| **Trossen WidowX-250** | 6 | 650mm | 0.5kg | ±1mm | Community | $2,500 | Education, Chapter 3.3 |
| **Interbotix PX100** | 4 | 300mm | 0.3kg | ±2mm | Native | $800 | Desktop learning |

**Gripper Options**:
- **Robotiq 2F-85**: Adaptive parallel gripper, 85mm stroke, $3,000
- **Custom 3D-printed**: Servo-driven, 50mm stroke, $50 (designs online)

### 5.3 Humanoid Platforms

| Robot | Height | DOF | Compute | Sensors | Price | Availability |
|-------|--------|-----|---------|---------|-------|--------------|
| **Unitree H1** | 1.8m | 27 (with hands) | Jetson AGX Orin | 4× cameras, LiDAR, IMU | $90,000 | 2024 |
| **Boston Dynamics Atlas** | 1.5m | 28 | Custom | Stereo cameras, IMU, LiDAR | Not for sale | Research only |
| **Figure 01** | 1.7m | 24 | Custom | 6× cameras, force sensors | Not for sale | Announced 2024 |
| **Tesla Optimus** | 1.73m | 28+ | FSD Computer | 8× cameras, force sensors | TBA | In development |

**Note**: Humanoid platforms are emerging; textbook uses simulated humanoids in Isaac Sim (Chapter 3.4, 4.1) due to cost/availability.

---

## 6. Networking & Connectivity

### 6.1 Robot-to-Workstation Communication

| Technology | Bandwidth | Latency | Range | Use Case |
|------------|-----------|---------|-------|----------|
| **Ethernet (1 Gbps)** | 125 MB/s | <1ms | 100m (tethered) | Fastest, lowest latency (preferred) |
| **Wi-Fi 6 (5 GHz)** | 600 Mbps | 5-10ms | 50m (indoor) | Untethered mobile robots |
| **Wi-Fi 6E (6 GHz)** | 1200 Mbps | 3-5ms | 30m (indoor) | High-bandwidth sensors (4K cameras) |
| **5G (mmWave)** | 1-2 Gbps | 10-20ms | 200m (line-of-sight) | Outdoor, future use |

**Bandwidth Requirements** (simultaneous streams):
- 4× RGB cameras (1080p 30 FPS): ~100 Mbps
- 1× LiDAR (100k points/sec): ~10 Mbps
- ROS 2 topics (odometry, control): ~1 Mbps
- **Total**: ~120 Mbps → Wi-Fi 6 sufficient, Ethernet recommended for stability

### 6.2 Multi-Robot Systems

**DDS Discovery** (ROS 2 default):
- Automatic peer discovery on same subnet
- Scales to 10-20 robots before discovery overhead becomes issue
- Use ROS_DOMAIN_ID (0-232) to isolate robot groups

**Centralized vs Distributed**:
- **Centralized**: Workstation runs Nav2 planner, robots execute commands (lower latency)
- **Distributed**: Each robot runs full stack on Jetson (more robust, higher per-unit cost)

---

## 7. Hardware Selection Matrix

| Use Case | Workstation GPU | Embedded Platform | Sensors | Budget |
|----------|----------------|-------------------|---------|--------|
| **Undergraduate Course (Simulation-Only)** | RTX 4060 Ti 16GB | None | None | $2,000 |
| **Graduate Research (Sim + Real Robot)** | RTX 4070 Ti | Jetson Orin Nano 8GB | RealSense D435i, RPLIDAR A1 | $4,500 |
| **Industry Prototype (Mobile Manipulation)** | RTX 4090 | Jetson AGX Orin 32GB | RealSense D455, Hokuyo, VectorNav | $12,000 |
| **Humanoid Research (Full VLA Pipeline)** | 2× RTX 4090 (NVLink) | Jetson AGX Orin 64GB | 4× OAK-D, Livox Mid-360, MTi-3 | $25,000+ |

---

## 8. Power & Thermal Management

### 8.1 Workstation Power Draw

| Component | Idle | Load (Isaac Sim + RL) | Peak |
|-----------|------|----------------------|------|
| RTX 4070 Ti (285W TDP) | 15W | 220W | 285W |
| Ryzen 7 7700X (105W TDP) | 30W | 80W | 142W |
| 32GB DDR5 RAM | 5W | 10W | 15W |
| 1TB NVMe SSD | 2W | 5W | 8W |
| Motherboard + Fans | 20W | 30W | 40W |
| **Total System** | 72W | 345W | 490W |

**PSU Recommendation**: 850W 80+ Gold (leaves 360W headroom for spikes)

### 8.2 Jetson Power Modes

| Jetson Model | Mode | Power | GPU Freq | CPU Cores | Use Case |
|--------------|------|-------|----------|-----------|----------|
| **Orin Nano 8GB** | MAXN | 15W | 625 MHz | 6 active | Maximum performance |
| | 15W | 15W | 625 MHz | 4 active | Balanced |
| | 7W | 7W | 307 MHz | 2 active | Power-saving (idle) |
| **AGX Orin 32GB** | MAXN | 40W | 765 MHz | 8 active | Maximum performance |
| | 30W | 30W | 624 MHz | 8 active | Balanced |
| | 15W | 15W | 420 MHz | 4 active | Battery operation |

**Battery Sizing** (mobile robots):
- **AGX Orin 32GB at 30W**: 12V 10Ah LiPo (~120Wh) → 4 hours runtime
- **Orin Nano 8GB at 15W**: 12V 5Ah LiPo (~60Wh) → 4 hours runtime
- Add 50-100W for motors/actuators → reduce runtime by 50%

### 8.3 Thermal Management

**Workstation (RTX 4070 Ti)**:
- Stock cooler: OK for stock clocks, loud under load (2000 RPM)
- Aftermarket (Noctua NH-D15): Quieter (1200 RPM), better temps (-10°C)
- AIO liquid cooling (280mm): Best for sustained loads (Isaac Sim 8+ hours)

**Jetson (Passive vs Active)**:
- **Passive heatsink**: Sufficient for 7-15W modes (natural convection)
- **Active fan**: Required for MAXN mode (40W sustained, 60°C limit)
- **Thermal throttling**: Kicks in at 85°C (performance drops 20-40%)

---

## Recommended Configurations by Chapter

### Chapter 1 (ROS 2): Minimal Hardware
- **Workstation**: Any laptop with 8GB RAM (no GPU required)
- **Embedded**: None
- **Total**: $500-1,000 (existing laptop)

### Chapter 2 (Digital Twin): GPU Required
- **Workstation**: RTX 4060 Ti 16GB or better
- **Embedded**: None
- **Total**: $2,000-2,500

### Chapter 3 (Isaac): Tier 2 Hardware
- **Workstation**: RTX 4070 Ti (12GB)
- **Embedded** (optional): Jetson Orin Nano 8GB
- **Total**: $2,500-3,000 (workstation), +$500 (Jetson)

### Chapter 4 (VLA): Full Stack
- **Workstation**: RTX 4070 Ti or 4080 Super
- **Embedded**: Jetson AGX Orin 32GB
- **Sensors**: RealSense D435i ($350)
- **Total**: $3,500-4,500

---

**Hardware reference complete. See Appendix A for installation instructions.**
