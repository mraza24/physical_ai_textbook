# Hardware Overview

This section summarizes the hardware platforms used throughout the textbook. Understanding the hardware landscape will help you make informed decisions about equipment for labs, projects, and future robot development.

---

## Hardware Tiers

We adopt a **tiered approach** to accommodate different budgets and access levels:

| Tier | Hardware | Cost | Modules Accessible |
|------|----------|------|--------------------|
| **Tier 1: Minimal** | Standard laptop/desktop + cloud GPU | ~$0-50 | All modules (cloud) |
| **Tier 2: Enhanced** | Desktop + RTX 4070 Ti GPU | ~$600-800 | All modules (local) |
| **Tier 3: Complete** | Tier 2 + Jetson + RealSense | ~$1300-1500 | All + edge deployment |

**Good news**: You can complete the entire textbook with **Tier 1** (minimal cost, cloud GPU as needed).

---

## Tier 1: Minimal Setup (All Students)

### Laptop/Desktop

**Requirements**:
- **CPU**: 4+ cores (Intel Core i5/AMD Ryzen 5 or better)
- **RAM**: 16GB minimum (32GB recommended)
- **Storage**: 50GB free space (SSD preferred)
- **OS**: Ubuntu 22.04 LTS (native, dual-boot, or VM)
- **Network**: Stable internet (for cloud GPU, downloads)

**Cost**: $0 (using existing hardware)

**What You Can Do**:
- All of Module 1 (ROS 2)
- All of Module 2 (Gazebo, Unity simulation)
- Some of Module 3 (Isaac basics without GPU acceleration)
- Module 4 (VLA) with cloud GPU access

### Cloud GPU Access (Modules 3-4)

When you need GPU acceleration (Weeks 8-13), use cloud instances:

**Providers**:
- **AWS EC2**: g4dn.xlarge (NVIDIA T4, $0.526/hr)
- **Google Cloud**: n1-standard-4 + T4 ($0.50/hr)
- **Azure**: NC4as_T4_v3 ($0.526/hr)
- **Lambda Labs**: RTX 4090 ($0.60-1.00/hr)

**Estimated Usage**:
- Module 3 (Isaac): ~20-30 hours → $10-30
- Module 4 (VLA): ~10-20 hours → $10-20
- **Total**: ~$20-50 for entire course

**How to Use**:
1. Spin up instance when needed (Week 8+)
2. SSH into machine, run labs
3. Shut down immediately after use (cost = only when running)
4. Use spot instances to save 60-70%

See **Appendix E: Cloud vs On-Premise GPU** for detailed setup guides.

---

## Tier 2: Enhanced Setup (Recommended)

If you plan to continue robotics development beyond this course, investing in local GPU hardware is valuable.

### Desktop GPU

**Requirements**:
- **Model**: NVIDIA RTX 4070 Ti or better
- **VRAM**: 12GB minimum (16GB preferred)
- **CUDA Compute**: 8.9+ (Ampere/Ada architecture)
- **Power**: Adequate PSU (750W+ for RTX 4070 Ti)

**Recommended Models** (as of 2025):
- **RTX 4070 Ti** (12GB): $599 — Minimum for Isaac Sim
- **RTX 4080** (16GB): $999 — Comfortable for all tasks
- **RTX 4090** (24GB): $1599 — Overkill but future-proof

**Cost**: $600-1000

**What You Can Do**:
- Everything in Tier 1, but **faster and local**
- Isaac Sim at 30-60 FPS (vs cloud latency)
- Longer training runs (RL experiments)
- No hourly cloud costs

**Trade-offs**:
- **Pros**: No internet dependency, faster iteration, long-term savings
- **Cons**: Upfront cost, power consumption, desktop only (not portable)

**Recommendation**: If you plan to work on robotics projects for 1+ year, local GPU pays for itself vs cloud costs.

---

## Tier 3: Complete Setup (Advanced)

For students interested in edge deployment and real-world robotics.

### NVIDIA Jetson Orin Nano

**Purpose**: Edge AI for physical robots (deploy trained models to embedded hardware)

**Specs**:
- **Model**: Jetson Orin Nano (8GB)
- **GPU**: 1024-core NVIDIA Ampere
- **CPU**: 6-core Arm Cortex-A78AE
- **RAM**: 8GB unified memory
- **Power**: 7-15W (vs 200-450W desktop GPU)

**Cost**: $499

**What You Can Do**:
- Run Isaac ROS nodes on embedded hardware
- Deploy TensorRT-optimized perception models
- Experiment with robot edge computing
- Learn power-constrained AI deployment

**When Used**: Chapter 3.4 (optional Jetson deployment section)

**Alternatives**:
- Raspberry Pi 4 + Coral TPU ($100-150): Lower performance, good for learning
- Jetson Xavier NX (discontinued but available used): $400-500

**Note**: This is **optional**—the textbook does not require physical Jetson hardware. All concepts can be understood via simulation and cloud GPU work.

---

### Intel RealSense Depth Camera

**Purpose**: Real-world RGB-D perception (depth sensing)

**Specs**:
- **Model**: RealSense D435 or D455
- **RGB**: 1920x1080 @ 30 FPS
- **Depth**: Stereo-based, up to 10m range
- **FOV**: ~87° × 58° (D435)
- **Interface**: USB 3.0

**Cost**: $180-250 (D435), $250-350 (D455)

**What You Can Do**:
- Capture real depth data for VSLAM
- Test Isaac perception on real images
- Experiment with sim-to-real transfer
- Generate datasets for VLA training

**When Used**: Chapter 2.4 (optional real hardware section)

**Alternatives**:
- Kinect Azure ($400): Higher accuracy, higher cost
- Oak-D ($149): Lower cost, good for prototyping
- Smartphone LiDAR (iPhone/iPad Pro): Free if you own one

**Note**: Also **optional**—textbook focuses on simulated sensors, but real camera enriches learning.

---

## Humanoid Robot Platforms (Instructor Demos Only)

For **classroom demonstrations** or **advanced labs** (not expected for individual students):

| Platform | Cost | DOF | Sensors | Purpose |
|----------|------|-----|---------|---------|
| **Robotis OP3** | $8,000 | 20 | IMU, camera | Education, walking research |
| **Unitree H1** | $90,000 | 25+ | Full suite | High-performance research |
| **NAO** | $9,000 | 25 | Cameras, mics | HRI, education |
| **TurtleBot3** | $500-1,000 | 0 (wheeled) | LiDAR, camera | Mobile robot basics |

**Recommendation**: Most courses use **TurtleBot3** or **simulated humanoids** (Gazebo, Isaac Sim) rather than expensive humanoid hardware.

---

## Hardware Decision Matrix

Use this table to decide which hardware tier fits your needs:

| Question | Tier 1 | Tier 2 | Tier 3 |
|----------|--------|--------|--------|
| Can I complete the textbook? | ✅ Yes (cloud) | ✅ Yes (local) | ✅ Yes (+ edge) |
| Do I have a limited budget? | ✅ Best choice | ❌ High upfront | ❌ Highest cost |
| Do I have reliable internet? | ⚠️ Required | ✅ Nice to have | ✅ Optional |
| Am I continuing robotics? | ❌ Cloud adds up | ✅ Recommended | ✅ Best long-term |
| Do I want edge deployment? | ❌ Not possible | ❌ Not possible | ✅ Jetson ready |
| Do I want physical sensors? | ❌ Sim only | ⚠️ Possible | ✅ Full setup |

---

## Hardware Setup Guides

Detailed setup instructions are provided in appendices:

- **Appendix A: Hardware Setup Guide** — Full step-by-step instructions for Tier 2 & 3
- **Appendix E: Cloud vs On-Premise GPU** — Cost analysis and cloud setup tutorials

---

## Summary: Our Recommendation

**For most students**: Start with **Tier 1** (minimal setup + cloud GPU).

- **Total cost**: $20-50 for cloud GPU hours
- **Pros**: Accessible, no upfront investment, try before buying
- **Cons**: Requires internet, hourly costs add up

**If you can afford it and plan to continue robotics**: Upgrade to **Tier 2** (local GPU).

- **Total cost**: $600-800 for RTX 4070 Ti or RTX 4080
- **Pros**: Faster iteration, long-term savings, no internet dependency
- **Cons**: Upfront cost, desktop-only

**Tier 3** (Jetson + RealSense) is optional enrichment—valuable for edge AI research but not required for textbook completion.

---

## Next: Software Overview

Hardware is only half the story. Next, see **Software Overview** for the complete software stack (ROS 2, Gazebo, Isaac, LLMs).
