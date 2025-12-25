--- sidebar_position: 5 title: Appendix E - Cloud vs On-Premise GPU Deployment ---
---
sidebar_position: 5
title: "Appendix E - Cloud vs On-Premise GPU Deployment"
---

# Appendix E: Cloud vs On-Premise GPU Deployment

Comprehensive comparison and decision guide for GPU infrastructure.

---

## Overview

Choosing between cloud and local GPU infrastructure is a critical decision affecting cost, performance, and workflow. This appendix provides data-driven guidance.

---

## Quick Comparison Table

| Factor | Cloud GPU | Local GPU | Winner |
|--------|-----------|-----------|--------|
| **Upfront Cost** | $0 | $2,000-3,000 | Cloud |
| **Long-term Cost (1 year)** | $1,200-2,400 | $2,500 (one-time) | Local* |
| **Latency** | 50-200ms (data upload) | 0ms | Local |
| **Scalability** | Instant (100s of GPUs) | Limited (1-2 GPUs) | Cloud |
| **Flexibility** | Pay-per-use | Always available | Depends |
| **Setup Time** | Minutes | Days (hardware delivery) | Cloud |
| **Privacy** | Data sent to cloud | Stays local | Local |

*Break-even at ~150-200 hours of use

---

## Cost Analysis

### Cloud GPU Pricing (2024)

**AWS EC2 G5 Instances**:
| Instance | GPU | VRAM | $/hour (on-demand) | $/hour (spot) | Use Case |
|----------|-----|------|---------------------|---------------|----------|
| g5.xlarge | A10G | 24GB | $1.006 | $0.30 | Development |
| g5.2xlarge | A10G | 24GB | $1.212 | $0.36 | Training (small) |
| g5.12xlarge | 4x A10G | 96GB | $5.672 | $1.70 | Training (large) |
| g5.48xlarge | 8x A10G | 192GB | $16.288 | $4.89 | Multi-GPU training |

**Google Cloud**:
| Instance | GPU | VRAM | $/hour | Use Case |
|----------|-----|------|--------|----------|
| n1-standard-8 + T4 | T4 | 16GB | $0.95 | Development |
| n1-standard-8 + V100 | V100 | 16GB | $2.48 | Training |
| a2-highgpu-1g | A100 | 40GB | $3.67 | High-end training |

**Paperspace (Gradient)**:
| Instance | GPU | VRAM | $/hour | Use Case |
|----------|-----|------|--------|----------|
| Free | M4000 | 8GB | $0 (limited) | Learning |
| P4000 | P4000 | 8GB | $0.51 | Development |
| A6000 | A6000 | 48GB | $1.89 | Production |

### Local GPU Costs

**RTX 4070 Ti Workstation** (Recommended):
- **GPU**: RTX 4070 Ti (12GB) - $800
- **CPU**: Ryzen 7 7700X - $350
- **RAM**: 32GB DDR5 - $150
- **Mobo**: B650 - $200
- **Storage**: 1TB NVMe - $100
- **PSU**: 750W Gold - $120
- **Case/Cooling**: $200
- **Total**: ~$2,500

**Operating Costs**:
- **Power**: 400W × 8 hours/day × 365 days × $0.15/kWh = $175/year
- **Maintenance**: $100/year (thermal paste, fans)
- **Total**: ~$275/year

### Break-Even Analysis

**Cloud** (g5.xlarge @ $1/hour):
- Year 1: $1/hour × 200 hours = $200
- Year 2: $1/hour × 200 hours = $200
- Year 3: $1/hour × 200 hours = $200
- **3-year total**: $600

**Local** (RTX 4070 Ti):
- Year 1: $2,500 (hardware) + $275 (power) = $2,775
- Year 2: $275
- Year 3: $275
- **3-year total**: $3,325

**Break-even**: ~200-250 hours per year

**Recommendation**:
- **`<150 hours/year`**: Use cloud
- **150-300 hours/year**: Either works, depends on other factors
- **More than 300 hours/year**: Buy local GPU

---

## Performance Comparison

### Inference Latency (YOLOv8 on 1920x1080 image)

| Platform | Latency | FPS | Notes |
|----------|---------|-----|-------|
| **Local RTX 4070 Ti** | 15ms | 66 | TensorRT FP16 |
| **Local RTX 4090** | 10ms | 100 | TensorRT FP16 |
| **AWS G5 (A10G)** | 18ms | 55 | +3ms (TensorRT compile) |
| **Cloud + Network** | 50-200ms | 5-20 | Data upload latency |

**Key Insight**: Cloud GPUs are fast, but data transfer kills real-time applications.

### Training Speed (RL Policy, 1M steps)

| Platform | Time | Cost |
|----------|------|------|
| Local RTX 4070 Ti | 8 hours | $2 (power) |
| AWS G5 (A10G) | 10 hours | $10 |
| AWS G5.12xlarge (4x A10G) | 3 hours | $17 |

**Key Insight**: Multi-GPU cloud training can be cost-effective for short bursts.

---

## Use Case Recommendations

### Scenario 1: Student Learning This Course

**Recommendation**: **Cloud** (Paperspace Free or AWS Spot)

**Reasoning**:
- Total GPU time needed: ~50-100 hours
- Cost: $30-100 (cloud) vs. $2,500 (local)
- Flexibility: Can upgrade to A100 for final project if needed

**Setup**:
```bash
# Launch Paperspace Gradient
paperspace-cli machines create --region "East Coast (NY2)" --machineType "A4000" --size 50

# Or AWS Spot Instance (save 70%)
aws ec2 request-spot-instances --instance-type g5.xlarge --spot-price "0.50"
```

---

### Scenario 2: Researcher with Daily GPU Use

**Recommendation**: **Local GPU** (RTX 4080/4090)

**Reasoning**:
- GPU time: 1,000+ hours/year
- Cost: $2,500 (year 1) vs. $1,000-2,000/year (cloud)
- Zero latency for interactive development
- Privacy (sensitive research data)

---

### Scenario 3: Startup Building Robot Perception

**Recommendation**: **Hybrid** (Local dev + Cloud training)

**Setup**:
- **Local**: RTX 4070 Ti for development, debugging
- **Cloud**: AWS G5.12xlarge (4x A10G) for batch training

**Reasoning**:
- Local GPU for fast iteration (80% of time)
- Cloud for scaling up (20% of time, large training runs)
- Optimizes cost and productivity

---

### Scenario 4: Edge Deployment Company

**Recommendation**: **Jetson Orin** for edge + **Local workstation** for dev

**Setup**:
- **Development**: RTX 4070 Ti workstation
- **Edge**: Jetson Orin Nano/NX on robots
- **Training**: Cloud bursts (AWS Spot for large models)

**Reasoning**:
- Must test on target hardware (Jetson)
- Local GPU mirrors Jetson architecture (same CUDA cores)
- Cloud for infrequent large-scale training

---

## Cloud Providers Detailed Comparison

### AWS EC2

**Pros**:
- ✅ Mature, reliable infrastructure
- ✅ Many GPU types (T4, A10G, V100, A100, H100)
- ✅ Spot instances (70% discount)
- ✅ Integration with S3, SageMaker

**Cons**:
- ❌ Complex pricing
- ❌ Requires VPC setup
- ❌ Can be expensive without optimization

**Best For**: Production deployments, large-scale training

---

### Google Cloud Platform (GCP)

**Pros**:
- ✅ Simple pricing
- ✅ Preemptible instances (cheaper)
- ✅ TPUs available (alternative to GPUs)
- ✅ Good for TensorFlow/JAX

**Cons**:
- ❌ Fewer GPU types than AWS
- ❌ Less ROS community support

**Best For**: TensorFlow/JAX workflows, TPU research

---

### Paperspace Gradient

**Pros**:
- ✅ **Free tier** (limited hours)
- ✅ Very simple UI (no AWS complexity)
- ✅ Jupyter notebooks built-in
- ✅ Cheap pricing ($0.51/hour for P4000)

**Cons**:
- ❌ Less reliable than AWS/GCP
- ❌ Limited regions
- ❌ Smaller machines (max 48GB VRAM)

**Best For**: Students, prototyping, learning

---

### Lambda Labs

**Pros**:
- ✅ Cheapest on-demand GPUs ($0.50/hour for A6000)
- ✅ Simple interface
- ✅ Reserved instances available

**Cons**:
- ❌ Often out of stock
- ❌ Limited support
- ❌ Smaller scale

**Best For**: Cost-conscious training

---

## Cloud Setup Examples

### AWS G5 Setup for Isaac ROS

```bash
# 1. Launch instance
aws ec2 run-instances \
  --image-id ami-0c7217cdde317cfec \  # Ubuntu 22.04
  --instance-type g5.xlarge \
  --key-name your-keypair \
  --security-group-ids sg-xxxxxx

# 2. SSH and install dependencies
ssh -i your-key.pem ubuntu@<instance-ip>
sudo apt update
sudo apt install -y nvidia-driver-525 cuda-toolkit-12-2

# 3. Install Docker + NVIDIA Container Toolkit
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo apt install -y nvidia-container-toolkit

# 4. Clone and run Isaac ROS
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
cd isaac_ros_common
./scripts/run_dev.sh
```

---

### Paperspace Gradient (Simplest)

```bash
# 1. Install CLI
pip install gradient

# 2. Login
gradient apiKey YOUR_API_KEY

# 3. Create notebook
gradient notebooks create \
  --name "ROS2-Isaac" \
  --machineType "A4000" \
  --container "nvidia/cuda:12.2.0-devel-ubuntu22.04"

# 4. Open browser, install ROS 2 in notebook
```

---

## Cost Optimization Strategies

### 1. Use Spot/Preemptible Instances
- **Savings**: 60-70% off on-demand pricing
- **Risk**: Can be terminated with 2-minute notice
- **Best for**: Training (can checkpoint and resume)
- **Not for**: Production inference

### 2. Right-Size Instances
- Don't over-provision: A10G (24GB) is enough for YOLOv8
- A100 (40GB) only if you need >24GB VRAM

### 3. Stop Instances When Not in Use
- **Mistake**: Leaving instance running overnight = $8-24 wasted
- **Solution**: Stop (not terminate) when done

### 4. Use Reserved Instances (Long-Term)
- 1-year commitment: 30-50% savings
- Only if you know you'll use it

### 5. Local Storage for Large Datasets
- **Problem**: Downloading 100GB dataset costs $$ and time
- **Solution**: Keep datasets on local storage, upload only results

---

## Privacy and Security

### Cloud Considerations
- Data uploaded to cloud (AWS, GCP stores data)
- Models trained on cloud servers
- Potential for data leakage

### Local Considerations
- Full control of data
- No egress fees
- Compliance-friendly (HIPAA, GDPR)

**Recommendation**: If handling sensitive data (medical, military, personal), prefer local or on-premise deployment.

---

## Final Recommendation Matrix

| Your Situation | Recommendation |
|----------------|----------------|
| Student, `<100 hours` | Paperspace Free or AWS Spot |
| Researcher, daily use | RTX 4070 Ti local workstation |
| Startup, moderate use | Hybrid (local dev + cloud training) |
| Enterprise, production | On-premise GPU servers + cloud backup |
| Edge deployment | Jetson Orin + local dev GPU |
| `Budget <$1,000` | Cloud only (no upfront cost) |
| Budget >$2,500 | Buy local GPU (better long-term) |

---

**See Also**:
- Appendix A: Hardware Setup Guide
- Appendix B: Software Installation Guide
- Chapter 3.2: Isaac Perception Pipeline
