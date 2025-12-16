---
sidebar_position: 6
title: Appendix F - Troubleshooting Guide
---

# Appendix F: Troubleshooting Guide

Comprehensive solutions to common errors across all modules.

---

## ROS 2 Issues

### Issue: "Package 'ros-humble-X' not found"

**Cause**: ROS 2 repository not configured or package doesn't exist.

**Solution**:
```bash
# Update package list
sudo apt update

# Verify repository
apt-cache search ros-humble | grep <package-name>

# If not found, check spelling or try:
rosdep install --from-paths src --ignore-src -r -y
```

---

### Issue: "No module named 'rclpy'"

**Cause**: ROS 2 not sourced in current terminal.

**Solution**:
```bash
source /opt/ros/humble/setup.bash

# Make permanent:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Issue: "Topic not appearing in `ros2 topic list`"

**Cause**:
1. Node not running
2. DDS discovery issue
3. QoS mismatch

**Solution**:
```bash
# 1. Check node is running
ros2 node list

# 2. Check DDS discovery
export ROS_DOMAIN_ID=0  # Use same ID across all terminals
ros2 doctor

# 3. Check QoS
ros2 topic info /your_topic --verbose
```

---

### Issue: "Subscriber not receiving messages"

**Cause**: QoS policy mismatch (Reliable vs. Best Effort).

**Solution**:
```python
# In subscriber, match publisher's QoS:
from rclpy.qos import QoSProfile, ReliabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Match publisher
    depth=10
)

self.subscription = self.create_subscription(
    String, 'topic', self.callback, qos
)
```

---

### Issue: "Transform lookup failed" (tf2)

**Cause**: Transform not published or frame names incorrect.

**Solution**:
```bash
# 1. Check available frames
ros2 run tf2_tools view_frames

# 2. Check specific transform
ros2 run tf2_ros tf2_echo <source_frame> <target_frame>

# 3. Verify frame names (case-sensitive!)
ros2 topic echo /tf_static
```

---

## Gazebo Issues

### Issue: "gz: command not found"

**Cause**: Gazebo not installed or not in PATH.

**Solution**:
```bash
# Install Gazebo Harmonic
sudo apt-get install gz-harmonic

# Check installation
gz sim --version

# If still not found, add to PATH:
export PATH=$PATH:/usr/local/bin
```

---

### Issue: "Model falls through ground" or "Robot explodes"

**Cause**: Physics instability (time step too large, contact parameters wrong).

**Solution**:
```xml
<!-- In world SDF file, reduce time step: -->
<physics>
  <max_step_size>0.001</max_step_size>  <!-- Default: 0.004 -->
  <real_time_factor>1.0</real_time_factor>
</physics>

<!-- Adjust contact parameters: -->
<surface>
  <contact>
    <ode>
      <kp>1e6</kp>  <!-- Contact stiffness -->
      <kd>100</kd>  <!-- Contact damping -->
    </ode>
  </contact>
</surface>
```

---

### Issue: "Gazebo simulation running very slow"

**Cause**: Real-time factor < 1.0 (physics can't keep up).

**Solution**:
```bash
# 1. Check current real-time factor
gz stats  # Look for "Real time factor"

# 2. Reduce physics complexity:
# - Simplify collision meshes
# - Reduce max_step_size
# - Disable shadows/reflections

# 3. Use faster physics engine (ODE instead of DART)
```

---

### Issue: "ROS 2 - Gazebo bridge not working"

**Cause**: ros_gz_bridge not configured or topic names wrong.

**Solution**:
```bash
# Install bridge
sudo apt install ros-humble-ros-gz-bridge

# Test bridge manually
ros2 run ros_gz_bridge parameter_bridge /topic_name@std_msgs/msg/String@gz.msgs.StringMsg

# Check if messages flow:
gz topic -l  # Gazebo topics
ros2 topic list  # ROS 2 topics
```

---

## Isaac ROS Issues

### Issue: "CUDA error: out of memory"

**Cause**: Model + batch size exceeds GPU VRAM.

**Solution**:
```python
# 1. Reduce batch size
batch_size = 1  # Or smaller

# 2. Use FP16 instead of FP32
model = model.half()

# 3. Clear CUDA cache
import torch
torch.cuda.empty_cache()

# 4. Check current usage
nvidia-smi
```

---

### Issue: "TensorRT engine build fails"

**Cause**: ONNX model incompatible or TensorRT version mismatch.

**Solution**:
```python
# 1. Verify ONNX model
import onnx
model = onnx.load("model.onnx")
onnx.checker.check_model(model)

# 2. Simplify ONNX
import onnxsim
model_simp, check = onnxsim.simplify(model)
onnx.save(model_simp, "model_simplified.onnx")

# 3. Use compatible TensorRT version
pip install tensorrt==8.6.1  # Match CUDA version
```

---

### Issue: "Isaac ROS container won't start"

**Cause**: NVIDIA Container Toolkit not installed or Docker permissions.

**Solution**:
```bash
# 1. Verify NVIDIA Container Toolkit
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi

# 2. If error, reinstall:
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# 3. Add user to docker group
sudo usermod -aG docker $USER
newgrp docker
```

---

### Issue: "Isaac Visual SLAM tracking lost"

**Cause**:
1. Insufficient visual features
2. Fast motion (motion blur)
3. Bad calibration

**Solution**:
```bash
# 1. Check camera calibration
ros2 topic echo /camera_info

# 2. Reduce motion speed
# 3. Improve lighting (add features)
# 4. Tune VSLAM parameters (lower feature threshold)
```

---

## Unity Issues

### Issue: "ROS-TCP-Connector: Connection refused"

**Cause**: ROS-TCP-Endpoint not running or wrong IP/port.

**Solution**:
```bash
# 1. Start ROS-TCP-Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# 2. In Unity, verify:
# Robotics > ROS Settings
# ROS IP Address: 127.0.0.1 (if local) or <your-ip>
# ROS Port: 10000 (default)

# 3. Test connection
ping <ROS_IP>
telnet <ROS_IP> 10000
```

---

### Issue: "URDF import fails in Unity"

**Cause**:
1. Malformed URDF
2. Missing meshes
3. Unsupported joint types

**Solution**:
```bash
# 1. Validate URDF
check_urdf your_robot.urdf

# 2. Verify mesh paths (must be absolute or relative to URDF)
<mesh filename="package://my_robot/meshes/base.stl"/>

# 3. Simplify URDF (remove unsupported elements)
# Unity supports: revolute, prismatic, fixed, continuous
```

---

### Issue: "Robot physics unstable in Unity"

**Cause**: ArticulationBody parameters not tuned.

**Solution**:
```csharp
// In Unity ArticulationBody component:
// - Increase Solver Iterations: 20 (default: 4)
// - Increase Velocity Iterations: 10 (default: 1)
// - Set Max Depenetration Velocity: 1.0
// - Reduce Max Joint Force if exploding
```

---

## VLA / LLM Issues

### Issue: "OpenAI API rate limit exceeded"

**Cause**: Too many requests or exceeded quota.

**Solution**:
```python
# 1. Add retry logic
import openai
from tenacity import retry, stop_after_attempt, wait_exponential

@retry(stop=stop_after_attempt(3), wait=wait_exponential(min=1, max=10))
def call_gpt4(prompt):
    return openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}]
    )

# 2. Check usage
openai.api_quota.retrieve()

# 3. Upgrade plan or wait
```

---

### Issue: "Whisper transcription incorrect"

**Cause**:
1. Background noise
2. Wrong language model
3. Poor audio quality

**Solution**:
```python
import whisper
import noisereduce as nr
import soundfile as sf

# 1. Load audio and reduce noise
audio, sr = sf.read("audio.wav")
reduced_audio = nr.reduce_noise(y=audio, sr=sr)

# 2. Use larger Whisper model
model = whisper.load_model("large")  # Better accuracy

# 3. Specify language
result = model.transcribe(reduced_audio, language="en")
```

---

### Issue: "LLM outputs invalid JSON"

**Cause**: Prompt not constraining output format.

**Solution**:
```python
# Use structured prompts:
prompt = """
You are a robot task planner. Output ONLY valid JSON.

User command: "Pick up the red cup"

Output format:
{
  "task": "pick_and_place",
  "steps": [
    {"action": "detect_object", "target": "red cup"},
    {"action": "navigate_to", "target": "red cup"},
    {"action": "grasp", "target": "red cup"}
  ]
}

JSON:
"""

# Parse with error handling
import json
try:
    result = json.loads(llm_output)
except json.JSONDecodeError:
    # Retry or fallback
    pass
```

---

## Build Errors

### Issue: "colcon build fails with missing dependencies"

**Cause**: Dependencies not installed.

**Solution**:
```bash
# Install dependencies automatically
rosdep install --from-paths src --ignore-src -r -y

# If specific package missing:
sudo apt install ros-humble-<package-name>
```

---

### Issue: "CMake Error: Could not find package"

**Cause**: Package not in CMakeLists.txt or not installed.

**Solution**:
```cmake
# Add to CMakeLists.txt:
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

ament_target_dependencies(my_node
  rclcpp
  std_msgs
)
```

---

## Performance Issues

### Issue: "High latency in perception pipeline"

**Cause**: CPU bottleneck or inefficient code.

**Solution**:
```bash
# 1. Profile with ros2 topic hz
ros2 topic hz /camera/image  # Check input rate
ros2 topic hz /detections    # Check output rate

# 2. Identify bottleneck
ros2 run rqt_graph rqt_graph  # Visualize pipeline

# 3. Optimize:
# - Use TensorRT for DNN inference
# - Reduce image resolution
# - Increase QoS queue size
```

---

### Issue: "Robot control jerky / unstable"

**Cause**: Low control frequency or network latency.

**Solution**:
```bash
# 1. Check control loop frequency
ros2 topic hz /cmd_vel

# 2. Increase frequency (target: 10-50 Hz)
self.timer = self.create_timer(0.02, self.control_callback)  # 50 Hz

# 3. Use real-time kernel (advanced)
sudo apt install linux-lowlatency
```

---

## Debugging Tools

### General ROS 2 Debugging
```bash
# Introspect system
ros2 doctor
ros2 wtf  # Warnings, Errors, Failures

# Monitor performance
ros2 run rqt_top rqt_top  # CPU usage per node
ros2 topic bw /topic  # Bandwidth usage

# Record and playback
ros2 bag record -a  # Record all topics
ros2 bag play <bagfile>
```

### GPU Debugging
```bash
# Monitor GPU
watch -n 1 nvidia-smi

# Detailed GPU metrics
nvtop  # Install: sudo apt install nvtop

# Profile CUDA code
nsys profile python your_script.py
```

---

## Getting Help

### Before Asking for Help

1. **Read the error message carefully** (often tells you what's wrong)
2. **Google the exact error** (usually others have seen it)
3. **Check official documentation**
4. **Try minimal reproducible example** (isolate the problem)

### Where to Ask

1. **ROS Discourse**: https://discourse.ros.org/
2. **NVIDIA Isaac Forums**: https://forums.developer.nvidia.com/
3. **Stack Overflow**: Tag questions with `ros2`, `gazebo`, `isaac`, etc.
4. **GitHub Issues**: For package-specific bugs

### How to Ask (Get Better Answers)

**Good Question**:
```
Title: "Isaac ROS DNN Inference crashes with CUDA out of memory"

Environment:
- Ubuntu 22.04
- ROS 2 Humble
- NVIDIA RTX 4070 Ti (12GB VRAM)
- Isaac ROS version: 2.0.0

Problem:
Running isaac_ros_dnn_inference with YOLOv8 model crashes after 10 seconds with:
[ERROR] [dnn_inference]: CUDA out of memory: tried to allocate 2.00 GB

Tried:
1. Reduced batch size to 1
2. Converted model to FP16
3. Still crashes

Logs:
[paste logs here]

Question: How can I reduce memory usage further?
```

**Bad Question**:
```
"Isaac doesn't work, help!!!"
```

---

**See Also**:
- Chapter-specific troubleshooting sections
- Appendix B: Software Installation Guide
- Official documentation for each package
