# Appendix D: Troubleshooting Guide

> **Common issues, error messages, and systematic debugging strategies for ROS 2, simulation, and AI robotics.**

## Quick Diagnostic Checklist

Before diving into specific issues, run these commands:

```bash
# 1. Check ROS 2 environment
printenv | grep ROS
ros2 doctor --report

# 2. Check NVIDIA GPU
nvidia-smi

# 3. Check disk space
df -h

# 4. Check network connectivity
ping -c 3 8.8.8.8

# 5. Check Python environment
python3 --version
pip list | grep -E "(torch|ros|gazebo)"
```

---

## ROS 2 Issues

### Issue: "Package not found" after installation

**Symptoms**:
```bash
ros2 run my_package my_node
# Package 'my_package' not found
```

**Root Causes**:
1. Workspace not sourced
2. Package not built
3. Wrong workspace sourced

**Solution**:
```bash
# Check if package exists
ros2 pkg list | grep my_package

# Rebuild workspace
cd ~/ros2_ws
colcon build --packages-select my_package

# Source correct workspace (order matters!)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Verify sourcing
echo $AMENT_PREFIX_PATH  # Should show both /opt/ros/humble and ~/ros2_ws/install
```

---

### Issue: "colcon build" fails with CMake errors

**Symptoms**:
```
CMake Error: Could not find a package configuration file provided by "rclcpp"
```

**Root Causes**:
1. Missing dependencies
2. ROS 2 not sourced before build
3. Corrupt build cache

**Solution**:
```bash
# 1. Source ROS 2 first
source /opt/ros/humble/setup.bash

# 2. Install missing dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Clean build (if corrupted)
rm -rf build/ install/ log/
colcon build

# 4. Check specific package dependencies
rosdep check --from-paths src/my_package
```

---

### Issue: Nodes can't communicate (topics not visible)

**Symptoms**:
```bash
ros2 topic list  # Shows only /rosout, not custom topics
```

**Root Causes**:
1. Different ROS_DOMAIN_ID
2. Firewall blocking DDS
3. Wrong DDS middleware

**Solution**:
```bash
# 1. Check ROS_DOMAIN_ID (must match across nodes)
echo $ROS_DOMAIN_ID  # Run on both machines

# Set same domain (0-232)
export ROS_DOMAIN_ID=0

# 2. Disable firewall temporarily (test only!)
sudo ufw disable

# 3. Test with simple pub/sub
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener

# 4. Check DDS discovery
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

---

### Issue: "QoS mismatch" warnings

**Symptoms**:
```
[WARN]: New subscription discovered on topic '/camera/image', requesting incompatible QoS
```

**Root Causes**:
- Publisher uses BEST_EFFORT, subscriber uses RELIABLE (or vice versa)

**Solution**:
```python
# Match QoS settings (in subscriber)
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT  # Match publisher
)

self.subscription = self.create_subscription(
    Image,
    '/camera/image',
    self.callback,
    qos
)
```

**When to use**:
- **RELIABLE**: Commands, critical data (default)
- **BEST_EFFORT**: Sensor streams (camera, LiDAR) where latest data matters

---

## Gazebo Issues

### Issue: Gazebo crashes on launch

**Symptoms**:
```
gz sim --version
Segmentation fault (core dumped)
```

**Root Causes**:
1. NVIDIA driver issues
2. Vulkan/OpenGL misconfiguration
3. Conflicting Gazebo versions

**Solution**:
```bash
# 1. Check graphics drivers
glxinfo | grep "OpenGL version"  # Should show Mesa or NVIDIA
vulkaninfo | grep "deviceName"

# 2. Install Vulkan drivers
sudo apt install mesa-vulkan-drivers vulkan-tools

# 3. Purge old Gazebo Classic (if mixed installations)
sudo apt purge gazebo* libgazebo*
sudo apt autoremove

# 4. Reinstall Gazebo Harmonic
sudo apt install gz-harmonic

# 5. Test with simple world
gz sim shapes.sdf
```

---

### Issue: Robot model not visible in Gazebo

**Symptoms**:
- Gazebo launches, world loads, but robot is invisible

**Root Causes**:
1. URDF/SDF parsing errors
2. Mesh file paths incorrect
3. Model spawned outside world bounds

**Solution**:
```bash
# 1. Validate URDF/SDF syntax
check_urdf my_robot.urdf
gz sdf -k my_robot.sdf  # Check SDF

# 2. Check console for errors
gz sim --verbose 4 my_world.sdf 2>&1 | grep -i error

# 3. Verify mesh paths
# In URDF: <mesh filename="package://my_pkg/meshes/base.stl"/>
# Ensure package path resolves correctly

# 4. Manually spawn at origin
gz service -s /world/default/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "my_robot.sdf", pose: {position: {z: 1.0}}'
```

---

### Issue: Physics behaves unrealistically (robot falls through floor)

**Root Causes**:
1. Collision geometry missing
2. Mass/inertia not set
3. Physics engine timestep too large

**Solution**:
```xml
<!-- In SDF model -->
<link name="base_link">
  <!-- Add collision geometry (not just visual) -->
  <collision name="collision">
    <geometry>
      <box><size>0.5 0.5 0.2</size></box>
    </geometry>
  </collision>

  <!-- Set proper inertial properties -->
  <inertial>
    <mass>10.0</mass>
    <inertia>
      <ixx>0.4</ixx><iyy>0.4</iyy><izz>0.2</izz>
    </inertia>
  </inertial>
</link>

<!-- In world file, adjust physics -->
<physics name="1ms" type="ignored">
  <max_step_size>0.001</max_step_size>  <!-- 1ms timestep -->
  <real_time_factor>1.0</real_time_factor>
</physics>
```

---

## Isaac Sim Issues

### Issue: Isaac Sim won't launch (crashes immediately)

**Symptoms**:
```
isaac-sim.sh
[Error] [omni.physx.plugin] PhysX GPU acceleration not available
```

**Root Causes**:
1. Insufficient VRAM (<8GB)
2. CUDA/driver mismatch
3. Vulkan not available

**Solution**:
```bash
# 1. Check GPU VRAM
nvidia-smi --query-gpu=memory.total --format=csv

# 2. Check CUDA compatibility
nvcc --version
nvidia-smi | grep "CUDA Version"

# 3. Launch with reduced settings
~/.local/share/ov/pkg/isaac-sim-4.5.0/isaac-sim.sh \
  --no-window \
  --/rtx/raytracing/enabled=false \
  --/rtx/reflections/enabled=false

# 4. Check Omniverse logs
tail -f ~/.nvidia-omniverse/logs/Isaac-Sim/*/isaac-sim.log
```

---

### Issue: Isaac Sim + ROS 2 bridge not working

**Symptoms**:
- Isaac Sim launches, but `ros2 topic list` shows no topics

**Root Causes**:
1. ROS 2 bridge extension not enabled
2. Wrong ROS_DOMAIN_ID
3. Isaac Sim using different Python than ROS 2

**Solution**:
```bash
# In Isaac Sim GUI:
# Window → Extensions → Search "ROS2 Bridge" → Enable

# In Isaac Sim Python console (inside Isaac Sim):
import rosgraph
print(rosgraph.is_master_online())  # Should return True if rosmaster exists (ROS 1 only)

# For ROS 2, check with:
from rclpy.utilities import get_available_rmw_implementations
print(get_available_rmw_implementations())

# Alternative: Use standalone ROS 2 bridge script
cd ~/.local/share/ov/pkg/isaac-sim-4.5.0
./python.sh standalone_examples/api/omni.isaac.ros2_bridge/ros2_bridge.py
```

---

## NVIDIA GPU Issues

### Issue: "CUDA out of memory" errors

**Symptoms**:
```python
RuntimeError: CUDA out of memory. Tried to allocate 2.00 GiB
```

**Root Causes**:
1. Batch size too large
2. Multiple GPU processes
3. Memory leak in code

**Solution**:
```bash
# 1. Check current VRAM usage
nvidia-smi --query-gpu=memory.used,memory.total --format=csv

# 2. Kill zombie processes
nvidia-smi | grep python
kill -9 <PID>

# 3. Clear PyTorch cache
python3 -c "import torch; torch.cuda.empty_cache()"

# 4. Reduce batch size in code
# Before: batch_size = 32
# After:  batch_size = 8

# 5. Enable gradient checkpointing (saves memory)
model.gradient_checkpointing_enable()
```

---

### Issue: TensorRT engine build fails

**Symptoms**:
```
[TensorRT] ERROR: Cannot find binding for tensor: input
```

**Root Causes**:
1. ONNX model input names mismatch
2. TensorRT version incompatible with ONNX opset
3. Unsupported layer types

**Solution**:
```python
# 1. Inspect ONNX model
import onnx
model = onnx.load("model.onnx")
print([input.name for input in model.graph.input])  # Check input names

# 2. Simplify ONNX (remove unsupported ops)
import onnxsim
model_simp, check = onnxsim.simplify(model)
onnx.save(model_simp, "model_simplified.onnx")

# 3. Build TensorRT engine with explicit input name
import tensorrt as trt

builder = trt.Builder(logger)
network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
parser = trt.OnnxParser(network, logger)

with open("model_simplified.onnx", "rb") as f:
    parser.parse(f.read())

# Check network inputs
print(f"Network inputs: {[network.get_input(i).name for i in range(network.num_inputs)]}")
```

---

## Unity Robotics Hub Issues

### Issue: Unity-ROS connection timeout

**Symptoms**:
```
[Unity] ROS-TCP-Connector: Failed to connect to ROS endpoint at 127.0.0.1:10000
```

**Root Causes**:
1. ROS-TCP-Endpoint not running
2. Firewall blocking port 10000
3. Wrong IP address in Unity settings

**Solution**:
```bash
# 1. Start ROS-TCP-Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# 2. Verify endpoint is listening
netstat -an | grep 10000
# Should show: tcp 0 0 0.0.0.0:10000 0.0.0.0:* LISTEN

# 3. Test connection from command line
nc -zv 127.0.0.1 10000  # Should succeed

# 4. In Unity: Robotics → ROS Settings
# - ROS IP Address: 127.0.0.1 (localhost) or workstation IP
# - ROS Port: 10000
# - Protocol: ROS2

# 5. Allow firewall (if remote connection)
sudo ufw allow 10000/tcp
```

---

## VLA & LLM Issues

### Issue: OpenVLA inference extremely slow on Jetson

**Symptoms**:
- Inference takes 5-10 seconds per frame (target: <100ms)

**Root Causes**:
1. Model not quantized
2. Running on CPU instead of GPU
3. Not using TensorRT optimization

**Solution**:
```python
# 1. Verify CUDA is being used
import torch
print(torch.cuda.is_available())  # Must be True

device = torch.device("cuda:0")
model = model.to(device)

# 2. Quantize model to INT8
from transformers import AutoModelForCausalLM
model = AutoModelForCausalLM.from_pretrained(
    "openvla/openvla-7b",
    torch_dtype=torch.float16,  # FP16 first
    device_map="cuda:0"
)

# 3. Convert to TensorRT (advanced)
import torch_tensorrt
trt_model = torch_tensorrt.compile(
    model,
    inputs=[torch_tensorrt.Input(shape=[1, 3, 224, 224])],
    enabled_precisions={torch.int8}
)
```

---

### Issue: Whisper transcription language mismatch

**Symptoms**:
- English input transcribed as foreign language

**Solution**:
```python
import whisper

model = whisper.load_model("base")

# Force English language
result = model.transcribe(
    audio_path,
    language="en",  # Explicit language
    task="transcribe"  # Not "translate"
)

# Or use English-only model (faster)
model = whisper.load_model("base.en")
```

---

### Issue: Claude/GPT-4 API rate limits

**Symptoms**:
```
anthropic.RateLimitError: 429 Too Many Requests
```

**Solution**:
```python
import time
from anthropic import Anthropic, RateLimitError

client = Anthropic(api_key=os.environ["ANTHROPIC_API_KEY"])

def call_with_retry(prompt, max_retries=3):
    for attempt in range(max_retries):
        try:
            message = client.messages.create(
                model="claude-3-5-sonnet-20241022",
                max_tokens=1000,
                messages=[{"role": "user", "content": prompt}]
            )
            return message.content[0].text
        except RateLimitError:
            if attempt < max_retries - 1:
                wait_time = 2 ** attempt  # Exponential backoff
                time.sleep(wait_time)
            else:
                raise

# Tier limits (as of 2025):
# Claude Sonnet: 50 requests/min, 200k tokens/min
# GPT-4 Turbo: 500 requests/min, 300k tokens/min
```

---

## Systematic Debugging Strategy

### 1. Isolate the Problem
- Does it work with simple examples? (e.g., `ros2 run demo_nodes_cpp talker`)
- Does it work on a different machine?
- Does it work with older software versions?

### 2. Check Logs
```bash
# ROS 2 logs
ros2 run rqt_console rqt_console

# Gazebo logs
gz log --list
gz log --replay <log_file>

# Isaac Sim logs
tail -f ~/.nvidia-omniverse/logs/Isaac-Sim/*/isaac-sim.log

# System logs
journalctl -xe
dmesg | tail -20
```

### 3. Minimal Reproducible Example
- Strip down code to minimal version that shows the bug
- Share on ROS Discourse, Gazebo Answers, NVIDIA Forums

### 4. Community Resources
- **ROS Discourse**: https://discourse.ros.org/
- **ROS Answers**: https://answers.ros.org/
- **Gazebo Community**: https://community.gazebosim.org/
- **NVIDIA Isaac Forums**: https://forums.developer.nvidia.com/c/isaac/
- **Stack Overflow**: Tag with `ros2`, `gazebo`, `tensorrt`

---

**For installation issues, see Appendix A. For software version conflicts, see Appendix C.**
