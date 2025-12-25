# Appendix F: Exercise Solutions Summary

> **Guidance and hints for textbook exercises. Full solutions available to instructors (see Appendix G).**

## How to Use This Appendix

**For Students**:
- Attempt exercises independently first
- Use hints if stuck after 15-20 minutes
- Solutions provide guidance, not complete code (encourages learning)
- Cross-reference with chapter code examples

**For Instructors**:
- Full worked solutions available in instructor materials (Appendix G)
- Solutions include runnable code, expected outputs, common mistakes
- Rubrics provided for grading programming assignments

---

## Module 1: ROS 2 Nervous System

### Chapter 1.1: Exercises (3 total)

**Exercise 1.1.1** (Easy): Create pub/sub nodes for sensor data
- **Hint**: Use `Float32` message type for temperature, `publish()` at 10 Hz
- **Expected outcome**: `ros2 topic echo /temperature` shows continuous stream
- **Common mistake**: Forgetting to call `spin()` to keep node alive

**Exercise 1.1.2** (Medium): Implement QoS reliability comparison
- **Hint**: Create two subscribers with RELIABLE and BEST_EFFORT, drop packets with `tc` command
- **Key insight**: RELIABLE retransmits lost messages, BEST_EFFORT skips them
- **Test**: `sudo tc qdisc add dev lo root netem loss 20%`

**Exercise 1.1.3** (Hard): Multi-node computational graph
- **Hint**: Use launch file to start 4 nodes, connect topics in chain: A→B→C→D
- **Verification**: `rqt_graph` should show linear topology
- **Extension**: Add branching (A→B→C, A→D)

### Chapter 1.2: Exercises (3 total)

**Exercise 1.2.1** (Easy): Service for arithmetic operations
- **Hint**: Define `AddTwoInts.srv` interface, implement server with `add_service()`
- **Test**: `ros2 service call /add example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"`

**Exercise 1.2.2** (Medium): Action for long-running task
- **Hint**: Use `Fibonacci.action` interface, publish feedback every 0.5s in loop
- **Key**: `goal_handle.publish_feedback()` for intermediate results
- **Cancellation**: Handle `is_cancel_requested()` gracefully

**Exercise 1.2.3** (Hard): Hybrid service+action navigation
- **Approach**: Service checks feasibility (fast), action executes navigation (slow with feedback)
- **ROS 2 pattern**: Common in Nav2 architecture

### Chapter 1.3: Exercises (3 total)

**Exercise 1.3.1** (Easy): Launch file with parameters
- **Hint**: Use `DeclareLaunchArgument` for camera FPS, read with `LaunchConfiguration`
- **Test**: `ros2 launch my_pkg camera.launch.py fps:=60`

**Exercise 1.3.2** (Medium): Conditional node launching
- **Hint**: Use `IfCondition` with launch argument (e.g., `use_sim:=true`)
- **Pattern**: `Node(..., condition=IfCondition(LaunchConfiguration('use_sim')))`

**Exercise 1.3.3** (Hard): Multi-robot namespace launch
- **Approach**: Use `GroupAction` with `PushRosNamespace` for each robot
- **Verification**: `ros2 topic list` shows `/robot1/camera`, `/robot2/camera`

### Chapter 1.4: Exercises (2 total)

**Exercise 1.4.1** (Easy): Create custom message package
- **Hint**: Add `rosidl_default_generators` to `package.xml`, define in `msg/` folder
- **Build**: `colcon build --packages-select my_msgs`, source, test with `ros2 interface show`

**Exercise 1.4.2** (Medium): Multi-package workspace with dependencies
- **Hint**: Package B depends on Package A (add to `package.xml` and `CMakeLists.txt`)
- **Build order**: `colcon build` automatically handles dependency order
- **Test**: `colcon graph` shows dependency tree

---

## Module 2: Digital Twin Simulation

### Chapter 2.1: Exercises (3 total)

**Exercise 2.1.1** (Easy): Compare physics engines in Gazebo
- **Hint**: Launch same world with ODE, Bullet, DART via `<physics type=...>`
- **Metrics**: Measure simulation time with `time gz sim ...`
- **Expected**: ODE fastest, DART most accurate

**Exercise 2.1.2** (Medium): URDF to SDF conversion
- **Hint**: Use `gz sdf -p robot.urdf > robot.sdf`, compare files
- **Key differences**: SDF has `<world>`, multiple `<model>`, URDF is single robot

**Exercise 2.1.3** (Hard): Domain randomization implementation
- **Approach**: Randomize friction (0.3-0.9), mass (±20%), light position
- **Code**: Use `SetEntityWorldPose()`, `SetLinkMass()` in Gazebo plugin
- **Validation**: Train RL policy, test on non-randomized world (expect degradation)

### Chapter 2.2: Exercises (3 total)

**Exercise 2.2.1** (Easy): Sensor plugin addition
- **Hint**: Add `<plugin name="camera" filename="libgazebo_ros_camera.so">` to SDF link
- **Verification**: `ros2 topic list | grep camera` shows `/camera/image_raw`

**Exercise 2.2.2** (Medium): Custom Gazebo plugin (LED controller)
- **Hint**: Inherit from `gazebo::ModelPlugin`, subscribe to ROS 2 topic
- **LED visual**: Change `<visual><material><ambient>` color via plugin

**Exercise 2.2.3** (Hard): Multi-robot spawn via ROS 2 service
- **Approach**: Call `/spawn_entity` service in loop with unique names
- **Namespace**: Use `--robot_namespace robot_N` in spawn args

### Chapter 2.3: Exercises (3 total)

**Exercise 2.3.1** (Easy): Unity scene with ROS 2 topic publishing
- **Hint**: Attach `ROS2Publisher` script to GameObject, publish `Float32` on Update()
- **Verification**: `ros2 topic echo /unity/sensor`

**Exercise 2.3.2** (Medium): URDF import and articulation
- **Hint**: Use URDF Importer, check "Create ArticulationBody" for joints
- **Test**: Apply forces to joints, verify realistic physics

**Exercise 2.3.3** (Hard): HDRP rendering for photorealistic dataset
- **Approach**: Enable HDRP, add HDRI skybox, configure post-processing
- **Dataset**: Capture 1000 images with random poses/lighting
- **Use case**: Train object detection model

### Chapter 2.4: Exercises (3 total)

**Exercise 2.4.1** (Easy): Sensor noise characterization
- **Hint**: Record IMU data stationary, compute Allan variance plot
- **Tool**: `allan_variance` Python package
- **Expected**: Identify bias instability, random walk coefficients

**Exercise 2.4.2** (Medium): ORB-SLAM3 with simulated camera
- **Hint**: Launch Gazebo camera, remap topics to ORB-SLAM3 input
- **Config**: Adjust `Camera.fx`, `Camera.fy` to match Gazebo intrinsics

**Exercise 2.4.3** (Hard): Sim-to-real VSLAM comparison
- **Approach**: Run ORB-SLAM3 on (1) Gazebo, (2) Unity, (3) real RealSense
- **Metrics**: ATE (Absolute Trajectory Error) vs ground truth
- **Insight**: Unity photorealistic rendering reduces sim-to-real gap vs Gazebo

---

## Module 3: Isaac AI Brain

### Chapter 3.1: Exercises (2 total)

**Exercise 3.1.1** (Easy): Isaac Sim + ROS 2 Carter robot navigation
- **Hint**: Load `carter_warehouse_navigation.usd`, enable ROS 2 bridge
- **Test**: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...`

**Exercise 3.1.2** (Medium): TensorRT YOLOv8 conversion and benchmarking
- **Hint**: Export PyTorch → ONNX → TensorRT, measure FPS with `nsys profile`
- **Expected speedup**: 3-5× on RTX 4070 Ti (PyTorch 25 FPS → TensorRT 90 FPS)

### Chapter 3.2: Exercises (3 total)

**Exercise 3.2.1** (Easy): INT8 calibration for segmentation model
- **Hint**: Use 500 calibration images, `trtexec --int8 --calib=...`
- **Accuracy check**: Compare IoU before/after quantization (expect -0.5% to -1%)

**Exercise 3.2.2** (Medium): Zero-copy pipeline implementation
- **Approach**: Use `cupy` for GPU arrays, avoid `.cpu()` calls
- **Measurement**: Profile with `torch.cuda.Event()` timestamps
- **Expected**: Eliminate 10-20ms CPU↔GPU transfer latency

**Exercise 3.2.3** (Hard): Multi-camera fusion for 360° perception
- **Approach**: 4 cameras (front/rear/left/right), merge depth maps on GPU
- **Isaac ROS**: Use `isaac_ros_nvblox` with multi-camera input

### Chapter 3.3: Exercises (3 total)

**Exercise 3.3.1** (Easy): Nav2 with Nvblox costmap
- **Hint**: Launch `isaac_ros_nvblox` + `nav2_bringup`, configure costmap plugin
- **Verification**: `rviz2` shows 3D voxel map + 2D costmap overlay

**Exercise 3.3.2** (Medium): MoveIt2 pick-and-place pipeline
- **Approach**: Detect object (YOLO), estimate pose (FoundationPose), plan grasp (MoveIt2)
- **Key**: Convert pose from camera frame to robot base frame via `tf2`

**Exercise 3.3.3** (Hard): Jetson Orin Nano deployment optimization
- **Approach**: Profile on workstation, identify bottlenecks, quantize models
- **Target**: 30 Hz VSLAM + Nvblox + Nav2 on Orin Nano 8GB (15W mode)

### Chapter 3.4: Exercises (3 total)

**Exercise 3.4.1** (Easy): PPO training with Isaac Lab (Cartpole)
- **Hint**: Use provided config, train 1M steps (~15 min RTX 4070 Ti)
- **Success**: Policy balances pole for 500 steps

**Exercise 3.4.2** (Medium): Domain randomization for quadruped locomotion
- **Approach**: Randomize mass (±30%), friction (0.3-0.9), terrain slope (0-15°)
- **Training**: 10M steps with 2048 parallel envs (~2 hours)
- **Sim-to-real**: Deploy policy on Unitree Go2, measure success rate

**Exercise 3.4.3** (Hard): TensorRT policy deployment
- **Approach**: Export ONNX from Isaac Lab, build TensorRT engine, integrate in ROS 2 node
- **Target**: 50 Hz policy inference on Jetson AGX Orin 32GB

---

## Module 4: VLA Systems

### Chapter 4.1: Exercises (2 total)

**Exercise 4.1.1** (Easy): OpenVLA zero-shot inference
- **Hint**: Load pre-trained OpenVLA-7B, test on household object manipulation
- **Input**: RGB image + text "pick up the red cup"
- **Output**: 7D action (x, y, z, roll, pitch, yaw, gripper)

**Exercise 4.1.2** (Medium): LoRA fine-tuning on custom dataset
- **Approach**: Collect 100 demonstrations, fine-tune with `peft` library (rank=8)
- **Dataset**: Use teleoperation or scripted policies in Isaac Sim
- **Expected improvement**: 15-25% success rate increase on target task

### Chapter 4.2: Exercises (3 total)

**Exercise 4.2.1** (Easy): Whisper transcription with VAD
- **Hint**: Segment audio with Silero VAD, transcribe each segment independently
- **Benefit**: Avoid transcribing silence, reduce latency

**Exercise 4.2.2** (Medium): Faster-Whisper Jetson optimization
- **Approach**: INT8 quantization with CTranslate2, benchmark vs standard Whisper
- **Expected speedup**: 4-6× (base model: 400ms → 70ms on Jetson Orin Nano)

**Exercise 4.2.3** (Hard): Real-time voice control pipeline (ROS 2)
- **Architecture**: Microphone → VAD → Whisper → LLM → VLA → Robot
- **Latency budget**: <2s end-to-end (voice input to robot motion start)

### Chapter 4.3: Exercises (3 total)

**Exercise 4.3.1** (Easy): ReAct loop for "clean table" task
- **Hint**: Implement Thought→Action→Observation loop with 5 max iterations
- **Actions**: `detect_objects()`, `pick()`, `place()`, `done()`

**Exercise 4.3.2** (Medium): Chain-of-Thought prompt engineering
- **Approach**: Add "Let's break this down step-by-step:" prefix to prompt
- **Comparison**: Measure success rate with/without CoT (expect +10-20% on multi-step tasks)

**Exercise 4.3.3** (Hard): LLM tool use with function calling
- **Approach**: Define tools as JSON schemas, parse LLM output for function calls
- **Example tools**: `get_object_pose(name)`, `check_gripper_force()`, `move_to(x, y, z)`

### Chapter 4.4: Exercises (3 total)

**Exercise 4.4.1** (Easy): End-to-end latency profiling
- **Hint**: Add `time.time()` timestamps at each pipeline stage, compute deltas
- **Bottlenecks**: Typically LLM inference (2-3s), robot motion (3-5s)

**Exercise 4.4.2** (Medium): Hybrid System 1/System 2 architecture
- **Approach**: System 1 (RL) runs at 20 Hz, System 2 (VLA) runs at 1 Hz
- **Coordination**: System 2 sets goals, System 1 executes reactive control

**Exercise 4.4.3** (Hard): Graceful degradation implementation
- **Approach**: Fallback chain (VLA → cached LLM plans → hard-coded primitives)
- **Monitoring**: Use ROS 2 diagnostics to detect model failures, trigger fallbacks

---

## Self-Assessment Questions

After completing exercises, ask yourself:

1. **Functionality**: Does my solution produce expected output?
2. **Code Quality**: Is code readable, modular, with error handling?
3. **Performance**: Meets latency/throughput requirements?
4. **Understanding**: Can I explain how it works to a peer?
5. **Extensions**: What would I change for real-world deployment?

**If stuck on an exercise**:
1. Re-read chapter section related to exercise
2. Check chapter code examples for similar patterns
3. Use hints in this appendix
4. Search ROS Answers, GitHub issues for similar problems
5. Ask on ROS Discourse with minimal reproducible example
6. (Instructors only) Refer to full solutions in Appendix G

---

**Exercises designed for incremental learning. Complete all three difficulty levels per chapter for mastery.**
