# Glossary

> **Comprehensive glossary of robotics, AI, and embodied intelligence terms used throughout this textbook.**

This glossary contains **122 terms** extracted from all four modules, alphabetically organized with chapter references. Terms are defined in the context of modern robotics systems integrating ROS 2, simulation, GPU-accelerated perception, and vision-language-action models.

**Organization**: Terms are listed alphabetically with their definition and the chapter where they first appear (e.g., **Ch 1.2** = Module 1, Chapter 2).

---

## A

**Action** (Ch 1.1)
A goal-oriented task pattern in ROS 2 supporting long-running operations with progress feedback and cancellation (e.g., "navigate to waypoint"). Uses action servers and clients for asynchronous execution.

**Action Client** (Ch 1.2)
Node component that sends goals to action servers, monitors feedback during execution, and handles results upon completion.

**Action Server** (Ch 1.2)
Node component that accepts goals, executes long-running tasks, publishes periodic feedback, and returns results when complete or canceled.

**Action Tokenization** (Ch 4.1)
Converting continuous robot actions (joint angles, gripper state) into discrete tokens (256-1024 vocabulary) for transformer processing in VLA models. Enables language model architectures to output robot control commands.

**Allan Variance** (Ch 2.4)
Statistical method to characterize IMU noise components (random walk, bias instability, rate noise) across different time scales, essential for accurate sensor modeling.

**ASR (Automatic Speech Recognition)** (Ch 4.2)
Converting spoken language to text, industry term for speech-to-text systems. Modern implementations use encoder-decoder transformers (e.g., Whisper).

**Asynchronous Call** (Ch 1.2)
Non-blocking operation that returns immediately, allowing other work to proceed concurrently. Used for topics and actions in ROS 2.

---

## B

**Beam Search** (Ch 4.2)
Decoding algorithm exploring multiple candidate transcripts (typically top-5 sequences), selecting highest-probability sequence via likelihood scoring. Used in Whisper for robust speech recognition.

**Behavior Tree** (Ch 3.3)
Hierarchical state machine for robot task orchestration—composes complex behaviors from simple actions (navigate, wait, grasp) with fallback/recovery logic for error handling.

**Bundle Adjustment** (Ch 2.4)
Optimization technique jointly refining camera poses and 3D point positions to minimize reprojection errors in VSLAM systems.

---

## C

**Callback** (Ch 1.2)
Function invoked automatically when events occur (message received on topic, service requested, action goal accepted). Core pattern for event-driven ROS 2 programming.

**Chain-of-Thought (CoT)** (Ch 4.3)
Prompting technique encouraging LLMs to show step-by-step reasoning before final answer ("Let's think step by step"), improving logical consistency and complex task performance.

**colcon** (Ch 1.4)
Command-line build tool for compiling ROS 2 workspaces. Supports parallel builds, dependency resolution, and multi-package compilation with `colcon build`.

**Composition** (Ch 1.3)
Running multiple ROS 2 nodes in a single process for efficiency, reducing inter-process communication overhead and memory usage.

**Costmap** (Ch 3.3)
2D occupancy grid representing environment as cells marked free space (0), obstacles (254), or inflation zones (1-253) providing safety margins around obstacles for navigation.

**Cross-Modal Fusion** (Ch 4.1)
Attention mechanisms aligning visual embeddings (from DINOv2) with language embeddings (from LLM) in shared latent space, enabling VLA models to ground language instructions in visual scenes.

**CTC (Connectionist Temporal Classification)** (Ch 4.2)
Alternative ASR approach aligning audio frames to text without explicit segmentation (not used in Whisper, mentioned for comparison with encoder-decoder methods).

**CUDA** (Ch 3.1)
NVIDIA's parallel computing platform enabling GPU acceleration for general-purpose computation, foundational for Isaac ecosystem.

---

## D

**DDS (Data Distribution Service)** (Ch 1.1)
Industry-standard middleware (OMG specification) providing reliable, real-time communication across processes and networks. Underlies ROS 2 publish-subscribe architecture.

**Depth Estimation** (Ch 3.2)
Stereo vision technique computing distance from camera to scene points using disparity between left/right images. Isaac ROS achieves 60+ FPS on GPU vs 5-10 FPS CPU.

**Digital Twin** (Ch 2.1)
Virtual replica of a physical robot mirroring its geometry, physics properties, sensor characteristics, and control systems for simulation and testing.

**Domain Randomization** (Ch 2.1, 2.4, 3.4)
Training technique varying simulation parameters (lighting, textures, physics properties, sensor noise) to improve robustness when transferring policies from simulation to real robots.

**DWB (Dynamic Window Approach)** (Ch 3.3)
Local planner generating collision-free trajectories by sampling velocity space and scoring candidates against obstacles, goal alignment, and path smoothness.

---

## E

**Edge Deployment** (Ch 4.4)
Running all AI models locally on robot compute (Jetson) vs. cloud deployment (API calls), enables offline operation and reduces latency at cost of lower model capabilities.

**Embodied Intelligence** (Ch 4.1)
AI systems with physical form that learn through interaction with the real world, contrasting with disembodied AI trained purely on text/images.

**Encoder-Decoder** (Ch 4.2)
Two-part transformer architecture—encoder processes input sequences (e.g., mel spectrogram), decoder generates output sequences (e.g., text tokens) autoregressively.

**End-to-End Latency** (Ch 4.4)
Total time from user input to robot action completion. Target <2s for interactive tasks. Example: Whisper 180ms + LLM 2.5s + VLA 80ms + motion 3s = 5.9s.

---

## F

**Feature Point** (Ch 2.4)
Distinctive image location (corner, blob) trackable across video frames using descriptors (ORB, SIFT, SURF), foundational for VSLAM.

**Few-Shot Learning** (Ch 4.3)
Providing 2-5 example input-output pairs in LLM prompt to guide behavior without fine-tuning (e.g., "water plant" example → "clean spill" example).

---

## G

**GameObject** (Ch 2.3)
Fundamental Unity entity representing objects in 3D scenes (robots, cameras, lights), each with transform, mesh, and script components.

**Gazebo (New)** (Ch 2.2)
Modern simulator (formerly Ignition Gazebo), modular architecture designed for ROS 2, replaces Gazebo Classic (EOL January 2025).

**Gazebo Classic** (Ch 2.2)
Legacy simulator (versions 1-11), end-of-life January 2025, tightly coupled to ROS 1 and early ROS 2.

**Gazebo Plugin** (Ch 2.2)
Shared library (.so) extending simulator functionality (sensors, actuators, custom physics, GUI widgets) via standardized API.

**Global Planner** (Ch 3.3)
Finds optimal path from start to goal using graph search algorithms (A*, Dijkstra, Theta*) on costmap representation, providing waypoints for local planner.

**Goal State** (Ch 1.2)
Current status of an action goal: ACCEPTED, EXECUTING, SUCCEEDED, ABORTED, or CANCELED. Clients monitor state transitions.

**Graceful Degradation** (Ch 4.4)
System remains functional when components fail through fallback mechanisms (edge LLM fails → cloud API, cloud unavailable → cached plans, VLA fails → RL primitive).

**GR00T** (Ch 4.1)
NVIDIA's VLA framework for humanoid robots (March 2025), featuring dual-system architecture (System 1: reactive 20 Hz RL, System 2: deliberative 1 Hz VLA).

**Grounding** (Ch 4.3)
Connecting LLM symbolic reasoning to physical world state via perception sensors (cameras, joint encoders) and scene descriptions, enabling language to drive robot actions.

---

## H

**HDRP (High Definition Render Pipeline)** (Ch 2.3)
Unity's photorealistic rendering system with ray tracing, global illumination, and advanced materials for high-fidelity robot simulation.

**Hybrid Architecture** (Ch 4.4)
Combining fast reactive control (System 1: RL primitives 20 Hz, 50ms reaction) with slow deliberative reasoning (System 2: LLM/VLA 1 Hz, 1-2s planning).

---

## I

**Install Space (install/)** (Ch 1.4)
Directory containing compiled executables and resources ready for use after `colcon build`. Sourced via `source install/setup.bash`.

**INT8 Calibration** (Ch 3.2)
Quantization process converting FP32 neural network to 8-bit integers using calibration dataset (100-500 images) to determine optimal scaling factors. Achieves 4× speedup with 0.5-1.0% accuracy loss.

**Isaac Gym** (Ch 3.4)
GPU-accelerated physics simulator for massively parallel RL training using PhysX GPU backend. Deprecated 2024, replaced by Isaac Lab.

**Isaac Lab** (Ch 3.4)
Next-generation RL framework built on Isaac Sim 4.0+ with improved API, realistic rendering, and integration with Omniverse ecosystem.

**Isaac ROS** (Ch 3.1)
Collection of ROS 2 packages providing GPU-accelerated perception (DNN inference, stereo depth), localization (VSLAM), and manipulation algorithms.

**Isaac SDK** (Ch 3.1)
NVIDIA's comprehensive robotics toolkit providing libraries, frameworks, and AI models for robot development across edge and cloud.

**Isaac Sim** (Ch 3.1)
GPU-accelerated robot simulator built on NVIDIA Omniverse, featuring RTX ray tracing and photorealistic rendering for digital twin applications.

---

## J

**Jetson** (Ch 3.1)
NVIDIA's embedded AI computing platform for edge deployment (Jetson Orin Nano 4GB/8GB, AGX Orin 32GB/64GB). Runs full Isaac ROS stack on-device.

---

## L

**Launch Argument** (Ch 1.3)
Command-line parameter customizing launch file behavior (e.g., `robot_name:=my_robot`). Accessed via `DeclareLaunchArgument` and `LaunchConfiguration`.

**Launch File** (Ch 1.3)
Python script describing how to start and configure multiple ROS 2 nodes, parameters, and includes. Uses `LaunchDescription` API.

**LaunchDescription** (Ch 1.3)
ROS 2 object containing all launch actions (nodes, parameters, includes, conditionals). Return value of `generate_launch_description()`.

**Loop Closure** (Ch 2.4)
Detecting when robot returns to previously visited location in VSLAM, enabling drift correction through pose graph optimization.

---

## M

**Mel Spectrogram** (Ch 4.2)
Time-frequency representation of audio using mel scale (mimics logarithmic human hearing), 80 bins × T frames. Standard input for Whisper speech recognition.

**Model** (Ch 2.2)
Self-contained robot or object definition in SDF format (links, joints, collision geometry, visual meshes, sensors, plugins).

**Model Quantization** (Ch 4.4)
Reducing neural network precision (FP32 → FP16 → INT8 → INT4) for faster inference and smaller memory footprint at minor accuracy cost (2-5% typical).

**Monitoring** (Ch 4.4)
Real-time tracking of system health via ROS 2 diagnostics (latency per component, success rate, error rate, uptime), visualized through dashboards (Grafana, RViz).

**MoveIt2** (Ch 3.3)
ROS 2 motion planning framework for manipulators—computes collision-free arm trajectories using OMPL planners (RRT, PRM) and inverse kinematics solvers.

---

## N

**Nav2** (Ch 3.3)
ROS 2 navigation framework providing path planning, obstacle avoidance, behavior trees, and recovery behaviors for autonomous mobile robots.

**Node** (Ch 1.1)
Independent process in ROS 2 computational graph performing specific computation (e.g., camera driver, object detector, motion planner). Communicates via topics/services/actions.

**Node Launch Action** (Ch 1.3)
Declaration to start a ROS 2 node with specified package, executable, parameters, and remappings in launch file.

**Nsight Systems (nsys)** (Ch 3.2)
NVIDIA profiling tool for GPU performance analysis—shows kernel execution timeline, memory transfers, CPU-GPU synchronization bottlenecks.

**Nvblox** (Ch 3.3)
GPU-accelerated 3D voxel mapping using Truncated Signed Distance Field (TSDF) for real-time scene reconstruction at 30 Hz from depth images.

---

## O

**Object Tracking** (Ch 3.2)
Associating detected objects across video frames using tracking-by-detection (YOLOv8 + DeepSORT) with persistent IDs maintained across 30-frame occlusions.

**OpenVLA** (Ch 4.1)
Open-source 7B-parameter VLA (2024) trained on 970k robot demonstrations from Open X-Embodiment dataset, based on Llama 2 architecture. Supports LoRA fine-tuning.

**Overlay** (Ch 1.4)
Workspace layered on top of underlay, allowing package extensions/modifications without changing base installation. Sourced after underlay.

---

## P

**Package** (Ch 1.4)
Smallest unit of ROS 2 software distribution, containing nodes, launch files, configuration, and dependencies declared in `package.xml`.

**package.xml** (Ch 1.4)
Manifest file declaring package metadata (name, version, maintainer), build/run dependencies, and licensing information.

**Parallel Environments** (Ch 3.4)
Multiple simulation instances running simultaneously on GPU—Isaac Gym supports 1,000-10,000 environments achieving 66× training speedup vs single CPU environment.

**Parameter** (Ch 1.3)
Named configuration value accessible to ROS 2 nodes at runtime (e.g., `camera_fps: 30`). Set via YAML files or command-line.

**Photometric Calibration** (Ch 2.4)
Characterizing camera response to light intensity (gamma curve, vignetting, exposure time) for accurate brightness modeling in simulation.

**Physics Engine** (Ch 2.1)
Software library computing rigid body dynamics, collisions, constraints. Gazebo supports ODE, Bullet, DART, Simbody with different speed/accuracy trade-offs.

**Policy** (Ch 3.4)
Neural network (typically MLP) mapping robot observations (joint angles, velocities, IMU, vision) to actions (joint torques or target positions) in reinforcement learning.

**Pose Estimation** (Ch 3.3)
Computing 6D object pose (x, y, z position + roll, pitch, yaw orientation) for robotic grasping. Isaac ROS uses FoundationPose deep learning algorithm.

**Power Budget** (Ch 4.4)
Maximum continuous power draw constraint. Mobile robots target 10-20W for compute (Whisper 2W + LLM 6W + VLA 2W), 50-100W total with actuation.

**PPO (Proximal Policy Optimization)** (Ch 3.4)
Stable on-policy RL algorithm using clipped surrogate loss to prevent catastrophic policy collapse. Typical hyperparameters: lr=3e-4, clip_range=0.2.

**Prefab** (Ch 2.3)
Reusable Unity asset template (e.g., robot model with scripts, collision meshes, joint controllers) instantiated in scenes.

**Prompt Engineering** (Ch 4.3)
Crafting LLM inputs (system prompt defining capabilities/constraints, user prompt with context, few-shot examples) to elicit desired structured outputs for robotics tasks.

**Publisher** (Ch 1.1)
Node component that sends messages to a topic. Multiple publishers can write to same topic with QoS-controlled behavior.

---

## Q

**QoS (Quality of Service)** (Ch 1.1)
Configurable policies (reliability: best-effort vs reliable, durability: volatile vs transient-local, deadline, liveliness) determining message delivery behavior in ROS 2 DDS.

---

## R

**ReAct** (Ch 4.3)
Reasoning and Acting—LLM pattern interleaving Thought (reasoning), Action (execution), Observation (feedback) in iterative loop for adaptive task execution with replanning.

**Reinforcement Learning (RL)** (Ch 3.4)
Machine learning paradigm where agent learns optimal actions through trial-and-error interactions with environment, guided by reward signals.

**Remapping** (Ch 1.3)
Redirecting topic/service names without modifying node code (e.g., `/camera/image` → `/my_camera/image`). Essential for multi-robot systems.

**Reward Function** (Ch 3.4)
Scalar signal indicating task success—designed by humans to incentivize desired behavior (e.g., +forward_velocity -energy_cost -10×fallen).

**ROS-TCP-Connector** (Ch 2.3)
Unity package enabling bidirectional communication with ROS via TCP/IP protocol, supporting topic pub/sub and service calls.

**ROS-TCP-Endpoint** (Ch 2.3)
ROS 2 package (Python node) bridging TCP messages from Unity to DDS topics, enabling Unity-ROS integration.

**RT-1 (Robotics Transformer 1)** (Ch 4.1)
Google's first large-scale VLA for real-world manipulation (2022). 35M params, 130k episodes, 97% success seen tasks, introduced action tokenization.

**RT-2 (Robotics Transformer 2)** (Ch 4.1)
Google's VLA leveraging web-scale vision-language knowledge (2023). PaLM-E 562B backbone, 62% zero-shot emergent tasks, learns from internet images.

**RTX** (Ch 3.1)
NVIDIA's real-time ray tracing technology for photorealistic lighting, shadows, reflections in Isaac Sim.

---

## S

**Safety Layer** (Ch 4.4)
Validation module checking LLM/VLA outputs before execution—validates joint limits, collision-free paths, singularity avoidance, speed limits to prevent damage.

**SDF (Simulation Description Format)** (Ch 2.1, 2.2)
XML format describing complete simulation worlds including robots, environments, physics settings, and sensors. Native format for Gazebo.

**Semantic Segmentation** (Ch 3.2)
Pixel-wise classification assigning class labels to every pixel (e.g., road, sidewalk, person) using deep networks (DeepLabV3, U-Net).

**Sensor Fusion** (Ch 3.2)
Combining data from multiple sensors (cameras, LiDAR, IMU, GPS) with synchronized timestamps and coordinate transforms for robust perception.

**Sensor Model** (Ch 2.1)
Mathematical representation of sensor behavior (Gaussian noise, latency, field of view, range limits) for accurate simulation.

**Service** (Ch 1.1)
Synchronous request-response communication pattern in ROS 2 for querying information or triggering actions that complete quickly (<1 second typically).

**Service Client** (Ch 1.2)
Node component calling a service, sending requests and awaiting responses. Blocks until response received or timeout.

**Service Server** (Ch 1.2)
Node component providing a service, processing requests and returning responses via callback function.

**Signal-to-Noise Ratio (SNR)** (Ch 2.4)
Ratio of signal power to noise power, measured in decibels (dB). Higher SNR = cleaner sensor data. Typical camera SNR: 40-50 dB.

**Sim-to-Real Gap** (Ch 2.1)
Discrepancy between simulated and real-world robot behavior due to modeling inaccuracies (physics approximations, sensor noise, unmodeled dynamics).

**Sim-to-Real Transfer** (Ch 2.1)
Process of deploying policies, controllers, or models trained in simulation to real robots, often requiring domain randomization or real-world fine-tuning.

**Source Space (src/)** (Ch 1.4)
Directory containing package source code in ROS 2 workspace. Organized as `src/package_name/`.

**Spawn** (Ch 2.2)
Action of inserting a model into a running Gazebo world via `/spawn_entity` service call, dynamically adding robots/objects.

**Subscriber** (Ch 1.2)
Node component receiving messages from a topic via callback function. Multiple subscribers can read from same topic independently.

**Synchronous Call** (Ch 1.2)
Blocking operation waiting for completion before continuing execution. Used for services (request-response) in ROS 2.

**System Prompt** (Ch 4.3)
Instructions defining LLM role, robot capabilities, constraints, output format—remains constant across requests (vs. user prompt which changes per task).

---

## T

**Task Planning** (Ch 4.3)
Decomposing high-level goals ("make breakfast") into ordered sequence of executable actions (pick, place, pour, press, wait) using LLMs.

**TensorRT** (Ch 3.2)
NVIDIA's deep learning inference optimizer converting trained models (PyTorch, ONNX) to optimized CUDA engines with layer fusion, kernel tuning, and quantization (FP32→FP16→INT8). Achieves 2-5× speedup.

**Tool Use** (Ch 4.3)
LLM capability to call external functions (robot actions like pick/place, perception queries like get_scene) via function calling or structured JSON output.

**Topic** (Ch 1.1)
Named communication channel in ROS 2 using asynchronous publish-subscribe messaging, ideal for streaming sensor data (camera images, laser scans, odometry).

---

## U

**Underlay** (Ch 1.4)
Base workspace (e.g., `/opt/ros/humble/`) providing foundational packages. Sourced before overlay workspaces.

**Unity Editor** (Ch 2.3)
Interactive development environment for creating Unity scenes, attaching scripts, configuring GameObjects, and testing robot simulations.

**Unity Robotics Hub** (Ch 2.3)
Official Unity package for ROS integration, providing ROS-TCP-Connector, URDF Importer, pick-and-place demo, and Nav2 examples.

**URDF (Unified Robot Description Format)** (Ch 2.1)
XML format describing robot kinematic tree (links, joints) and dynamic properties (mass, inertia). Native format for ROS, converted to SDF for Gazebo.

**URDF Importer** (Ch 2.3)
Unity tool importing ROS URDF files and converting to Unity GameObjects with ArticulationBody joints preserving kinematics.

**URDF-to-SDF Conversion** (Ch 2.2)
Automatic translation from URDF (ROS-native format) to SDF (Gazebo-native format) at runtime via `gz_ros2_control` plugin.

---

## V

**VAD (Voice Activity Detection)** (Ch 4.2)
Algorithm detecting speech vs. silence/noise in audio stream. Silero VAD: 1.5M params, 2ms latency, probability threshold >0.5 = speech detected.

**VLA (Vision-Language-Action)** (Ch 4.1)
Multimodal foundation model mapping images + text instructions directly to robot action sequences, eliminating traditional modular sense-plan-act pipelines.

**VSLAM (Visual Simultaneous Localization and Mapping)** (Ch 2.4)
Technique to estimate robot pose and build map using only camera images, without LiDAR. Uses feature points, loop closure, bundle adjustment.

---

## W

**Wake Word** (Ch 4.2)
Trigger phrase activating speech recognition (e.g., "Hey robot", "computer"), implemented with Porcupine wake word engine for on-device detection.

**Whisper** (Ch 4.2)
OpenAI's open-source speech-to-text model (2022) trained on 680k hours of multilingual audio, supports 98 languages with encoder-decoder transformer architecture.

**World File** (Ch 2.2)
SDF file describing complete simulation environment (ground plane, lighting, models, gravity, physics solver settings) loaded at Gazebo startup.

**Workspace** (Ch 1.4)
Top-level directory containing source code (`src/`), build artifacts (`build/`), compiled packages (`install/`), and logs (`log/`). Built with `colcon build`.

---

## Y

**YAML Parameter File** (Ch 1.3)
Human-readable configuration file mapping parameter names to values. Loaded via `--params-file` argument in launch files.

---

## Z

**Zero-Copy Pipeline** (Ch 3.2)
GPU memory architecture eliminating CPU transfers—data flows directly from camera → GPU → TensorRT → post-processing without roundtrip to CPU RAM, reducing latency by 10-50ms.

---

## Glossary Statistics

- **Total Terms**: 122
- **Module 1 (ROS 2)**: 32 terms
- **Module 2 (Simulation)**: 30 terms
- **Module 3 (Isaac)**: 28 terms
- **Module 4 (VLA)**: 32 terms

**Categories**:
- Communication & Middleware: 18 terms (Topics, Services, Actions, DDS, QoS, etc.)
- Simulation & Digital Twins: 24 terms (Gazebo, Unity, URDF, SDF, Physics Engines, etc.)
- GPU & Perception: 16 terms (TensorRT, Isaac ROS, Depth Estimation, Segmentation, etc.)
- Navigation & Control: 12 terms (Nav2, MoveIt2, Costmap, Behavior Trees, etc.)
- Machine Learning: 18 terms (RL, Policy, PPO, VLA, LLMs, Domain Randomization, etc.)
- Build & Development: 10 terms (Workspace, Package, colcon, Launch Files, etc.)
- Hardware: 8 terms (Jetson, CUDA, RTX, etc.)
- Voice & Language: 16 terms (Whisper, VAD, Prompt Engineering, ReAct, etc.)

**Cross-References**: Terms marked with multiple chapter references appear in context across modules, indicating foundational concepts (e.g., Domain Randomization in Ch 2.1, 2.4, 3.4).

---

**Usage Notes for Students**:
- Terms in **bold** are entry headings
- Chapter references (Ch X.Y) indicate first/primary usage
- Italicized terms within definitions link to related concepts
- Technical acronyms fully expanded on first use
- Terms progress from basic (ROS 2 fundamentals) to advanced (hybrid VLA architectures)

**Usage Notes for Instructors**:
- Use as quiz/exam reference material
- Cross-reference glossary when introducing new concepts
- Assign students to create flashcards from subset of terms
- Reference when students encounter unfamiliar terminology in exercises
