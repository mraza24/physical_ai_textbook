# 13-Week Course Mapping

**Purpose**: Map textbook chapters to 13-week semester course structure.

**Target Audience**: Graduate students in robotics, AI, or related fields

**Prerequisites**: Basic Python programming, Linux command line familiarity

---

## Course Overview

| Week | Module | Topics | Chapters | Lab/Assignment |
|------|--------|--------|----------|----------------|
| 1 | Intro | Course overview, setup, robotics fundamentals | Front Matter, Hardware/Software Setup | Lab: Environment setup (Ubuntu 22.04, ROS 2 Humble installation) |
| 2 | Intro | Robot anatomy, sensors, actuators, course roadmap | Module intros, Appendices A & B | Lab: Verify installations, test hardware |
| 3 | Module 1 | ROS 2 fundamentals, computational graph, pub-sub | Chapter 1.1: ROS 2 Fundamentals | Lab: Publisher-subscriber nodes |
| 4 | Module 1 | Nodes, topics, services, actions | Chapter 1.2: Nodes & Communication | Lab: Service client-server |
| 5 | Module 1 | Launch files, parameters, configuration | Chapters 1.3 & 1.4: Launch Files, Packages | Lab: Multi-node system with launch files |
| 6 | Module 2 | Digital twin concept, Gazebo setup, URDF | Chapters 2.1 & 2.2: Digital Twin, Gazebo | Lab: Humanoid simulation in Gazebo |
| 7 | Module 2 | Unity visualization, VSLAM, sim-to-real | Chapters 2.3 & 2.4: Unity, VSLAM | Lab: VSLAM with RealSense in simulation |
| 8 | Module 3 | Isaac ecosystem, GPU setup, perception pipeline | Chapters 3.1 & 3.2: Isaac Overview, Perception | Lab: Object detection with Isaac |
| 9 | Module 3 | Manipulation, navigation, Nav2 integration | Chapter 3.3: Manipulation & Navigation | Lab: Autonomous navigation with Nav2 + Isaac |
| 10 | Module 3 | Reinforcement learning, Isaac Gym, PPO | Chapter 3.4: Reinforcement Learning | Lab: RL training for bipedal locomotion |
| 11 | Module 4 | VLA concepts, embodied intelligence, RT-1/RT-2 | Chapter 4.1: VLA Concepts | Lab: Review VLA architectures, OpenVLA exploration |
| 12 | Module 4 | LLM integration, Whisper voice commands | Chapters 4.2 & 4.3: LLM, Whisper | Lab: Voice-controlled robot commands |
| 13 | Module 4 | End-to-end VLA system, ethics, final project | Chapter 4.4: End-to-End System | Final Project: "Pick up the red cup" demo |

---

## Detailed Week-by-Week Breakdown

### Week 1: Course Overview & Setup

**Objectives**:
- Understand course structure and learning outcomes
- Set up development environment (Ubuntu 22.04, ROS 2 Humble)
- Review prerequisite knowledge (Python, Linux)

**Reading**:
- Front Matter: Preface, How to Use This Book
- Appendix B: Software Installation Guide (skim)

**Lab/Assignment**:
- Install Ubuntu 22.04 (dual-boot, VM, or native)
- Install ROS 2 Humble following Appendix B
- Verify installation with `ros2 --version`
- Complete environment test exercises

**Deliverable**: Screenshot of successful ROS 2 installation

---

### Week 2: Robot Fundamentals & Roadmap

**Objectives**:
- Understand humanoid robot components (sensors, actuators)
- Learn about hardware requirements (GPU, Jetson, RealSense)
- Preview all 4 modules and their dependencies

**Reading**:
- Front Matter: Hardware Overview, Software Overview
- Module 1-4 Introductions (brief overview)
- Appendix A: Hardware Setup Guide (skim)

**Lab/Assignment**:
- Install additional tools (Gazebo, Python packages)
- Verify GPU drivers (NVIDIA, CUDA) if available
- Optional: Set up Jetson Orin Nano if available

**Deliverable**: Environment checklist completed (ROS 2, Gazebo, Python packages verified)

---

### Week 3: ROS 2 Fundamentals

**Objectives**:
- Understand ROS 2 computational graph (nodes, topics, services, actions)
- Learn publish-subscribe communication pattern
- Create first ROS 2 nodes in Python

**Reading**:
- Chapter 1.1: ROS 2 Fundamentals

**Lab/Assignment**:
- Implement publisher node (publishes sensor data)
- Implement subscriber node (receives and processes data)
- Visualize computational graph with `rqt_graph`

**Deliverable**: Working publisher-subscriber system with screenshot of `rqt_graph`

---

### Week 4: Advanced Communication Patterns

**Objectives**:
- Deep dive into topics, services, and actions
- Understand synchronous vs asynchronous communication
- Implement service client-server pattern

**Reading**:
- Chapter 1.2: Nodes & Communication

**Lab/Assignment**:
- Create service server (provides robot status)
- Create service client (queries status)
- Implement action server for long-running task (e.g., move to goal)

**Deliverable**: Service and action implementation with demonstration video

---

### Week 5: System Configuration & Build

**Objectives**:
- Master launch files for multi-node systems
- Configure parameters with YAML files
- Build and manage ROS 2 workspaces

**Reading**:
- Chapter 1.3: Launch Files & Configuration
- Chapter 1.4: Building Packages & Workspaces

**Lab/Assignment**:
- Create launch file for multi-node system (3+ nodes)
- Use parameter files for configuration
- Build custom ROS 2 package from scratch

**Deliverable**: Custom package with launch file, demonstrates multi-node coordination

---

### Week 6: Digital Twin & Gazebo Simulation

**Objectives**:
- Understand digital twin concept and benefits
- Set up Gazebo simulation environment
- Load and control humanoid robot in simulation

**Reading**:
- Chapter 2.1: Digital Twin Concepts
- Chapter 2.2: Gazebo Fundamentals

**Lab/Assignment**:
- Load humanoid URDF into Gazebo
- Simulate basic movements (joint control)
- Add sensors (camera, IMU) to simulation

**Deliverable**: Gazebo simulation video showing humanoid robot with sensor data published to ROS 2

---

### Week 7: Unity Visualization & VSLAM

**Objectives**:
- Set up Unity for robotics visualization
- Integrate RealSense camera (simulated or real)
- Implement Visual SLAM for localization

**Reading**:
- Chapter 2.3: Unity for Robotics
- Chapter 2.4: Sensors & VSLAM

**Lab/Assignment**:
- Connect Unity to ROS 2 via TCP connector
- Simulate RealSense camera in Gazebo
- Run ORB-SLAM3 or Cartographer for VSLAM

**Deliverable**: VSLAM output (map + localization) with visualization in Unity or RViz

---

### Week 8: Isaac Ecosystem & Perception

**Objectives**:
- Understand Isaac SDK, Isaac Sim, Isaac ROS components
- Set up Isaac perception pipeline
- Run object detection with TensorRT optimization

**Reading**:
- Chapter 3.1: Isaac Overview
- Chapter 3.2: Isaac Perception Pipeline

**Lab/Assignment**:
- Install Isaac SDK and Isaac Sim
- Run Isaac perception demo (object detection)
- Optimize DNN model with TensorRT

**Deliverable**: Object detection output on test images, performance benchmarks (FPS, latency)

**Note**: GPU required (RTX 4070 Ti+ or cloud instance)

---

### Week 9: Autonomous Navigation

**Objectives**:
- Integrate Isaac perception with Nav2
- Configure costmaps and path planning
- Achieve autonomous navigation in simulation

**Reading**:
- Chapter 3.3: Isaac Manipulation & Navigation

**Lab/Assignment**:
- Set up Nav2 navigation stack
- Integrate Isaac perception for obstacle detection
- Demonstrate autonomous navigation (point A to B)

**Deliverable**: Autonomous navigation demo video, costmap visualization

---

### Week 10: Reinforcement Learning

**Objectives**:
- Understand RL fundamentals (PPO algorithm)
- Set up Isaac Gym for GPU-accelerated training
- Train bipedal locomotion policy

**Reading**:
- Chapter 3.4: Reinforcement Learning with Isaac Gym

**Lab/Assignment**:
- Set up Isaac Gym environment
- Train PPO policy for bipedal walking (use provided task)
- Evaluate policy performance

**Deliverable**: Training curves (reward vs episodes), policy demonstration video

**Note**: GPU required, training may take several hours

---

### Week 11: Vision-Language-Action Concepts

**Objectives**:
- Understand VLA models (RT-1, RT-2, OpenVLA)
- Learn embodied intelligence principles
- Explore multimodal reasoning

**Reading**:
- Chapter 4.1: VLA Concepts

**Lab/Assignment**:
- Review RT-1 and RT-2 papers
- Explore OpenVLA GitHub repository
- Brainstorm VLA project ideas

**Deliverable**: Short report (2-3 pages) comparing VLA approaches

---

### Week 12: LLM Integration & Voice Control

**Objectives**:
- Integrate LLM API (OpenAI or Anthropic) with ROS 2
- Implement Whisper for speech-to-text
- Build voice command pipeline

**Reading**:
- Chapter 4.2: LLM Integration for Robotics
- Chapter 4.3: Whisper for Voice Commands

**Lab/Assignment**:
- Create ROS 2 node that calls LLM API
- Parse LLM output into robot commands
- Integrate Whisper for voice input

**Deliverable**: Demo of voice command → LLM reasoning → robot action primitive

**Note**: LLM API access required (OpenAI or Anthropic)

---

### Week 13: End-to-End VLA System & Final Project

**Objectives**:
- Integrate all modules (ROS 2, Gazebo, Isaac, LLM, Whisper)
- Build complete VLA system: voice → vision → reasoning → action
- Demonstrate ethical AI considerations

**Reading**:
- Chapter 4.4: End-to-End VLA System

**Lab/Assignment**: **Final Project**
- **Task**: Create "Pick up the red cup" demo
- **Requirements**:
  - Voice input: "Pick up the red cup"
  - Whisper converts speech to text
  - LLM plans task (locate, approach, grasp, lift)
  - Isaac perception identifies red cup
  - Robot executes grasp and lift
- **Deliverable**: Complete demo video + code repository

**Alternative Projects** (if hardware limitations):
- Simulation-only VLA system in Gazebo/Isaac Sim
- Voice-controlled navigation ("Go to the kitchen")
- Multi-step task execution ("Set the table")

---

## Grading Breakdown (Suggested)

| Component | Weight | Description |
|-----------|--------|-------------|
| Weekly Labs (Weeks 3-10) | 40% | 8 labs × 5% each |
| Week 11 VLA Report | 10% | Comparative analysis of VLA models |
| Week 12 Voice Control Demo | 10% | LLM + Whisper integration |
| Week 13 Final Project | 30% | End-to-end VLA system |
| Participation & Quizzes | 10% | In-class engagement, concept checks |

---

## Hardware Allocation Strategy

### Minimal Setup (All Students)
- **Development Machine**: Laptop/Desktop with Ubuntu 22.04
- **ROS 2**: Humble Hawksbill
- **Simulation**: Gazebo 11, Unity (optional)
- **Cloud GPU**: Access to AWS/GCP/Azure for Isaac labs (Weeks 8-10)

### Enhanced Setup (Recommended)
- **Local GPU**: RTX 4070 Ti or better
- **Jetson Orin Nano**: For edge deployment (optional)
- **RealSense D435**: For real-world VSLAM and perception (optional)

### Lab/Classroom Setup
- **Shared GPU Workstations**: For Isaac labs (4-6 students per workstation)
- **Humanoid Robots**: 1-2 for final project demonstrations (Unitree, OP3, or similar)

---

## Prerequisites Check

Before Week 1, students should have:
- [ ] Basic Python programming (functions, classes, loops)
- [ ] Linux command line familiarity (cd, ls, nano/vim)
- [ ] Git basics (clone, commit, push)
- [ ] Linear algebra fundamentals (vectors, matrices)
- [ ] Probability basics (helpful for RL module)

---

## Course Policies

### Late Submissions
- 10% penalty per day late (max 3 days)
- Labs due Sunday 11:59 PM each week
- Final project has no late submissions

### Collaboration
- Labs: Discuss concepts, but submit individual code
- Final Project: Groups of 2-3 students allowed

### AI Tool Usage
- ChatGPT, Copilot allowed for debugging and learning
- Must understand and explain all submitted code
- Document any AI assistance in submission README

---

## Additional Resources

### Office Hours
- Instructor: Tuesdays 2-4 PM
- TA: Thursdays 3-5 PM
- Online: Discord server for async questions

### Recommended Readings
- Siciliano et al. (2010) - *Robotics: Modelling, Planning and Control*
- Craig (2017) - *Introduction to Robotics: Mechanics and Control*
- ROS 2 Official Documentation - https://docs.ros.org/en/humble/

### Community Resources
- ROS Discourse: https://discourse.ros.org/
- Isaac ROS Forums: https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/
- Robotics Stack Exchange: https://robotics.stackexchange.com/

---

**Validation**: This mapping covers all 13 weeks ✓, maps to all 16 chapters ✓, aligns with spec requirements (FR-005, SC-006) ✓
