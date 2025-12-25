# 13-Week Course Mapping

This table shows how textbook chapters map to a 13-week semester course. Use this to plan your weekly reading and lab assignments.

---

## Quick Reference Table

| Week | Module | Topics | Reading | Lab/Assignment |
|------|--------|--------|---------|----------------|
| **1** | Intro | Course overview, environment setup | Front Matter, Appendices A & B | Lab: Install Ubuntu 22.04, ROS 2 Humble |
| **2** | Intro | Hardware/software requirements, course roadmap | Hardware & Software Overviews, Module Intros | Lab: Verify installations, GPU test |
| **3** | M1 | ROS 2 fundamentals, computational graph | Chapter 1.1 | Lab: Publisher-subscriber nodes |
| **4** | M1 | Topics, services, actions | Chapter 1.2 | Lab: Service client-server |
| **5** | M1 | Launch files, packages, workspaces | Chapters 1.3 & 1.4 | Lab: Multi-node launch system |
| **6** | M2 | Digital twin, Gazebo, URDF | Chapters 2.1 & 2.2 | Lab: Humanoid in Gazebo |
| **7** | M2 | Unity visualization, VSLAM | Chapters 2.3 & 2.4 | Lab: RealSense VSLAM simulation |
| **8** | M3 | Isaac ecosystem, perception pipeline | Chapters 3.1 & 3.2 | Lab: Object detection with Isaac |
| **9** | M3 | Navigation, Nav2 integration | Chapter 3.3 | Lab: Autonomous navigation |
| **10** | M3 | Reinforcement learning, Isaac Gym | Chapter 3.4 | Lab: Bipedal locomotion training |
| **11** | M4 | VLA concepts, RT-1/RT-2/OpenVLA | Chapter 4.1 | Report: VLA architecture comparison |
| **12** | M4 | LLM integration, Whisper voice | Chapters 4.2 & 4.3 | Lab: Voice-controlled commands |
| **13** | M4 | End-to-end VLA system, ethics | Chapter 4.4 | **Final Project**: "Pick up red cup" demo |

---

## Detailed Week-by-Week Breakdown

### Week 1: Course Overview & Setup
**Focus**: Installation and environment verification

**Reading**:
- Title Page & Preface
- How to Use This Book
- Appendix B: Software Installation Guide (full read)

**Lab**:
- Install Ubuntu 22.04 (dual-boot, VM, or native)
- Install ROS 2 Humble following Appendix B
- Verify: `ros2 --version`, `gazebo --version`

**Deliverable**: Screenshot of successful installations

---

### Week 2: Hardware & Software Landscape
**Focus**: Understanding the technology stack

**Reading**:
- Hardware Overview
- Software Overview
- Module 1-4 Introductions (skim for big picture)
- Appendix A: Hardware Setup Guide (skim)

**Lab**:
- Install additional tools (Python packages, Gazebo plugins)
- Verify GPU drivers if available (NVIDIA, CUDA)
- Run ROS 2 demo (`ros2 run demo_nodes_cpp talker`)

**Deliverable**: Environment checklist completed

---

### Week 3: ROS 2 Fundamentals
**Focus**: Understanding the robotic nervous system

**Reading**:
- **Chapter 1.1: ROS 2 Fundamentals** (full)

**Key Concepts**:
- Computational graph architecture
- Nodes, topics, services, actions
- DDS middleware layer
- Publisher-subscriber pattern

**Lab**:
- Create publisher node (Python)
- Create subscriber node (Python)
- Visualize graph with `rqt_graph`

**Deliverable**: Working pub-sub system

---

### Week 4: Advanced Communication
**Focus**: Services and actions

**Reading**:
- **Chapter 1.2: Nodes & Communication** (full)

**Key Concepts**:
- Service request-response pattern
- Action goal-feedback-result pattern
- Synchronous vs asynchronous communication

**Lab**:
- Implement service server (robot status query)
- Implement service client
- Create action server for long-running task

**Deliverable**: Service and action demos

---

### Week 5: System Configuration & Build
**Focus**: Multi-node systems and workspaces

**Reading**:
- **Chapter 1.3: Launch Files & Configuration**
- **Chapter 1.4: Building Packages & Workspaces**

**Key Concepts**:
- Python launch files
- YAML parameter configuration
- Workspace structure (src/, build/, install/)
- colcon build system

**Lab**:
- Create launch file for 3+ nodes
- Use parameter files for configuration
- Build custom ROS 2 package from scratch

**Deliverable**: Custom package with launch file

**Checkpoint**: Module 1 quiz + pub-sub system demo

---

### Week 6: Digital Twins & Gazebo
**Focus**: Simulation fundamentals

**Reading**:
- **Chapter 2.1: Digital Twin Concepts**
- **Chapter 2.2: Gazebo Fundamentals**

**Key Concepts**:
- Digital twin definition and benefits
- Physics engines (ODE, Bullet)
- URDF robot descriptions
- Sensor simulation

**Lab**:
- Load humanoid URDF into Gazebo
- Simulate joint movements
- Add camera and IMU sensors
- Publish sensor data to ROS 2

**Deliverable**: Gazebo simulation video

---

### Week 7: Unity & Visual SLAM
**Focus**: Visualization and localization

**Reading**:
- **Chapter 2.3: Unity for Robotics**
- **Chapter 2.4: Sensors & VSLAM**

**Key Concepts**:
- Unity Robotics Hub
- ROS-TCP-Connector
- Visual SLAM algorithms (ORB-SLAM3)
- Sim-to-real transfer challenges

**Lab**:
- Connect Unity to ROS 2
- Simulate RealSense camera
- Run ORB-SLAM3 or Cartographer
- Generate map + localization

**Deliverable**: VSLAM output with map visualization

**Checkpoint**: Module 2 quiz + Gazebo simulation demo

---

### Week 8: Isaac Overview & Perception
**Focus**: GPU-accelerated AI for robots

**Reading**:
- **Chapter 3.1: Isaac Overview**
- **Chapter 3.2: Isaac Perception Pipeline**

**Key Concepts**:
- Isaac SDK, Isaac Sim, Isaac ROS
- DNN inference with TensorRT
- Object detection pipeline
- Semantic segmentation

**Lab**:
- Install Isaac SDK and Isaac Sim
- Run Isaac perception demo (object detection)
- Optimize model with TensorRT
- Measure FPS and latency

**Deliverable**: Perception output + performance benchmarks

**Note**: GPU required (RTX 4070 Ti+ or cloud instance)

---

### Week 9: Navigation & Manipulation
**Focus**: Autonomous navigation

**Reading**:
- **Chapter 3.3: Isaac Manipulation & Navigation**

**Key Concepts**:
- Nav2 navigation stack
- Costmap generation
- Path planning and control
- Isaac perception integration

**Lab**:
- Set up Nav2 with Isaac perception
- Configure costmaps
- Demonstrate autonomous navigation (A → B)

**Deliverable**: Navigation demo video + costmap visualization

---

### Week 10: Reinforcement Learning
**Focus**: Learning-based robot control

**Reading**:
- **Chapter 3.4: Reinforcement Learning with Isaac Gym**

**Key Concepts**:
- RL fundamentals (PPO algorithm)
- Isaac Gym environment
- Task definition and reward shaping
- Sim-to-real RL

**Lab**:
- Set up Isaac Gym
- Train PPO policy for bipedal walking
- Evaluate policy performance

**Deliverable**: Training curves + policy demo video

**Checkpoint**: Module 3 quiz + Isaac perception demo

---

### Week 11: Vision-Language-Action Concepts
**Focus**: Embodied intelligence foundations

**Reading**:
- **Chapter 4.1: VLA Concepts**

**Key Concepts**:
- VLA definition and paradigm
- RT-1, RT-2, OpenVLA architectures
- Embodied intelligence principles
- Multimodal reasoning

**Lab/Assignment**:
- Read RT-1, RT-2, OpenVLA papers
- Explore OpenVLA GitHub repository
- Write 2-3 page comparison report

**Deliverable**: VLA architecture comparison report

---

### Week 12: LLM & Voice Integration
**Focus**: Natural language robot control

**Reading**:
- **Chapter 4.2: LLM Integration for Robotics**
- **Chapter 4.3: Whisper for Voice Commands**

**Key Concepts**:
- LLM API integration (OpenAI, Anthropic)
- Prompt engineering for robotics
- Whisper speech-to-text
- Voice → text → reasoning → action pipeline

**Lab**:
- Create ROS 2 node calling LLM API
- Parse LLM output into robot commands
- Integrate Whisper for voice input
- Demo: "Move forward" voice command

**Deliverable**: Voice command demo

**Note**: LLM API access required

---

### Week 13: End-to-End VLA System
**Focus**: Capstone integration project

**Reading**:
- **Chapter 4.4: End-to-End VLA System**

**Key Concepts**:
- Full system integration (ROS 2 + Gazebo + Isaac + LLM + Whisper)
- Ethical considerations (safety, bias, privacy)
- Future directions in VLA research

**Final Project**:
**Task**: Build "Pick up the red cup" demo
- Voice input: "Pick up the red cup" (Whisper)
- Vision: Identify red cup location (Isaac perception)
- Reasoning: Plan grasp sequence (LLM)
- Action: Execute grasp and lift (ROS 2 + simulation)

**Alternative Projects** (if hardware limited):
- Simulation-only VLA in Gazebo/Isaac Sim
- Voice-controlled navigation ("Go to kitchen")
- Multi-step task ("Set the table")

**Deliverable**: Demo video + code repository + project report

---

## Grading Breakdown (Suggested)

| Component | Weight | Description |
|-----------|--------|-------------|
| Weekly Labs (Weeks 3-10) | 40% | 8 labs @ 5% each |
| Week 11 VLA Report | 10% | Comparative analysis |
| Week 12 Voice Control Demo | 10% | LLM + Whisper integration |
| Week 13 Final Project | 30% | End-to-end VLA system |
| Participation & Quizzes | 10% | In-class engagement |

---

## Milestones & Checkpoints

**Checkpoint 1 (Week 5)**: Module 1 mastery
- Pass Module 1 quiz (10 questions)
- Demo working ROS 2 multi-node system

**Checkpoint 2 (Week 7)**: Module 2 mastery
- Pass Module 2 quiz
- Demo Gazebo humanoid simulation

**Checkpoint 3 (Week 10)**: Module 3 mastery
- Pass Module 3 quiz
- Demo Isaac perception on test images

**Final Checkpoint (Week 13)**: Full VLA system
- Complete end-to-end voice-to-action demo
- Submit code + documentation

---

## Flexibility & Adaptation

**Accelerated Learners**:
- Complete multiple chapters per week
- Attempt "Hard" difficulty exercises
- Explore optional extensions in appendices

**Students Needing Support**:
- Take 2 weeks per module if needed
- Focus on "Easy" and "Medium" exercises first
- Use office hours and discussion forums

**Hardware Limitations**:
- Use cloud GPU for Modules 3-4 (cost: ~$20-40 total)
- Skip optional Jetson and RealSense labs
- Focus on simulation rather than physical hardware

---

**Ready to Start? → Week 1: Install Ubuntu 22.04 + ROS 2 Humble**
