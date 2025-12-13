---
sidebar_position: 4
title: Course-to-Chapter Mapping
---

# Course-to-Chapter Mapping

## 13-Week Semester Course Schedule

This textbook is designed for a **13-week semester course** with 3 hours per week (2 hours lecture + 1 hour lab).

---

## Week-by-Week Schedule

### **Week 1: Introduction & ROS 2 Fundamentals**

**Module**: 1 - The Robotic Nervous System
**Chapter**: [1.1 - ROS 2 Fundamentals](./module1/chapter1-1-ros2-fundamentals)

**Learning Objectives**:
- Understand ROS 2 architecture and computational graph
- Create basic publishers and subscribers
- Configure Quality of Service (QoS) policies

**Lab**:
- Install ROS 2 Humble on Ubuntu 22.04
- Create first "Hello World" publisher/subscriber
- Visualize ROS 2 graph with `rqt_graph`

**Deliverable**: Simple pub/sub node publishing sensor data

---

### **Week 2: ROS 2 Communication Patterns**

**Module**: 1 - The Robotic Nervous System
**Chapter**: [1.2 - ROS 2 Nodes and Communication](./module1/chapter1-2-nodes-communication)

**Learning Objectives**:
- Implement services for request-response patterns
- Create action servers for long-running tasks
- Understand node lifecycle management

**Lab**:
- Build service client/server for robot queries
- Implement action server for navigation task
- Test different QoS reliability modes

**Deliverable**: Multi-node system with topics, services, and actions

---

### **Week 3: ROS 2 System Configuration**

**Module**: 1 - The Robotic Nervous System
**Chapter**: [1.3 - Launch Files and Configuration](./module1/chapter1-3-launch-files)

**Learning Objectives**:
- Write Python launch files
- Configure parameters with YAML files
- Use namespaces for multi-robot systems

**Lab**:
- Create launch file starting 5+ nodes
- Configure node parameters from YAML
- Test namespace remapping

**Deliverable**: Configurable robot system with launch file

---

### **Week 4: Building ROS 2 Applications**

**Module**: 1 - The Robotic Nervous System
**Chapter**: [1.4 - Building ROS 2 Packages](./module1/chapter1-4-packages)

**Learning Objectives**:
- Create custom ROS 2 packages (C++ and Python)
- Write package.xml and CMakeLists.txt
- Build with colcon and manage dependencies

**Lab**:
- Create custom package with nodes, messages, services
- Build workspace with colcon
- Install and source workspace

**Deliverable**: Custom ROS 2 package with documentation

**Milestone**: 🎯 **Module 1 Complete** | **Midterm Exam** (optional)

---

### **Week 5: Digital Twin Concepts**

**Module**: 2 - The Digital Twin
**Chapter**: [2.1 - Introduction to Digital Twins](./module2/chapter2-1-digital-twin-intro)

**Learning Objectives**:
- Understand digital twin architecture
- Identify use cases for simulation
- Plan sim-to-real transfer strategy

**Lab**:
- Install Gazebo Harmonic or Isaac Sim
- Explore pre-built robot models
- Set up basic world environment

**Deliverable**: Running Gazebo/Isaac Sim with robot model

---

### **Week 6: Gazebo Simulation**

**Module**: 2 - The Digital Twin
**Chapter**: [2.2 - Gazebo Simulation Fundamentals](./module2/chapter2-2-gazebo-fundamentals)

**Learning Objectives**:
- Import URDF models into Gazebo
- Configure physics engines (DART, ODE, Bullet)
- Simulate sensors (camera, IMU, LiDAR)

**Lab**:
- Import humanoid URDF model
- Add camera and IMU sensors
- Publish sensor data to ROS 2 topics

**Deliverable**: Simulated humanoid with functional sensors

---

### **Week 7: Unity Robotics Integration**

**Module**: 2 - The Digital Twin
**Chapter**: [2.3 - Unity for Robotics](./module2/chapter2-3-unity-robotics)

**Learning Objectives**:
- Set up Unity-ROS 2 bridge (ROS-TCP-Connector)
- Import URDF models into Unity
- Visualize robot state in Unity

**Lab**:
- Install Unity and Robotics Hub packages
- Connect Unity to ROS 2 via TCP
- Synchronize robot pose between ROS 2 and Unity

**Deliverable**: Unity digital twin mirroring ROS 2 robot

---

### **Week 8: Vision and SLAM**

**Module**: 2 - The Digital Twin
**Chapter**: [2.4 - Sensor Simulation and VSLAM](./module2/chapter2-4-sensors-vslam)

**Learning Objectives**:
- Simulate depth cameras (RealSense D435)
- Implement Visual SLAM (ORB-SLAM3 or Cartographer)
- Map environments autonomously

**Lab**:
- Add depth camera to simulated robot
- Run Visual SLAM in Gazebo
- Generate 3D map of environment

**Deliverable**: SLAM-based autonomous navigation

**Milestone**: 🎯 **Module 2 Complete**

---

### **Week 9: NVIDIA Isaac Ecosystem**

**Module**: 3 - The AI-Robot Brain
**Chapter**: [3.1 - NVIDIA Isaac Overview](./module3/chapter3-1-isaac-overview)

**Learning Objectives**:
- Understand Isaac SDK, Isaac Sim, Isaac ROS architecture
- Install Isaac packages
- Run first Isaac ROS node

**Lab**:
- Install Isaac Sim (or use cloud instance)
- Install Isaac ROS packages
- Test Isaac-ROS 2 bridge with example

**Deliverable**: Isaac Sim environment with ROS 2 integration

---

### **Week 10: AI Perception Pipeline**

**Module**: 3 - The AI-Robot Brain
**Chapter**: [3.2 - Isaac Perception Pipeline](./module3/chapter3-2-isaac-perception)

**Learning Objectives**:
- Deploy TensorRT-optimized DNNs
- Run object detection (YOLOv5/v8)
- Measure inference performance (FPS, latency)

**Lab**:
- Download pre-trained YOLO model
- Convert to TensorRT engine
- Deploy with Isaac ROS DNN inference node
- Test on camera feed

**Deliverable**: Real-time object detection (20+ FPS)

---

### **Week 11: Navigation and Manipulation**

**Module**: 3 - The AI-Robot Brain
**Chapter**: [3.3 - Isaac Manipulation and Navigation](./module3/chapter3-3-isaac-manipulation-nav)

**Learning Objectives**:
- Integrate Isaac Visual SLAM with Nav2
- Use cuMotion for GPU-accelerated planning
- Implement autonomous navigation

**Lab**:
- Set up Nav2 with Isaac SLAM
- Configure costmaps and planners
- Test navigation to waypoints

**Deliverable**: Autonomous navigation in Isaac Sim

**Milestone**: 🎯 **Module 3 Complete**

---

### **Week 12: Vision-Language-Action Models**

**Module**: 4 - Vision-Language-Action
**Chapters**:
- [4.1 - Introduction to VLA](./module4/chapter4-1-vla-intro)
- [4.2 - LLM Integration](./module4/chapter4-2-llm-integration)
- [4.3 - Whisper Voice Commands](./module4/chapter4-3-whisper-voice)

**Learning Objectives**:
- Understand VLA architectures (RT-1, RT-2, OpenVLA)
- Integrate LLM APIs (GPT-4, Claude) with ROS 2
- Use Whisper for speech-to-text

**Lab**:
- Set up OpenAI or Anthropic API keys
- Create ROS 2 service for LLM task planning
- Integrate Whisper for voice input
- Test: "Pick up the red object" → LLM plan → ROS 2 actions

**Deliverable**: Voice-controlled robot with LLM reasoning

---

### **Week 13: End-to-End VLA System & Final Project**

**Module**: 4 - Vision-Language-Action
**Chapter**: [4.4 - End-to-End VLA System](./module4/chapter4-4-vla-system)

**Learning Objectives**:
- Integrate all modules into complete system
- Deploy VLA pipeline: Voice → LLM → Perception → Action
- Evaluate system performance

**Lab / Final Project**:
Build complete VLA-powered humanoid system:

1. **Input**: Voice command via Whisper
   - "Pick up the red cup and place it on the blue table"

2. **Planning**: GPT-4 decomposes task
   - Step 1: Detect red cup (Isaac perception)
   - Step 2: Plan grasp approach (cuMotion)
   - Step 3: Execute pick (ROS 2 control)
   - Step 4: Detect blue table
   - Step 5: Plan place motion
   - Step 6: Execute place

3. **Execution**: Robot performs task in simulation

4. **Visualization**: Unity shows real-time progress

**Deliverable**:
- **Final Project Demo** (live or recorded video)
- **Project Report** (architecture, challenges, results)
- **Code Repository** (GitHub with README)

**Milestone**: 🎯 **Module 4 Complete** | **Course Complete** 🎉

---

## Assessment Schedule

| Week | Assessment | Weight | Description |
|------|------------|--------|-------------|
| 1-13 | Weekly Labs | 40% | Deliverables submitted each week |
| 4 | Midterm Exam (optional) | 20% | ROS 2 written + practical |
| 13 | Final Project | 30% | VLA system demo + report |
| 13 | Final Exam (optional) | 10% | Cumulative written exam |

---

## Learning Outcomes by Module

### After Module 1 (Week 4)
✅ Create ROS 2 nodes in Python and C++
✅ Implement pub/sub, services, actions
✅ Write launch files and configure parameters
✅ Build custom packages with colcon

### After Module 2 (Week 8)
✅ Simulate robots in Gazebo and Unity
✅ Model robots with URDF
✅ Simulate sensors (camera, IMU, LiDAR)
✅ Implement Visual SLAM

### After Module 3 (Week 11)
✅ Deploy TensorRT-optimized DNNs
✅ Run GPU-accelerated perception (object detection, SLAM)
✅ Implement autonomous navigation with Isaac + Nav2
✅ Understand edge deployment (Jetson)

### After Module 4 (Week 13)
✅ Integrate LLMs for task planning
✅ Use Whisper for voice control
✅ Deploy VLA models (RT-2, OpenVLA)
✅ Build end-to-end voice-controlled robot system

---

## Flexible Scheduling Options

### Accelerated (8 Weeks)
Combine weeks: 1+2, 3+4, 5+6, 7+8, 9+10, 11, 12+13

### Extended (16 Weeks)
Add extra weeks for:
- Week 5: ROS 2 review and practice
- Week 9: Digital twin deep dive
- Week 13: Isaac RL (optional Module 3.4)
- Week 16-17: Final project development

### Self-Paced (Variable)
Follow at your own speed, but maintain sequence

---

## Prerequisites Check

Before starting, ensure you have:

**Software**:
- [ ] Ubuntu 22.04 LTS installed
- [ ] ROS 2 Humble installed
- [ ] Python 3.10+ and pip
- [ ] Git configured

**Skills**:
- [ ] Python or C++ programming
- [ ] Linux command line basics (cd, ls, apt, vim/nano)
- [ ] Version control (git add, commit, push)
- [ ] Basic calculus and linear algebra

**Hardware** (Minimum):
- [ ] 16GB RAM
- [ ] 50GB free disk space
- [ ] Internet connection for package installation

**Hardware** (Recommended for Modules 3-4):
- [ ] NVIDIA GPU (RTX 4070 Ti or better)
- [ ] 32GB RAM
- [ ] 100GB+ free disk space

---

## Weekly Time Commitment

| Activity | Hours/Week |
|----------|------------|
| Pre-lecture reading | 2-3 |
| Lecture attendance | 2 |
| Lab session | 2-3 |
| Homework/exercises | 3-4 |
| **Total** | **9-12 hours** |

---

## Office Hours & Support

**When to Attend Office Hours**:
- Stuck on lab for >1 hour
- Conceptual confusion after lecture
- Need help debugging code
- Questions about final project

**How to Get Help**:
1. Try debugging yourself (15-30 min)
2. Check troubleshooting sections
3. Search ROS Discourse / Stack Overflow
4. Post question with error details
5. Attend office hours as last resort

---

## Final Project Guidelines

### Requirements
- **Functionality** (40%): System works as specified
- **Integration** (20%): All modules integrated (ROS 2, simulation, Isaac, VLA)
- **Code Quality** (15%): Clean, documented, tested
- **Demo** (15%): Live demo or clear video
- **Report** (10%): Written explanation of architecture and results

### Deliverables
1. GitHub repository with code
2. README with setup instructions
3. Demo video (3-5 minutes)
4. Project report (5-10 pages)

### Grading Rubric
Available in course syllabus and final project specification document.

---

**Ready to start?** Proceed to **[Module 1: The Robotic Nervous System](./module1/intro)**
