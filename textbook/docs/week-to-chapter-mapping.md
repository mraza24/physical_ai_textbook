# Week-to-Chapter Mapping: 13-Week Course

**Purpose**: Map the textbook chapters to a 13-week semester course.

---

## Course Structure Overview

**Total Weeks**: 13
**Modules**: 4
**Chapters**: 16
**Format**: 3 hours/week (2 hours lecture + 1 hour lab)

---

## Weekly Schedule

### **Week 1: Introduction & ROS 2 Fundamentals**
- **Chapter**: 1.1 - ROS 2 Fundamentals
- **Topics**: ROS 2 architecture, nodes, topics, pub/sub
- **Lab**: Install ROS 2, create first publisher/subscriber
- **Deliverables**: Hello World ROS 2 node

---

### **Week 2: ROS 2 Communication Patterns**
- **Chapter**: 1.2 - ROS 2 Nodes and Communication
- **Topics**: Services, actions, QoS policies, lifecycle nodes
- **Lab**: Implement service client/server, action server
- **Deliverables**: Multi-node communication system

---

### **Week 3: ROS 2 System Configuration**
- **Chapter**: 1.3 - Launch Files and Configuration
- **Topics**: Launch files (Python), parameters, namespaces
- **Lab**: Create launch files for multi-node systems
- **Deliverables**: Configurable robot system with parameters

---

### **Week 4: Building ROS 2 Applications**
- **Chapter**: 1.4 - Building ROS 2 Packages
- **Topics**: Workspace setup, colcon, package.xml, CMakeLists.txt
- **Lab**: Create custom ROS 2 package (C++ and Python)
- **Deliverables**: Custom package with nodes, messages, services
- **Milestone**: Module 1 Complete

---

### **Week 5: Digital Twin Concepts**
- **Chapter**: 2.1 - Introduction to Digital Twins
- **Topics**: Digital twin architecture, sim-to-real, benefits
- **Lab**: Set up Gazebo environment
- **Deliverables**: Basic Gazebo world with robot model

---

### **Week 6: Gazebo Simulation**
- **Chapter**: 2.2 - Gazebo Simulation Fundamentals
- **Topics**: URDF models, physics engines, sensor simulation
- **Lab**: Import humanoid URDF, simulate sensors (camera, IMU)
- **Deliverables**: Functioning humanoid simulation

---

### **Week 7: Unity Robotics Integration**
- **Chapter**: 2.3 - Unity for Robotics
- **Topics**: Unity-ROS 2 bridge, visualization, synthetic data
- **Lab**: Set up Unity-ROS 2 connection, visualize robot
- **Deliverables**: Unity digital twin synchronized with ROS 2

---

### **Week 8: Vision and SLAM**
- **Chapter**: 2.4 - Sensor Simulation and VSLAM
- **Topics**: Visual SLAM, ORB-SLAM3, RealSense simulation
- **Lab**: Implement VSLAM in Gazebo with camera feed
- **Deliverables**: SLAM-based navigation in simulation
- **Milestone**: Module 2 Complete

---

### **Week 9: NVIDIA Isaac Ecosystem**
- **Chapter**: 3.1 - NVIDIA Isaac Overview
- **Topics**: Isaac SDK, Isaac Sim, Isaac ROS architecture
- **Lab**: Install Isaac Sim, run first Isaac ROS node
- **Deliverables**: Isaac Sim environment with ROS 2 bridge

---

### **Week 10: AI Perception**
- **Chapter**: 3.2 - Isaac Perception Pipeline
- **Topics**: TensorRT, DNN inference, object detection (YOLO)
- **Lab**: Deploy YOLO model with TensorRT on Isaac ROS
- **Deliverables**: Real-time object detection system

---

### **Week 11: Navigation and Manipulation**
- **Chapter**: 3.3 - Isaac Manipulation and Navigation
- **Topics**: cuMotion, Nav2 integration, Nvblox mapping
- **Lab**: Implement autonomous navigation with Isaac
- **Deliverables**: Humanoid robot navigating in Isaac Sim
- **Milestone**: Module 3 Complete

---

### **Week 12: Vision-Language-Action Models**
- **Chapters**:
  - 4.1 - Introduction to VLA
  - 4.2 - LLM Integration for Robotics
  - 4.3 - Whisper for Voice Commands
- **Topics**: RT-1, RT-2, OpenVLA, LLM APIs, Whisper ASR
- **Lab**: Set up LLM API, implement voice command system
- **Deliverables**: Voice-controlled robot with LLM task planning

---

### **Week 13: End-to-End VLA System & Final Project**
- **Chapter**: 4.4 - End-to-End VLA System
- **Topics**: Full VLA pipeline, deployment, evaluation
- **Lab**: Integrate all modules into complete VLA system
- **Deliverables**: **Final Project Demo**
  - Voice command → LLM planning → Isaac perception → Robot execution
  - "Pick up the red cup and place it on the table"
- **Milestone**: Module 4 Complete, Course Complete

---

## Midterm Exam (Optional)

**Timing**: After Week 4 (Module 1 Complete)

**Topics**:
- ROS 2 architecture and communication
- Nodes, topics, services, actions
- Package development
- Launch files and configuration

**Format**: Written exam + practical coding

---

## Final Exam (Optional)

**Timing**: After Week 13 (All Modules Complete)

**Topics**:
- All modules (cumulative)
- Digital twin concepts
- Isaac perception and navigation
- VLA models and deployment

**Format**: Written exam + final project presentation

---

## Final Project Rubric

**Requirements** (100 points total):

1. **ROS 2 Integration (20 pts)**
   - Proper node architecture
   - Topic/service/action usage
   - Launch file configuration

2. **Simulation (20 pts)**
   - Gazebo or Isaac Sim environment
   - Sensor integration (camera, IMU)
   - Realistic physics

3. **AI Perception (20 pts)**
   - Object detection or SLAM
   - TensorRT optimization
   - Real-time performance (>10 FPS)

4. **VLA Integration (20 pts)**
   - Voice command input (Whisper)
   - LLM task planning
   - Action execution

5. **Demo & Documentation (20 pts)**
   - Live demonstration
   - Clear documentation
   - Code quality and comments

---

## Recommended Reading by Week

### Weeks 1-4 (Module 1)
- ROS 2 Official Documentation
- "Programming Robots with ROS 2" (Goebel et al.)

### Weeks 5-8 (Module 2)
- Gazebo Documentation
- Unity Robotics Hub Documentation

### Weeks 9-11 (Module 3)
- Isaac SDK Documentation
- "Probabilistic Robotics" (Thrun, Burgard, Fox)

### Weeks 12-13 (Module 4)
- RT-1, RT-2 papers
- OpenVLA documentation
- LLM API documentation

---

## Lab Equipment Requirements

### Per Student/Group
- **Workstation**: RTX 4070 Ti or better (for simulation)
- **Edge Device** (optional): Jetson Orin Nano
- **Sensor** (optional): RealSense D435/D455
- **Robot** (optional): Unitree Go2/G1, OP3, or Hiwonder

### Software
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic or Isaac Sim
- Unity (optional)
- Python 3.10+
- CUDA 12.x, TensorRT

---

## Assessment Weights

| Component | Weight |
|-----------|--------|
| Weekly Labs | 40% |
| Midterm Exam (optional) | 20% |
| Final Project | 30% |
| Final Exam (optional) | 10% |

---

**Status**: ✅ Complete. 13-week course mapped to 16 chapters.
