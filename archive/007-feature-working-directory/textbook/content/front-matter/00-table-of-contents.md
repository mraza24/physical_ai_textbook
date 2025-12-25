# Table of Contents: Front Matter

---

## Front Matter Navigation

| Section | Page |
|---------|------|
| [Title Page](#title-page) | i |
| [Copyright & Licensing](#copyright--licensing) | ii |
| [Dedication](#dedication) | iv |
| [Preface](#preface) | v |
| [How to Use This Book](#how-to-use-this-book) | vii |
| [13-Week Course Mapping](#course-mapping) | xi |
| [Hardware Overview](#hardware-overview) | xv |
| [Software Overview](#software-overview) | xviii |
| [Module 1 Introduction](#module-1-introduction) | xxiii |
| [Module 2 Introduction](#module-2-introduction) | xxvi |
| [Module 3 Introduction](#module-3-introduction) | xxix |
| [Module 4 Introduction](#module-4-introduction) | xxxii |
| [Module Navigation & Learning Paths](#module-navigation) | xxxv |

---

## Detailed Contents

### Title Page
- Book title: *Physical AI & Humanoid Robotics: A Practical Textbook*
- Module list (Modules 1-4)
- Target audience
- Edition and publication information
- Open Educational Resource license

**File**: `00-title-page.md` | **Page**: i

---

### Copyright & Licensing
- Copyright notice
- Creative Commons CC BY-NC-SA 4.0 license (full terms)
- Third-party materials attribution (ROS 2, Gazebo, Unity, Isaac, VLA models)
- Code examples license (MIT)
- Citation formats (APA and BibTeX)
- Disclaimer
- Errata information

**File**: `01-copyright.md` | **Page**: ii-iii

---

### Dedication
- To students, educators, and researchers
- To the open-source community
- To the humanoid robots of tomorrow

**File**: `02-dedication.md` | **Page**: iv

---

### Preface
- The Physical AI transformation
- Gap in traditional robotics curricula
- Target audience (graduate students, early-career engineers)
- Pedagogical philosophy (relentlessly practical, scaffolded learning)
- Why now? (ROS 2 Humble LTS, Isaac Sim 5.0, VLA maturity, LLM reasoning)
- Open Educational Resource commitment
- Acknowledgments

**File**: `03-preface.md` | **Page**: v-vi

**Word Count**: 688 words

---

### How to Use This Book
#### Book Structure
- Four modules, sixteen chapters
- Module dependencies (M2→M1, M3→M1&2, M4→M1-3)
- Chapter elements (7 standard sections per chapter)

#### Prerequisites
- Python programming, Linux command line, Git basics
- Mathematical fundamentals (linear algebra, probability)
- Readiness checklist

#### Weekly Pacing (13-Week Course)
- Weeks 1-2: Setup & Introduction
- Weeks 3-5: Module 1 (ROS 2)
- Weeks 6-7: Module 2 (Digital Twin)
- Weeks 8-10: Module 3 (Isaac)
- Weeks 11-13: Module 4 (VLA)

#### Self-Paced Learning
- Accelerated path (8 weeks)
- Deep dive path (16 weeks)
- Focus tracks (simulation-focused, AI-focused, end-to-end)

#### Exercise System
- Difficulty levels (Easy, Medium, Hard)
- How to approach exercises
- Where to get help

#### Hardware & Software Requirements
- Minimal setup: Laptop + cloud GPU ($0-50)
- Enhanced setup: Desktop + RTX 4070 Ti ($600-800)
- Complete setup: + Jetson + RealSense ($1300-1500)

#### Tips for Success
- Follow linear path, type code, run examples immediately
- Complete at least 1 exercise per chapter
- Test in simulation first before hardware

#### For Instructors
- Course customization (lecture, lab, flipped classroom, hybrid)
- Suggested assessments (weekly labs 40%, VLA report 10%, voice demo 10%, final project 30%, participation 10%)
- Checkpoint recommendations (Weeks 5, 7, 10)
- Lab infrastructure guidance

**File**: `04-how-to-use-this-book.md` | **Page**: vii-x

---

### Course Mapping
#### Quick Reference Table
- 13-week schedule with Module/Topics/Reading/Lab columns

#### Detailed Week-by-Week Breakdown
- **Week 1**: Setup & installation (Ubuntu, ROS 2, Gazebo)
- **Week 2**: Hardware/software landscape verification
- **Week 3**: ROS 2 fundamentals, computational graph
- **Week 4**: ROS 2 services and actions
- **Week 5**: Launch files, packages, workspaces
  - **Checkpoint 1**: Module 1 quiz + pub-sub demo
- **Week 6**: Digital twin, Gazebo simulation
- **Week 7**: Unity visualization, Visual SLAM
  - **Checkpoint 2**: Module 2 quiz + Gazebo demo
- **Week 8**: Isaac overview, perception pipeline
- **Week 9**: Navigation, Nav2 integration
- **Week 10**: Reinforcement learning, Isaac Gym
  - **Checkpoint 3**: Module 3 quiz + Isaac perception demo
- **Week 11**: VLA concepts, architecture comparison report
- **Week 12**: LLM integration, Whisper voice commands
- **Week 13**: End-to-end VLA system, final project
  - **Final**: "Pick up the red cup" demo

#### Grading Breakdown
- Weekly Labs (Weeks 3-10): 40% (8 labs @ 5% each)
- Week 11 VLA Report: 10%
- Week 12 Voice Control Demo: 10%
- Week 13 Final Project: 30%
- Participation & Quizzes: 10%

#### Milestones & Checkpoints
- 3 major checkpoints (Weeks 5, 7, 10)
- Final checkpoint (Week 13)

#### Flexibility & Adaptation
- Accelerated learners, students needing support
- Hardware limitations (cloud GPU alternatives)

**File**: `05-course-mapping.md` | **Page**: xi-xiv

---

### Hardware Overview
#### Hardware Tiers
- **Tier 1 (Minimal)**: Laptop + cloud GPU (~$0-50 total)
- **Tier 2 (Enhanced)**: Desktop + RTX 4070 Ti GPU (~$600-800)
- **Tier 3 (Complete)**: Tier 2 + Jetson Orin Nano + RealSense (~$1300-1500)

#### Tier 1: Minimal Setup
- Laptop/desktop requirements (4+ cores, 16GB RAM, 50GB storage)
- Cloud GPU access (AWS, GCP, Azure, Lambda Labs)
- Estimated usage: $20-50 for entire course

#### Tier 2: Enhanced Setup
- Desktop GPU (RTX 4070 Ti, RTX 4080, RTX 4090)
- Pros: Faster iteration, long-term savings, no internet dependency
- Cons: Upfront cost, desktop-only

#### Tier 3: Complete Setup
- NVIDIA Jetson Orin Nano (8GB) for edge AI ($499)
- Intel RealSense D435/D455 depth camera ($180-350)
- Optional enrichment (not required for textbook completion)

#### Humanoid Robot Platforms
- Instructor demos only (Robotis OP3, Unitree H1, NAO, TurtleBot3)

#### Hardware Decision Matrix
- 6 questions across 3 tiers to guide selection

**File**: `06-hardware-overview.md` | **Page**: xv-xvii

---

### Software Overview
#### Software Stack Architecture
- **Layer 0**: Ubuntu 22.04, NVIDIA Drivers, CUDA
- **Layer 1**: ROS 2 Humble (LTS until May 2027)
- **Layer 2**: Gazebo, Unity, Isaac Sim
- **Layer 3**: Isaac SDK, TensorRT, PyTorch
- **Layer 4**: OpenVLA, LLMs, Whisper

#### Layer Details
- **Operating System**: Ubuntu 22.04 LTS (support until April 2027)
- **ROS 2 Humble**: DDS middleware, Python & C++, colcon build system
- **Gazebo 11**: Open-source physics (ODE, Bullet, Simbody, DART), URDF support
- **Unity**: Photorealistic rendering, ROS-TCP-Connector
- **Isaac Sim 5.0**: RTX ray tracing, PhysX 5.0, ROS 2 native
- **Isaac SDK**: Isaac ROS (TensorRT-optimized nodes), Isaac Gym (RL)
- **TensorRT 8.6+**: 5-50× inference acceleration, INT8 quantization
- **OpenVLA**: 7B parameters, 970k demos, MIT licensed
- **LLMs**: GPT-4, Claude, Gemini (task planning)
- **Whisper**: Speech-to-text, 99 languages, real-time on GPU

#### Software Version Matrix
- 11 software components with versions, release dates, support timelines

#### Installation Order
- **Week 1**: Ubuntu, CUDA, ROS 2, Gazebo
- **Week 8**: Isaac SDK, TensorRT
- **Week 11**: PyTorch, Whisper, OpenVLA, LLM API keys

#### Troubleshooting Quick Reference
- 6 common issues with solutions

#### Licensing Summary
- 12 components with license type, cost, commercial use

**File**: `07-software-overview.md` | **Page**: xviii-xxii

---

### Module 1 Introduction: ROS 2
#### The Communication Challenge
- Coordination problem in multi-component robot systems
- Why ROS 2? (de facto standard, used by Boston Dynamics, NASA, Waymo)

#### What You Will Learn
- Computational graph architecture
- Publisher-subscriber communication
- Service-action architectures
- Multi-node system configuration

#### What You Will Build
- Publisher-subscriber system (30 Hz camera → detector → visualizer)
- Service system (robot status query, mode switcher)
- Action system (long-running "Move to Goal" with feedback)
- Multi-node application (5+ nodes with launch file)

#### Prerequisites
- Basic Python, Linux CLI, Git, math fundamentals

#### Time Commitment
- 3 weeks (Weeks 3-5), ~25-30 hours total

#### Success Criteria
- 7 checkpoints before Module 2

#### Chapter Roadmap
- **Chapter 1.1**: ROS 2 Fundamentals (Week 3)
- **Chapter 1.2**: Nodes & Communication (Week 4)
- **Chapter 1.3**: Launch Files & Configuration (Week 5a)
- **Chapter 1.4**: Building Packages & Workspaces (Week 5b)

**File**: `08-module1-introduction.md` | **Page**: xxiii-xxv

---

### Module 2 Introduction: Digital Twin
#### The Physical Robot Problem
- Risk (robot damage), availability (shared hardware), iteration speed
- Digital twin solution: Build → Simulate → Test → Deploy

#### What Is a Digital Twin?
- Virtual replica: geometry (URDF), physics, sensors, actuators
- Gazebo (open-source), Unity (photorealistic), Isaac Sim (GPU-accelerated)

#### What You Will Learn
- Digital twin concepts and sim-to-real transfer
- Gazebo fundamentals (SDF worlds, physics engines, sensors)
- Unity for robotics (Robotics Hub, TCP-based ROS 2)
- Sensors & Visual SLAM (depth cameras, ORB-SLAM3, Cartographer)

#### What You Will Build
- Gazebo humanoid simulation with camera, IMU, joint controllers
- URDF robot model (custom geometry, sensors)
- Unity visualization (photorealistic rendering + ROS 2)
- Visual SLAM system (ORB-SLAM3/Cartographer on simulated RealSense)

#### Prerequisites
- ✅ Module 1 complete, URDF familiarity, 3D spatial reasoning

#### Time Commitment
- 2 weeks (Weeks 6-7), ~20-24 hours total

#### Success Criteria
- 7 checkpoints before Module 3

#### Chapter Roadmap
- **Chapter 2.1**: Digital Twin Concepts (Week 6a)
- **Chapter 2.2**: Gazebo Fundamentals (Week 6b)
- **Chapter 2.3**: Unity for Robotics (Week 7a)
- **Chapter 2.4**: Sensors & Visual SLAM (Week 7b)

**File**: `09-module2-introduction.md` | **Page**: xxvi-xxviii

---

### Module 3 Introduction: Isaac
#### The Perception Bottleneck
- CPU inference: 500-1000ms per frame (1-2 FPS)
- GPU-accelerated (TensorRT): 10-20ms per frame (50-100 FPS)
- 20-50× speedup unlocks real-time robotics

#### What Is NVIDIA Isaac?
- **Isaac ROS**: Pre-built ROS 2 nodes (perception, SLAM, navigation)
- **Isaac Sim**: GPU-accelerated simulator (RTX ray tracing, PhysX 5.0)
- **Isaac Gym**: RL environment (1000s of parallel simulations)

#### What You Will Learn
- Isaac ecosystem overview (ROS, Sim, Gym integration)
- Perception pipeline (object detection, segmentation, TensorRT)
- Manipulation & navigation (grasp planning, Nav2, costmaps)
- Reinforcement learning (PPO, bipedal locomotion, sim-to-real)

#### What You Will Build
- Real-time object detector (YOLO at 50+ FPS with Isaac ROS)
- Autonomous navigation system (Isaac perception + Nav2)
- Semantic segmentation pipeline (pixel-level scene understanding)
- Bipedal locomotion policy (RL in Isaac Gym)

#### Prerequisites
- ✅ Modules 1-2 complete, GPU access (RTX 3060+ or cloud), CUDA installed

#### Time Commitment
- 3 weeks (Weeks 8-10), ~30-40 hours (includes RL training)

#### Success Criteria
- 7 checkpoints before Module 4

#### Chapter Roadmap
- **Chapter 3.1**: Isaac Overview (Week 8a)
- **Chapter 3.2**: Isaac Perception Pipeline (Week 8b)
- **Chapter 3.3**: Manipulation & Navigation (Week 9)
- **Chapter 3.4**: Reinforcement Learning (Week 10)

**File**: `10-module3-introduction.md` | **Page**: xxix-xxxi

---

### Module 4 Introduction: VLA
#### The Embodied Intelligence Challenge
- Traditional robots: Explicit programs for every scenario
- VLA robots: Learned policies from 970k+ demonstrations
- Natural language control: "Pick up the red cup" → execution

#### What Are VLA Models?
- Map (Camera Image, Language Instruction) → Robot Actions
- RT-1 (2022), RT-2 (2023), OpenVLA (2024)
- Transfer learning from vision-language models (CLIP, LLaMA)

#### What You Will Learn
- VLA concepts & architectures (RT-1, RT-2, OpenVLA, GR00T)
- LLM integration (task planning, GPT-4/Claude/Gemini APIs)
- Whisper voice commands (speech-to-text, ROS 2 node)
- End-to-end VLA system (voice → perception → reasoning → action)

#### What You Will Build
- Voice-controlled robot (Whisper → ROS 2 commands)
- LLM task planner ("Set the table" → step-by-step execution)
- **"Pick Up the Red Cup" Demo**: Full VLA pipeline (voice, vision, reasoning, action)
- Comparative VLA analysis (RT-1 vs RT-2 vs OpenVLA report)

#### Prerequisites
- ✅ Modules 1-3 complete, GPU access, LLM API (~$5 budget), PyTorch basics

#### Time Commitment
- 3 weeks (Weeks 11-13), ~35-40 hours (includes final project)

#### Success Criteria
- 7 checkpoints for course completion

#### Chapter Roadmap
- **Chapter 4.1**: VLA Concepts (Week 11)
- **Chapter 4.2**: LLM Integration (Week 12a)
- **Chapter 4.3**: Whisper Voice Commands (Week 12b)
- **Chapter 4.4**: End-to-End VLA System (Week 13)

#### Final Project
- "Pick up the red cup" voice-to-action demo
- Deliverables: Demo video, code repository, project report

**File**: `11-module4-introduction.md` | **Page**: xxxii-xxxiv

---

### Module Navigation & Learning Paths
#### Complete Module Flow Diagram
- Mermaid diagram: Setup → M1 → M2/M3 → M4 → Final Project
- All dependencies visualized (M2→M1, M3→M1&2, M4→M1-3)

#### Dependency Explanation
- Why M2 depends on M1 (ROS 2 topics for simulation)
- Why M3 depends on M1&2 (Isaac ROS + simulation testing)
- Why M4 depends on M1-3 (VLA integrates all layers)

#### Alternative Learning Paths
- **Path 1: Linear** (13 weeks, recommended)
- **Path 2: Accelerated** (8 weeks, fast-paced)
- **Path 3: Simulation-Focused** (emphasize Gazebo/Unity/Isaac Sim)
- **Path 4: AI-Focused** (emphasize perception and VLA)
- **Path 5: Self-Paced Deep Dive** (16 weeks, thorough)

#### Key Concepts by Module
- Hierarchical breakdown of concepts per module

#### Checkpoint Questions
- Before M2, M3, M4, and course completion

#### Time Investment Summary
- 122 hours total: 44h reading + 63h labs + 15h final project

#### Module Interdependency Matrix
- 4×4 table showing which modules require which

**File**: `12-module-navigation-diagram.md` | **Page**: xxxv-xxxviii

---

## Quick Navigation Guide

**Getting Started?**
→ Start with [Preface](#preface), then [How to Use This Book](#how-to-use-this-book)

**Planning your schedule?**
→ See [13-Week Course Mapping](#course-mapping)

**Choosing hardware?**
→ Read [Hardware Overview](#hardware-overview) for 3-tier comparison

**Need software setup?**
→ Check [Software Overview](#software-overview) + Appendix B

**Want to skip ahead?**
→ See [Module Navigation](#module-navigation) for prerequisites

---

## Main Content Structure (After Front Matter)

### Module 1: ROS 2 (Chapters 1.1 - 1.4)
- 4 chapters, 3 weeks, Weeks 3-5

### Module 2: Digital Twin (Chapters 2.1 - 2.4)
- 4 chapters, 2 weeks, Weeks 6-7

### Module 3: Isaac (Chapters 3.1 - 3.4)
- 4 chapters, 3 weeks, Weeks 8-10

### Module 4: VLA (Chapters 4.1 - 4.4)
- 4 chapters, 3 weeks, Weeks 11-13

### Back Matter
- Glossary (≥40 terms)
- References (≥20 sources, APA 7)
- 7 Appendices (A-G)
- 3 Master Diagrams
- Index

---

**Total Front Matter**: 13 sections, ~15,000 words, ~40 pages (estimated)

**Ready to begin? → Chapter 1.1: ROS 2 Fundamentals**
