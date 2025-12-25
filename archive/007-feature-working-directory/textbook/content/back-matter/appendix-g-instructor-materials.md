# Appendix G: Instructor Materials

> **Teaching resources, course structures, lab assignments, and assessment rubrics for instructors using this textbook.**

**Access Note**: Full materials (complete exercise solutions, answer keys, exam banks) available upon request to verified instructors at [instructor-access@textbook-website.com].

---

## Course Structures

### Option 1: 13-Week Undergraduate Course (3 credits, 3 hours/week)

**Target**: CS/EE juniors/seniors with Python experience, no robotics background

**Weekly Structure**:
| Week | Module | Topics | Lab | Assignment Due |
|------|--------|--------|-----|----------------|
| 1 | Intro | Course overview, setup (Appendix A) | ROS 2 installation | - |
| 2 | 1.1 | ROS 2 fundamentals, pub/sub | Ch1.1 exercises | - |
| 3 | 1.2 | Services, actions | Multi-node system lab | HW1 (Ch1.1) |
| 4 | 1.3-1.4 | Launch files, workspaces | Custom package lab | HW2 (Ch1.2) |
| 5 | 2.1-2.2 | Digital twins, Gazebo | Robot simulation lab | - |
| 6 | 2.3-2.4 | Unity, sensor modeling | VSLAM comparison lab | HW3 (Ch1.3-1.4) |
| 7 | **Midterm Exam** | Modules 1-2 | - | Project proposal |
| 8 | 3.1-3.2 | Isaac Sim, TensorRT | GPU perception lab | - |
| 9 | 3.3 | Navigation, manipulation | Nav2 + Nvblox lab | HW4 (Ch2) |
| 10 | 3.4 | Reinforcement learning | RL training lab | - |
| 11 | 4.1-4.2 | VLA, Whisper | Voice control lab | HW5 (Ch3) |
| 12 | 4.3-4.4 | LLM planning, integration | End-to-end VLA lab | - |
| 13 | **Final Project Presentations** | Student demos | - | Final project report |

**Grading Breakdown**:
- Homework (5 × 10% = 50%): Programming assignments
- Midterm Exam (20%): Written + coding problems
- Final Project (25%): Open-ended robot application
- Participation (5%): Lab attendance, forum engagement

---

### Option 2: 8-Week Intensive Graduate Seminar

**Target**: MS/PhD students in robotics, focus on research-level topics

**Accelerated Pace** (2 meetings/week, 3 hours each):
- **Week 1**: Module 1 (ROS 2) speedrun - assumes prior exposure
- **Week 2**: Module 2 (Simulation) with emphasis on sim-to-real research
- **Week 3-4**: Module 3 (Isaac) + RL paper discussions
- **Week 5-6**: Module 4 (VLA) + recent papers (RT-2, OpenVLA, GR00T)
- **Week 7-8**: Research project implementation + presentation

**Assessment**:
- No homework (assumes self-directed learning)
- Paper presentations (30%): Each student presents 2 papers
- Research project (60%): Novel contribution (e.g., new VLA architecture, domain randomization technique)
- Class participation (10%): Technical discussions

---

### Option 3: Self-Paced Online Course (MOOCs)

**Platform**: Coursera, edX, Udacity

**Structure**:
- 4 modules = 4 courses (can be taken independently)
- Each module: 4-5 weeks of video lectures + quizzes + programming labs
- Capstone project: Multi-module integration

**Auto-Grading**:
- Multiple-choice quizzes (concepts)
- Code submission with unit tests (programming)
- Peer review for open-ended projects

---

## Lab Equipment Recommendations

### Tier 1: Simulation-Only Course (Budget: $0-500/student)
**Minimum**:
- Students use personal laptops (8GB RAM minimum)
- Cloud GPU credits (AWS g5.xlarge, $50-75/student/semester)
- ROS 2 + Gazebo (no Isaac Sim due to GPU requirements)

**Deliverables**: All exercises completed in simulation

---

### Tier 2: Shared Physical Robot Lab (Budget: $10k-20k total)
**Equipment** (for 20-student course):
- 4× TurtleBot 4 robots ($2,500 each = $10k)
- 4× Workstations with RTX 4060 Ti ($2,000 each = $8k)
- Wi-Fi 6 access points, shared tools
- Students work in groups of 5, rotate robot access

**Lab Sessions**: 2 hours/week, 4 groups in parallel

---

### Tier 3: Individual Robot Kits (Budget: $1k-2k/student)
**Equipment** (per student):
- RTX 4060 Ti workstation or cloud GPU access
- Optional: Jetson Orin Nano kit ($500) + RealSense camera ($350)
- Students can build custom 3D-printed robots ($200 in parts)

**Deliverables**: Final project deployed on physical robot

---

## Assessment Rubrics

### Programming Assignment Rubric (100 points)

| Criterion | Excellent (90-100) | Good (75-89) | Satisfactory (60-74) | Needs Improvement (<60) |
|-----------|-------------------|--------------|----------------------|-------------------------|
| **Functionality** (40%) | All requirements met, edge cases handled | Core functionality works, minor issues | Partially working, missing features | Does not run or major bugs |
| **Code Quality** (20%) | Clean, modular, PEP8 style, error handling | Readable with minor style issues | Inconsistent style, minimal error handling | Unreadable, no error handling |
| **Documentation** (15%) | Clear docstrings, README, inline comments | Basic documentation present | Minimal documentation | No documentation |
| **Performance** (15%) | Meets all latency/throughput targets | Meets most targets, minor issues | Slow but functional | Unacceptably slow or fails |
| **Testing** (10%) | Comprehensive unit tests, edge cases | Basic tests provided | Minimal testing | No tests |

**Example**: Module 1 HW1 (Ch1.1 Exercises)
- Exercise 1.1.1 pub/sub: 20 points
- Exercise 1.1.2 QoS comparison: 30 points (harder)
- Exercise 1.1.3 multi-node graph: 50 points (complex)

---

### Final Project Rubric (100 points)

| Criterion | Points | Description |
|-----------|--------|-------------|
| **Problem Definition** | 10 | Clear motivation, requirements, success criteria |
| **Technical Implementation** | 40 | ROS 2 architecture, integration of ≥2 modules, working system |
| **Innovation** | 15 | Novel approach, creative solution, or extension beyond course material |
| **Documentation** | 15 | Code documentation, system architecture diagram, usage instructions |
| **Demo/Presentation** | 10 | Live demo, clear explanation, answers to questions |
| **Report** | 10 | Written report (5-10 pages) with results, analysis, limitations |

**Project Ideas** (by difficulty):
- **Easy**: Voice-controlled mobile robot (Modules 1, 4)
- **Medium**: Warehouse navigation with obstacle avoidance (Modules 1-3)
- **Hard**: Tabletop manipulation with VLA (all modules)
- **Research**: Novel VLA architecture, sim-to-real transfer technique

---

## Exam Question Banks

### Sample Midterm Questions (Modules 1-2)

**Multiple Choice** (2 points each):
1. Which DDS QoS policy controls whether messages are retransmitted on packet loss?
   - A. Durability
   - B. Reliability ✓
   - C. Deadline
   - D. Liveliness

2. What is the primary advantage of domain randomization in sim-to-real transfer?
   - A. Faster simulation speed
   - B. Improved policy robustness to real-world variations ✓
   - C. Reduced training time
   - D. Better graphics quality

**Short Answer** (10 points each):
3. Explain the difference between synchronous services and asynchronous actions in ROS 2. When would you use each? (Expected: 3-5 sentences)

4. Describe the sim-to-real gap. Give two techniques to reduce it. (Expected: 4-6 sentences)

**Coding Problems** (20 points each):
5. Write a ROS 2 Python node that subscribes to `/scan` (LaserScan message) and publishes `/obstacle_detected` (Bool) if any range value is <0.5 meters. Include error handling.

6. Write a Gazebo SDF snippet for a wheeled robot with: (1) box chassis (0.5×0.3×0.2m, mass 10kg), (2) two wheels (radius 0.1m), (3) differential drive plugin.

---

### Sample Final Exam Questions (All Modules)

**Conceptual** (10 points each):
1. Compare TensorRT INT8 quantization vs FP32 inference: accuracy loss, speedup, memory usage, and use cases. (Expected: 6-8 sentences)

2. Explain the dual-system architecture in GR00T (System 1 vs System 2). Why is this useful for humanoid robots? (Expected: 5-7 sentences)

**Design Problem** (30 points):
3. Design a complete system architecture for a voice-controlled coffee-making robot. Include:
   - Hardware (compute, sensors, actuators)
   - Software stack (ROS 2 nodes, topics, services)
   - ML models (Whisper, LLM, VLA or alternatives)
   - Data flow diagram
   - Estimated latency breakdown

**Coding Problem** (40 points):
4. Implement a simplified ReAct loop in Python that controls a simulated robot arm to "clean a table". Provide:
   - Thought-Action-Observation loop structure
   - Mock functions: `detect_objects()`, `pick(obj)`, `place(obj, location)`
   - LLM prompt template
   - Max 5 iterations with success/failure detection

---

## Common Student Mistakes & How to Address

### 1. Not Sourcing ROS 2 Setup Files
**Symptom**: "ros2: command not found" or "Package not found"
**Fix**: Add to troubleshooting lecture, create shell check script
**Prevention**: Include in lab setup checklist, auto-check in assignments

### 2. Mixing Python 2 and Python 3
**Symptom**: Syntax errors (`print` statements, division), import failures
**Fix**: Emphasize Python 3.10 requirement in syllabus
**Prevention**: Use `#!/usr/bin/env python3` shebang, include version check in code template

### 3. Hardcoded Paths Instead of ROS 2 Package Paths
**Symptom**: Code works on student's machine, fails on autograder
**Fix**: Teach `ament_index_python.packages.get_package_share_directory()`
**Prevention**: Provide code templates with correct patterns, deduct points for hardcoded paths

### 4. Not Handling Node Shutdown Gracefully
**Symptom**: `Ctrl+C` leaves zombie processes, port conflicts
**Fix**: Teach signal handling, context managers (`rclpy.ok()` in loops)
**Prevention**: Include shutdown tests in autograder

### 5. Overfitting to Simulation
**Symptom**: Policy works perfectly in Isaac Sim, fails immediately on real robot
**Fix**: Lecture on domain randomization, sensor noise, action delays
**Prevention**: Require analysis of sim-to-real gap in project reports

---

## Guest Lecture Suggestions

Invite industry practitioners for 1-2 guest lectures:

1. **Robotics Engineer from Warehouse Automation Company**
   - Topics: Real-world challenges, Nav2 deployment at scale, fleet management
   - Timing: After Chapter 3.3 (Navigation)

2. **ML Researcher from VLA/LLM Robotics Lab**
   - Topics: State-of-the-art VLA models, future directions, research opportunities
   - Timing: During Module 4 or final week

3. **Embedded Systems Engineer (NVIDIA/Qualcomm)**
   - Topics: Jetson optimization, power management, TensorRT best practices
   - Timing: After Chapter 3.2 or 4.4 (Deployment)

---

## Supplementary Materials

**Provided to Instructors Upon Request**:
1. **Slide Decks** (400+ slides, Keynote/PowerPoint/PDF)
   - One deck per chapter, fully editable
   - Includes diagrams, code snippets, demo videos

2. **Complete Exercise Solutions** (Runnable code + explanations)
   - Python scripts, launch files, SDF models
   - Expected outputs, common variants

3. **Lab Handouts** (PDF, 1-2 pages each)
   - Step-by-step instructions for 13 labs
   - Pre-lab checklist, objectives, deliverables

4. **Exam Banks** (50 multiple-choice, 20 short answer, 10 coding problems)
   - Versions A/B/C for exam security
   - Answer keys with partial credit rubrics

5. **Autograder Scripts** (Python + Docker)
   - Unit tests for programming assignments
   - ROS 2 bag file validators for robot outputs

6. **Video Tutorials** (30+ hours)
   - Chapter summaries (10-15 min each)
   - Software installation walkthroughs
   - Common debugging scenarios

---

## Contact for Instructor Access

**Email**: instructor-access@textbook-website.com
**Required Information**:
- University name
- Course number and semester
- Proof of instructor status (university email or faculty page link)
- Brief description of how you plan to use the textbook

**Expected Response Time**: 2-3 business days

---

**Teaching resources designed for flexible adoption—adapt to your course structure and student level.**
