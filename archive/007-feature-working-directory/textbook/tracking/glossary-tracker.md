# Glossary Tracking System

**Purpose**: Track all technical terms as they are introduced in chapters, ensuring we reach ≥40 unique terms with clear definitions and chapter references.

**Target**: ≥40 technical terms (Spec FR-047, Success Criteria SC-002)

**Current Count**: 0 terms

---

## Instructions

1. **During Chapter Writing**: When you introduce a technical term, add it to this tracker
2. **Term Selection Criteria**:
   - Domain-specific (not general English)
   - Requires explanation for graduate students
   - Used in multiple contexts or chapters
3. **Placeholders**: Add term now, write definition later (Phase 7)
4. **Chapter References**: Note where term is first introduced and where it appears

---

## Tracked Terms

| # | Term | Module | Chapters | Definition Status | Notes |
|---|------|--------|----------|-------------------|-------|
| 001 | Node | 1 | 1.1, 1.2 | PENDING | ROS 2 concept, introduced in Chapter 1.1 |
| 002 | Topic | 1 | 1.1, 1.2 | PENDING | ROS 2 pub-sub communication |
| 003 | Service | 1 | 1.1, 1.2 | PENDING | ROS 2 request-response pattern |
| 004 | Action | 1 | 1.1, 1.2 | PENDING | ROS 2 long-running tasks |
| 005 | DDS | 1 | 1.1 | PENDING | Data Distribution Service middleware |
| 006 | URDF | 1, 2 | 1.2, 2.2 | PENDING | Unified Robot Description Format |
| 007 | Launch File | 1 | 1.3 | PENDING | ROS 2 system startup configuration |
| 008 | Parameter | 1 | 1.3 | PENDING | ROS 2 configuration values |
| 009 | Workspace | 1 | 1.4 | PENDING | ROS 2 development environment |
| 010 | Package | 1 | 1.4 | PENDING | ROS 2 code organization unit |
| 011 | Digital Twin | 2 | 2.1 | PENDING | Virtual replica of physical system |
| 012 | Simulation | 2 | 2.1, 2.2, 2.3 | PENDING | Computer-based robot environment |
| 013 | Gazebo | 2 | 2.2 | PENDING | Physics-based robot simulator |
| 014 | Unity | 2 | 2.3 | PENDING | Game engine for robotics visualization |
| 015 | VSLAM | 2 | 2.4 | PENDING | Visual Simultaneous Localization and Mapping |
| 016 | Sim-to-Real | 2 | 2.1, 2.4 | PENDING | Transferring from simulation to physical robot |
| 017 | Domain Randomization | 2 | 2.4 | PENDING | Varying simulation parameters for robustness |
| 018 | Isaac SDK | 3 | 3.1 | PENDING | NVIDIA robotics development kit |
| 019 | Isaac Sim | 3 | 3.1 | PENDING | NVIDIA GPU-accelerated simulator |
| 020 | Isaac ROS | 3 | 3.1 | PENDING | NVIDIA ROS 2 packages for AI |
| 021 | Perception | 3 | 3.2 | PENDING | Robot sensing and understanding environment |
| 022 | TensorRT | 3 | 3.2 | PENDING | NVIDIA deep learning inference optimizer |
| 023 | Nav2 | 3 | 3.3 | PENDING | ROS 2 navigation stack |
| 024 | Costmap | 3 | 3.3 | PENDING | Obstacle representation for navigation |
| 025 | Reinforcement Learning | 3 | 3.4 | PENDING | Learning through trial and error |
| 026 | Isaac Gym | 3 | 3.4 | PENDING | NVIDIA GPU-accelerated RL training |
| 027 | PPO | 3 | 3.4 | PENDING | Proximal Policy Optimization algorithm |
| 028 | VLA | 4 | 4.1, 4.2, 4.3, 4.4 | PENDING | Vision-Language-Action model |
| 029 | Embodied Intelligence | 4 | 4.1 | PENDING | AI grounded in physical interaction |
| 030 | LLM | 4 | 4.2 | PENDING | Large Language Model |
| 031 | Prompt Engineering | 4 | 4.2 | PENDING | Crafting effective LLM instructions |
| 032 | Task Planning | 4 | 4.2 | PENDING | Breaking down high-level goals |
| 033 | Whisper | 4 | 4.3 | PENDING | OpenAI speech-to-text model |
| 034 | Speech-to-Text | 4 | 4.3 | PENDING | Converting audio to text |
| 035 | Multimodal | 4 | 4.4 | PENDING | Processing multiple input types (vision, language) |
| 036 | Jetson Orin Nano | 3, Appendix A | 3.1, 3.2 | PENDING | NVIDIA edge AI computer |
| 037 | RealSense | 2, Appendix A | 2.4 | PENDING | Intel depth camera |
| 038 | RGB-D | 2, 3 | 2.4, 3.2 | PENDING | Color + depth image |
| 039 | Odometry | 2 | 2.4 | PENDING | Robot motion estimation |
| 040 | Kinematics | 2 | 2.2 | PENDING | Study of motion without forces |
| 041 | | | | | *Add more terms during chapter writing* |

---

## Terms by Category

### ROS 2 & Middleware (10 terms)
- Node, Topic, Service, Action, DDS, URDF, Launch File, Parameter, Workspace, Package

### Simulation & Digital Twin (7 terms)
- Digital Twin, Simulation, Gazebo, Unity, Sim-to-Real, Domain Randomization, VSLAM

### NVIDIA Isaac & AI (9 terms)
- Isaac SDK, Isaac Sim, Isaac ROS, Perception, TensorRT, Nav2, Costmap, Reinforcement Learning, Isaac Gym, PPO

### VLA & Language Models (6 terms)
- VLA, Embodied Intelligence, LLM, Prompt Engineering, Task Planning, Whisper, Speech-to-Text, Multimodal

### Hardware & Sensors (5 terms)
- Jetson Orin Nano, RealSense, RGB-D, Odometry, Kinematics

### Additional Categories
- *Add more as chapters are written*

---

## Definition Writing Guidelines (Phase 7)

When writing definitions in Phase 7:

1. **Format**:
   ```markdown
   **Term Name**: [1-3 sentence clear definition]. (See Chapter X.Y)
   ```

2. **Definition Quality**:
   - Avoid circular definitions (don't use the term to define itself)
   - Be precise but accessible to target audience (graduate students)
   - Include context of how term relates to robotics/AI
   - Mention key applications or use cases

3. **Example**:
   ```markdown
   **Node**: A process in the ROS 2 computational graph that performs computation. Nodes communicate with each other via topics (publish-subscribe), services (request-response), and actions (long-running tasks). Every ROS 2 application consists of one or more nodes working together. (See Chapter 1.1)
   ```

---

## Validation Checklist

Before finalizing glossary:

- [ ] Total terms ≥40 (Spec FR-047, SC-002)
- [ ] All terms have clear definitions (Spec FR-048)
- [ ] All terms have chapter references (Spec FR-048)
- [ ] Terms alphabetically sorted (Spec FR-049)
- [ ] Required terms present (Spec FR-050):
  - [ ] Node, Topic, Service, Action
  - [ ] URDF, Gazebo, Unity
  - [ ] Isaac SDK, Isaac Sim, Isaac ROS
  - [ ] VLA, LLM, Whisper
  - [ ] VSLAM, DDS, Nav2, TensorRT
  - [ ] Embodied Intelligence, Digital Twin
  - [ ] Sim-to-Real, Domain Randomization
  - [ ] Reinforcement Learning, PPO

---

## Glossary Generation (Phase 7)

Once all terms are collected and defined:

1. **Sort alphabetically** (case-insensitive)
2. **Format consistently** in `content/back-matter/glossary.md`
3. **Cross-reference chapters** (ensure chapter numbers are correct)
4. **Validate count** (must be ≥40)
5. **Review for completeness** (no undefined terms)

---

**Status**: Ready for term collection during chapter writing (Phases 3-6)
