---
sidebar_position: 7
title: Appendix G - Additional Resources
---

# Appendix G: Additional Resources

Curated list of learning materials, communities, and tools beyond this textbook.

---

## Official Documentation

### ROS 2
- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **ROS 2 Tutorials**: https://docs.ros.org/en/humble/Tutorials.html
- **ROS 2 Design**: https://design.ros2.org/
- **rclpy API**: https://docs.ros2.org/latest/api/rclpy/
- **rclcpp API**: https://docs.ros2.org/latest/api/rclcpp/

### Gazebo
- **Gazebo Sim**: https://gazebosim.org/docs
- **SDF Format Spec**: http://sdformat.org/
- **Gazebo Plugins**: https://gazebosim.org/api/sim/7/namespacegz.html

### Unity
- **Unity Robotics Hub**: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- **Unity Manual**: https://docs.unity3d.com/Manual/index.html
- **Unity Robotics Forum**: https://forum.unity.com/forums/robotics.506/

### NVIDIA Isaac
- **Isaac ROS**: https://nvidia-isaac-ros.github.io/
- **Isaac Sim**: https://docs.omniverse.nvidia.com/isaacsim/
- **Isaac Gym**: https://developer.nvidia.com/isaac-gym
- **TensorRT**: https://docs.nvidia.com/deeplearning/tensorrt/

### VLA / LLM
- **OpenAI API**: https://platform.openai.com/docs
- **Anthropic Claude**: https://docs.anthropic.com/
- **Whisper**: https://github.com/openai/whisper
- **OpenVLA**: https://github.com/openvla/openvla

---

## Online Courses

### ROS 2
- **The Construct**: ROS 2 Basics in 5 Days (Python)
  - https://www.theconstructsim.com/
  - Online ROS simulation environment

- **Udemy**: ROS 2 for Beginners
  - Practical ROS 2 development
  - Mobile robot examples

### Robotics Fundamentals
- **Coursera**: Modern Robotics (Northwestern University)
  - Kinematics, dynamics, control
  - Theoretical foundation

- **MIT OpenCourseWare**: Introduction to Robotics
  - https://ocw.mit.edu/
  - Free lecture notes and videos

### Deep Learning for Robotics
- **Stanford CS231n**: Convolutional Neural Networks
  - https://cs231n.stanford.edu/
  - Computer vision fundamentals

- **DeepLearning.AI**: Deep Learning Specialization
  - https://www.deeplearning.ai/
  - Neural networks, CNNs, RNNs

---

## Books

### Robotics Fundamentals
1. Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer Handbook of Robotics* (2nd ed.).
   - Comprehensive reference
   - Covers all robotics subfields

2. Murray, R. M., Li, Z., & Sastry, S. S. (1994). *A Mathematical Introduction to Robotic Manipulation*.
   - Kinematics, dynamics, control
   - Free PDF: http://www.cds.caltech.edu/~murray/mlswiki/

### ROS 2 Specific
3. Newbury, R., & Martinez, A. (2023). *ROS 2 Robotics Developer's Handbook*.
   - Practical ROS 2 development
   - Real-world examples

4. Fairchild, C., & Harman, T. (2023). *ROS 2 for Ubuntu 22.04*.
   - Step-by-step tutorials
   - From basics to advanced

### Deep Learning
5. Goodfellow, I., Bengio, Y., & Courville, A. (2016). *Deep Learning*.
   - ML fundamentals
   - Free online: https://www.deeplearningbook.org/

6. Zhang, A., et al. (2023). *Dive into Deep Learning*.
   - Interactive book with code
   - https://d2l.ai/

---

## Research Papers (Key Papers)

### ROS 2
- Macenski, S., et al. (2022). Robot Operating System 2: Design, architecture, and uses in the wild. *Science Robotics*.

### SLAM
- Mur-Artal, R., & Tardós, J. D. (2017). ORB-SLAM2. *IEEE Transactions on Robotics*.
- Campos, C., et al. (2021). ORB-SLAM3. *IEEE Transactions on Robotics*.

### Vision-Language-Action
- Brohan, A., et al. (2022). RT-1: Robotics Transformer. *arXiv:2212.06817*.
- Brohan, A., et al. (2023). RT-2: Vision-language-action models. *arXiv:2307.15818*.

### Reinforcement Learning
- Schulman, J., et al. (2017). Proximal Policy Optimization. *arXiv:1707.06347*.
- Makoviychuk, V., et al. (2021). Isaac Gym. *arXiv:2108.10470*.

### Sim-to-Real Transfer
- Tobin, J., et al. (2017). Domain Randomization. *IROS*.
- Peng, X. B., et al. (2018). Sim-to-Real Transfer with Dynamics Randomization. *ICRA*.

---

## Community and Forums

### ROS Community
- **ROS Discourse**: https://discourse.ros.org/
  - Official ROS forum
  - Active community, quick responses

- **ROS Answers**: https://answers.ros.org/
  - Q&A site (older, less active than Discourse)

- **r/ROS (Reddit)**: https://www.reddit.com/r/ROS/
  - Community discussions
  - Project showcases

### Robotics Community
- **r/robotics (Reddit)**: https://www.reddit.com/r/robotics/
  - General robotics discussion

- **Robotics Stack Exchange**: https://robotics.stackexchange.com/
  - Technical Q&A

### NVIDIA Community
- **NVIDIA Developer Forums**: https://forums.developer.nvidia.com/
  - Isaac Sim, Isaac ROS, Jetson support

- **Omniverse Discord**: Active community for Isaac Sim

---

## Tools and Software

### Visualization
- **RViz2**: ROS 2 3D visualization
  - `ros2 run rviz2 rviz2`

- **PlotJuggler**: Real-time plotting for ROS 2 topics
  - `sudo apt install ros-humble-plotjuggler-ros`

- **rqt_graph**: Visualize ROS 2 computational graph
  - `ros2 run rqt_graph rqt_graph`

### Simulation
- **MuJoCo**: Fast physics engine (now open-source)
  - https://mujoco.org/

- **PyBullet**: Python physics simulation
  - `pip install pybullet`

- **CoppeliaSim**: Robot simulator (formerly V-REP)
  - https://www.coppeliarobotics.com/

### Development
- **VS Code**: Recommended IDE
  - Extensions: ROS, Python, C/C++, CMake Tools

- **Terminator**: Terminal multiplexer
  - `sudo apt install terminator`

- **Docker**: Containerization for reproducible environments
  - https://www.docker.com/

---

## Datasets

### Robot Manipulation
- **RT-X Dataset**: Open X-Embodiment dataset
  - https://robotics-transformer-x.github.io/
  - 1M+ robot trajectories

- **Fractal Dataset**: Google's robot manipulation data
  - https://sites.google.com/view/fractal-data/

### Computer Vision
- **COCO**: Common Objects in Context
  - https://cocodataset.org/
  - Object detection, segmentation

- **ImageNet**: Large-scale image dataset
  - https://www.image-net.org/

### SLAM / 3D Vision
- **TUM RGB-D**: RGB-D SLAM benchmark
  - https://vision.in.tum.de/data/datasets/rgbd-dataset

- **KITTI**: Autonomous driving dataset
  - https://www.cvlibs.net/datasets/kitti/

---

## Conferences (Where to Follow Research)

### Robotics
- **RSS** (Robotics: Science and Systems)
  - https://roboticsconference.org/

- **ICRA** (International Conference on Robotics and Automation)
  - https://www.ieee-ras.org/conferences-workshops/fully-sponsored/icra

- **IROS** (Intelligent Robots and Systems)
  - https://www.iros.org/

- **CoRL** (Conference on Robot Learning)
  - https://www.robot-learning.org/

### AI / ML
- **NeurIPS** (Neural Information Processing Systems)
- **ICML** (International Conference on Machine Learning)
- **ICLR** (International Conference on Learning Representations)

### Computer Vision
- **CVPR** (Computer Vision and Pattern Recognition)
- **ICCV** (International Conference on Computer Vision)
- **ECCV** (European Conference on Computer Vision)

---

## YouTube Channels

### ROS 2 Tutorials
- **The Construct**: ROS 2 tutorials and courses
- **Articulated Robotics**: Beginner-friendly ROS 2 content
- **Robotics Back-End**: Practical ROS 2 projects

### General Robotics
- **MIT CSAIL**: Research talks and demos
- **Boston Dynamics**: Robot demos (Atlas, Spot)
- **NVIDIA Developer**: Isaac Sim, Isaac ROS tutorials

### Deep Learning
- **Yannic Kilcher**: Paper reviews and explanations
- **Two Minute Papers**: Latest AI research summaries
- **Lex Fridman**: Interviews with AI researchers

---

## Code Repositories

### ROS 2 Packages
- **Navigation2**: https://github.com/ros-planning/navigation2
- **MoveIt 2**: https://github.com/ros-planning/moveit2
- **Isaac ROS**: https://github.com/NVIDIA-ISAAC-ROS

### Example Projects
- **TurtleBot4**: https://github.com/turtlebot/turtlebot4
  - Complete mobile robot examples

- **ROBOTIS OP3**: https://github.com/ROBOTIS-GIT/ROBOTIS-OP3
  - Humanoid robot ROS 2 code

### Perception
- **YOLOv8**: https://github.com/ultralytics/ultralytics
- **ORB-SLAM3**: https://github.com/UZ-SLAMLab/ORB_SLAM3
- **Segment Anything**: https://github.com/facebookresearch/segment-anything

---

## Newsletters and Blogs

### Robotics News
- **The Robot Report**: https://www.therobotreport.com/
- **IEEE Spectrum Robotics**: https://spectrum.ieee.org/robotics

### AI/ML News
- **The Batch (DeepLearning.AI)**: https://www.deeplearning.ai/the-batch/
- **Import AI (Jack Clark)**: https://importai.substack.com/

### Technical Blogs
- **NVIDIA Developer Blog**: https://developer.nvidia.com/blog
- **Open Robotics**: https://www.openrobotics.org/blog
- **Google AI Blog**: https://ai.googleblog.com/

---

## Job Boards (For After Completion)

### Robotics Companies
- **Boston Dynamics**: https://www.bostondynamics.com/careers
- **NVIDIA**: https://www.nvidia.com/en-us/about-nvidia/careers/
- **Waymo**: https://waymo.com/careers/
- **Tesla (Autopilot/Optimus)**: https://www.tesla.com/careers

### Startups
- **AngelList**: Filter by "Robotics"
- **Y Combinator Companies**: Many robotics startups

### General Tech
- **LinkedIn Jobs**: Search "ROS 2", "Robotics Engineer"
- **Indeed**: Filter by robotics, automation

---

## Professional Organizations

### IEEE
- **IEEE Robotics and Automation Society**: https://www.ieee-ras.org/
  - Conferences, publications, community

### Academic
- **Association for the Advancement of Artificial Intelligence (AAAI)**: https://www.aaai.org/

---

## Open-Source Contributions

Want to give back? Contribute to:
- **ROS 2 packages**: https://github.com/ros2
- **Gazebo**: https://github.com/gazebosim
- **Isaac ROS**: https://github.com/NVIDIA-ISAAC-ROS
- **OpenVLA**: https://github.com/openvla/openvla

**How to start**:
1. Find "good first issue" labels
2. Read contributing guidelines
3. Fix bugs, add features, improve documentation

---

## Staying Current

Robotics evolves rapidly. To stay updated:

1. **Follow key researchers on Twitter/X**: (search for authors of papers you like)
2. **Subscribe to arXiv alerts**: http://arxiv.org/ (cs.RO, cs.CV, cs.AI)
3. **Attend virtual conferences**: Many have free registration
4. **Join Discord/Slack communities**: Real-time discussions
5. **Build projects regularly**: Best way to learn new techniques

---

**Thank you for using this textbook!** We hope these resources help you continue your journey in physical AI and robotics.

**Feedback**: If you have additional resources to suggest, please contribute to the textbook's GitHub repository.

---

**🎓 Keep learning. Keep building. The future of robotics is in your hands! 🤖**
