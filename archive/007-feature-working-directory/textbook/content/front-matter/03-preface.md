# Preface

The field of robotics is undergoing a transformative shift. For decades, robots have been constrained to structured environments—factory floors, warehouses, and laboratories—where tasks are repetitive and predictable. Today, we stand at the threshold of a new era: **Physical AI**, where robots understand natural language, perceive their surroundings, and adapt to unstructured, dynamic environments. Humanoid robots that can respond to voice commands like "Pick up the red cup" or "Set the table" are no longer science fiction—they are emerging realities powered by vision-language-action (VLA) models and embodied intelligence.

This textbook was born from the recognition that traditional robotics curricula have not kept pace with these rapid advancements. Most textbooks focus either on classical robotics (kinematics, dynamics, control theory) or on artificial intelligence (machine learning, computer vision, natural language processing) but rarely bridge the two in a practical, integrated manner. Students complete robotics courses without ever integrating a large language model into a robot system. They learn computer vision but never deploy it on a physical humanoid navigating a real environment. This gap between theory and practice—between AI and robotics—is what this textbook aims to close.

## Who This Book Is For

This textbook is designed for **graduate students and early-career robotics engineers** who possess basic programming skills (Python, Linux command line) and want to build complete, intelligent robot systems. It assumes no prior knowledge of ROS 2, simulation environments, or modern AI frameworks, but it does expect mathematical maturity (linear algebra, probability) and a willingness to engage with complex, multi-layered systems.

Whether you are a computer science student curious about embodied AI, a mechanical engineering student seeking to add intelligence to your robot designs, or a professional engineer transitioning into robotics, this textbook provides a structured path from foundational concepts to cutting-edge VLA systems.

## Pedagogical Philosophy

Our approach is **relentlessly practical**. Every concept is introduced with a clear motivation ("Why does this matter?"), explained with concrete examples, and reinforced with hands-on exercises. Code examples are not pseudocode—they are tested, runnable Python scripts using ROS 2 Humble on Ubuntu 22.04. Diagrams are not abstract—they show real computational graphs, data flows, and system architectures. Exercises are not toy problems—they guide you to build publisher-subscriber systems, simulate humanoid robots, deploy GPU-accelerated perception, and create voice-controlled robot systems.

The textbook follows a **scaffolded learning path** across four modules, each building on the previous:

1. **Module 1 (ROS 2)** establishes the "nervous system"—the communication infrastructure that connects sensors, actuators, and decision-making processes.

2. **Module 2 (Digital Twin)** introduces simulation environments (Gazebo, Unity) where you can safely test robot behaviors before deploying to physical hardware.

3. **Module 3 (Isaac)** adds the "AI brain"—NVIDIA Isaac's GPU-accelerated perception, navigation, and reinforcement learning capabilities.

4. **Module 4 (VLA)** integrates everything into voice-controlled, vision-language-action systems that embody the future of human-robot interaction.

By the end of this textbook, you will have built not just isolated components but a **complete system**: a humanoid robot that listens to voice commands via Whisper, reasons about tasks using large language models, perceives its environment with Isaac, and executes actions in simulation (or on real hardware if available).

## Why Now?

The timing of this textbook reflects several converging trends. **ROS 2 Humble**, released in 2022 with long-term support until 2027, has matured into a stable, production-ready robotics framework. **NVIDIA Isaac Sim 5.0** (2025) brings GPU-accelerated simulation with photorealistic rendering and advanced sensor models. **Vision-language-action models** like RT-1, RT-2, and OpenVLA (2022-2024) have demonstrated that robots can learn from vast internet-scale data, not just narrow task-specific datasets. **Large language models** (GPT-4, Claude, Gemini) have reached a level of reasoning that makes natural language robot control practical.

In short, all the pieces are in place for a new generation of intelligent, adaptable robots. This textbook provides the blueprint for building them.

## Open Educational Resource

In the spirit of the open-source robotics community, this textbook is an **Open Educational Resource (OER)** licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0. We believe knowledge should be accessible to all. Share it, adapt it, build upon it—and contribute your improvements back to the community.

## Acknowledgments

This textbook would not exist without the tireless work of the open-source robotics community: the ROS 2 developers at Open Robotics, the Gazebo and Unity Robotics teams, the NVIDIA Isaac engineers, and the researchers advancing VLA models. We stand on the shoulders of giants.

Now, let us begin the journey from computational graphs to embodied intelligence.

**[Author Name(s)]**

**December 2025**

---

**Word Count**: 688 words
