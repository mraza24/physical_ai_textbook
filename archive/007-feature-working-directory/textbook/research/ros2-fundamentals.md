# Research Notes: ROS 2 Humble Fundamentals

**Research Track**: ROS 2
**Module**: 1 - Robotic Nervous System
**Date**: 2025-12-11
**Status**: Complete

---

## Official Source Information

### Primary Source
- **Title**: ROS 2 Documentation: Humble Hawksbill
- **URL**: https://docs.ros.org/en/humble/
- **Version**: Humble Hawksbill (May 2022, LTS)
- **Publisher**: Open Robotics

### APA 7 Citation
```
Open Robotics. (2023). ROS 2 documentation: Humble Hawksbill. https://docs.ros.org/en/humble/
```

---

## Key Concepts

### Concept 1: Computational Graph (Nodes, Topics, Services, Actions)

**Definition**: ROS 2 uses a computational graph architecture where nodes (processes) communicate via topics (asynchronous pub-sub), services (synchronous request-response), and actions (long-running goal-oriented tasks).

**Explanation**: The computational graph is the fundamental abstraction in ROS 2. Nodes are independent processes that perform specific computations. They communicate through three primary mechanisms: (1) Topics enable many-to-many asynchronous communication ideal for sensor data streaming, (2) Services provide one-to-one synchronous communication for queries and commands, and (3) Actions support goal-oriented tasks with feedback, cancellation, and result reporting.

**Relevance to Textbook**: Chapter 1.1 introduces computational graph; Chapter 1.2 covers nodes and communication patterns in depth. This is the foundation for all subsequent modules.

**Example**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2'
        self.publisher.publish(msg)
```

---

### Concept 2: DDS Middleware Layer

**Definition**: ROS 2 uses Data Distribution Service (DDS) as its middleware layer for inter-process communication, enabling real-time, reliable data exchange across networks.

**Explanation**: Unlike ROS 1's custom TCPROS protocol, ROS 2 leverages industry-standard DDS implementations (Fast DDS by default, also supports Connext 6.0.1 via rmw_connextdds). DDS provides Quality-of-Service (QoS) policies, automatic discovery, and scalable publish-subscribe communication. The middleware abstraction (RMW) allows swapping DDS implementations at runtime.

**Relevance to Textbook**: Chapter 1.1 explains DDS layer; important for understanding ROS 2's reliability and real-time capabilities compared to ROS 1.

**Example**: Middleware is transparent to most users but can be configured via environment variables or QoS settings.

---

### Concept 3: Content Filtered Topics

**Definition**: Content Filtered Topics allow subscribers to specify filters that reduce the amount of data received by only accepting messages matching specific criteria.

**Explanation**: This advanced feature enables subscribers to request content-based subscriptions when the underlying RMW implementation supports it. Instead of receiving all messages on a topic and filtering locally, the filtering happens at the publisher or middleware level, reducing bandwidth and processing overhead.

**Relevance to Textbook**: Advanced topic for Chapter 1.2; useful for high-frequency sensor data scenarios where only certain values are relevant.

---

### Concept 4: Topic Publishing Behavior (--times/--once flags)

**Definition**: ros2 topic pub command waits for matching subscribers before publishing to avoid message loss from early publishing.

**Explanation**: When using command-line topic publishing with flags like `--times` or `--once`, the ros2 CLI node now waits for at least one matching subscription to be discovered before starting to publish. This prevents the common issue of initial messages being lost because the publisher started before any subscribers were ready.

**Relevance to Textbook**: Practical detail for Chapter 1.1 exercises and debugging tips.

---

## Technical Specifications

### Version Information
- **Software Version**: ROS 2 Humble Hawksbill (Released May 2022, LTS until May 2027)
- **OS Requirements**: Ubuntu 22.04 LTS (Jammy Jellyfish), Windows 10, macOS (see REP 2000 for full platform support)
- **Python Version**: Python 3.10+
- **Dependencies**:
  - DDS implementation (Fast DDS default, Connext 6.0.1 optional)
  - colcon build system
  - ament packages for CMake/Python

### Installation Notes
```bash
# Ubuntu 22.04 installation
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash

# Verify installation
ros2 --version
```

---

## Important Details

### Features/Capabilities
- **Multi-platform**: Linux, Windows, macOS support
- **Real-time**: DDS middleware enables deterministic communication
- **Security**: Built-in support for encrypted and authenticated communication (SROS2)
- **QoS Policies**: Configurable reliability, durability, liveliness, and deadline policies
- **Python & C++ APIs**: First-class support for both languages
- **Composition**: Multiple nodes can run in single process for efficiency

### Limitations/Constraints
- **Learning Curve**: Steeper than ROS 1 due to DDS complexity and lifecycle management
- **Debugging**: Graph visualization tools (rqt_graph) less mature than ROS 1
- **Migration**: Existing ROS 1 packages require porting (though ros1_bridge exists)
- **QoS Complexity**: Mismatched QoS settings between publishers/subscribers can prevent communication

### Best Practices
1. Use lifecycle nodes for managed state transitions in production systems
2. Configure appropriate QoS policies for your use case (e.g., reliable for commands, best-effort for high-frequency sensor data)
3. Leverage composition for performance-critical multi-node applications
4. Use domain IDs to isolate multiple ROS 2 systems on same network
5. Test with different DDS implementations if experiencing network issues

---

## Chapter Applications

### Chapter 1.1: ROS 2 Fundamentals

**Concepts to Include**:
- Computational graph architecture
- Nodes as independent processes
- Topics for pub-sub communication
- DDS middleware overview
- Quality of Service basics

**Code Examples Needed**:
- Simple publisher node (Python)
- Simple subscriber node (Python)
- Running nodes and visualizing graph with rqt_graph

**Figures Needed**:
- ROS 2 computational graph showing nodes connected by topics
- Comparison of topics vs services vs actions (decision tree diagram)

---

### Chapter 1.2: Nodes & Communication

**Concepts to Include**:
- Deep dive into topics, services, actions
- Synchronous vs asynchronous communication
- Service client-server pattern
- Action goal-feedback-result pattern
- Content filtered topics (advanced)

**Code Examples Needed**:
- Service server implementation
- Service client calling server
- Action server with feedback
- Action client with goal cancellation

**Figures Needed**:
- Service call sequence diagram (Mermaid)
- Action lifecycle state diagram

---

### Chapter 1.3: Launch Files & Configuration

**Concepts to Include**:
- Python launch file syntax
- Parameter declaration and use
- YAML configuration files
- Remapping topics/services
- Composable node containers

**Code Examples Needed**:
- Multi-node launch file
- Parameter file (YAML)
- Launch file with remapping

**Figures Needed**:
- Launch file structure diagram
- Parameter flow diagram

---

### Chapter 1.4: Building Packages & Workspaces

**Concepts to Include**:
- Colcon build system
- Workspace structure (src/, build/, install/, log/)
- package.xml manifest
- CMakeLists.txt (C++) or setup.py (Python)
- Dependency management

**Code Examples Needed**:
- Creating new package with ros2 pkg create
- Building workspace with colcon build
- Source overlay concept

**Figures Needed**:
- ROS 2 workspace directory tree
- Build process flowchart

---

## Glossary Terms Identified

| Term | Brief Definition | Chapter Reference |
|------|------------------|-------------------|
| Node | Independent process in ROS 2 computational graph | 1.1 |
| Topic | Named bus for asynchronous message passing | 1.1 |
| Service | Synchronous request-response communication pattern | 1.1, 1.2 |
| Action | Goal-oriented task with feedback and cancellation | 1.1, 1.2 |
| DDS | Data Distribution Service middleware standard | 1.1 |
| QoS | Quality of Service policies for communication reliability | 1.1, 1.2 |
| Publisher | Node that sends messages to a topic | 1.1 |
| Subscriber | Node that receives messages from a topic | 1.1 |
| RMW | ROS Middleware abstraction layer | 1.1 |
| Colcon | Build tool for ROS 2 workspaces | 1.4 |

---

## References to Collect

### Official Documentation
- Open Robotics. (2023). *ROS 2 documentation: Humble Hawksbill*. https://docs.ros.org/en/humble/
- Open Robotics. (2023). *Humble Hawksbill release notes*. https://docs.ros.org/en/rolling/Releases/Release-Humble-Hawksbill.html
- Open Robotics. (2023). *ROS 2 tutorials*. https://docs.ros.org/en/humble/Tutorials.html

### Platform Support
- Open Robotics. *REP 2000: ROS 2 platform support*. https://www.ros.org/reps/rep-2000.html

---

## Notes & Observations

- **ROS 1 vs ROS 2**: Key difference is DDS middleware (ROS 2) vs custom TCPROS (ROS 1). This enables better real-time performance, security, and multi-platform support.
- **LTS Status**: Humble is LTS release (supported until May 2027), making it ideal choice for textbook rather than newer but non-LTS releases.
- **Common Pitfall**: Students often struggle with QoS mismatches causing "silent" communication failures. Important to emphasize QoS compatibility in exercises.
- **Ecosystem Maturity**: By 2025, ROS 2 ecosystem is mature with most popular ROS 1 packages ported. Migration from ROS 1 largely complete in community.

---

**Sources**:
- [ROS 2 Documentation: Humble Hawksbill](https://docs.ros.org/en/humble/)
- [Humble Hawksbill Release Notes](https://docs.ros.org/en/rolling/Releases/Release-Humble-Hawksbill.html)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
