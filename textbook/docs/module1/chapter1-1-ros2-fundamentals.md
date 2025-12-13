---
sidebar_position: 2
title: Chapter 1.1 - ROS 2 Fundamentals
---

# Chapter 1.1: ROS 2 Fundamentals

**Module**: 1 - The Robotic Nervous System
**Week**: 1
**Estimated Reading Time**: 30 minutes

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. Explain the ROS 2 architecture and computational graph
2. Create basic publisher and subscriber nodes in Python
3. Configure Quality of Service (QoS) policies for different scenarios
4. Use ROS 2 command-line tools to inspect and debug systems
5. Understand the role of DDS middleware in ROS 2

---

## Prerequisites

- Python 3.10+ installed
- ROS 2 Humble installed on Ubuntu 22.04
- Basic Linux command-line knowledge
- Understanding of object-oriented programming

---

## Introduction

Welcome to your first chapter on ROS 2! In this chapter, you'll learn the fundamental building blocks of Robot Operating System 2—the middleware that powers modern robotics.

ROS 2 is not an operating system in the traditional sense. It's a **middleware framework** that provides:
- Communication infrastructure between software components
- Standard message types for sensors and actuators
- Tools for building, testing, and deploying robot software
- A vast ecosystem of reusable packages

Think of ROS 2 as the **nervous system** of your robot, enabling different parts to communicate and coordinate.

---

## Key Terms

:::info Glossary Terms
- **Node**: An independent process that performs computation
- **Topic**: A named bus over which nodes exchange messages
- **Publisher**: A node component that sends messages to a topic
- **Subscriber**: A node component that receives messages from a topic
- **DDS**: Data Distribution Service, the middleware layer under ROS 2
- **QoS**: Quality of Service policies that control message delivery
:::

---

## Core Concepts

### 1. ROS 2 Architecture

ROS 2 uses a **decentralized** architecture built on DDS (Data Distribution Service):

```mermaid
graph LR
    A[Node 1<br/>Camera] -->|publishes| B[/camera/image<br/>Topic]
    B -->|subscribes| C[Node 2<br/>Processing]
    C -->|publishes| D[/detections<br/>Topic]
    D -->|subscribes| E[Node 3<br/>Control]
```

**Key Components**:
- **Nodes**: Independent processes (e.g., camera driver, image processor, controller)
- **Topics**: Named channels for data flow (e.g., `/camera/image`, `/cmd_vel`)
- **Messages**: Structured data types (e.g., `sensor_msgs/Image`, `geometry_msgs/Twist`)

**No roscore required!** Unlike ROS 1, nodes discover each other automatically via DDS.

---

### 2. Creating Your First Publisher

Let's create a simple node that publishes "Hello World" messages:

**File**: `hello_publisher.py`

```python
#!/usr/bin/env python3
"""
Simple ROS 2 publisher node that sends string messages.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloPublisher(Node):
    """Publisher node that sends hello messages at 1 Hz."""

    def __init__(self):
        super().__init__('hello_publisher')

        # Create publisher: topic name, message type, queue size
        self.publisher = self.create_publisher(String, 'hello_topic', 10)

        # Create timer: callback runs every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.counter = 0
        self.get_logger().info('Hello Publisher node started')

    def timer_callback(self):
        """Callback function executed by timer."""
        msg = String()
        msg.data = f'Hello World: {self.counter}'

        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

        self.counter += 1

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create node
    node = HelloPublisher()

    try:
        # Spin = process callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run it**:
```bash
python3 hello_publisher.py
```

**Expected Output**:
```
[INFO] [hello_publisher]: Hello Publisher node started
[INFO] [hello_publisher]: Published: "Hello World: 0"
[INFO] [hello_publisher]: Published: "Hello World: 1"
[INFO] [hello_publisher]: Published: "Hello World: 2"
...
```

---

### 3. Creating a Subscriber

Now let's create a subscriber that receives those messages:

**File**: `hello_subscriber.py`

```python
#!/usr/bin/env python3
"""
Simple ROS 2 subscriber node that receives string messages.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloSubscriber(Node):
    """Subscriber node that receives hello messages."""

    def __init__(self):
        super().__init__('hello_subscriber')

        # Create subscriber: topic name, message type, callback, queue size
        self.subscription = self.create_subscription(
            String,
            'hello_topic',
            self.listener_callback,
            10
        )

        self.get_logger().info('Hello Subscriber node started')

    def listener_callback(self, msg):
        """Callback function executed when message received."""
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = HelloSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run in another terminal**:
```bash
python3 hello_subscriber.py
```

**Expected Output**:
```
[INFO] [hello_subscriber]: Hello Subscriber node started
[INFO] [hello_subscriber]: Received: "Hello World: 5"
[INFO] [hello_subscriber]: Received: "Hello World: 6"
...
```

🎉 **Congratulations!** You've created your first pub/sub system!

---

### 4. Quality of Service (QoS)

QoS policies control how messages are delivered. Key policies:

| Policy | Options | Use Case |
|--------|---------|----------|
| **Reliability** | Best Effort, Reliable | Sensor data vs critical commands |
| **Durability** | Volatile, Transient Local | Temporary vs persistent messages |
| **History** | Keep Last N, Keep All | How many messages to store |

**Example**: Reliable delivery for critical commands

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=10
)

self.publisher = self.create_publisher(String, 'critical_topic', qos)
```

---

### 5. ROS 2 Command-Line Tools

Essential tools for debugging:

```bash
# List all running nodes
ros2 node list

# List all topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /hello_topic

# Check topic frequency
ros2 topic hz /hello_topic

# Show topic info
ros2 topic info /hello_topic

# Visualize graph
ros2 run rqt_graph rqt_graph
```

---

## Practical Example: Robot Velocity Publisher

Real-world example: Publishing velocity commands to a mobile robot.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)  # 10 Hz

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.5  # Forward 0.5 m/s
        msg.angular.z = 0.2  # Rotate 0.2 rad/s
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

This publishes velocity commands at 10 Hz, moving the robot forward while turning.

---

## Summary

In this chapter, you learned:

1. ✅ ROS 2 architecture with nodes, topics, and DDS
2. ✅ How to create publishers and subscribers in Python
3. ✅ Quality of Service (QoS) policies for reliable communication
4. ✅ Essential ROS 2 command-line tools
5. ✅ Real-world example: robot velocity control

**Key Figures**:
- Figure 1.1: ROS 2 computational graph (Mermaid diagram above)

**Code Examples**:
- Publisher: `hello_publisher.py`
- Subscriber: `hello_subscriber.py`
- Velocity control: `velocity_publisher.py`

---

## End-of-Chapter Exercises

### Exercise 1: Modify Publisher Frequency (Difficulty: Easy)

Modify `hello_publisher.py` to publish at 10 Hz instead of 1 Hz.

**Expected Outcome**: Messages published 10 times per second.

---

### Exercise 2: Create Temperature Sensor Node (Difficulty: Medium)

Create a node that publishes simulated temperature data.

**Requirements**:
- Publish to `/temperature` topic
- Use `std_msgs/Float32` message type
- Generate random temperature between 20-30°C
- Publish at 5 Hz

---

### Exercise 3: Build Publisher-Subscriber Pair (Difficulty: Hard)

Create two nodes:
1. **Sensor Node**: Publishes random "sensor readings" (0-100)
2. **Monitor Node**: Subscribes and logs warning if reading > 80

**Test**: Verify warnings appear when values exceed threshold.

---

## Further Reading

### Required
1. Open Robotics. (2024). *ROS 2 Humble documentation*. https://docs.ros.org/en/humble/
2. ROS 2 Design documentation: https://design.ros2.org/

### Recommended
3. Macenski, S., et al. (2022). Robot Operating System 2: Design, architecture, and uses in the wild. *Science Robotics*.

---

## Troubleshooting

### Issue: "No module named 'rclpy'"
**Solution**: Ensure ROS 2 is sourced:
```bash
source /opt/ros/humble/setup.bash
```

### Issue: "Topic not appearing"
**Solution**: Check node is running and publishing:
```bash
ros2 node list
ros2 topic list
ros2 topic echo /your_topic
```

---

## Next Chapter

Continue to **[Chapter 1.2: Nodes and Communication](./chapter1-2-nodes-communication)** to learn about services and actions.

---

**🎓 Chapter 1.1 Complete!** You now understand ROS 2 fundamentals.
