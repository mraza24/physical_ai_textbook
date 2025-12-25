# Chapter 1.1: ROS 2 Fundamentals

> **Module**: 1 - Robotic Nervous System (ROS 2)
> **Week**: 3
> **Estimated Reading Time**: 45 minutes

---

## Summary

This chapter introduces the Robot Operating System 2 (ROS 2) as the communication middleware for modern robotics, explaining its computational graph architecture where independent processes (nodes) exchange data via topics, services, and actions. Understanding ROS 2 is essential because it provides the "nervous system" that connects sensors, actuators, and decision-making processes in every subsequent module of this textbook.

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Explain** the computational graph architecture and why it's fundamental to ROS 2
2. **Differentiate** between topics, services, and actions—knowing when to use each communication pattern
3. **Understand** the role of DDS middleware and Quality of Service (QoS) policies in reliable robot communication
4. **Implement** a basic publisher and subscriber node in Python
5. **Visualize** and debug ROS 2 systems using command-line tools (`ros2 topic`, `ros2 node`, `rqt_graph`)

**Prerequisite Knowledge**: Basic Python programming (functions, classes, loops), Linux command-line navigation (`cd`, `ls`, running scripts)

---

## Key Terms

This chapter introduces the following technical terms (see Glossary for detailed definitions):

- **Node**: An independent process in the ROS 2 computational graph that performs a specific computation (e.g., camera driver, object detector, motion planner)
- **Topic**: A named communication channel using asynchronous publish-subscribe messaging, ideal for streaming sensor data
- **Service**: A synchronous request-response communication pattern for querying information or triggering actions that complete quickly
- **Action**: A goal-oriented task pattern supporting long-running operations with progress feedback and cancellation (e.g., "navigate to waypoint")
- **DDS (Data Distribution Service)**: Industry-standard middleware providing reliable, real-time communication across processes and networks
- **Publisher**: A node component that sends messages to a topic
- **Subscriber**: A node component that receives messages from a topic
- **QoS (Quality of Service)**: Configurable policies (reliability, durability, deadline) that determine message delivery behavior

---

## Core Concepts

### 1. The Computational Graph: Orchestrating Robot Intelligence

Imagine a humanoid robot performing a simple task: picking up a cup. This action requires coordination between dozens of components operating simultaneously:
- Camera drivers capturing RGB-D images at 30 Hz
- Object detection algorithms identifying the cup's 3D pose at 10 Hz
- Motion planners computing collision-free arm trajectories at 50 Hz
- Joint controllers commanding motor torques at 1000 Hz
- Safety monitors checking for human proximity at 100 Hz

Each of these components runs as an **independent process** with its own lifecycle, failure modes, and computational requirements. Yet they must exchange data seamlessly, without any single component blocking others or causing system-wide failures.

ROS 2 solves this coordination problem through the **computational graph**—a runtime system where processes (called **nodes**) communicate via three primary mechanisms:

1. **Topics** (asynchronous pub-sub): Many-to-many data streaming
2. **Services** (synchronous request-response): One-to-one queries and commands
3. **Actions** (goal-feedback-result): Long-running tasks with progress updates

This architecture provides **loose coupling**: nodes don't need to know which other nodes exist, only the names and types of the communication channels they use. A camera node publishes images to `/camera/image_raw` without knowing whether zero, one, or ten nodes are subscribing. An object detector subscribes to images without caring whether they come from a real camera, a simulator, or a recorded bag file.

**Key Points**:
- The computational graph is dynamic—nodes can join or leave at runtime without restarting the entire system
- Communication is language-agnostic—Python nodes communicate seamlessly with C++ nodes
- The graph is distributed—nodes can run on different machines connected by a network
- ROS 2 uses **DDS middleware** for communication, providing real-time guarantees and reliability that HTTP/REST APIs cannot match

### 2. Topics: The Data Highways of ROS 2

**Topics** are the most common communication pattern in ROS 2, designed for continuous data streams like sensor readings, robot state, and control commands.

**How Topics Work**:

A **publisher** creates a topic with a specific name (e.g., `/camera/image_raw`) and message type (e.g., `sensor_msgs/Image`). It then sends messages to that topic at whatever rate it chooses—30 Hz for a camera, 200 Hz for an IMU, 1 kHz for joint states.

**Subscribers** declare interest in a topic by name and type. The DDS middleware handles **discovery** automatically—publishers and subscribers find each other without a central server (unlike ROS 1's roscore). Multiple subscribers can listen to the same topic, and multiple publishers can send to the same topic (though the latter is less common and requires careful handling).

**Key characteristics of topics**:
- **Asynchronous**: Publishers don't wait for subscribers to process messages before continuing
- **One-way**: Data flows from publisher to subscriber (no response expected)
- **Buffered**: Messages can be queued if subscribers process slower than publishers publish (configurable via QoS)
- **Typed**: Both publisher and subscriber must agree on the message type (enforced by middleware)

**Example scenario**: A camera node publishes 1920×1080 RGB images at 30 Hz to `/camera/color/image_raw`. Three nodes subscribe to this topic: (1) an object detector processing every frame, (2) a visualization tool displaying images at 10 Hz (dropping frames), and (3) a data logger saving images to disk. Each subscriber operates independently without affecting the others.

**When to use topics**:
- Sensor data streams (camera, LiDAR, IMU, GPS)
- Robot state (joint positions, velocities, battery level)
- Control commands (velocity setpoints, joint trajectories)
- Any high-frequency, continuous data flow

### 3. Services: Request-Response Communication

While topics handle continuous streams, **services** provide synchronous, one-shot communication for queries and commands that expect immediate responses.

**How Services Work**:

A **service server** advertises a service with a name (e.g., `/robot/get_battery_level`) and service type (e.g., `example_interfaces/srv/Trigger`). When a **service client** calls the service, it sends a **request** message and blocks (waits) until receiving a **response** message from the server.

**Key characteristics of services**:
- **Synchronous**: Client blocks until receiving response (or timeout)
- **One-to-one**: Each request goes to exactly one server
- **Two-way**: Request from client, response from server
- **Transient**: No history—if the call fails, it must be retried

**Example scenario**: A navigation node needs to query the robot's current battery level before planning a long-distance path. It calls the `/robot/get_battery_level` service, which immediately returns `85%`. Based on this response, the navigator decides whether to proceed or return to the charging dock.

**When to use services**:
- Querying robot state ("What's the battery level?", "What mode am I in?")
- Triggering actions that complete quickly ("Switch to manual mode", "Reset odometry")
- Configuration requests ("Get/set parameter values")
- Any operation where you need a response before continuing

**Services vs Topics**:
- Use **topics** when data flows continuously (sensor streams, state updates)
- Use **services** when you need confirmation that a command succeeded or when querying for information

### 4. Actions: Long-Running Tasks with Feedback

**Actions** extend the service pattern to support tasks that take significant time to complete (seconds to minutes) and benefit from progress feedback and cancellation.

**How Actions Work**:

An **action server** advertises an action (e.g., `/navigate_to_pose`) with a goal type, feedback type, and result type. An **action client** sends a **goal** ("Navigate to x=5.0, y=3.0"), receives periodic **feedback** ("50% complete, ETA 10 seconds"), and eventually gets a **result** ("Success: reached waypoint" or "Aborted: path blocked").

Critically, actions support **cancellation**—the client can request the server abort the goal mid-execution, useful when plans change or obstacles appear.

**Key characteristics of actions**:
- **Asynchronous**: Client doesn't block waiting for result (though it can if desired)
- **Three-phase**: Goal → Feedback (multiple) → Result
- **Cancelable**: Client can request early termination
- **Persistent**: Action servers maintain goal state and handle concurrent goals

**Example scenario**: A navigation system receives a goal to move from (0,0) to (10,0). It begins execution, sending feedback every second: "20% complete, ETA 15 seconds", "40% complete, ETA 12 seconds". At 60% completion, a human steps into the robot's path. The safety monitor cancels the navigation goal, and the robot stops safely before attempting a new path.

**When to use actions**:
- Navigation ("Move to waypoint")
- Manipulation ("Grasp object", "Pour liquid")
- Perception tasks ("Scan environment", "Track object")
- Any task that takes >1 second, benefits from progress updates, or needs cancellation

### 5. DDS Middleware: The Hidden Foundation

Unlike ROS 1, which used a custom TCPROS protocol, ROS 2 builds on **DDS (Data Distribution Service)**—an industry-standard middleware used in military systems, medical devices, and industrial automation.

**Why DDS Matters**:

DDS provides capabilities critical for real-time robotics:
1. **Automatic Discovery**: Nodes find each other without a central server (no single point of failure)
2. **Quality of Service (QoS)**: Configure reliability, durability, and deadline policies per topic
3. **Real-Time Performance**: Deterministic message delivery with bounded latency
4. **Security**: Built-in authentication and encryption (SROS2)
5. **Multi-Platform**: Works across Linux, Windows, macOS, and embedded systems

**QoS Policies**:

ROS 2 allows configuring communication reliability via QoS policies. The most important:

- **Reliability**:
  - `RELIABLE`: Guarantee delivery (TCP-like, resends lost messages)
  - `BEST_EFFORT`: No guarantees (UDP-like, faster but can drop messages)

- **Durability**:
  - `VOLATILE`: New subscribers only get messages published after subscription
  - `TRANSIENT_LOCAL`: New subscribers get recent message history

- **History**:
  - `KEEP_LAST(N)`: Buffer last N messages
  - `KEEP_ALL`: Buffer all messages (risk of memory growth)

**Example**: A camera publishing at 30 Hz uses `BEST_EFFORT` QoS (losing occasional frames is acceptable for real-time perception). A command topic uses `RELIABLE` QoS (every "emergency stop" command must arrive).

**Common Pitfall**: If a publisher uses `RELIABLE` QoS and a subscriber uses `BEST_EFFORT`, they are **incompatible** and will not communicate. ROS 2 provides tools (`ros2 topic info -v`) to debug QoS mismatches.

**Key Points**:
- DDS is transparent to most users—you don't need to understand DDS internals to use ROS 2
- QoS configuration is critical for robust systems (covered more in Chapter 1.2)
- ROS 2 supports multiple DDS implementations (Fast DDS by default, also Connext, Cyclone DDS)—you can switch via environment variables if needed

---

## Practical Example: Publisher and Subscriber in Python

### Overview

This example demonstrates the fundamental pub-sub pattern by creating two nodes: a `talker` that publishes string messages at 1 Hz, and a `listener` that subscribes to and prints these messages. This is the "Hello World" of ROS 2—simple yet illustrative of the core communication mechanism.

### Prerequisites

- Software: ROS 2 Humble installed, Python 3.10+
- Knowledge: Basic Python (classes, functions)
- Files: None required

### Implementation

**Step 1: Create the Publisher Node**

Create a file `publisher_node.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')  # Node name
        # Create publisher: topic name 'chatter', message type String, queue size 10
        self.publisher = self.create_publisher(String, 'chatter', 10)
        # Create timer: call timer_callback every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        self.get_logger().info('Talker node started, publishing to /chatter')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation**:
- `rclpy.init()` initializes the ROS 2 client library
- `TalkerNode` inherits from `Node`, providing ROS 2 functionality
- `create_publisher(String, 'chatter', 10)` creates a publisher for topic `/chatter` with queue size 10
- `create_timer(1.0, self.timer_callback)` calls `timer_callback` every 1 second
- `rclpy.spin(node)` keeps the node running and processing callbacks

**Step 2: Create the Subscriber Node**

Create a file `subscriber_node.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')  # Node name
        # Create subscriber: topic 'chatter', message type String, callback function
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10  # QoS profile: queue size 10
        )
        self.get_logger().info('Listener node started, subscribed to /chatter')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
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

**Explanation**:
- `create_subscription()` registers interest in topic `/chatter`
- `listener_callback(msg)` is called automatically whenever a message arrives
- Messages are processed asynchronously—the subscriber doesn't need to poll

**Step 3: Run the Nodes**

Make scripts executable:
```bash
chmod +x publisher_node.py subscriber_node.py
```

Run the publisher in one terminal:
```bash
source /opt/ros/humble/setup.bash
python3 publisher_node.py
```

Run the subscriber in another terminal:
```bash
source /opt/ros/humble/setup.bash
python3 subscriber_node.py
```

### Expected Output

**Publisher terminal**:
```
[INFO] [talker]: Talker node started, publishing to /chatter
[INFO] [talker]: Published: "Hello ROS 2! Count: 0"
[INFO] [talker]: Published: "Hello ROS 2! Count: 1"
[INFO] [talker]: Published: "Hello ROS 2! Count: 2"
...
```

**Subscriber terminal**:
```
[INFO] [listener]: Listener node started, subscribed to /chatter
[INFO] [listener]: Received: "Hello ROS 2! Count: 0"
[INFO] [listener]: Received: "Hello ROS 2! Count: 1"
[INFO] [listener]: Received: "Hello ROS 2! Count: 2"
...
```

### Troubleshooting

- **Issue**: `ModuleNotFoundError: No module named 'rclpy'`
  **Solution**: Ensure you sourced ROS 2: `source /opt/ros/humble/setup.bash`. Add this line to `~/.bashrc` to avoid repeating.

- **Issue**: Publisher and subscriber don't communicate (no messages received)
  **Solution**: Check that both nodes are using the same topic name (`/chatter`) and message type (`String`). Use `ros2 topic list` to verify topic exists, and `ros2 topic info /chatter -v` to check publishers/subscribers.

- **Issue**: "Permission denied" when running script
  **Solution**: Make script executable: `chmod +x publisher_node.py`

### Further Exploration

- Modify the publisher to send messages at different rates (0.1 Hz, 10 Hz, 100 Hz)—observe subscriber behavior
- Add a second subscriber to `/chatter`—both should receive messages independently
- Use `ros2 topic echo /chatter` to monitor messages from the command line without writing a subscriber
- Experiment with QoS: Change queue size to 1 and publish at high frequency—messages will be dropped if subscriber is slow

---

## Figures & Diagrams

### Figure 1.1-1: ROS 2 Computational Graph

![ROS 2 Computational Graph](../../diagrams/fig1.1-computational-graph.svg)

**Caption**: A ROS 2 computational graph showing four nodes communicating via topics (blue arrows), a service (green arrow), and an action (orange arrows). The `/camera` node publishes images to `/image_raw`, which both `/detector` and `/logger` subscribe to. The `/planner` node calls a `/get_map` service on `/map_server` and sends goals to the `/navigator` action server. This loose coupling allows nodes to be added, removed, or replaced without modifying other nodes.

**Reference**: This figure illustrates the computational graph concept discussed in Section 1, showing how nodes interact through typed communication channels without direct dependencies.

---

### Figure 1.1-2: Topics vs Services vs Actions Decision Tree

![Communication Pattern Decision Tree](../../diagrams/fig1.1-decision-tree.svg)

**Caption**: Decision tree for selecting the appropriate ROS 2 communication pattern. Start with "Is data continuous?" If yes, use topics (pub-sub). If no, ask "Do you need feedback during execution?" If yes, use actions (goal-feedback-result). If no, use services (request-response). Examples provided for each branch guide practical application.

**Reference**: This decision tree summarizes the communication patterns introduced in Sections 2-4, providing a practical guide for architecture decisions.

---

## Exercises

### Exercise 1: Modify Publisher Frequency (Difficulty: Easy)

**Objective**: Test your understanding of ROS 2 timers and publication rates

**Task**: Modify the `TalkerNode` from the practical example to publish at 10 Hz instead of 1 Hz. Observe how the subscriber handles the higher message rate.

**Requirements**:
- Change only one line of code (the timer period)
- Publisher should send 10 messages per second
- Subscriber should receive and print all messages

**Expected Outcome**:
Publisher terminal shows messages published 10 times per second:
```
[INFO] [talker]: Published: "Hello ROS 2! Count: 0"
[INFO] [talker]: Published: "Hello ROS 2! Count: 1"
...  (10 messages per second)
```
Subscriber terminal receives all messages at the same rate.

**Hints**:
- Timer period is specified in seconds in `create_timer(period, callback)`
- 10 Hz = 10 times per second = 1 message every 0.1 seconds

**Estimated Time**: 5 minutes

---

### Exercise 2: Create a Counter Subscriber (Difficulty: Medium)

**Objective**: Practice creating ROS 2 subscribers and processing message data

**Task**: Create a new subscriber node called `counter_node.py` that subscribes to `/chatter`, counts received messages, and prints statistics every 5 seconds (total messages received, average messages per second).

**Requirements**:
- Subscribe to `/chatter` topic
- Count messages received
- Every 5 seconds, print: "Received X messages (Y messages/sec average)"
- Handle node shutdown gracefully (Ctrl+C)

**Expected Outcome**:
```bash
$ python3 counter_node.py
[INFO] [counter]: Counter node started
[INFO] [counter]: Received 50 messages (10.0 messages/sec average)
[INFO] [counter]: Received 100 messages (10.0 messages/sec average)
...
```

**Hints**:
- Use `create_timer(5.0, self.stats_callback)` for periodic statistics printing
- Track message count in an instance variable (`self.msg_count`)
- Calculate average using total messages / elapsed time

**Estimated Time**: 15 minutes

---

### Exercise 3: Explore QoS Policies (Difficulty: Hard)

**Objective**: Understand Quality of Service and its impact on communication reliability

**Task**: Modify the publisher and subscriber to use different QoS settings:
1. Publisher with `RELIABLE` reliability, subscriber with `BEST_EFFORT`—observe they don't communicate
2. Both with `RELIABLE` reliability—observe successful communication
3. Publisher with queue depth 1, publish at 100 Hz, subscriber with slow processing (add `time.sleep(0.5)` in callback)—observe message loss

**Requirements**:
- Import QoS profiles: `from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy`
- Create custom QoS profiles for publisher and subscriber
- Document observations for each scenario in comments

**Expected Outcome**: Understanding that:
- QoS mismatch prevents communication
- Small queue sizes cause message drops with slow subscribers
- RELIABLE QoS provides delivery guarantees at cost of latency

**Hints**:
- Example QoS profile:
  ```python
  qos_profile = QoSProfile(
      reliability=ReliabilityPolicy.RELIABLE,
      history=HistoryPolicy.KEEP_LAST,
      depth=10
  )
  self.create_publisher(String, 'chatter', qos_profile)
  ```
- Use `ros2 topic info /chatter -v` to inspect QoS settings

**Estimated Time**: 30 minutes

---

## Summary & Key Takeaways

In this chapter, you learned:

- **ROS 2's computational graph** architecture enables loosely coupled nodes to communicate via topics, services, and actions—the foundation for scalable robot systems
- **Topics** provide asynchronous pub-sub communication ideal for continuous data streams (sensors, state)
- **Services** offer synchronous request-response for queries and quick commands
- **Actions** support long-running tasks with feedback and cancellation
- **DDS middleware** provides real-time, reliable communication with configurable Quality of Service policies
- **Practical implementation** of publisher-subscriber nodes in Python demonstrates core ROS 2 patterns

**Connection to Chapter 1.2**: Now that you understand the computational graph and basic pub-sub communication, Chapter 1.2 will dive deeper into services and actions, exploring advanced communication patterns including service client-server implementations, action goal management, and QoS policy tuning for real-world scenarios.

---

## Additional Resources

### Official Documentation
- **ROS 2 Documentation**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/) - Complete reference for all ROS 2 concepts
- **ROS 2 Tutorials**: [https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html) - Step-by-step guides from beginner to advanced
- **DDS-ROS 2 Middleware**: [https://design.ros2.org/articles/ros_on_dds.html](https://design.ros2.org/articles/ros_on_dds.html) - Design rationale for using DDS

### Recommended Reading
- **REP 2000**: ROS 2 platform support and release lifecycle - Understand LTS release strategy
- **ROS 2 QoS Policies**: [https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html) - Deep dive into QoS configuration

### Community Resources
- **ROS Discourse**: [https://discourse.ros.org/](https://discourse.ros.org/) - Official community forum for questions and discussions
- **ROS Answers**: [https://answers.ros.org/](https://answers.ros.org/) - Q&A site for ROS-specific problems
- **GitHub Discussions**: [https://github.com/ros2/ros2/discussions](https://github.com/ros2/ros2/discussions) - Development discussions and feature requests

---

## Notes for Instructors

**Teaching Tips**:
- **Start with visualization**: Run `rqt_graph` while students launch nodes to show the computational graph in real time—visual feedback solidifies abstract concepts
- **Common misconception**: Students often confuse topics with function calls. Emphasize that pub-sub is fire-and-forget (publisher doesn't wait for subscriber) unlike services (which block until response)
- **Debugging practice**: Intentionally introduce QoS mismatch (publisher RELIABLE, subscriber BEST_EFFORT) and have students use `ros2 topic info -v` to diagnose—builds troubleshooting skills early

**Lab Exercise Ideas**:
- **Multi-node system**: Have students create a simple pipeline: `sensor_node` (publishes random data) → `filter_node` (subscribes, filters, republishes) → `logger_node` (subscribes and logs). This demonstrates how data flows through topic chains.
- **Service integration**: Extend the example with a `/start_publishing` service that the talker node uses to enable/disable publishing based on service calls. Introduces service servers in familiar context.

**Assessment Suggestions**:
- **Code review criteria**: Check that students use appropriate QoS profiles (not all RELIABLE by default), handle node shutdown gracefully (destroy_node, rclpy.shutdown), and add descriptive log messages
- **Concept explanation rubric**: Can students explain in writing when to use topics vs services vs actions, with concrete examples from their own robot design?

**Time Management**:
- **Core lecture**: 60-75 minutes (computational graph 15 min, topics 20 min, services/actions 20 min, DDS 15 min, examples 10 min)
- **Hands-on lab**: 90 minutes (setup 15 min, implement pub-sub 30 min, exercises 45 min)
- **Homework**: Exercises 2-3 (30-45 minutes outside class)

---

**Chapter Metadata**:
- **Word Count**: ~2800 words (core concepts)
- **Figures**: 2 (computational graph, decision tree)
- **Code Examples**: 2 (publisher, subscriber)
- **Exercises**: 3 (easy, medium, hard)
- **Glossary Terms**: 8
- **Cross-References**: Prerequisites (Module 1 intro), Forward reference (Chapter 1.2)
