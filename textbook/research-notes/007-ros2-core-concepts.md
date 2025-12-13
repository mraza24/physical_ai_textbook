# Research Notes: ROS 2 Core Concepts

**Task**: 007
**Date**: 2025-12-10
**Status**: Complete
**Sources**: ROS 2 Official Documentation (Humble Hawksbill)

---

## Official Documentation Sources

- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **ROS 2 Concepts**: https://docs.ros.org/en/humble/Concepts.html
- **ROS 2 Design**: https://design.ros2.org/
- **ROS 2 Tutorials**: https://docs.ros.org/en/humble/Tutorials.html

---

## Core ROS 2 Concepts

### 1. Nodes

**Definition**: Independent processes that perform computation.

**Key Characteristics**:
- Written in C++ or Python
- Communicate via topics, services, actions, parameters
- Can be composed into single processes (component nodes)
- Support lifecycle management (unconfigured, inactive, active, finalized)

**API Reference**: `rclcpp` (C++), `rclpy` (Python)

**Example Node Creation**:
```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node started')

def main():
    rclpy.init()
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

### 2. Topics

**Definition**: Named buses for asynchronous, many-to-many publish-subscribe communication.

**Key Characteristics**:
- Unidirectional data flow
- Publishers send messages, subscribers receive
- Multiple publishers/subscribers per topic
- QoS (Quality of Service) policies configurable
- Message types defined in `.msg` files

**Message Types**: `std_msgs`, `sensor_msgs`, `geometry_msgs`, custom messages

**QoS Policies**:
- **Reliability**: Best effort vs reliable
- **Durability**: Volatile vs transient local
- **History**: Keep last N vs keep all
- **Deadline**: Maximum expected time between messages
- **Lifespan**: Maximum duration of message validity

**Example Publisher/Subscriber**:
```python
# Publisher
publisher = node.create_publisher(String, 'topic_name', 10)
publisher.publish(String(data='Hello ROS 2'))

# Subscriber
def callback(msg):
    node.get_logger().info(f'Received: {msg.data}')
subscriber = node.create_subscription(String, 'topic_name', callback, 10)
```

---

### 3. Services

**Definition**: Synchronous request-response communication between nodes.

**Key Characteristics**:
- Client sends request, server sends response
- Blocking by default (async options available)
- One-to-one communication
- Service types defined in `.srv` files
- Suitable for short-duration tasks (configuration, queries)

**Service Interface**:
```
# Example: AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

**Example Service Server/Client**:
```python
# Server
def add_callback(request, response):
    response.sum = request.a + request.b
    return response

service = node.create_service(AddTwoInts, 'add_two_ints', add_callback)

# Client
client = node.create_client(AddTwoInts, 'add_two_ints')
request = AddTwoInts.Request()
request.a = 5
request.b = 3
future = client.call_async(request)
```

---

### 4. Actions

**Definition**: Asynchronous request-response with feedback and cancellation.

**Key Characteristics**:
- Three communication patterns: goal, feedback, result
- Long-running tasks (navigation, manipulation)
- Preemptable (can be cancelled mid-execution)
- Action types defined in `.action` files
- Built on top of topics and services

**Action Interface**:
```
# Goal
geometry_msgs/PoseStamped target_pose
---
# Result
bool success
---
# Feedback
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
```

**Example Action Server/Client**:
```python
# Server provides feedback during execution
# Client can monitor progress, cancel, or wait for result
```

---

### 5. Parameters

**Definition**: Configuration values for nodes, stored in parameter server.

**Key Characteristics**:
- Per-node parameter storage
- Types: bool, int, double, string, byte arrays, lists
- Can be declared with default values
- Can be updated dynamically
- Loaded from YAML files via launch files

**Example Parameter Usage**:
```python
# Declare parameter
node.declare_parameter('my_param', 42)

# Get parameter
value = node.get_parameter('my_param').value

# Set parameter (from client)
node.set_parameters([rclpy.parameter.Parameter('my_param', value=100)])
```

---

### 6. DDS (Data Distribution Service)

**Definition**: Underlying middleware for ROS 2 communication.

**Key Characteristics**:
- Industry standard (OMG DDS)
- Multiple vendor implementations: Fast DDS, CycloneDDS, RTI Connext
- Discovery: Automatic node/topic discovery on network
- Zero-copy transport for efficiency
- Security: DDS-Security for encrypted communication

**DDS Implementations**:
- **Fast DDS** (default): eProsima, used in most ROS 2 installations
- **CycloneDDS**: Eclipse Foundation, lightweight alternative
- **RTI Connext**: Commercial-grade, used in safety-critical systems

**Environment Variable**:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp  # Fast DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # Cyclone DDS
```

---

### 7. ROS 2 Graph Architecture

**Computational Graph Components**:
- **Nodes**: Processes
- **Topics**: Data buses
- **Services**: Request-response
- **Actions**: Long-running tasks
- **Parameters**: Configuration

**Introspection Tools**:
```bash
ros2 node list           # List all nodes
ros2 topic list          # List all topics
ros2 topic echo /topic   # Echo topic data
ros2 topic hz /topic     # Measure topic frequency
ros2 service list        # List all services
ros2 action list         # List all actions
ros2 run rqt_graph rqt_graph  # Visualize graph
```

---

## ROS 2 vs ROS 1 Key Differences

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Architecture** | roscore master required | Decentralized (DDS) |
| **Languages** | C++, Python | C++, Python (+ others) |
| **Real-time** | Limited | RTOS support |
| **Security** | None | DDS-Security |
| **Cross-platform** | Linux only | Linux, Windows, macOS |
| **QoS** | Limited | Extensive QoS policies |
| **Lifecycle** | No standardized lifecycle | Managed lifecycle |

---

## ROS 2 Distributions

| Distribution | Release Date | Ubuntu | Support Until |
|--------------|--------------|--------|---------------|
| Foxy Fitzroy | June 2020 | 20.04 | May 2023 |
| Galactic Geochelone | May 2021 | 20.04 | November 2022 |
| Humble Hawksbill | May 2022 | 22.04 | **May 2027 (LTS)** |
| Iron Irwini | May 2023 | 22.04 | November 2024 |
| Jazzy Jalisco | May 2024 | 24.04 | May 2029 (LTS) |

**Recommended for Textbook**: **Humble Hawksbill** (LTS, widely adopted)

---

## Key Packages

### Essential Packages
- **rclcpp**: ROS 2 C++ client library
- **rclpy**: ROS 2 Python client library
- **std_msgs**: Standard message types
- **sensor_msgs**: Sensor data messages (Image, PointCloud2, LaserScan)
- **geometry_msgs**: Geometric primitives (Pose, Twist, Transform)
- **nav_msgs**: Navigation messages (Odometry, Path, OccupancyGrid)

### Common Utility Packages
- **tf2**: Transform library (coordinate frames)
- **rosbag2**: Data recording and playback
- **rqt**: GUI tools for debugging
- **rviz2**: 3D visualization tool

---

## Installation (Ubuntu 22.04)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop  # Full install with GUI tools

# Source setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Workspace Structure

```
ros2_ws/
├── src/                    # Source code
│   ├── package1/
│   │   ├── package.xml     # Package manifest
│   │   ├── CMakeLists.txt  # Build configuration
│   │   ├── src/            # C++ source
│   │   ├── include/        # C++ headers
│   │   └── launch/         # Launch files
│   └── package2/
├── build/                  # Build artifacts (auto-generated)
├── install/                # Install space (auto-generated)
└── log/                    # Build logs (auto-generated)
```

---

## Build Tool: colcon

```bash
# Build workspace
cd ros2_ws
colcon build

# Build specific package
colcon build --packages-select my_package

# Build with symlinks (Python development)
colcon build --symlink-install

# Source install space
source install/setup.bash
```

---

## Best Practices

1. **Use Lifecycle Nodes** for managed startup/shutdown
2. **Configure QoS** appropriately for each topic
3. **Use Composition** to reduce process overhead
4. **Implement Namespaces** for multi-robot systems
5. **Use tf2** for all coordinate transformations
6. **Record Data with rosbag2** for debugging and testing
7. **Write Unit Tests** using gtest (C++) or pytest (Python)

---

## References for Chapters

1. Open Robotics. (2024). *ROS 2 Humble documentation*. Retrieved December 10, 2025, from https://docs.ros.org/en/humble/
2. Open Robotics. (2024). *ROS 2 design documentation*. Retrieved December 10, 2025, from https://design.ros2.org/
3. Macenski, S., Foote, T., Gerkey, B., Lalancette, C., & Woodall, W. (2022). Robot Operating System 2: Design, architecture, and uses in the wild. *Science Robotics*, *7*(66). https://doi.org/10.1126/scirobotics.abm6074

---

## Topics for Chapter Coverage

**Chapter 1.1: ROS 2 Fundamentals**
- ROS 2 architecture overview
- Nodes, topics, services, actions
- DDS middleware
- Basic publish-subscribe example

**Chapter 1.2: Nodes and Communication**
- Node lifecycle management
- QoS policies
- Intra-process communication
- Multi-node systems

**Chapter 1.3: Launch Files and Configuration**
- Launch file syntax (Python)
- Parameter YAML files
- Node composition
- Namespaces and remapping

**Chapter 1.4: Building ROS 2 Packages**
- Workspace setup
- Package creation (C++ and Python)
- package.xml and CMakeLists.txt
- Building and installing with colcon

---

**Status**: ✅ Research complete. Ready for chapter writing.
