# Research Notes: Unity for Robotics

**Task**: 009
**Date**: 2025-12-10
**Status**: Complete
**Sources**: Unity Robotics Hub Official Documentation

---

## Official Documentation Sources

- **Unity Robotics Hub**: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- **ROS-TCP-Connector**: https://github.com/Unity-Technologies/ROS-TCP-Connector
- **URDF Importer**: https://github.com/Unity-Technologies/URDF-Importer
- **Unity Learn**: https://learn.unity.com/

---

## Why Unity for Robotics?

**Advantages over Gazebo**:
- **High-fidelity graphics**: Photorealistic rendering for vision algorithms
- **Synthetic data generation**: Large-scale labeled datasets
- **GPU acceleration**: Native NVIDIA GPU support
- **Computer vision**: Integration with ML frameworks
- **VR/AR support**: Immersive robot teleoperation
- **Asset store**: Pre-built environments and models

**Use Cases**:
- Training vision models (object detection, segmentation)
- Sim-to-real transfer with domain randomization
- Human-robot interaction visualization
- Digital twin visualization
- Multi-robot system visualization

---

## Unity Robotics Architecture

```
┌─────────────────────┐         ┌──────────────────────┐
│   Unity Simulation  │         │   ROS 2 System       │
│                     │         │                      │
│  ┌───────────────┐  │         │  ┌────────────────┐  │
│  │ Robot Model   │  │◄────────┤  │ ROS 2 Nodes    │  │
│  │ (URDF Import) │  │  TCP    │  │                │  │
│  └───────────────┘  │◄────────┤  │ Controllers    │  │
│                     │         │  │                │  │
│  ┌───────────────┐  │         │  │ Planners       │  │
│  │ Sensors       │  ├────────►│  └────────────────┘  │
│  │ (Camera, etc) │  │         │                      │
│  └───────────────┘  │         │                      │
└─────────────────────┘         └──────────────────────┘
```

---

## Key Components

### 1. ROS-TCP-Connector

**Purpose**: Enable Unity-ROS 2 communication over TCP.

**Features**:
- Publish/subscribe to ROS 2 topics
- Call ROS 2 services
- Send/receive custom messages
- Network-based (no DDS required in Unity)

**Installation**:
```bash
# Unity Package Manager
# Add package from git URL:
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

**ROS 2 Side (ROS-TCP-Endpoint)**:
```bash
sudo apt install ros-humble-ros-tcp-endpoint

# Or from source
cd ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint
cd ../
colcon build --packages-select ros_tcp_endpoint
```

**Launch Endpoint**:
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

---

### 2. URDF Importer

**Purpose**: Import ROS URDF models into Unity.

**Features**:
- Converts URDF to Unity GameObjects
- Maps joints to Unity articulation bodies
- Imports meshes (STL, DAE, OBJ)
- Preserves joint limits and dynamics
- Creates controllers for joint actuation

**Usage**:
1. Install URDF Importer package
2. Assets → Import Robot from URDF → Select .urdf file
3. Unity creates GameObject hierarchy with ArticulationBody components

---

### 3. Unity Perception Package

**Purpose**: Generate synthetic datasets for computer vision.

**Features**:
- Bounding box annotations
- Semantic segmentation
- Instance segmentation
- Keypoint labeling
- Depth maps
- Random domain generation

**Use Case**: Train object detection models on synthetic humanoid robot data.

---

## Unity-ROS 2 Message Passing

### Publishing from Unity to ROS 2

**Unity C# Script**:
```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class ROSPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "unity_topic";
    public float publishRate = 10f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
        InvokeRepeating("PublishMessage", 1f, 1f / publishRate);
    }

    void PublishMessage()
    {
        StringMsg msg = new StringMsg("Hello from Unity");
        ros.Publish(topicName, msg);
    }
}
```

### Subscribing in Unity to ROS 2

**Unity C# Script**:
```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class ROSSubscriber : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "ros_topic";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>(topicName, ReceiveMessage);
    }

    void ReceiveMessage(StringMsg msg)
    {
        Debug.Log($"Received: {msg.data}");
    }
}
```

---

## Unity Physics for Robotics

### ArticulationBody

**Purpose**: Unity's component for robotic joints (replaces Rigidbody for articulated robots).

**Joint Types**:
- **Fixed**: No motion
- **Revolute**: Rotation around axis
- **Prismatic**: Linear motion
- **Spherical**: Ball-and-socket

**Joint Drive Types**:
- **Position**: Target angle/position
- **Velocity**: Target velocity
- **Force**: Applied force/torque

**Example Configuration**:
```csharp
ArticulationBody joint = GetComponent<ArticulationBody>();
ArticulationDrive drive = joint.xDrive;
drive.stiffness = 10000;
drive.damping = 100;
drive.forceLimit = 1000;
drive.target = 45;  // Target angle in degrees
joint.xDrive = drive;
```

---

## Unity Sensors for Robotics

### Camera Sensors

**RGB Camera**:
```csharp
Camera cam = GetComponent<Camera>();
RenderTexture rt = new RenderTexture(640, 480, 24);
cam.targetTexture = rt;
```

**Depth Camera** (using Perception package):
```csharp
// Add Perception Camera component
// Enable depth output
// Publish depth map as sensor_msgs/Image to ROS 2
```

### Simulating RealSense D435
- Use Unity camera with RGB + Depth
- Match field of view (69° H × 42° V)
- Set resolution to 640×480 or 1280×720
- Add noise to depth data for realism

---

## Unity Environments for Humanoid Robots

### Recommended Assets
- **Realistic indoor environments**: Architectural visualization assets
- **Outdoor environments**: Terrain tools, vegetation
- **Obstacles and props**: Furniture, doors, stairs
- **Lighting**: HDRP for realistic lighting

### Performance Optimization
- Use LOD (Level of Detail) for distant objects
- Occlusion culling
- GPU instancing for repeated objects
- Simplified collision meshes
- Target frame rate: 60 FPS for real-time

---

## Digital Twin with Unity

### Architecture
```
Physical Robot → Sensors → ROS 2 → TCP → Unity Visualization
                                    ↓
Unity Simulation → Control Commands → ROS 2 → Physical Robot
```

### Features
- **State synchronization**: Mirror robot pose in Unity
- **Sensor visualization**: Display camera feeds, LiDAR
- **Trajectory preview**: Show planned paths
- **Teleoperation**: Control robot from Unity VR interface

---

## Unity ML-Agents (Optional Advanced Topic)

**Purpose**: Train RL agents for robot control using Unity.

**Features**:
- PPO, SAC, POCA algorithms
- Integration with PyTorch
- Parallel environments for faster training
- Curriculum learning support

**Use Case**: Train bipedal locomotion controllers.

---

## Installation

### Unity Hub and Editor
```
1. Download Unity Hub: https://unity.com/download
2. Install Unity Editor 2021.3 LTS or 2022.3 LTS
3. Include modules:
   - Linux Build Support
   - Visual Studio (for scripting)
```

### Unity Robotics Packages
```
1. Open Unity project
2. Window → Package Manager
3. Add packages from git URL:
   - ROS-TCP-Connector
   - URDF-Importer
   - Perception (optional)
```

---

## Chapter Topics

**Chapter 2.3: Unity for Robotics**
- Unity Robotics Hub overview
- URDF import and robot setup
- ROS-TCP-Connector configuration
- Publishing/subscribing to ROS 2 topics
- Camera and sensor simulation
- Digital twin visualization

---

## References

1. Unity Technologies. (2024). *Unity Robotics Hub*. GitHub. https://github.com/Unity-Technologies/Unity-Robotics-Hub
2. Unity Technologies. (2024). *ROS-TCP-Connector documentation*. GitHub. https://github.com/Unity-Technologies/ROS-TCP-Connector
3. Juliani, A., et al. (2020). Unity: A general platform for intelligent agents. *arXiv preprint* arXiv:1809.02627. https://arxiv.org/abs/1809.02627

---

**Status**: ✅ Research complete. Ready for chapter writing.
