---
sidebar_position: 4
title: Chapter 2.3 - Unity for Robotics
---

# Chapter 2.3: Unity for Robotics

**Module**: 2 - The Digital Twin
**Week**: 7
**Estimated Reading Time**: 30 minutes

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. Install Unity and Robotics Hub packages
2. Set up ROS-TCP-Connector for Unity-ROS 2 communication
3. Import URDF models into Unity
4. Synchronize robot state between ROS 2 and Unity
5. Use Unity for photorealistic visualization

---

## Prerequisites

- Completed Chapters 2.1 and 2.2
- Unity 2022.3 LTS or later installed
- Understanding of ROS 2 topics and messages

---

## Introduction

**Unity** is a real-time 3D development platform originally designed for game development, now widely adopted for robotics simulation and visualization. Unlike Gazebo's focus on physics accuracy, Unity excels at **photorealistic rendering**, making it ideal for computer vision research, human-robot interaction studies, and demonstration videos.

### Why Unity for Robotics?

**Strengths**:
1. **Photorealistic Graphics**: Ray tracing, global illumination, post-processing effects
2. **Cross-Platform**: Windows, Linux, macOS, mobile, VR/AR headsets
3. **Asset Ecosystem**: Unity Asset Store (environments, 3D models, animations)
4. **ML-Agents**: Built-in reinforcement learning framework
5. **Real-Time Performance**: Optimized rendering pipeline (60+ FPS on laptops)

**Trade-offs vs. Gazebo**:
- **Physics**: Unity's PhysX is faster but less accurate than Bullet/DART
- **ROS Integration**: Requires TCP bridge (not native ROS 2)
- **Learning Curve**: C# scripting vs. Python/C++ in ROS ecosystem
- **Cost**: Free for hobbyists, paid licenses for commercial use

### Unity Robotics Hub

The **Unity Robotics Hub** is a collection of open-source packages that connect Unity with ROS/ROS 2:

1. **ROS-TCP-Connector**: Bidirectional message passing (Unity ↔ ROS 2)
2. **URDF Importer**: Converts URDF files to Unity GameObjects
3. **Articulation Body**: Unity's equivalent to ROS joints
4. **Robotics Object Pose Estimation**: AI-based 6D pose detection

**Installation**:
```bash
# 1. Install Unity Hub
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHubSetup.AppImage
chmod +x UnityHubSetup.AppImage
./UnityHubSetup.AppImage

# 2. Install Unity 2022.3 LTS via Hub GUI
# License: Personal (free for non-commercial use)

# 3. Create new Unity project
# Template: 3D (URP) for better graphics
```

---

## Key Terms

:::info Glossary Terms
- **Unity**: Game engine adapted for robotics simulation and visualization
- **ROS-TCP-Connector**: Unity package for ROS 2 communication
- **ArticulationBody**: Unity component for robot joint simulation
- **URDF Importer**: Tool to convert URDF to Unity GameObjects
- **GameObject**: Base entity in Unity scene (like links in URDF)
- **Prefab**: Reusable Unity asset (like xacro macros)
- **URP**: Universal Render Pipeline (optimized graphics)
:::

---

## Core Concepts

### 1. Unity Robotics Hub Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  Unity Editor (C#)                      │
│  ┌──────────────┐  ┌────────────────┐  ┌─────────────┐ │
│  │ URDF Importer│  │ ArticulationBody│  │ ROS-TCP-    │ │
│  │              │  │ (Joints/Physics) │  │ Connector   │ │
│  └──────────────┘  └────────────────┘  └──────┬──────┘ │
└────────────────────────────────────────────────┼────────┘
                                                  │ TCP Socket
┌─────────────────────────────────────────────────┼────────┐
│                  ROS 2 (Python/C++)             │        │
│  ┌──────────────────────────────────────────────▼──────┐ │
│  │ ROS-TCP-Endpoint (Bridge Node)                     │ │
│  │ - Subscribes to /joint_states, /tf, /cmd_vel       │ │
│  │ - Publishes to Unity-specific topics               │ │
│  └──────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

**Key Components**:

1. **Unity Side (C#)**:
   - Receives ROS messages as JSON/binary
   - Updates GameObject transforms and joint angles
   - Renders scene at 60+ FPS

2. **ROS 2 Side (Python)**:
   - `ROSConnection` node bridges Unity ↔ ROS 2
   - Translates ROS 2 messages to Unity-compatible format
   - Runs alongside your ROS 2 nodes

### 2. ROS-TCP-Connector Setup

#### Step 1: Install Unity Packages

1. Open Unity project
2. Window → Package Manager
3. Add package from git URL:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
   ```

#### Step 2: Configure Robotics Settings

1. Robotics → ROS Settings
2. Set ROS IP Address: `192.168.1.100` (your Ubuntu machine IP)
3. Set ROS Port: `10000` (default)
4. Protocol: ROS 2

#### Step 3: Install ROS 2 Endpoint

On Ubuntu machine:
```bash
# Install ROS-TCP-Endpoint package
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash

# Launch endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Verify Connection**:
```bash
# In Unity: Robotics → ROS Settings → Test Connection
# Expected: "Connection successful!"
```

### 3. URDF Import Workflow

Unity's URDF Importer converts ROS URDF files to Unity GameObjects with ArticulationBody components.

#### Example: Import TurtleBot3

**Step 1: Prepare URDF**
```bash
# Get TurtleBot3 URDF
sudo apt install ros-humble-turtlebot3-description
cd $(ros2 pkg prefix turtlebot3_description)/share/turtlebot3_description/urdf

# Copy to Unity project
cp -r ~/ros2_ws/src/turtlebot3/turtlebot3_description ~/UnityProjects/MyRobotProject/Assets/URDF/
```

**Step 2: Import in Unity**
1. Assets → Import Robot from URDF
2. Select `turtlebot3_burger.urdf`
3. Settings:
   - **Axis Type**: Y Axis (Unity uses Y-up, ROS uses Z-up)
   - **Mesh Decomposer**: VHACD (for accurate collisions)
   - **Override Inertia**: Yes (Unity PhysX requires balanced inertia)

**Step 3: Verify Import**
- Hierarchy window shows `turtlebot3_burger` with child links
- Inspector shows ArticulationBody on each link
- Scene view shows robot model

#### ArticulationBody Configuration

Unity's ArticulationBody replaces URDF joints:

**Continuous Joint** (e.g., wheel):
```csharp
ArticulationBody wheelBody = wheelGameObject.GetComponent<ArticulationBody>();
wheelBody.jointType = ArticulationJointType.RevoluteJoint;
wheelBody.matchAnchors = false;

// Set joint limits (optional for continuous)
wheelBody.twistLock = ArticulationDofLock.FreeMotion;

// Apply velocity (rad/s)
ArticulationDrive drive = wheelBody.xDrive;
drive.target = velocityRadPerSec;
wheelBody.xDrive = drive;
```

**Prismatic Joint** (e.g., elevator):
```csharp
ArticulationBody liftBody = liftGameObject.GetComponent<ArticulationBody>();
liftBody.jointType = ArticulationJointType.PrismaticJoint;

// Set limits
liftBody.linearLockY = ArticulationDofLock.LimitedMotion;
ArticulationDrive yDrive = liftBody.yDrive;
yDrive.lowerLimit = 0.0f;    // meters
yDrive.upperLimit = 1.0f;
liftBody.yDrive = yDrive;
```

### 4. Real-Time Synchronization

Synchronize Unity's robot with ROS 2's `/joint_states` topic.

#### ROS 2 Publisher (Python)

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.01, self.publish_joint_states)  # 100 Hz
        self.wheel_angle = 0.0

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['wheel_left_joint', 'wheel_right_joint']

        # Simulate rotating wheels
        self.wheel_angle += 0.1  # rad
        msg.position = [self.wheel_angle, self.wheel_angle]
        msg.velocity = [1.0, 1.0]  # rad/s

        self.publisher.publish(msg)
        self.get_logger().debug(f'Published joint states: {self.wheel_angle:.2f} rad')
```

#### Unity Subscriber (C#)

**File**: `Assets/Scripts/JointStateSubscriber.cs`
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    public ArticulationBody wheelLeft;
    public ArticulationBody wheelRight;

    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>("/joint_states", UpdateJointStates);
    }

    void UpdateJointStates(JointStateMsg msg)
    {
        // Find joint indices
        int leftIdx = System.Array.IndexOf(msg.name, "wheel_left_joint");
        int rightIdx = System.Array.IndexOf(msg.name, "wheel_right_joint");

        if (leftIdx != -1)
        {
            // Convert radians to degrees (Unity uses degrees)
            float angleDeg = (float)(msg.position[leftIdx] * Mathf.Rad2Deg);
            ArticulationDrive drive = wheelLeft.xDrive;
            drive.target = angleDeg;
            wheelLeft.xDrive = drive;
        }

        if (rightIdx != -1)
        {
            float angleDeg = (float)(msg.position[rightIdx] * Mathf.Rad2Deg);
            ArticulationDrive drive = wheelRight.xDrive;
            drive.target = angleDeg;
            wheelRight.xDrive = drive;
        }
    }
}
```

**Attach Script**:
1. Select robot root GameObject
2. Add Component → Joint State Subscriber
3. Assign wheel ArticulationBodies to public fields

---

## Practical Examples

### Example 1: Create Warehouse Environment

**Step 1: Import 3D Assets**
```
Asset Store → Search "Warehouse" → Download "Industrial Warehouse Pack"
Import to project
```

**Step 2: Build Scene**
1. Drag warehouse prefab to Scene
2. Add ground plane: GameObject → 3D Object → Plane
3. Scale plane: (10, 1, 10)
4. Add lighting: GameObject → Light → Directional Light

**Step 3: Add Robot**
1. Import TurtleBot3 URDF (see Section 3)
2. Position robot at origin (0, 0.05, 0)
3. Add camera: GameObject → Camera
4. Position camera above robot (0, 5, -5), rotate (45°, 0°, 0°)

**Step 4: Configure Physics**
1. Edit → Project Settings → Physics
2. Gravity: Y = -9.81 (matches ROS convention)
3. Default Solver Iterations: 6 (balance speed vs stability)

### Example 2: TurtleBot3 Teleoperation

**Goal**: Drive Unity robot using ROS 2 `teleop_twist_keyboard`

**Unity Script** (`Assets/Scripts/TwistSubscriber.cs`):
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class TwistSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    public ArticulationBody baseBody;

    // TurtleBot3 parameters
    public float wheelRadius = 0.033f;  // meters
    public float wheelSeparation = 0.160f;  // meters

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("/cmd_vel", ApplyTwist);
    }

    void ApplyTwist(TwistMsg msg)
    {
        float linearVel = (float)msg.linear.z;  // Unity Z-forward
        float angularVel = (float)msg.angular.y;  // Unity Y-up

        // Differential drive kinematics
        float leftVel = (linearVel - angularVel * wheelSeparation / 2.0f) / wheelRadius;
        float rightVel = (linearVel + angularVel * wheelSeparation / 2.0f) / wheelRadius;

        // Apply to base (simplified - normally apply to wheels)
        baseBody.velocity = new Vector3(0, 0, linearVel);
        baseBody.angularVelocity = new Vector3(0, angularVel, 0);
    }
}
```

**Test**:
```bash
# Terminal 1: Launch ROS-TCP-Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint

# Terminal 2: Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Press keys in Terminal 2, watch robot move in Unity
```

### Example 3: Camera Feed to ROS 2

**Goal**: Publish Unity camera images to ROS 2 `/camera/image_raw`

**Unity Script** (`Assets/Scripts/CameraPublisher.cs`):
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraPublisher : MonoBehaviour
{
    private ROSConnection ros;
    public Camera robotCamera;
    public int publishRate = 30;  // Hz

    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private float nextPublishTime = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>("/camera/image_raw");

        // Create RenderTexture for camera
        renderTexture = new RenderTexture(640, 480, 24);
        robotCamera.targetTexture = renderTexture;

        texture2D = new Texture2D(640, 480, TextureFormat.RGB24, false);
    }

    void Update()
    {
        if (Time.time >= nextPublishTime)
        {
            PublishImage();
            nextPublishTime = Time.time + 1.0f / publishRate;
        }
    }

    void PublishImage()
    {
        // Read pixels from RenderTexture
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, 640, 480), 0, 0);
        texture2D.Apply();

        // Convert to ROS Image message
        byte[] imageData = texture2D.GetRawTextureData();

        ImageMsg msg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1.0f) * 1e9)
                }
            },
            height = 480,
            width = 640,
            encoding = "rgb8",
            step = 640 * 3,
            data = imageData
        };

        ros.Publish("/camera/image_raw", msg);
    }
}
```

**View in ROS 2**:
```bash
# Terminal 1: Launch ROS-TCP-Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint

# Terminal 2: View image
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

---

## Performance Optimization

### 1. Graphics Settings

**For Real-Time Performance**:
- Edit → Project Settings → Quality
- Level: Medium (balance quality vs FPS)
- V-Sync: Off (reduces latency)
- Anti-Aliasing: MSAA 2x (good compromise)

**For Photorealistic Rendering**:
- Level: Ultra
- Enable Raytracing (RTX GPUs only)
- Post-Processing: Enable all effects

### 2. Physics Optimization

**Reduce Solver Iterations**:
```csharp
// For simple robots (differential drive)
Physics.defaultSolverIterations = 4;

// For complex manipulators (6+ DOF)
Physics.defaultSolverIterations = 10;
```

**Simplify Collision Meshes**:
- Use primitive colliders (Box, Sphere, Capsule) instead of Mesh Colliders
- Reduce mesh decomposer resolution in URDF import

### 3. Network Optimization

**Reduce Message Frequency**:
```csharp
// Publish camera at 10 Hz instead of 30 Hz
public int publishRate = 10;
```

**Compress Images**:
```csharp
// Use JPEG encoding instead of raw RGB
msg.encoding = "jpeg";
msg.data = texture2D.EncodeToJPG(75);  // Quality 75%
```

---

## Comparison: Unity vs Gazebo

| Feature | Unity | Gazebo |
|---------|-------|--------|
| **Graphics Quality** | Photorealistic (ray tracing) | Functional (OGRE) |
| **Physics Accuracy** | Medium (PhysX) | High (Bullet/DART) |
| **Learning Curve** | Moderate (C#) | Easy (Python/XML) |
| **ROS Integration** | TCP bridge (not native) | Native (ros_gz) |
| **Asset Ecosystem** | Unity Asset Store (100k+) | Gazebo Fuel (limited) |
| **ML Integration** | ML-Agents (built-in RL) | Custom (requires setup) |
| **Cross-Platform** | Windows/Linux/Mac/Mobile | Linux (primarily) |
| **Performance** | 60+ FPS (optimized) | 30-60 FPS (varies) |
| **Best Use Case** | Vision, HRI, demos | Physics, manipulation |

---

## Common Issues and Solutions

### Issue 1: Robot Falls Through Floor

**Cause**: Collision meshes not generated or too complex

**Solution**:
1. Re-import URDF with VHACD mesh decomposer
2. Simplify collision geometries in URDF to primitives:
```xml
<collision>
  <geometry>
    <box size="0.3 0.3 0.1"/>  <!-- Simple box instead of mesh -->
  </geometry>
</collision>
```

### Issue 2: Joints Jitter or Explode

**Cause**: Inertia values too small or unbalanced

**Solution**:
```csharp
// Override inertia for stable joints
ArticulationBody body = GetComponent<ArticulationBody>();
body.automaticInertiaTensor = false;
body.inertiaTensor = new Vector3(0.1f, 0.1f, 0.1f);
body.inertiaTensorRotation = Quaternion.identity;
```

### Issue 3: ROS Connection Timeout

**Symptom**: "Failed to connect to ROS endpoint"

**Debug Steps**:
```bash
# 1. Check ROS-TCP-Endpoint is running
ps aux | grep ros_tcp_endpoint

# 2. Verify IP address
ifconfig  # Note IP (e.g., 192.168.1.100)

# 3. Test connection
telnet 192.168.1.100 10000
# Expected: Connection successful

# 4. Check firewall
sudo ufw allow 10000/tcp
```

---

## Summary

This chapter introduced **Unity for robotics simulation**:

1. **Unity Robotics Hub**: ROS-TCP-Connector bridges Unity ↔ ROS 2
2. **URDF Import**: Converts ROS robots to Unity GameObjects
3. **ArticulationBody**: Unity's joint simulation system
4. **Real-Time Sync**: Bidirectional communication (joint states, cmd_vel, camera)
5. **Graphics Excellence**: Photorealistic rendering for vision research

**Key Takeaways**:
- Unity excels at visualization, Gazebo at physics accuracy
- Use Unity for computer vision datasets, human-robot interaction studies
- ROS-TCP bridge adds latency (~10-50ms) vs native ROS 2
- Asset Store provides rich environments (offices, warehouses, outdoor)

**Next Steps**:
- Chapter 2.4: Sensor simulation and visual SLAM
- Unity ML-Agents for reinforcement learning (Module 3)

---

## End-of-Chapter Exercises

### Exercise 1: Create Unity Digital Twin (Difficulty: Medium)

**Objective**: Mirror a real ROS 2 robot in Unity with synchronized joint states.

**Tasks**:
1. Import TurtleBot3 URDF into Unity
2. Launch ROS-TCP-Endpoint on Ubuntu machine
3. Publish `/joint_states` from ROS 2 at 10 Hz
4. Create C# subscriber to update Unity robot in real-time
5. Add camera to Unity, publish images to `/camera/image_raw`

**Success Criteria**:
- Unity robot mirrors ROS 2 robot motion with <100ms latency
- Camera feed visible in `rqt_image_view`
- No joint jitter or physics explosions

### Exercise 2: Warehouse Navigation Visualization (Difficulty: Hard)

**Scenario**: Visualize Nav2 path planning in photorealistic Unity environment.

**Tasks**:
1. Import warehouse 3D assets from Unity Asset Store
2. Import TurtleBot3 and configure ArticulationBodies
3. Subscribe to `/plan` (Path message) in Unity
4. Draw path as LineRenderer in Unity scene
5. Subscribe to `/cmd_vel` and move robot along path

**Success Criteria**:
- Path visible as colored line in 3D space
- Robot follows path smoothly
- Unity scene maintains 60+ FPS

### Exercise 3: Multi-Camera Dataset Generation (Difficulty: Advanced)

**Objective**: Generate synthetic training data with multiple camera angles.

**Tasks**:
1. Create Unity scene with varied lighting conditions
2. Place 4 cameras around robot (front, back, left, right)
3. Publish all 4 camera feeds to different topics
4. Record rosbag with images and robot pose
5. Verify dataset contains 10,000+ images with labels

**Success Criteria**:
- Rosbag size > 5GB
- Images show varied lighting, shadows, reflections
- Camera info messages correct (intrinsics, extrinsics)

---

## Further Reading

### Required
1. Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub
2. ROS-TCP-Connector: https://github.com/Unity-Technologies/ROS-TCP-Connector
3. Unity Manual - Physics: https://docs.unity3d.com/Manual/PhysicsSection.html

### Optional
4. "Using Unity for Robotics Simulation" (Unity blog post)
5. Unity ML-Agents Documentation: https://github.com/Unity-Technologies/ml-agents
6. ROS# (alternative Unity-ROS bridge): https://github.com/siemens/ros-sharp

---

## Next Chapter

Continue to **[Chapter 2.4: Sensor Simulation and VSLAM](./chapter2-4-sensors-vslam)** to learn advanced sensor modeling and visual SLAM algorithms.
