# Chapter 2.2: Gazebo Simulation

> **Module**: 2 - Digital Twin Simulation
> **Week**: 6
> **Estimated Reading Time**: 28 minutes

---

## Summary

Gazebo is the most widely-used open-source robot simulator in the ROS ecosystem, providing physics-based simulation, sensor plugins, and seamless ROS 2 integration. This chapter covers Gazebo architecture, URDF/SDF model formats, physics engine configuration, and practical simulation workflows.

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Distinguish** between Gazebo Classic (version 11) and new Gazebo (formerly Ignition), and choose appropriate version
2. **Convert** URDF robot descriptions to SDF format and add Gazebo-specific plugins
3. **Configure** physics engines (ODE, Bullet, DART) with appropriate time steps and solver parameters
4. **Implement** sensor plugins (camera, depth camera, LiDAR) with realistic noise models
5. **Launch** Gazebo worlds with ROS 2 integration using `ros2_control` and Gazebo plugins

**Prerequisite Knowledge**: Chapter 2.1 (Digital Twin Concepts), Chapter 1.3 (Launch Files), basic XML syntax

---

## Key Terms

This chapter introduces the following technical terms (see Glossary for detailed definitions):

- **Gazebo Classic**: Legacy simulator (versions 1-11), end-of-life January 2025, tightly coupled to ROS 1 and early ROS 2
- **Gazebo (New)**: Modern simulator (formerly Ignition Gazebo), modular architecture, designed for ROS 2 from the ground up
- **SDF (Simulation Description Format)**: XML format for complete world descriptions including robots, environments, physics, and sensors
- **Gazebo Plugin**: Shared library that extends simulator functionality (sensors, actuators, custom physics, GUI widgets)
- **World File**: SDF file describing the complete simulation environment (ground plane, lighting, models, physics settings)
- **Model**: Self-contained robot or object definition in SDF format (links, joints, sensors, plugins)
- **Spawn**: Action of inserting a model into a running Gazebo world via service call
- **URDF-to-SDF Conversion**: Automatic translation from URDF (ROS-native format) to SDF (Gazebo-native format) at runtime

---

## Core Concepts

### 1. Gazebo Classic vs. New Gazebo

As of January 2025, Gazebo is transitioning from "Gazebo Classic" (version 11, end-of-life) to "new Gazebo" (codenamed Ignition Fortress, Fortress LTS, later renamed back to just "Gazebo"). Understanding the differences is critical for choosing the right tool.

#### Gazebo Classic (Version 11)

**Status**: ❌ **End-of-life January 2025** (no further updates, security patches only until Ubuntu 22.04 EOL in 2027)

**Architecture**:
- Monolithic design (physics, rendering, GUI in single process)
- Tight coupling to ROS 1 (via `gazebo_ros_pkgs`)
- ROS 2 support added later via `gazebo_ros2_control`

**Pros**:
- ✅ Mature, stable, extensive documentation
- ✅ Large ecosystem of existing models and worlds
- ✅ Well-tested with ROS 2 Humble (common pairing for Ubuntu 22.04)

**Cons**:
- ❌ No new features after EOL
- ❌ Monolithic architecture limits performance (single-threaded physics)
- ❌ ODE physics engine default (adequate but not state-of-the-art)

**When to Use**:
- Legacy projects already using Gazebo Classic
- Ubuntu 22.04 + ROS 2 Humble deployments (pre-installed via `ros-humble-gazebo-*`)
- Courses/tutorials written for Classic (most existing materials)

---

#### New Gazebo (Fortress, Garden, Harmonic releases)

**Status**: ✅ **Active development**, LTS releases every 2 years

**Architecture**:
- Modular design (separate processes for physics, rendering, GUI)
- Native ROS 2 integration from day one
- Plugin-based everything (sensors, actuators, custom physics)

**Pros**:
- ✅ Modern C++17 codebase, multithreaded physics
- ✅ Better rendering (Ogre2 with PBR materials, ray tracing support)
- ✅ Distributed simulation (run physics/rendering on separate machines)

**Cons**:
- ❌ Smaller ecosystem (fewer pre-built models)
- ❌ Breaking changes between versions (Fortress → Garden → Harmonic)
- ❌ Steeper learning curve (more configuration required)

**When to Use**:
- New projects starting in 2025 or later
- High-performance needs (large worlds, many robots, GPU rendering)
- Distributed simulation (cloud-based robotics)

---

**Migration Path**:

```
Gazebo Classic 11 (Ubuntu 22.04, ROS 2 Humble)
           ↓
  [Recommended for this textbook: use Classic for consistency with Humble]
           ↓
New Gazebo Fortress+ (Ubuntu 24.04, ROS 2 Jazzy)
           ↓
  [Future: transition to new Gazebo when ROS 2 moves beyond Humble LTS]
```

**Textbook Decision**: This chapter focuses on **Gazebo Classic 11** paired with **ROS 2 Humble** (dominant configuration as of 2025), with notes on new Gazebo differences where relevant.

---

### 2. URDF vs. SDF: Two Formats, One Ecosystem

URDF (Unified Robot Description Format) and SDF (Simulation Description Format) are both XML-based, but serve different purposes.

#### URDF: ROS-Native Robot Description

**Purpose**: Describe robot kinematics and dynamics for ROS tools (`robot_state_publisher`, `joint_state_publisher`, `tf2`)

**Strengths**:
- ✅ Simple, focused on robots (not environments)
- ✅ Well-integrated with ROS visualization (RViz) and planning (MoveIt)
- ✅ Xacro support for macros, conditionals, math

**Limitations**:
- ❌ No simulation-specific features (sensor plugins, physics settings)
- ❌ Cannot describe complete worlds (ground plane, lighting, multiple models)
- ❌ Limited inertia specification (no off-diagonal terms in inertia matrix)

**Example** (from Chapter 2.1):
```xml
<link name="base_link">
  <visual>
    <geometry><box size="0.5 0.3 0.1"/></geometry>
  </visual>
  <collision>
    <geometry><box size="0.5 0.3 0.1"/></geometry>
  </collision>
  <inertial>
    <mass value="10.0"/>
    <inertia ixx="0.083" iyy="0.225" izz="0.292"
             ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>
```

---

#### SDF: Gazebo-Native Simulation Format

**Purpose**: Describe complete simulation worlds (robots, environments, physics, sensors, plugins)

**Strengths**:
- ✅ Comprehensive: supports lights, cameras, physics settings, sensor noise
- ✅ Modular: models can be nested and reused
- ✅ Plugin system: extend with custom sensors, actuators, world dynamics

**Limitations**:
- ❌ More complex than URDF (steeper learning curve)
- ❌ Less ROS tooling support (no RViz direct loading, no xacro)

**Example**:
```xml
<sdf version="1.7">
  <model name="mobile_robot">
    <link name="base_link">
      <visual name="visual">
        <geometry><box><size>0.5 0.3 0.1</size></box></geometry>
      </visual>
      <collision name="collision">
        <geometry><box><size>0.5 0.3 0.1</size></box></geometry>
      </collision>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.083</ixx><iyy>0.225</iyy><izz>0.292</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <sensor name="camera" type="camera">
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image><width>640</width><height>480</height></image>
        </camera>
        <plugin name="camera_plugin" filename="libgazebo_ros_camera.so"/>
      </sensor>
    </link>
  </model>
</sdf>
```

**Key Differences**:

| Feature | URDF | SDF |
|---------|------|-----|
| **Purpose** | Robot description | Complete world description |
| **Sensors** | ❌ No native support | ✅ First-class elements |
| **Plugins** | ❌ Via `<gazebo>` tags | ✅ Native `<plugin>` elements |
| **Worlds** | ❌ Robot only | ✅ Environments, lighting, physics |
| **Macros** | ✅ Xacro support | ❌ No macro system (use includes) |
| **ROS Integration** | ✅ Direct (RViz, MoveIt) | ⚠️ Via conversion or Gazebo |

---

#### URDF-to-SDF Conversion

Gazebo automatically converts URDF to SDF at runtime using `<gazebo>` tags to add simulation-specific features.

**Example**: Adding a camera plugin to URDF

```xml
<!-- URDF: Base robot definition -->
<robot name="mobile_robot">
  <link name="camera_link">
    <visual>
      <geometry><box size="0.05 0.05 0.05"/></geometry>
    </visual>
  </link>
</robot>

<!-- Gazebo tags: Simulation-specific additions -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov> <!-- 80 degrees -->
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>image_raw:=camera/image</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

**Conversion Process**:
1. Gazebo reads URDF (parses links, joints, inertias)
2. Converts to SDF (adds default collision properties, sensor configs)
3. Applies `<gazebo>` tags (overlays plugins, custom physics)
4. Loads resulting SDF model into simulation

**Best Practice**: Use URDF for robot structure (portable to RViz, MoveIt), add `<gazebo>` tags for simulation features.

---

### 3. Physics Engine Configuration

Gazebo supports multiple physics engines via a plugin architecture. Choosing and tuning the engine affects simulation accuracy, speed, and stability.

#### Selecting a Physics Engine

In `world.sdf` or via command-line:

```xml
<world name="default">
  <physics name="default_physics" type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>
</world>
```

**Options for `type` attribute**:
- `ode`: Open Dynamics Engine (default, fast, stable for mobile robots)
- `bullet`: Bullet Physics (good for legged robots, GPU broad-phase)
- `dart`: DART (accurate, slower, best for manipulation)
- `simbody`: Simbody (slow, biomechanics focus)

---

#### Key Physics Parameters

**1. Max Step Size** (`max_step_size`):
- **Meaning**: Simulation time step (seconds per iteration)
- **Typical values**: 0.001 (1 ms) for real-time, 0.0001 (0.1 ms) for high precision
- **Trade-off**: Smaller steps = more accurate but slower

**Example**:
```xml
<max_step_size>0.001</max_step_size>  <!-- 1 kHz update rate -->
```

If simulation runs slower than real-time (e.g., 0.5x speed), increase step size to 0.002 or reduce scene complexity.

---

**2. Real-Time Factor** (`real_time_factor`):
- **Meaning**: Target simulation speed relative to wall-clock time
- **Values**: 1.0 = real-time, 2.0 = 2x faster, 0.5 = 2x slower
- **Use case**: Set > 1.0 for faster-than-real-time training, < 1.0 for complex scenes

**Example**:
```xml
<real_time_factor>1.0</real_time_factor>  <!-- Run at wall-clock speed -->
```

**Caveat**: Gazebo cannot always achieve target factor—if physics is too slow, actual factor will be lower (e.g., target 1.0, achieve 0.7).

---

**3. Solver Iterations**:
- **Meaning**: Number of iterations to solve constraints (contacts, joints)
- **Typical values**: 50-200 (ODE), 10-50 (Bullet)
- **Trade-off**: More iterations = more stable but slower

**Example** (ODE-specific):
```xml
<physics name="ode_physics" type="ode">
  <ode>
    <solver>
      <type>quick</type>  <!-- Fast iterative solver -->
      <iters>50</iters>   <!-- Constraint solver iterations -->
      <sor>1.3</sor>      <!-- Successive over-relaxation parameter -->
    </solver>
    <constraints>
      <cfm>0.0</cfm>  <!-- Constraint force mixing (softness) -->
      <erp>0.2</erp>  <!-- Error reduction parameter (stiffness) -->
    </constraints>
  </ode>
</physics>
```

**Common Issue**: Robots vibrating or jittering → Increase `iters`, decrease `cfm`, increase `erp`.

---

**4. Gravity**:
```xml
<gravity>0 0 -9.81</gravity>  <!-- Earth gravity in m/s² -->
```

For space robotics or underwater simulation, modify accordingly (e.g., `0 0 -1.62` for lunar gravity).

---

#### Benchmarking Physics Engines

**Test Scene**: Mobile robot with 4 wheels, 100 objects (boxes) in world

| Engine | Step Time | Real-Time Factor | Stability (1-5) |
|--------|-----------|------------------|-----------------|
| ODE | 0.8 ms | 1.2x | ★★★☆☆ |
| Bullet | 1.2 ms | 0.9x | ★★★★☆ |
| DART | 4.5 ms | 0.3x | ★★★★★ |
| Simbody | 12.0 ms | 0.1x | ★★★★★ |

**Recommendation**: Start with ODE, switch to DART if stability issues arise, avoid Simbody unless biomechanics required.

---

### 4. Sensor Plugins: Cameras, Depth, LiDAR

Gazebo sensors are implemented as plugins that publish ROS 2 topics. Accurate sensor configuration is critical for sim-to-real transfer.

#### Camera Sensor

**Plugin**: `libgazebo_ros_camera.so`

**Configuration**:
```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov> <!-- 80 degrees -->
      <image>
        <width>1920</width>
        <height>1080</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev> <!-- ~2/255 per channel -->
      </noise>
      <distortion>
        <k1>-0.05</k1> <!-- Radial distortion -->
        <k2>0.01</k2>
        <p1>0.0</p1>   <!-- Tangential distortion -->
        <p2>0.0</p2>
      </distortion>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>image_raw:=camera/image</remapping>
        <remapping>camera_info:=camera/camera_info</remapping>
      </ros>
      <camera_name>front_camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Published Topics**:
- `/robot/camera/image` (`sensor_msgs/Image`): RGB image
- `/robot/camera/camera_info` (`sensor_msgs/CameraInfo`): Intrinsics, distortion coefficients

**Tuning Tips**:
- Match `width`, `height`, `horizontal_fov` to real camera specs
- Add `<noise>` to avoid overfitting (set `stddev` to match real sensor)
- Include `<distortion>` if real camera has noticeable barrel/pincushion distortion

---

#### Depth Camera (RGB-D)

**Plugin**: `libgazebo_ros_camera.so` (with depth mode)

**Configuration**:
```xml
<sensor name="depth_camera" type="depth">
  <update_rate>20</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.3</near>  <!-- Min range -->
      <far>10.0</far>   <!-- Max range -->
    </clip>
  </camera>
  <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>depth/image_raw:=depth_camera/depth/image</remapping>
      <remapping>depth/points:=depth_camera/points</remapping>
    </ros>
    <min_depth>0.3</min_depth>
    <max_depth>10.0</max_depth>
  </plugin>
</sensor>
```

**Published Topics**:
- `/robot/depth_camera/depth/image` (`sensor_msgs/Image`, 32FC1 encoding): Depth image
- `/robot/depth_camera/points` (`sensor_msgs/PointCloud2`): 3D point cloud

**Realistic Noise** (RealSense D435 example):
```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.005</stddev> <!-- 5 mm depth noise -->
</noise>
```

---

#### 2D LiDAR (Laser Scanner)

**Plugin**: `libgazebo_ros_ray_sensor.so`

**Configuration**:
```xml
<sensor name="laser" type="ray">
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples> <!-- 0.5° resolution -->
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle> <!-- -180° -->
        <max_angle>3.14159</max_angle>  <!-- +180° -->
      </horizontal>
    </scan>
    <range>
      <min>0.3</min>
      <max>30.0</max>
      <resolution>0.01</resolution> <!-- 1 cm -->
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev> <!-- 2 cm range noise -->
    </noise>
  </ray>
  <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>scan:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>laser_link</frame_name>
  </plugin>
</sensor>
```

**Published Topic**:
- `/robot/scan` (`sensor_msgs/LaserScan`): Range measurements, 720 points per scan

**Matching Real Hardware** (Hokuyo URG-04LX example):
```xml
<samples>683</samples>       <!-- 240° FOV / 0.352° resolution -->
<min_angle>-2.0944</min_angle> <!-- -120° -->
<max_angle>2.0944</max_angle>  <!-- +120° -->
<min>0.06</min>
<max>5.6</max>
<stddev>0.03</stddev>        <!-- 3 cm accuracy -->
```

---

#### 3D LiDAR (Velodyne-style)

**Plugin**: `libgazebo_ros_ray_sensor.so` with vertical scan

**Configuration**:
```xml
<sensor name="velodyne" type="gpu_ray">
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>1800</samples> <!-- 0.2° horizontal resolution -->
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>   <!-- 16 laser beams -->
        <resolution>1</resolution>
        <min_angle>-0.2618</min_angle> <!-- -15° -->
        <max_angle>0.2618</max_angle>  <!-- +15° -->
      </vertical>
    </scan>
    <range>
      <min>0.9</min>
      <max>100.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="velodyne_controller" filename="libgazebo_ros_ray_sensor.so">
    <output_type>sensor_msgs/PointCloud2</output_type>
    <frame_name>velodyne_link</frame_name>
  </plugin>
</sensor>
```

**Note**: `gpu_ray` uses GPU for faster ray tracing (requires NVIDIA GPU with CUDA support). Falls back to CPU if unavailable.

---

### 5. Launching Gazebo with ROS 2

Integrating Gazebo with ROS 2 requires spawning the robot, starting controllers, and connecting plugins.

#### Minimal Launch File

`launch/gazebo_sim.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    robot_pkg = get_package_share_directory('my_robot_description')
    world_file = os.path.join(robot_pkg, 'worlds', 'empty_world.world')
    urdf_file = os.path.join(robot_pkg, 'urdf', 'mobile_robot.urdf')

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                gazebo_pkg, '/launch/gazebo.launch.py'
            ]),
            launch_arguments={'world': world_file}.items()
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'mobile_robot',
                       '-topic', 'robot_description',
                       '-x', '0', '-y', '0', '-z', '0.5'],
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        )
    ])
```

**Explanation**:
1. **Gazebo Server**: `gazebo.launch.py` starts physics engine, rendering, GUI
2. **Spawn Robot**: `spawn_entity.py` inserts robot model at specified pose (x, y, z)
3. **Robot State Publisher**: Publishes TF transforms from URDF for RViz visualization

---

#### Running the Simulation

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash
ros2 launch my_robot_description gazebo_sim.launch.py
```

**Expected Result**:
- Gazebo GUI window opens with empty world
- Mobile robot spawns at origin (0, 0, 0.5 meters above ground)
- ROS 2 topics available: `/robot/camera/image`, `/robot/scan`, etc.

---

**Common Errors**:

**Error 1**: `[spawn_entity.py] Service call failed`
- **Cause**: Gazebo not fully started when spawn attempted
- **Solution**: Add 5-second delay before spawn:
  ```python
  from launch.actions import TimerAction
  spawn_node = Node(...)
  delayed_spawn = TimerAction(period=5.0, actions=[spawn_node])
  ```

**Error 2**: Robot falls through ground plane
- **Cause**: Missing inertial properties or collision geometry
- **Solution**: Verify all `<link>` elements have `<inertial>` and `<collision>`

**Error 3**: Topics not publishing
- **Cause**: Plugin not loaded or incorrect namespace
- **Solution**: Check Gazebo terminal output for plugin load errors, verify `<ros><namespace>` matches expected

---

## Practical Example: Simulating a LiDAR-Equipped Robot

### Overview

We'll extend the mobile robot from Chapter 2.1 by adding a 2D LiDAR sensor, spawning it in Gazebo, and visualizing scan data in RViz.

### Prerequisites

- Software: ROS 2 Humble, Gazebo Classic 11, RViz2
- Files: `mobile_robot.urdf` from Chapter 2.1

### Implementation

**Step 1: Add LiDAR Link to URDF**

Append to `mobile_robot.urdf`:

```xml
<!-- LiDAR link -->
<link name="laser_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.05"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" iyy="0.0001" izz="0.0001"
             ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>

<!-- Joint: base_link to laser_link -->
<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="laser_link"/>
  <origin xyz="0.2 0 0.1" rpy="0 0 0"/> <!-- Mount on front of robot -->
</joint>
```

---

**Step 2: Add Gazebo LiDAR Plugin**

Create `mobile_robot.gazebo`:

```xml
<?xml version="1.0"?>
<robot>
  <!-- Gazebo-specific tags -->
  <gazebo reference="laser_link">
    <sensor name="laser" type="ray">
      <update_rate>10</update_rate>
      <visualize>true</visualize> <!-- Show rays in Gazebo GUI -->
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.3</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.02</stddev>
        </noise>
      </ray>
      <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <remapping>scan:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>laser_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Add differential drive plugin for wheel control -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
      <update_rate>20</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.35</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>
</robot>
```

---

**Step 3: Include Gazebo Tags in Launch File**

Modify `urdf/mobile_robot.urdf` to include Gazebo extensions:

```xml
<robot name="mobile_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include URDF content from Chapter 2.1 -->
  ...

  <!-- Include Gazebo-specific tags -->
  <xacro:include filename="$(find my_robot_description)/urdf/mobile_robot.gazebo"/>
</robot>
```

*(If not using xacro, manually paste Gazebo tags into URDF file)*

---

**Step 4: Create World File**

`worlds/empty_world.world`:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <!-- Physics settings -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Add obstacles for LiDAR testing -->
    <include>
      <uri>model://box</uri>
      <pose>2 0 0.5 0 0 0</pose>
    </include>
    <include>
      <uri>model://box</uri>
      <pose>-1 2 0.5 0 0 0</pose>
    </include>
  </world>
</sdf>
```

---

**Step 5: Launch and Visualize**

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch my_robot_description gazebo_sim.launch.py

# Terminal 2: Launch RViz for visualization
rviz2
```

**In RViz**:
1. Set Fixed Frame to `odom`
2. Add "LaserScan" display, set Topic to `/scan`
3. Add "TF" display to see coordinate frames
4. Add "RobotModel" display, set Description Topic to `/robot_description`

---

**Step 6: Teleoperate Robot**

```bash
# Terminal 3: Control robot with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use keyboard (i/j/k/l) to drive robot around. Observe LiDAR detecting obstacles in RViz.

### Expected Output

- Gazebo shows mobile robot with red LiDAR sensor on top, two boxes nearby
- RViz displays laser scan rays (red points) showing obstacle detections
- Driving robot forward causes scan to show decreasing range to boxes

### Troubleshooting

- **Issue**: No `/scan` topic
  **Solution**: Check Gazebo terminal for plugin load errors, verify `libgazebo_ros_ray_sensor.so` installed (`sudo apt install ros-humble-gazebo-plugins`)

- **Issue**: Robot doesn't move with `teleop_twist_keyboard`
  **Solution**: Verify differential drive plugin loaded, check joint names match URDF (`left_wheel_joint`, `right_wheel_joint`)

### Further Exploration

- Add walls to the world to create a maze, test obstacle avoidance
- Change LiDAR `<samples>` to 180 (1° resolution) and observe scan rate impact
- Increase `<noise><stddev>` to 0.1 and observe scan jitter

---

## Figures & Diagrams

### Figure 2.2-1: Physics Engine Comparison

*(See separate diagram file `fig2.2-physics-comparison.md` for comparison table/flowchart)*

**Caption**: Comparison of physics engines (ODE, Bullet, DART, Simbody) showing trade-offs between speed, accuracy, and stability. Decision tree guides engine selection based on robot type and simulation requirements.

**Reference**: This figure supports Section 3 (Physics Engine Configuration) by visualizing performance characteristics.

---

## Exercises

### Exercise 1: World Building (Difficulty: Easy)

**Objective**: Practice creating Gazebo worlds with multiple models

**Task**: Create a warehouse environment with:
- 4 walls (1m high, 0.1m thick, forming 10m × 10m enclosure)
- 8 boxes (0.5m × 0.5m × 0.5m) scattered inside
- 2 cylinders (0.3m radius, 1m height) as pillars

**Requirements**:
- Use SDF `<world>` file format
- Set appropriate physics parameters (1 kHz, real-time factor 1.0)
- Place models at non-overlapping positions

**Expected Outcome**: Launching Gazebo displays warehouse scene, robot can navigate without walls/obstacles overlapping

**Hints**:
- Use `<pose>x y z roll pitch yaw</pose>` for positioning
- Check Gazebo model database: `ls /usr/share/gazebo-11/models/`
- Test collisions by spawning robot and driving around

**Estimated Time**: 25 minutes

---

### Exercise 2: Sensor Calibration (Difficulty: Medium)

**Objective**: Match simulated sensor to real hardware specifications

**Task**: Configure a simulated Intel RealSense D435 depth camera with these specs:
- Resolution: 1280 × 720
- FOV: 87° horizontal × 58° vertical
- Depth range: 0.3m to 10m
- Depth noise: ±2% at 2m distance
- Frame rate: 30 Hz

**Requirements**:
- Write complete `<gazebo>` sensor configuration
- Calculate appropriate `horizontal_fov` in radians
- Set Gaussian noise `stddev` matching 2% error at 2m

**Expected Outcome**: Simulated depth camera publishes `/camera/depth/image` matching RealSense D435 characteristics

**Hints**:
- Convert FOV: 87° = 1.518 radians
- 2% error at 2m = 0.04m stddev
- Use `type="depth"` sensor, `libgazebo_ros_camera.so` plugin

**Estimated Time**: 30 minutes

---

### Exercise 3: Multi-Robot Simulation (Difficulty: Hard)

**Objective**: Spawn multiple robots with unique namespaces

**Task**: Create a launch file that spawns 3 mobile robots at different positions (0, 0), (2, 0), (0, 2) with namespaces `/robot1`, `/robot2`, `/robot3`

**Requirements**:
- Each robot publishes to separate topics (`/robot1/scan`, `/robot2/scan`, etc.)
- Use loop in Python launch file to avoid code duplication
- Ensure TF frames are namespaced (`robot1/base_link`, not `base_link`)

**Expected Outcome**: Three robots visible in Gazebo, `ros2 topic list` shows 9 scan topics (3 robots × 3 topics each)

**Hints**:
- Pass namespace to `spawn_entity.py` via `-robot_namespace` argument
- Set `<tf_prefix>` in robot_state_publisher parameters
- Use Python loop: `for i in range(3): ...`

**Estimated Time**: 60 minutes

---

## Summary & Key Takeaways

In this chapter, you learned:

- **Gazebo Classic** (version 11) is end-of-life January 2025 but remains the default choice for ROS 2 Humble + Ubuntu 22.04 deployments
- **URDF** describes robot structure for ROS, **SDF** describes complete simulation worlds—Gazebo converts URDF to SDF automatically
- **Physics engines** (ODE, Bullet, DART, Simbody) trade off speed vs. accuracy—start with ODE, switch to DART for manipulation
- **Sensor plugins** (camera, depth, LiDAR) require careful configuration (resolution, FOV, noise) to match real hardware
- **ROS 2 integration** uses `gazebo_ros` plugins to publish sensor data and subscribe to control commands via standard ROS 2 topics

**Connection to Chapter 2.3**: While Gazebo excels at physics simulation, Chapter 2.3 introduces Unity Robotics Hub as an alternative for photorealistic rendering, game engine integration, and VR/AR applications.

---

## Additional Resources

### Official Documentation
- Gazebo Classic Tutorials: http://classic.gazebosim.org/tutorials - Comprehensive guides for sensors, plugins, worlds
- Gazebo ROS 2 Integration: https://github.com/ros-simulation/gazebo_ros_pkgs - Plugin documentation and examples
- SDF Specification: http://sdformat.org/spec - Complete SDF XML reference

### Recommended Reading
- "Effective Robotics Programming with ROS" by Anil Mahtani et al. - Chapter 9 covers Gazebo simulation in depth
- Gazebo Physics Engine Comparison: http://classic.gazebosim.org/tutorials?tut=physics_params - Official benchmark results

### Community Resources
- Gazebo Answers: https://answers.gazebosim.org/ - Q&A for troubleshooting simulation issues
- ROS Discourse (Simulation): https://discourse.ros.org/c/general/simulation - General simulation discussions

---

## Notes for Instructors

**Teaching Tips**:
- Start with visual comparison: Show same robot in RViz (URDF only) vs. Gazebo (physics + sensors)
- Common misconception: "Gazebo is just for visualization"—emphasize physics simulation vs. RViz visualization
- Live demo: Change physics engine mid-lecture (modify world file, relaunch) and show stability differences

**Lab Exercise Ideas**:
- **Lab 1**: Physics engine comparison—students measure simulation speed and stability for stacking task across ODE/Bullet/DART
- **Lab 2**: Sensor noise tuning—students train vision model with/without noise, compare real-world performance
- **Lab 3**: Multi-robot coordination—students spawn 5 robots, implement leader-follower formation control

**Assessment Suggestions**:
- **Simulation assignment**: Provide broken URDF (missing inertias, wrong joint axes), require students to debug via Gazebo testing
- **World building project**: Students create custom environment (office, warehouse, outdoor) with obstacles, lighting, spawn robot
- **Sensor integration quiz**: Given real sensor datasheet, write complete Gazebo sensor configuration matching specifications

---

**Chapter Metadata**:
- **Word Count**: 3,800 words (core concepts)
- **Figures**: 1 (physics engine comparison)
- **Code Examples**: 8 (URDF, SDF, plugins, launch files)
- **Exercises**: 3 (easy, medium, hard)
- **Glossary Terms**: 8
- **Cross-References**: Chapter 2.1, forward to Chapter 2.3
