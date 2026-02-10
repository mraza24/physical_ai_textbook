---
sidebar_position: 3
title: Chapter 2.2 - Gazebo Simulation Fundamentals
---

# Chapter 2.2: Gazebo Simulation Fundamentals

**Module**: 2 - The Digital Twin
**Week**: 6
**Estimated Reading Time**: 30 minutes

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. Install and configure Gazebo Harmonic
2. Import URDF models into Gazebo
3. Configure physics engines and simulation parameters
4. Add simulated sensors (camera, IMU, LiDAR)
5. Integrate Gazebo with ROS 2 using ros_gz_bridge

---

## Prerequisites

- Completed Chapter 2.1 (Digital Twin Concepts)
- ROS 2 Humble installed
- Basic understanding of XML and URDF

---

## Introduction

**Gazebo** is the most widely used open-source robot simulator in the ROS ecosystem. Originating in 2002 at the University of Southern California, Gazebo has evolved into a sophisticated simulation platform used by research labs, universities, and companies worldwide.

### Why Gazebo for Robotics?

1. **Open-Source and Free**: No licensing costs, full source code access
2. **ROS 2 Native**: First-class integration with ros_gz packages
3. **High-Fidelity Physics**: Multiple physics engines (Bullet, ODE, DART, Simbody)
4. **Rich Sensor Suite**: Camera, LiDAR, IMU, GPS, depth sensors
5. **Plugin Architecture**: Extensible with C++ plugins for custom behaviors
6. **Active Community**: Large user base, extensive tutorials, forum support

### Gazebo Versions: Classic vs. Modern

| Feature | Gazebo Classic (v11) | Gazebo (Harmonic, Ionic) |
|---------|----------------------|--------------------------|
| **Architecture** | Monolithic | Modular (Ignition libraries) |
| **ROS 2 Support** | Limited | Native |
| **Rendering** | OGRE 1.x | OGRE 2.x, Optix |
| **Performance** | CPU-bound | GPU-accelerated |
| **Status** | Legacy (2025 EOL) | Active development |

**This chapter focuses on Gazebo Harmonic (2024)**, the recommended version for new ROS 2 projects.

---

## Key Terms

:::info Glossary Terms
- **Gazebo**: Open-source robot simulator with physics engine
- **SDF**: Simulation Description Format for Gazebo worlds
- **Physics Engine**: Software that simulates forces, collisions, dynamics
- **ros_gz_bridge**: ROS 2 package for Gazebo communication
- **World**: Complete simulation environment (models + physics + environment)
- **Model**: A simulated entity (robot, object, sensor)
- **Plugin**: C++ code that extends Gazebo functionality
:::

---

## Core Concepts

### 1. Gazebo Architecture

Gazebo uses a **client-server architecture** to separate simulation logic from visualization:

```
┌─────────────────────────────────────────┐
│           Gazebo Client (GUI)           │
│  - Renders 3D visuals                   │
│  - User interaction                     │
│  - Camera views                         │
└────────────────┬────────────────────────┘
                 │ Network (TCP)
┌────────────────▼────────────────────────┐
│        Gazebo Server (gzserver)         │
│  - Physics simulation                   │
│  - Sensor data generation               │
│  - Plugin execution                     │
│  - World state management               │
└────────────────┬────────────────────────┘
                 │
┌────────────────▼────────────────────────┐
│         ROS 2 via ros_gz_bridge         │
│  - Topic bridging (/cmd_vel, /odom)    │
│  - Sensor data publishing               │
└─────────────────────────────────────────┘
```

**Key Components**:

- **gzserver**: Headless simulation engine (runs physics, sensors)
- **gz sim**: GUI client (3D visualization, scene tree, plugin panels)
- **gz topic**: Gazebo's internal transport system (Protobuf messages)
- **ros_gz_bridge**: Bidirectional ROS 2 ↔ Gazebo message conversion

**Benefits of Separation**:
- Run simulation headless on servers (no GPU needed for physics)
- Multiple clients can connect to one simulation
- Restart GUI without resetting simulation state

### 2. URDF vs. SDF

Both formats describe robots, but they serve different purposes:

#### URDF (Unified Robot Description Format)

**Purpose**: ROS-native robot description
**Use Case**: Robot kinematics, joint definitions, TF tree
**Strengths**:
- Native ROS 2 support (`robot_state_publisher`)
- Simple joint definitions
- xacro macros for reusability

**Limitations**:
- No world descriptions (only robots)
- Limited physics parameters
- No closed kinematic loops

**Example** (TurtleBot3 Burger):
```xml
<robot name="turtlebot3_burger">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.14 0.14 0.04"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.14 0.14 0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="0.0 0.08 -0.02" rpy="-1.57 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

#### SDF (Simulation Description Format)

**Purpose**: Gazebo-native world and model description
**Use Case**: Complete simulation environments
**Strengths**:
- Describes worlds, lights, physics parameters
- Advanced features (closed loops, nested models)
- Physics engine configuration
- Sensor noise models

**Limitations**:
- Not understood by ROS 2 directly (requires conversion)
- More complex syntax

**Example** (Empty World):
```xml
<sdf version="1.9">
  <world name="empty_world">
    <!-- Physics Engine -->
    <physics name="1ms" type="bullet">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  </world>
</sdf>
```

#### When to Use Each Format

| Scenario | Format | Reason |
|----------|--------|--------|
| **Robot hardware description** | URDF | ROS 2 tools expect URDF |
| **Gazebo simulation** | SDF | Full physics control |
| **Hybrid approach** | URDF → SDF | Use xacro macros, convert to SDF |
| **World environments** | SDF | Only SDF supports worlds |

**Best Practice**: Define robots in URDF (for ROS 2 compatibility), then convert to SDF for Gazebo simulation using:
```bash
gz sdf -p robot.urdf > robot.sdf
```

### 3. Physics Configuration

Gazebo supports multiple physics engines, each with trade-offs:

#### Available Physics Engines

| Engine | Speed | Accuracy | Best For |
|--------|-------|----------|----------|
| **Bullet** | Fast | Medium | Mobile robots, manipulation |
| **ODE** | Medium | Medium | Legacy projects (Gazebo Classic default) |
| **DART** | Slow | High | Humanoid robots, complex contacts |
| **Simbody** | Slow | Very High | Biomechanics, precise simulations |

**Default Recommendation**: **Bullet** (good speed/accuracy trade-off)

#### Configuring Physics Parameters

**In SDF World File**:
```xml
<world name="my_world">
  <physics name="bullet_physics" type="bullet">
    <!-- Simulation timestep -->
    <max_step_size>0.001</max_step_size> <!-- 1ms = 1000 Hz -->

    <!-- Real-time factor (1.0 = real-time, 0.5 = half speed) -->
    <real_time_factor>1.0</real_time_factor>

    <!-- Solver iterations (higher = more stable) -->
    <solver>
      <iterations>50</iterations>
    </solver>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>
  </physics>
</world>
```

#### Tuning for Stability vs. Speed

**Problem**: Fast-moving robots may experience physics jitter or pass through walls.

**Solution**: Adjust timestep and solver iterations:

```xml
<!-- STABLE but SLOW (for high-speed robots) -->
<max_step_size>0.0005</max_step_size> <!-- 2000 Hz -->
<solver><iterations>100</iterations></solver>

<!-- FAST but UNSTABLE (for simple mobile robots) -->
<max_step_size>0.01</max_step_size> <!-- 100 Hz -->
<solver><iterations>20</iterations></solver>
```

**Rule of Thumb**: For stable simulation, ensure:
```
max_step_size < 1 / (10 × max_robot_velocity)
```

Example: Robot moving at 2 m/s → `max_step_size < 0.05s` (50 Hz minimum)

### 4. Sensor Plugins

Gazebo generates sensor data using **plugins** that attach to robot links.

#### Camera Sensor

**SDF Configuration**:
```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format> <!-- RGB -->
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <!-- Noise model (realistic sensor) -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <update_rate>30</update_rate> <!-- 30 FPS -->
  <visualize>true</visualize>
</sensor>
```

**ROS 2 Bridge** (publish to `/camera/image_raw`):
```xml
<plugin filename="libgazebo_ros_camera.so" name="camera_controller">
  <ros>
    <namespace>/camera</namespace>
    <remapping>image_raw:=image_raw</remapping>
    <remapping>camera_info:=camera_info</remapping>
  </ros>
  <camera_name>my_camera</camera_name>
  <frame_name>camera_link</frame_name>
</plugin>
```

#### IMU Sensor (Inertial Measurement Unit)

**SDF Configuration**:
```xml
<sensor name="imu" type="imu">
  <imu>
    <!-- Accelerometer -->
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev> <!-- m/s² -->
        </noise>
      </x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></z>
    </linear_acceleration>

    <!-- Gyroscope -->
    <angular_velocity>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev></noise></z>
    </angular_velocity>
  </imu>
  <update_rate>100</update_rate> <!-- 100 Hz -->
</sensor>
```

#### LiDAR Sensor (2D Laser Scanner)

**SDF Configuration**:
```xml
<sensor name="lidar" type="gpu_lidar">
  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples> <!-- 360 rays -->
        <resolution>1.0</resolution>
        <min_angle>-3.14159</min_angle> <!-- -180° -->
        <max_angle>3.14159</max_angle>  <!-- +180° -->
      </horizontal>
    </scan>
    <range>
      <min>0.12</min> <!-- 12 cm minimum range -->
      <max>10.0</max> <!-- 10 m maximum range -->
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev> <!-- 1 cm noise -->
    </noise>
  </lidar>
  <update_rate>10</update_rate> <!-- 10 Hz -->
  <visualize>true</visualize>
</sensor>
```

**Why `gpu_lidar`?**: Uses GPU ray-casting for 10-100x speedup vs. CPU-based `ray` sensor.

#### Depth Camera (RealSense, Kinect)

```xml
<sensor name="depth_camera" type="depth_camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <update_rate>30</update_rate>
</sensor>
```

---

## Practical Examples

### Example 1: Spawning TurtleBot3 in Empty World

**Step 1: Install TurtleBot3 Packages**
```bash
sudo apt install ros-humble-turtlebot3-gazebo
```

**Step 2: Set Robot Model**
```bash
export TURTLEBOT3_MODEL=burger
```

**Step 3: Launch Gazebo World**
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**What Happens**:
1. Gazebo server starts with empty world
2. TurtleBot3 URDF loaded via `spawn_entity.py` node
3. ros_gz_bridge publishes:
   - `/cmd_vel` (Twist) → Gazebo controls
   - `/odom` (Odometry) → ROS 2
   - `/scan` (LaserScan) → ROS 2
   - `/imu` (Imu) → ROS 2

**Step 4: Drive the Robot**
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### Example 2: Creating a Custom World with Obstacles

**File**: `my_warehouse.sdf`
```xml
<sdf version="1.9">
  <world name="warehouse">
    <!-- Physics -->
    <physics name="1ms" type="bullet">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Sun -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>

    <!-- Warehouse Walls -->
    <model name="wall_1">
      <pose>5 0 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.2 10 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 10 2</size></box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacles (Boxes) -->
    <model name="box_1">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.5 0.2 1</ambient>
            <diffuse>1 0.6 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
        </inertial>
      </link>
    </model>

    <!-- System Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  </world>
</sdf>
```

**Launch the World**:
```bash
gz sim my_warehouse.sdf
```

### Example 3: Attaching a Camera to a Robot (URDF)

**File**: `camera_robot.urdf.xacro`
```xml
<robot name="camera_bot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.3 0.3 0.1"/></geometry>
      <material name="blue"><color rgba="0 0 1 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.3 0.3 0.1"/></geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Camera Link -->
  <link name="camera_link">
    <visual>
      <geometry><box size="0.05 0.05 0.05"/></geometry>
      <material name="red"><color rgba="1 0 0 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.05 0.05 0.05"/></geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Camera Joint -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/> <!-- 15cm forward, 10cm up -->
  </joint>

  <!-- Camera Sensor (Gazebo Plugin) -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>50.0</far>
        </clip>
      </camera>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
      <plugin filename="libgazebo_ros_camera.so" name="camera_controller">
        <ros>
          <namespace>/camera_bot</namespace>
          <remapping>image_raw:=image</remapping>
        </ros>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

**View Camera Feed**:
```bash
ros2 run rqt_image_view rqt_image_view /camera_bot/image
```

### Example 4: ROS 2 ↔ Gazebo Topic Bridging

**Problem**: Gazebo publishes to `/gz/topics`, but ROS 2 expects standard topics like `/odom`.

**Solution**: Use `ros_gz_bridge`:

**Bridge Configuration File** (`bridge_config.yaml`):
```yaml
# Gazebo → ROS 2
- ros_topic_name: "/odom"
  gz_topic_name: "/model/my_robot/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

# ROS 2 → Gazebo
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/model/my_robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

# Bidirectional
- ros_topic_name: "/scan"
  gz_topic_name: "/model/my_robot/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: BIDIRECTIONAL
```

**Launch Bridge**:
```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge_config.yaml
```

**Verify Topics**:
```bash
ros2 topic list
# Should show: /odom, /cmd_vel, /scan
```

---

## Common Pitfalls and Debugging

### Pitfall 1: Robot Falls Through Ground

**Symptom**: Robot spawns and immediately falls infinitely.

**Cause**: Missing collision geometry or incorrect mass/inertia.

**Fix**:
1. Ensure `<collision>` tags match `<visual>` geometry
2. Add realistic inertia values:
```xml
<inertial>
  <mass value="1.0"/> <!-- Don't use 0! -->
  <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
</inertial>
```

### Pitfall 2: Physics is Too Fast or Slow

**Symptom**: Robot moves in slow motion or super speed.

**Cause**: `real_time_factor` mismatch or underpowered CPU.

**Fix**:
1. Check CPU usage: `top` (Gazebo should use <100% per core)
2. Reduce physics frequency:
```xml
<max_step_size>0.01</max_step_size> <!-- Increase from 0.001 -->
```
3. Disable GUI rendering:
```bash
gz sim -s my_world.sdf  # Server-only mode
```

### Pitfall 3: Sensors Publish No Data

**Symptom**: `/scan` or `/camera/image` topics exist but have no messages.

**Cause**: Missing `<update_rate>` or `visualize` set to false.

**Fix**:
```xml
<sensor name="lidar" type="gpu_lidar">
  <update_rate>10</update_rate> <!-- MUST be > 0 -->
  <visualize>true</visualize>   <!-- Enable for debugging -->
</sensor>
```

### Debugging Commands

```bash
# List Gazebo topics
gz topic -l

# Echo Gazebo topic
gz topic -e -t /model/my_robot/odometry

# List ROS 2 topics
ros2 topic list

# Check if bridge is running
ros2 node list | grep bridge

# View TF tree
ros2 run tf2_tools view_frames
```

---

## Performance Optimization Tips

1. **Use GPU Sensors**: Replace `ray` with `gpu_lidar`, `camera` with `gpu_camera`
2. **Simplify Collision Meshes**: Use primitive shapes (box, sphere, cylinder) instead of complex meshes
3. **Reduce Sensor Rates**: 10 Hz LiDAR is often sufficient (don't default to 100 Hz)
4. **Disable GUI During Training**: Use `gzserver` only (headless mode)
5. **Parallelize Simulations**: Run multiple Gazebo instances with different ports:
```bash
GZ_PARTITION=sim1 gz sim world1.sdf &
GZ_PARTITION=sim2 gz sim world2.sdf &
```

---

## Summary

This chapter introduced **Gazebo simulation fundamentals**:

1. **Architecture**: Client-server design separates physics from rendering
2. **Formats**: URDF for robots, SDF for worlds
3. **Physics**: Bullet engine recommended, tune `max_step_size` for stability
4. **Sensors**: Camera, IMU, LiDAR plugins with realistic noise models
5. **ROS 2 Integration**: `ros_gz_bridge` for topic translation

**Key Takeaways**:
- Start with existing worlds (TurtleBot3, empty_world) before creating custom environments
- Always add collision geometry and realistic inertia to models
- Use GPU-accelerated sensors for performance
- Debug with `gz topic` and `ros2 topic` commands

**Next Steps**:
- Chapter 2.3: Unity for photorealistic rendering
- Chapter 2.4: Advanced sensor simulation (vSLAM with stereo cameras)

---

## End-of-Chapter Exercises

### Exercise 1: Simulate a Mobile Robot (Difficulty: Medium)

**Objective**: Create a differential drive robot with LiDAR and camera, spawn in warehouse world.

**Tasks**:
1. Write a URDF file defining:
   - Base link (box 0.4×0.3×0.1 m, mass 10 kg)
   - Two wheels (cylinders, radius 0.05 m)
   - One caster wheel (sphere, radius 0.02 m)
   - Camera link (forward-facing, 60° FOV)
   - LiDAR link (top-mounted, 360° scan)

2. Add Gazebo plugins:
   - Differential drive controller
   - Camera sensor (publish to `/camera/image_raw`)
   - LiDAR sensor (publish to `/scan`)

3. Spawn robot in warehouse world from Example 2

4. Drive robot using teleop and verify sensor data:
```bash
ros2 topic echo /scan --once
ros2 run image_view image_view --ros-args --remap image:=/camera/image_raw
```

**Success Criteria**:
- Robot moves forward/backward and rotates
- `/scan` publishes `LaserScan` at 10 Hz
- Camera shows warehouse obstacles

**Bonus**: Implement autonomous wall-following using `/scan` data.

### Exercise 2: Physics Tuning Challenge (Difficulty: Hard)

**Scenario**: A humanoid robot walks but frequently falls over.

**Tasks**:
1. Start with provided unstable world file
2. Adjust physics parameters to stabilize walking:
   - Physics engine (try Bullet vs. DART)
   - Step size (0.001 to 0.0001)
   - Solver iterations (50 to 200)
3. Document changes and their effects

**Success Criteria**: Robot completes 10-meter walk without falling.

### Exercise 3: Multi-Robot Simulation (Difficulty: Advanced)

**Objective**: Spawn 3 TurtleBot3 robots in one world, control independently.

**Tasks**:
1. Modify TurtleBot3 URDF to add namespace parameter
2. Spawn 3 robots at different positions:
```bash
ros2 run gazebo_ros spawn_entity.py -entity robot1 -file robot.urdf -x 0 -y 0
ros2 run gazebo_ros spawn_entity.py -entity robot2 -file robot.urdf -x 2 -y 0
ros2 run gazebo_ros spawn_entity.py -entity robot3 -file robot.urdf -x 4 -y 0
```
3. Bridge topics with namespaces:
   - `/robot1/cmd_vel`, `/robot1/odom`, `/robot1/scan`
   - `/robot2/cmd_vel`, `/robot2/odom`, `/robot2/scan`
   - `/robot3/cmd_vel`, `/robot3/odom`, `/robot3/scan`
4. Drive each robot independently

**Success Criteria**: All 3 robots navigate without colliding.

---

## Further Reading

### Required
1. Gazebo Tutorials: https://gazebosim.org/docs/harmonic/tutorials
2. ros_gz documentation: https://github.com/gazebosim/ros_gz
3. SDF Specification: http://sdformat.org/spec

### Optional
4. "Simulation and Modeling of Robots" (Gazebo whitepaper)
5. TurtleBot3 Simulations: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/

---

## Next Chapter

Continue to **[Chapter 2.3: Unity for Robotics](./chapter2-3-unity-robotics)** to explore photorealistic rendering and ML-Agents integration.
