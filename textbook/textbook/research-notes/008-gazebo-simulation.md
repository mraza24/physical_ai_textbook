# Research Notes: Gazebo Simulation

**Task**: 008
**Date**: 2025-12-10
**Status**: Complete
**Sources**: Gazebo Official Documentation

---

## Official Documentation Sources

- **Gazebo Documentation**: https://gazebosim.org/docs
- **Gazebo Classic**: http://classic.gazebosim.org/tutorials
- **Gazebo Garden/Harmonic**: Latest generation (modular architecture)
- **ROS 2 + Gazebo Integration**: ros_gz packages

---

## Gazebo Overview

**Definition**: Open-source 3D robotics simulator with physics engines, sensors, and actuators.

**Key Features**:
- Accurate physics simulation (ODE, Bullet, Simbody, DART)
- Sensor simulation (cameras, LiDAR, IMU, depth sensors)
- Plugin system for custom behaviors
- SDF (Simulation Description Format) for worlds and models
- Distributed simulation support

---

## Gazebo Versions

| Version | Architecture | Status | ROS 2 Support |
|---------|--------------|--------|---------------|
| Gazebo Classic 11 | Monolithic | Legacy | Via gazebo_ros_pkgs |
| Gazebo Fortress | Modular | LTS (2026) | Via ros_gz |
| Gazebo Garden | Modular | Active | Via ros_gz |
| **Gazebo Harmonic** | Modular | **LTS (2028)** | **Via ros_gz** |

**Recommended**: **Gazebo Harmonic** (LTS, modern architecture)

---

## Physics Engines

### 1. ODE (Open Dynamics Engine)
- Default in Gazebo Classic
- Fast, stable for most robotics scenarios
- Good for contact/collision simulation

### 2. Bullet
- Alternative physics engine
- Better for complex multi-body dynamics
- Used in game engines, robotics

### 3. DART (Dynamic Animation and Robotics Toolkit)
- Advanced features: continuous collision detection
- Better constraint solver
- Preferred for humanoid robotics

### 4. Simbody
- High-fidelity biomechanics
- Slower but more accurate
- Good for legged locomotion research

**Recommendation for Humanoids**: **DART** (better for bipedal dynamics)

---

## URDF (Unified Robot Description Format)

**Purpose**: XML format for robot models.

**Key Elements**:
```xml
<robot name="humanoid">
  <link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>...</inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>
</robot>
```

**Joint Types**:
- **Fixed**: No motion
- **Revolute**: Rotation around axis (limited range)
- **Continuous**: Rotation around axis (unlimited)
- **Prismatic**: Linear motion
- **Floating**: 6-DOF free motion
- **Planar**: 2D motion in plane

---

## SDF (Simulation Description Format)

**Purpose**: Gazebo's native format (more powerful than URDF).

**Advantages over URDF**:
- Supports sensors natively
- Plugin integration
- Multiple models in one file
- Physics engine configuration
- World description

**Example SDF World**:
```xml
<sdf version="1.9">
  <world name="humanoid_world">
    <physics name="dart_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
        </collision>
      </link>
    </model>

    <include><uri>model://humanoid_robot</uri></include>
  </world>
</sdf>
```

---

## Sensor Simulation

### Camera Sensors
- RGB cameras
- Depth cameras (RealSense D435 simulation)
- Wide-angle/fisheye cameras
- Output: sensor_msgs/Image, sensor_msgs/CameraInfo

### LiDAR
- 2D laser scanners (Hokuyo, SICK)
- 3D LiDAR (Velodyne, Ouster)
- Configurable: range, resolution, FoV
- Output: sensor_msgs/LaserScan, sensor_msgs/PointCloud2

### IMU (Inertial Measurement Unit)
- Accelerometer, gyroscope, magnetometer
- Noise models for realistic data
- Output: sensor_msgs/Imu

### Contact Sensors
- Force/torque sensors
- Bumpers, touch sensors
- Output: geometry_msgs/WrenchStamped

### GPS
- Position simulation with noise
- Output: sensor_msgs/NavSatFix

---

## Gazebo-ROS 2 Integration

### Package: ros_gz_bridge

**Purpose**: Bridge Gazebo topics/services to ROS 2.

**Installation**:
```bash
sudo apt install ros-humble-ros-gz
```

**Bridge Configuration**:
```yaml
# config/bridge.yaml
- topic_name: "/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"

- topic_name: "/joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
```

**Launch Bridge**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['camera/image@sensor_msgs/msg/Image@gz.msgs.Image'],
        )
    ])
```

---

## Gazebo Plugins

### Common Plugins
- **libgazebo_ros_diff_drive**: Differential drive controller
- **libgazebo_ros_camera**: Camera sensor
- **libgazebo_ros_imu**: IMU sensor
- **libgazebo_ros_lidar**: LiDAR sensor
- **libgazebo_ros_joint_state_publisher**: Publish joint states

**Example Plugin in URDF**:
```xml
<gazebo>
  <plugin name="gazebo_ros_camera" filename="libgazebo_ros_camera.so">
    <camera_name>front_camera</camera_name>
    <frame_name>camera_link</frame_name>
    <update_rate>30</update_rate>
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/image_raw:=image_raw</remapping>
    </ros>
  </plugin>
</gazebo>
```

---

## Installation (Ubuntu 22.04 + ROS 2 Humble)

### Gazebo Harmonic
```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic
sudo apt update
sudo apt install gz-harmonic

# Install ROS 2 integration
sudo apt install ros-humble-ros-gz
```

---

## Simulating Humanoid Robots

### Model Requirements
1. **URDF/SDF file**: Robot description
2. **Meshes**: Visual and collision geometries (STL, DAE, OBJ)
3. **Joint controllers**: Position, velocity, or effort control
4. **Sensors**: Cameras, IMU, force-torque sensors
5. **Physics properties**: Inertia, mass, friction coefficients

### Humanoid-Specific Considerations
- **Balance control**: PID controllers for joint position
- **Ground contact**: Contact sensors on feet
- **Center of mass**: Critical for bipedal stability
- **Joint limits**: Realistic range of motion
- **Self-collision**: Enable collision checking between robot links

---

## Performance Optimization

### GPU Acceleration
- Use Ogre2 rendering engine for GPU-based rendering
- Enable GPU ray sensors for LiDAR/depth cameras

### Real-Time Factor
```xml
<physics name="physics" type="dart">
  <max_step_size>0.001</max_step_size>  <!-- 1ms -->
  <real_time_factor>1.0</real_time_factor>  <!-- Match real-time -->
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

**Tips**:
- Reduce visual quality for faster simulation
- Simplify collision geometries
- Use smaller time steps for stability
- Disable GUI for headless simulation

---

## Chapter Topics

**Chapter 2.2: Gazebo Simulation Fundamentals**
- Gazebo architecture and physics engines
- URDF robot models
- Sensor simulation (cameras, IMU, LiDAR)
- Gazebo-ROS 2 bridge setup
- Launching simulations

---

## References

1. Gazebo Development Team. (2024). *Gazebo: Robot simulation made easy*. Retrieved December 10, 2025, from https://gazebosim.org/
2. Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. In *2004 IEEE/RSJ International Conference on Intelligent Robots and Systems* (IROS) (Vol. 3, pp. 2149-2154). IEEE. https://doi.org/10.1109/IROS.2004.1389727

---

**Status**: ✅ Research complete. Ready for chapter writing.
