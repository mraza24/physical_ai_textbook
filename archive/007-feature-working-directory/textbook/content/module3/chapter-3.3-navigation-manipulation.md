# Chapter 3.3: Navigation & Manipulation

> **Module**: 3 - AI-Robot Brain (NVIDIA Isaac)
> **Week**: 9
> **Estimated Reading Time**: 24 minutes

---

## Summary

Isaac ROS integrates seamlessly with Nav2 (navigation) and MoveIt2 (manipulation), providing GPU-accelerated 3D mapping, localization, and motion planning. This chapter covers warehouse robot navigation using isaac_ros_nvblox, autonomous grasping with isaac_ros_pose_estimation, and deployment to Jetson platforms.

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Integrate** isaac_ros_nvblox with Nav2 for GPU-accelerated 3D mapping
2. **Configure** isaac_ros_visual_slam for real-time localization at 30+ Hz
3. **Implement** object grasping using isaac_ros_pose_estimation with MoveIt2
4. **Deploy** navigation stacks to Jetson Orin for autonomous mobile robots
5. **Optimize** Nav2 parameters for obstacle avoidance with GPU-based costmaps

**Prerequisite Knowledge**: Chapter 3.1 (Isaac Ecosystem), Chapter 3.2 (Perception), Module 1 (ROS 2), basic path planning algorithms

---

## Key Terms

- **Nav2**: ROS 2 navigation framework (path planning, obstacle avoidance, behavior trees)
- **Costmap**: 2D occupancy grid marking free space, obstacles, and inflation zones
- **Nvblox**: GPU-accelerated 3D voxel mapping for real-time scene reconstruction
- **MoveIt2**: ROS 2 motion planning framework for manipulators (arm trajectories, collision checking)
- **Pose Estimation**: Computing 6D object pose (position + orientation) for grasping
- **Behavior Tree**: Hierarchical state machine for robot task orchestration
- **DWB**: Dynamic Window Approach for local trajectory generation
- **Global Planner**: Finds path from start to goal (A*, Dijkstra, Theta*)

---

## Core Concepts

### 1. Nav2 Architecture with Isaac ROS

**Standard Nav2 Stack**:
```
Sensors → Costmap2D → Global Planner → Local Planner → cmd_vel
(LiDAR)    (CPU)         (CPU)           (CPU)         (motors)
```

**Isaac ROS Enhanced Stack**:
```
RGB-D Camera → isaac_ros_nvblox (GPU) → 3D Map → Costmap2D
isaac_ros_visual_slam (GPU) → Localization → Nav2 Core
                                            ↓
                                      Path Planning (CPU)
                                            ↓
                                      cmd_vel (motors)
```

**Key Improvements**:
- ✅ **3D Mapping**: Nvblox creates full 3D environment model (not just 2D slice)
- ✅ **GPU VSLAM**: Visual localization at 30 Hz (vs. 10 Hz CPU ORB-SLAM)
- ✅ **Dynamic Objects**: Nvblox handles moving obstacles (people, forklifts)
- ✅ **Jetson Deployment**: Full stack runs on Jetson Orin AGX

---

### 2. Isaac ROS Nvblox: GPU 3D Mapping

**What is Nvblox?**
- **Volumetric mapping**: Represent 3D space as voxels (3D pixels)
- **Truncated Signed Distance Field (TSDF)**: Each voxel stores distance to nearest surface
- **GPU-accelerated**: Process depth images at 30 Hz (vs. 3-5 Hz CPU)

**Workflow**:
```
1. RGB-D Camera captures depth image (30 Hz)
   ↓
2. Nvblox integrates depth into TSDF voxel grid (GPU)
   ↓
3. Extract 2D costmap slice at robot height
   ↓
4. Publish to Nav2 /map topic
```

**Installation**:
```bash
sudo apt install ros-humble-isaac-ros-nvblox
```

**Launch Configuration**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nvblox 3D mapper
        Node(
            package='isaac_ros_nvblox',
            executable='isaac_ros_nvblox',
            parameters=[{
                'voxel_size': 0.05,  # 5 cm voxels
                'esdf_max_distance': 10.0,  # meters
                'mesh_update_rate': 5.0  # Hz
            }],
            remappings=[
                ('depth/image', '/camera/depth/image'),
                ('depth/camera_info', '/camera/depth/camera_info'),
                ('color/image', '/camera/color/image')
            ]
        ),

        # Nav2 integration
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            parameters=[{'costmap_topic': '/nvblox_costmap'}]
        )
    ])
```

**Performance**:
- **CPU (OctoMap)**: 3-5 Hz, 1 cm voxels, 10m³ volume
- **GPU (Nvblox)**: 30 Hz, 5 cm voxels, 100m³ volume
- **Speedup**: 6-10× faster, 10× larger volume

---

### 3. Isaac ROS Visual SLAM for Localization

**Why GPU VSLAM?**
- **Real-time**: 30 Hz pose updates (vs. 10 Hz CPU ORB-SLAM)
- **Robustness**: Better loop closure detection (GPU-accelerated Bag-of-Words)
- **Jetson-ready**: Runs on Jetson Orin Nano (8GB)

**Launch**:
```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

**Published Topics**:
- `/visual_slam/tracking/odometry` (nav_msgs/Odometry)
- `/visual_slam/tracking/vo_pose` (geometry_msgs/PoseStamped)
- `/visual_slam/tracking/vo_path` (nav_msgs/Path)

**Integration with Nav2**:
```python
# Nav2 AMCL replacement (use VSLAM pose instead of particle filter)
Node(
    package='nav2_amcl',
    executable='amcl',
    parameters=[{
        'use_sim_time': False,
        'global_frame_id': 'map',
        'odom_frame_id': 'visual_slam/odom',  # Use VSLAM odom
        'base_frame_id': 'base_link'
    }]
)
```

**Accuracy**:
- **Position error**: 0.02-0.05 m over 100 m trajectory (with loop closures)
- **Orientation error**: 0.5-1.0° over 360° rotation

---

### 4. MoveIt2 Integration for Manipulation

**Grasping Pipeline**:
```
RGB Image → Object Detection (YOLOv8) → Detected Objects
Depth Image → Pose Estimation (6D) → Object Poses
                    ↓
              MoveIt2 Motion Planning
                    ↓
              Grasp Execution (gripper)
```

**isaac_ros_pose_estimation**:
- **Input**: Bounding box (from YOLOv8) + depth image
- **Output**: 6D pose (x, y, z, roll, pitch, yaw)
- **Method**: FoundationPose algorithm (NVIDIA, 2024)

**Example Launch**:
```python
# Pose estimation node
Node(
    package='isaac_ros_foundationpose',
    executable='foundationpose_node',
    parameters=[{
        'model_file_path': 'foundationpose.engine',
        'mesh_file_path': 'object.obj'  # CAD model
    }],
    remappings=[
        ('detections', '/yolov8/detections'),
        ('depth', '/camera/depth/image'),
        ('poses', '/object_poses')
    ]
)

# MoveIt2 planning
Node(
    package='moveit_ros_planning',
    executable='move_group',
    parameters=[{
        'robot_description': robot_urdf,
        'planning_plugin': 'ompl_interface/OMPLPlanner'
    }]
)
```

**Workflow**:
1. Detect object (YOLOv8): "cup at pixel (320, 240)"
2. Estimate pose: "(x=0.5m, y=0.2m, z=0.8m, roll=0, pitch=0, yaw=45°)"
3. Plan grasp: MoveIt2 computes arm trajectory to pre-grasp pose
4. Execute: Robot arm moves, gripper closes
5. Verify: Force-torque sensor confirms grasp success

---

### 5. Deployment to Jetson Orin

**Hardware**: Jetson Orin AGX (64GB, 275 TOPS)

**Full Stack on Jetson**:
```
┌─────────────────────────────────────┐
│      Jetson Orin AGX (15-60W)       │
├─────────────────────────────────────┤
│ isaac_ros_nvblox     (30 Hz, 8GB)  │
│ isaac_ros_visual_slam (30 Hz, 2GB)  │
│ isaac_ros_yolov8      (60 Hz, 3GB)  │
│ Nav2 stack            (10 Hz, 1GB)  │
│ MoveIt2               (10 Hz, 2GB)  │
├─────────────────────────────────────┤
│ Total: 16GB VRAM, 30-40W power     │
└─────────────────────────────────────┘
```

**Performance vs. Laptop RTX**:
- **Laptop RTX 4070 Ti**: 200 FPS YOLOv8, 50 Hz Nvblox, 100W power
- **Jetson Orin AGX**: 60 FPS YOLOv8, 30 Hz Nvblox, 40W power
- **Trade-off**: 3× slower but 2.5× lower power (critical for battery robots)

**Deployment Workflow**:
```bash
# 1. Build on laptop (cross-compilation)
docker run --rm -it nvcr.io/nvidia/l4t-jetpack:r35.2.1
colcon build --packages-select isaac_ros_nvblox

# 2. Copy to Jetson
scp -r install/ jetson@192.168.1.100:~/ros2_ws/

# 3. Run on Jetson
ssh jetson@192.168.1.100
source ~/ros2_ws/install/setup.bash
ros2 launch warehouse_nav full_stack.launch.py
```

---

### 6. Nav2 Parameter Tuning for Isaac ROS

**Critical Parameters**:

**A. Costmap Configuration**:
```yaml
# config/costmap_common.yaml
obstacle_layer:
  enabled: true
  observation_sources: nvblox_pointcloud
  nvblox_pointcloud:
    data_type: PointCloud2
    topic: /nvblox/pointcloud
    marking: true
    clearing: true
    min_obstacle_height: 0.1  # Ignore floor
    max_obstacle_height: 2.0  # Ignore ceiling

inflation_layer:
  enabled: true
  inflation_radius: 0.5  # meters (robot radius + safety margin)
  cost_scaling_factor: 10.0
```

**B. DWB Local Planner**:
```yaml
# config/dwb.yaml
DWB:
  max_vel_x: 0.5  # m/s (tune for robot dynamics)
  max_vel_theta: 1.0  # rad/s
  min_vel_x: -0.2  # Allow backing up
  acc_lim_x: 2.5  # m/s² (acceleration limit)
  acc_lim_theta: 3.2  # rad/s²

  critics:
    - RotateToGoal
    - Oscillation
    - ObstacleFootprint
    - PathAlign
    - PathDist
    - GoalDist
```

**C. Behavior Tree** (warehouse use case):
```xml
<!-- config/behavior_tree.xml -->
<BehaviorTree>
  <Sequence>
    <RateController hz="1.0">
      <ComputePathToPose goal="{goal}"/>
    </RateController>
    <FollowPath path="{path}"/>
    <Wait wait_duration="2.0"/>  <!-- Verify arrived -->
  </Sequence>
</BehaviorTree>
```

---

## Practical Example: Warehouse Navigation

### Overview

Deploy autonomous mobile robot in warehouse—navigate between stations, avoid dynamic obstacles (people, forklifts).

### Implementation

**Step 1: Hardware Setup**
- **Robot**: Clearpath Jackal (differential drive)
- **Sensors**: RealSense D435i (RGB-D), ZED 2 (stereo)
- **Computer**: Jetson Orin AGX (64GB)

**Step 2: Launch Isaac ROS Stack**

```python
# warehouse_nav.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nvblox 3D mapping
        Node(package='isaac_ros_nvblox', executable='isaac_ros_nvblox',
             parameters=[{'voxel_size': 0.05}]),

        # Visual SLAM localization
        Node(package='isaac_ros_visual_slam', executable='visual_slam_node',
             parameters=[{'enable_slam_visualization': True}]),

        # Nav2 stack
        Node(package='nav2_bringup', executable='bringup_launch.py',
             parameters=[{'use_sim_time': False}])
    ])
```

**Step 3: Create Map**

```bash
# Drive robot around warehouse (teleoperation)
ros2 launch warehouse_nav mapping.launch.py

# Save map after complete loop
ros2 service call /nvblox/save_map nvblox_msgs/srv/SaveMap "{file_path: '/home/jetson/warehouse_map.nvblox'}"
```

**Step 4: Autonomous Navigation**

```bash
# Load saved map
ros2 launch warehouse_nav navigation.launch.py map:=warehouse_map.nvblox

# Send goal via RViz or command
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 10.0, y: 5.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

### Expected Outcome

- Robot navigates 50m warehouse in 90 seconds (0.55 m/s average)
- Avoids 3 dynamic obstacles (people walking)
- Re-plans path when forklift blocks corridor
- Arrives within 0.1m of goal pose

### Troubleshooting

- **Issue**: Robot oscillates near obstacles
  **Solution**: Increase `inflation_radius` (0.3 → 0.5m)

- **Issue**: VSLAM tracking lost in featureless corridor
  **Solution**: Add AprilTag fiducials on walls (isaac_ros_apriltag)

- **Issue**: Nvblox runs slow on Jetson
  **Solution**: Reduce `voxel_size` (0.05 → 0.10m), lower `mesh_update_rate` (5 → 2 Hz)

---

## Exercises

### Exercise 1: Nav2 Tuning (Difficulty: Easy)

**Objective**: Tune Nav2 parameters for smooth navigation

**Task**: Modify DWB parameters, test in simulation

**Requirements**:
- Start with default params (robot overshoots goals)
- Reduce `max_vel_x` from 0.5 → 0.3 m/s
- Test: Does robot arrive smoothly?

**Expected Outcome**: Smooth deceleration, no overshoot beyond 0.05m

**Estimated Time**: 20 minutes

---

### Exercise 2: MoveIt2 Grasping (Difficulty: Medium)

**Objective**: Implement pick-and-place with pose estimation

**Task**: Detect object (YOLOv8), estimate pose, plan grasp (MoveIt2)

**Requirements**:
- Use isaac_ros_foundationpose for 6D pose
- MoveIt2 plans collision-free trajectory
- Grasp success rate > 80% (10 trials)

**Expected Outcome**: Robot successfully grasps object 8/10 times

**Estimated Time**: 60 minutes

---

### Exercise 3: Jetson Deployment (Difficulty: Hard)

**Objective**: Deploy full navigation stack to Jetson

**Task**: Cross-compile, deploy, benchmark performance

**Requirements**:
- Build Isaac ROS packages for Jetson ARM64
- Deploy to Jetson Orin Nano (8GB)
- Measure FPS (Nvblox, VSLAM, YOLOv8), power consumption

**Expected Outcome**:
- Nvblox: 20+ Hz
- VSLAM: 20+ Hz
- YOLOv8: 30+ FPS
- Power: < 15W

**Estimated Time**: 2 hours

---

## Summary & Key Takeaways

- **Isaac ROS Nvblox** provides GPU-accelerated 3D mapping at 30 Hz (6× faster than CPU)
- **Isaac ROS Visual SLAM** enables real-time localization for Nav2 without LiDAR
- **MoveIt2 integration** with isaac_ros_pose_estimation enables autonomous grasping
- **Jetson deployment** brings full perception + navigation stack to edge (40W power)
- **Nav2 tuning** (costmap inflation, DWB params) critical for smooth, safe navigation

**Connection to Chapter 3.4**: With navigation and manipulation working, Chapter 3.4 explores reinforcement learning—training robots to learn complex behaviors (locomotion, manipulation) in Isaac Gym with 1000+ parallel environments.

---

## Additional Resources

- Nav2 Documentation: https://navigation.ros.org/
- Isaac ROS Nvblox: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/
- MoveIt2 Tutorials: https://moveit.picknik.ai/main/index.html

---

## Notes for Instructors

**Teaching Tips**:
- Demo: Show robot navigating with/without Nvblox (2D vs 3D obstacle avoidance)
- Compare: CPU VSLAM (10 Hz) vs GPU VSLAM (30 Hz) side-by-side

**Lab Ideas**:
- Lab 1: Build Nvblox map of classroom
- Lab 2: Implement autonomous delivery robot (station A → B)
- Lab 3: MoveIt2 pick-and-place competition

**Assessment**:
- Quiz: Explain Nav2 costmap layers
- Project: Deploy navigation to real robot, measure success rate

---

**Chapter Metadata**:
- **Word Count**: 3,000 words
- **Code Examples**: 6
- **Exercises**: 3
- **Glossary Terms**: 8
