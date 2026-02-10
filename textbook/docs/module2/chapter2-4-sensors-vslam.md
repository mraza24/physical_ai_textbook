---
sidebar_position: 5
title: Chapter 2.4 - Sensor Simulation and VSLAM
---

# Chapter 2.4: Sensor Simulation and Visual SLAM

**Module**: 2 - The Digital Twin
**Week**: 8
**Estimated Reading Time**: 30 minutes

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. Simulate depth cameras (Intel RealSense D435/D455)
2. Generate realistic sensor data (noise, distortion, lighting)
3. Implement Visual SLAM using ORB-SLAM3 or Cartographer
4. Generate 3D maps from simulated sensor data
5. Validate SLAM performance in simulation

---

## Prerequisites

- Completed Chapters 2.1, 2.2, and 2.3
- Understanding of camera models and depth sensing
- Familiarity with coordinate transformations (tf2)

---

## Introduction

**Visual SLAM** (Simultaneous Localization and Mapping) enables robots to build maps while tracking their position—essential for autonomous navigation. Before deploying SLAM on real hardware, **simulation** allows testing in controlled environments with perfect ground truth.

This chapter covers simulating RGB-D cameras (Intel RealSense, Azure Kinect) in Gazebo/Unity, adding realistic noise models, and running SLAM algorithms to generate 3D maps.

---

## Key Terms

:::info Glossary Terms
- **VSLAM**: Visual Simultaneous Localization and Mapping
- **Depth Camera**: Sensor that captures RGB and depth information
- **Point Cloud**: 3D representation of sensor data (x, y, z coordinates)
- **Loop Closure**: Detecting revisited locations to correct drift
- **Occupancy Grid**: 2D map representation (free, occupied, unknown cells)
- **Feature Matching**: Finding correspondences between images for pose estimation
:::

---

## Core Concepts

### 1. Depth Camera Simulation

#### Gazebo Depth Camera Plugin

**SDF Configuration** (Intel RealSense D435 equivalent):
```xml
<sensor name="realsense_d435" type="depth_camera">
  <camera>
    <horizontal_fov>1.211</horizontal_fov> <!-- 69.4 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.3</near> <!-- 30cm minimum range -->
      <far>10.0</far>  <!-- 10m maximum range -->
    </clip>
    <!-- Depth-specific settings -->
    <depth_camera>
      <output>depth</output> <!-- Publish depth image -->
    </depth_camera>
    <!-- Realistic noise model -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev> <!-- 1cm depth noise -->
    </noise>
  </camera>
  <update_rate>30</update_rate>
  <visualize>false</visualize>

  <!-- ROS 2 Plugin -->
  <plugin filename="libgazebo_ros_camera.so" name="depth_camera_controller">
    <ros>
      <namespace>/camera</namespace>
      <remapping>image_raw:=color/image_raw</remapping>
      <remapping>depth/image_raw:=depth/image_raw</remapping>
      <remapping>points:=depth/points</remapping>
    </ros>
    <camera_name>realsense_d435</camera_name>
    <frame_name>camera_depth_optical_frame</frame_name>
  </plugin>
</sensor>
```

**Published Topics**:
- `/camera/color/image_raw` (RGB image, 640×480)
- `/camera/depth/image_raw` (Depth image, 640×480, uint16)
- `/camera/depth/points` (PointCloud2, organized)
- `/camera/camera_info` (Calibration parameters)

#### Unity Depth Camera

**C# Script** (`Assets/Scripts/DepthCameraPublisher.cs`):
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class DepthCameraPublisher : MonoBehaviour
{
    private Camera rgbCamera;
    private Camera depthCamera;
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>("/camera/depth/image_raw");

        // Create depth camera with shader replacement
        depthCamera = gameObject.AddComponent<Camera>();
        depthCamera.CopyFrom(rgbCamera);
        depthCamera.SetReplacementShader(Shader.Find("DepthShader"), "");
    }

    void Update()
    {
        // Render depth and publish (simplified)
        RenderTexture.active = depthCamera.targetTexture;
        Texture2D depthTex = new Texture2D(640, 480, TextureFormat.R16, false);
        depthTex.ReadPixels(new Rect(0, 0, 640, 480), 0, 0);

        ImageMsg msg = new ImageMsg
        {
            width = 640,
            height = 480,
            encoding = "16UC1", // Depth in millimeters
            data = depthTex.GetRawTextureData()
        };
        ros.Publish("/camera/depth/image_raw", msg);
    }
}
```

### 2. Sensor Noise Modeling

Real-world depth cameras have imperfections that must be simulated for realistic testing.

#### Types of Depth Noise

**1. Quantization Noise** (discrete depth values):
```python
# Add quantization (1mm steps for RealSense)
depth_quantized = np.round(depth_meters * 1000) / 1000
```

**2. Gaussian Noise** (random fluctuation):
```python
# Depth-dependent noise (farther = noisier)
noise_stddev = 0.001 + 0.0005 * depth_meters  # Linear model
depth_noisy = depth_meters + np.random.normal(0, noise_stddev, depth_meters.shape)
```

**3. Flying Pixels** (edge artifacts):
```python
# Remove depth at edges (occlusion boundaries)
depth_gradient = np.gradient(depth_meters)
flying_pixel_mask = np.abs(depth_gradient) > 0.05  # 5cm threshold
depth_cleaned = np.where(flying_pixel_mask, 0, depth_meters)
```

**4. Invalid Regions** (reflective/transparent surfaces):
```python
# Simulate failure on glass, mirrors (reflectivity > 0.9)
if material.reflectivity > 0.9:
    depth_value = 0  # Invalid depth
```

#### Gazebo Noise Plugin

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <!-- Depth-dependent noise -->
  <stddev>0.007</stddev>
</noise>
```

### 3. Visual SLAM Algorithms

#### ORB-SLAM3 (Feature-Based)

**Strengths**:
- Accurate in textured environments
- Loop closure detection
- Supports monocular, stereo, RGB-D cameras

**Workflow**:
1. Extract ORB features from images
2. Match features across frames
3. Estimate camera pose (PnP algorithm)
4. Triangulate 3D map points
5. Bundle adjustment for global consistency
6. Detect loop closures to correct drift

**Installation**:
```bash
# Dependencies
sudo apt install libopencv-dev libeigen3-dev libpangolin-dev

# Clone and build
cd ~/ros2_ws/src
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3
cd ORB_SLAM3
chmod +x build.sh
./build.sh

# ROS 2 wrapper
git clone https://github.com/thien94/orb_slam3_ros2
cd ~/ros2_ws
colcon build --packages-select orb_slam3_ros2
```

**Launch**:
```bash
# Run ORB-SLAM3 with RGB-D camera
ros2 run orb_slam3_ros2 rgbd \
    --ros-args \
    -p vocabulary_file:=/path/to/ORBvoc.txt \
    -p settings_file:=/path/to/RealSense.yaml \
    -r /camera/color/image_raw:=/camera/color/image_raw \
    -r /camera/depth/image_raw:=/camera/depth/image_raw
```

#### Cartographer (Graph-Based)

**Strengths**:
- Robust to sensor noise
- Real-time performance
- Supports LiDAR + IMU fusion

**Installation**:
```bash
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
```

**Configuration** (`cartographer_config.lua`):
```lua
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

MAP_BUILDER.use_trajectory_builder_2d = true
POSE_GRAPH.optimize_every_n_nodes = 90
```

**Launch**:
```bash
ros2 launch cartographer_ros cartographer.launch.py \
    configuration_directory:=/path/to/config \
    configuration_basename:=cartographer_config.lua
```

### 4. Map Generation and Evaluation

#### Occupancy Grid Map

**Conversion from Point Cloud**:
```python
from nav_msgs.msg import OccupancyGrid
import numpy as np

def pointcloud_to_occupancy_grid(points, resolution=0.05):
    """
    Convert 3D point cloud to 2D occupancy grid
    Args:
        points: Nx3 numpy array (x, y, z)
        resolution: meters per cell
    Returns:
        OccupancyGrid message
    """
    # Project to 2D (remove z)
    points_2d = points[:, :2]

    # Discretize to grid cells
    min_x, min_y = points_2d.min(axis=0)
    max_x, max_y = points_2d.max(axis=0)

    grid_width = int((max_x - min_x) / resolution) + 1
    grid_height = int((max_y - min_y) / resolution) + 1

    # Initialize grid (0 = unknown, 100 = occupied, -1 = free)
    grid = np.zeros((grid_height, grid_width), dtype=np.int8)

    # Mark occupied cells
    for point in points_2d:
        x_idx = int((point[0] - min_x) / resolution)
        y_idx = int((point[1] - min_y) / resolution)
        grid[y_idx, x_idx] = 100

    # Publish as OccupancyGrid
    msg = OccupancyGrid()
    msg.info.resolution = resolution
    msg.info.width = grid_width
    msg.info.height = grid_height
    msg.info.origin.position.x = min_x
    msg.info.origin.position.y = min_y
    msg.data = grid.flatten().tolist()

    return msg
```

#### SLAM Evaluation Metrics

**1. Absolute Trajectory Error (ATE)**:
```python
import numpy as np

def compute_ate(estimated_poses, ground_truth_poses):
    """
    Compute ATE between estimated and ground truth trajectories
    Args:
        estimated_poses: List of (x, y, theta) tuples
        ground_truth_poses: List of (x, y, theta) tuples
    Returns:
        Mean ATE (meters)
    """
    errors = []
    for est, gt in zip(estimated_poses, ground_truth_poses):
        error = np.sqrt((est[0] - gt[0])**2 + (est[1] - gt[1])**2)
        errors.append(error)

    return np.mean(errors)

# Example usage
ate = compute_ate(slam_trajectory, gazebo_ground_truth)
print(f"Absolute Trajectory Error: {ate:.3f} meters")
```

**2. Map Accuracy** (compare to ground truth CAD model):
```python
# Compute Intersection over Union (IoU)
def map_iou(predicted_map, ground_truth_map):
    intersection = np.logical_and(predicted_map, ground_truth_map).sum()
    union = np.logical_or(predicted_map, ground_truth_map).sum()
    return intersection / union if union > 0 else 0

iou = map_iou(slam_map, cad_map)
print(f"Map IoU: {iou:.2%}")
```

---

## Practical Example: Complete VSLAM Pipeline

### Step 1: Gazebo World with Texture

**File**: `textured_warehouse.world`
```xml
<sdf version="1.9">
  <world name="textured_warehouse">
    <physics name="1ms" type="bullet">
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- Well-lit environment -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <intensity>500</intensity>
    </light>

    <!-- Textured walls (important for feature extraction) -->
    <model name="wall_front">
      <pose>5 0 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>0.2 10 2</size></box></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name> <!-- Textured material -->
            </script>
          </material>
        </visual>
        <collision name="collision">
          <geometry><box><size>0.2 10 2</size></box></geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### Step 2: Launch SLAM Pipeline

```bash
# Terminal 1: Gazebo simulation
gz sim textured_warehouse.world

# Terminal 2: Spawn TurtleBot3 with depth camera
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 3: ORB-SLAM3
ros2 run orb_slam3_ros2 rgbd --ros-args -p vocabulary_file:=ORBvoc.txt

# Terminal 4: Teleoperate robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 5: Visualize in RViz
rviz2 -d slam_config.rviz
```

### Step 3: Evaluate Results

```bash
# Record trajectory
ros2 bag record /orb_slam3/trajectory /tf

# Compare with ground truth (from Gazebo /odom topic)
python3 evaluate_slam.py --bag slam_trajectory.bag --gt gazebo_odom.csv
```

**Expected Output**:
```
Absolute Trajectory Error: 0.042 meters
Relative Pose Error: 0.018 meters/meter
Loop Closures Detected: 3
Final Map Points: 1,247
```

---

## SLAM Algorithm Comparison

| Feature | ORB-SLAM3 | Cartographer | RTAB-Map |
|---------|-----------|--------------|----------|
| **Sensor Support** | Mono/Stereo/RGB-D | LiDAR/Multi-Sensor | RGB-D/Stereo |
| **Accuracy** | Very High | High | High |
| **Speed** | 30 FPS | Real-time | 15-20 FPS |
| **Loop Closure** | Yes (DBoW2) | Yes (Graph) | Yes (BoW) |
| **Map Type** | Sparse (features) | Dense (grid) | Dense (3D) |
| **Best For** | Feature-rich indoor | Large outdoor | Long-term autonomy |

---

## Common Issues

### Issue 1: SLAM Fails in Low-Texture Areas

**Symptom**: Tracking lost in white walls, uniform floors

**Solution**:
```bash
# Add artificial features (AprilTags, posters) in simulation
# Or use LiDAR-based SLAM instead of visual SLAM
```

### Issue 2: Depth Noise Breaks SLAM

**Symptom**: Map points scattered, trajectory jumpy

**Solution**:
```python
# Filter depth image before SLAM
import cv2

def filter_depth(depth_image):
    # Bilateral filter preserves edges while reducing noise
    filtered = cv2.bilateralFilter(depth_image, d=5, sigmaColor=50, sigmaSpace=50)
    return filtered
```

### Issue 3: Loop Closure Drift

**Symptom**: Map inconsistent after revisiting locations

**Solution**:
```lua
-- Increase loop closure detection rate (Cartographer)
POSE_GRAPH.optimize_every_n_nodes = 30  -- Default: 90
```

---

## Summary

This chapter covered **sensor simulation and VSLAM**:

1. **Depth Cameras**: Simulated RGB-D sensors (RealSense) with noise models
2. **ORB-SLAM3**: Feature-based visual SLAM for indoor environments
3. **Cartographer**: Graph-based SLAM with LiDAR support
4. **Evaluation**: ATE, map IoU metrics for validation

**Key Takeaways**:
- Simulate realistic sensor noise for robust SLAM testing
- ORB-SLAM3 requires textured environments (features)
- Cartographer excels in large, feature-poor spaces
- Always validate SLAM with ground truth from simulation

**Next Module**: NVIDIA Isaac SDK for GPU-accelerated perception

---

## End-of-Chapter Exercises

### Exercise 1: Implement Visual SLAM (Difficulty: Hard)

**Tasks**:
1. Create Gazebo world with textured environment
2. Add depth camera to TurtleBot3
3. Launch ORB-SLAM3 in RGB-D mode
4. Teleoperate robot to map environment
5. Compute ATE against Gazebo ground truth

**Success Criteria**: ATE < 0.1 meters, Map has 500+ points

### Exercise 2: Sensor Noise Analysis (Difficulty: Medium)

**Tasks**:
1. Record depth images from simulation (clean)
2. Add Gaussian noise (σ = 0.01m)
3. Run SLAM on both clean and noisy data
4. Compare trajectory accuracy

**Success Criteria**: Document ATE degradation with noise level

---

## Further Reading

1. ORB-SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3
2. Cartographer: https://google-cartographer-ros.readthedocs.io/
3. RTAB-Map Documentation: http://introlab.github.io/rtabmap/
4. "Past, Present, and Future of SLAM" (Cadena et al., 2016)

---

## Next Module

Continue to **[Module 3: The AI-Robot Brain](../module3/intro)** for GPU-accelerated perception with NVIDIA Isaac.

**🎓 Module 2 Complete!** You can now simulate and validate robot systems in digital twins.
