# Chapter 2.4: Sensor Simulation & VSLAM

> **Module**: 2 - Digital Twin Simulation
> **Week**: 7
> **Estimated Reading Time**: 30 minutes

---

## Summary

Accurate sensor simulation is critical for closing the sim-to-real gap in vision-based and perception-heavy robots. This chapter covers realistic camera, LiDAR, and IMU models with noise characterization, introduces Visual SLAM (VSLAM) as a capstone application, and explores domain randomization techniques to improve real-world transfer.

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Configure** realistic sensor noise models (Gaussian, salt-and-pepper, dropout) matching real hardware specifications
2. **Characterize** sensor performance using Allan variance, signal-to-noise ratio (SNR), and range accuracy metrics
3. **Explain** the VSLAM pipeline (feature detection, matching, pose estimation, mapping, loop closure)
4. **Implement** domain randomization strategies for lighting, textures, and sensor parameters
5. **Deploy** ORB-SLAM3 in Gazebo simulation and analyze tracking performance

**Prerequisite Knowledge**: Chapter 2.1 (Digital Twin Concepts), Chapter 2.2 (Gazebo Simulation), basic computer vision (features, keypoints)

---

## Key Terms

This chapter introduces the following technical terms (see Glossary for detailed definitions):

- **Allan Variance**: Statistical method to characterize IMU noise (random walk, bias instability, rate noise)
- **Signal-to-Noise Ratio (SNR)**: Ratio of signal power to noise power, measured in decibels (dB)
- **VSLAM (Visual Simultaneous Localization and Mapping)**: Technique to estimate robot pose and build map using only camera images
- **Feature Point**: Distinctive image location (corner, blob) used for tracking across frames (e.g., ORB, SIFT, SURF)
- **Loop Closure**: Detecting when robot returns to previously visited location, correcting accumulated drift
- **Bundle Adjustment**: Optimization technique to refine camera poses and 3D points jointly
- **Domain Randomization**: Training technique varying simulation parameters (lighting, textures, noise) to improve robustness
- **Photometric Calibration**: Characterizing camera response to light intensity (gamma curve, vignetting, exposure)

---

## Core Concepts

### 1. Realistic Camera Simulation

Cameras are the most common sensors in robotics (low cost, rich information), but simulating them accurately is challenging due to optical effects and noise sources.

#### Camera Noise Models

Real cameras suffer from multiple noise sources—simulating them prevents overfitting to perfect synthetic images.

**1. Photon Shot Noise (Gaussian)**

**Source**: Quantum nature of light (photon arrival is Poisson process, approximates Gaussian for high intensity)

**Model**:
```python
noisy_pixel = clean_pixel + N(0, σ²)
where σ = k * sqrt(clean_pixel)  # Noise proportional to sqrt(intensity)
```

**Gazebo Configuration**:
```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.007</stddev>  <!-- ~2/255 for 8-bit image -->
</noise>
```

**Typical Values**:
- **Good camera** (Sony IMX sensors): σ = 0.003-0.005 (0.8-1.3 gray levels)
- **Consumer webcam**: σ = 0.01-0.02 (2.5-5 gray levels)
- **Low-light conditions**: σ increases (shot noise dominates)

---

**2. Salt-and-Pepper Noise (Dead/Hot Pixels)**

**Source**: Manufacturing defects, radiation damage

**Model**: Random pixels set to 0 (black) or 255 (white)

**Implementation** (post-processing in ROS node):
```python
import numpy as np

def add_salt_pepper(image, prob=0.001):
    noisy = image.copy()
    # Salt (white pixels)
    salt = np.random.rand(*image.shape[:2]) < prob/2
    noisy[salt] = 255
    # Pepper (black pixels)
    pepper = np.random.rand(*image.shape[:2]) < prob/2
    noisy[pepper] = 0
    return noisy
```

**Typical Values**: 0.1-0.5% of pixels (1-5 per 1000 pixels)

---

**3. Motion Blur**

**Source**: Camera moving during exposure (rolling shutter or global shutter)

**Model**: Convolve image with motion kernel

**Gazebo Limitation**: Not natively supported—requires post-processing plugin or Unity Perception Package

**Effect**: Edges appear smeared in direction of motion, reduces feature detection quality

---

**4. Lens Distortion**

**Types**:
- **Radial**: Barrel (negative k1) or pincushion (positive k1) distortion
- **Tangential**: Asymmetric distortion from lens misalignment

**Brown-Congiami Model**:
```
x_distorted = x_undistorted * (1 + k1*r² + k2*r⁴ + k3*r⁶) + [tangential terms]
```

**Gazebo Configuration**:
```xml
<camera>
  <distortion>
    <k1>-0.05</k1>  <!-- Barrel distortion -->
    <k2>0.01</k2>
    <k3>0.0</k3>
    <p1>0.0</p1>    <!-- Tangential p1 -->
    <p2>0.0</p2>    <!-- Tangential p2 -->
  </distortion>
</camera>
```

**When to Include**: Always include for wide-angle cameras (>90° FOV). Optional for narrow FOV (<60°).

---

#### Lighting and Photorealism

Poor lighting simulation is a major source of sim-to-real gap for vision systems.

**Gazebo Classic Limitations**:
- Basic Phong shading (diffuse + specular)
- No global illumination (light bounces)
- Simplified shadows (ray-traced or shadow maps)

**Unity/Isaac Sim Advantages**:
- Path-traced rendering (physically accurate light transport)
- HDR lighting (high dynamic range)
- Realistic materials (PBR: physically-based rendering)

**Domain Randomization Approach** (Gazebo):
```xml
<!-- Randomize lighting in Gazebo world -->
<light name='sun' type='directional'>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>  <!-- Randomize: 0.5-1.0 -->
  <specular>0.2 0.2 0.2 1</specular>
  <direction>-0.5 0.1 -0.9</direction>  <!-- Randomize sun angle -->
</light>
```

**Best Practice**: Train on images with randomized lighting (brightness ±30%, shadows on/off, sun angle varied).

---

### 2. LiDAR Simulation

LiDAR (Light Detection and Ranging) measures distance by timing laser reflections—critical for navigation and obstacle avoidance.

#### Range Noise Model

**Gaussian Noise**:
```python
measured_range = true_range + N(0, σ_range²)
```

**Typical Accuracy**:
- **2D LiDAR** (Hokuyo, SICK): σ = 0.02-0.03 m (2-3 cm)
- **3D LiDAR** (Velodyne VLP-16): σ = 0.03 m (3 cm)
- **High-end 3D** (Velodyne HDL-64E): σ = 0.02 m (2 cm)

**Gazebo Configuration**:
```xml
<sensor name="laser" type="ray">
  <ray>
    <range>
      <min>0.3</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.03</stddev>  <!-- 3 cm noise -->
    </noise>
  </ray>
</sensor>
```

---

#### Dropout Model (Missed Returns)

**Causes**:
- Dark surfaces (low reflectivity)
- Transparent materials (glass, water)
- Specular reflections (mirrors)
- Out-of-range targets

**Model**: Probability of missed return depends on material and angle

**Gazebo Limitation**: No native dropout model—post-process in ROS node:

```python
def add_lidar_dropout(scan, dropout_prob=0.05):
    noisy_scan = scan.copy()
    dropout_mask = np.random.rand(len(scan.ranges)) < dropout_prob
    noisy_scan.ranges[dropout_mask] = scan.range_max  # Invalid reading
    return noisy_scan
```

**Typical Values**: 2-5% dropout rate for indoor environments, higher outdoors (sunlight interference).

---

#### Multi-Path Reflections

**Problem**: Laser bounces off multiple surfaces before returning (e.g., corner reflectors, metal surfaces)

**Effect**: Measured range longer than true range (ghost points in point cloud)

**Gazebo**: Not modeled (assumes single-bounce ray tracing)

**Mitigation**: Train navigation algorithms to be robust to outliers (RANSAC, statistical outlier removal).

---

### 3. IMU Simulation

Inertial Measurement Units (IMUs) measure angular velocity (gyroscope) and linear acceleration (accelerometer)—essential for state estimation and sensor fusion.

#### Allan Variance Characterization

Allan variance decomposes IMU noise into components with different time scales:

1. **White Noise** (Angle Random Walk): High-frequency noise, σ ∝ 1/√τ
2. **Bias Instability**: Slowly drifting zero-offset, flat region in Allan variance plot
3. **Rate Random Walk**: Very low-frequency noise, σ ∝ √τ

**Typical IMU Specifications**:

| IMU Grade | Gyro Noise (°/h) | Accel Noise (mg) | Bias Stability (°/h) | Cost |
|-----------|------------------|------------------|---------------------|------|
| **Consumer** (MPU-6050) | 0.1 | 0.5 | 10 | $5 |
| **Tactical** (BMI088) | 0.01 | 0.1 | 1 | $50 |
| **Navigation** (VectorNav VN-100) | 0.001 | 0.01 | 0.1 | $500 |

---

#### Gazebo IMU Plugin Configuration

```xml
<sensor name="imu" type="imu">
  <update_rate>200</update_rate>  <!-- 200 Hz typical -->
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.02</stddev>  <!-- 0.02 rad/s = ~1.15 deg/s -->
        </noise>
      </x>
      <!-- Repeat for y, z -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.1</stddev>  <!-- 0.1 m/s² = ~0.01 g -->
        </noise>
      </x>
      <!-- Repeat for y, z -->
    </linear_acceleration>
  </imu>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <remapping>imu:=imu/data</remapping>
    </ros>
  </plugin>
</sensor>
```

**Common Mistake**: Forgetting gravity alignment—Gazebo IMU always aligns Z-axis with gravity (perfect alignment, unrealistic). Add bias to simulate misalignment.

---

### 4. Visual SLAM (VSLAM) Fundamentals

Visual SLAM estimates robot pose and builds a 3D map using only camera images—no GPS, LiDAR, or wheel odometry required.

#### VSLAM Pipeline (6 Stages)

**Stage 1: Feature Detection**

Extract distinctive keypoints (corners, blobs) from image.

**Algorithms**:
- **ORB** (Oriented FAST and Rotated BRIEF): Fast, rotation-invariant, used in ORB-SLAM
- **SIFT** (Scale-Invariant Feature Transform): Scale-invariant, patented (expired 2020)
- **SURF** (Speeded-Up Robust Features): Faster than SIFT, less accurate

**Output**: List of (x, y) pixel coordinates + descriptor (binary or float vector)

---

**Stage 2: Feature Matching**

Match features between current frame and previous frame (or map points).

**Methods**:
- **Brute-force**: Compare all pairs (O(N²), slow)
- **FLANN** (Fast Library for Approximate Nearest Neighbors): KD-tree or LSH (O(N log N))

**Output**: Correspondences (feature_id in frame t) ↔ (feature_id in frame t-1)

---

**Stage 3: Pose Estimation**

Compute camera motion (translation + rotation) from feature correspondences.

**Algorithm**: **PnP** (Perspective-n-Point) with RANSAC
- Input: 2D image points + known 3D map points
- Output: Camera pose [R|t] (rotation matrix + translation vector)
- RANSAC: Outlier rejection (mismatched features)

**Minimum**: 4 correspondences (P4P), more is better (overdetermined system)

---

**Stage 4: Triangulation (Depth Estimation)**

Estimate 3D position of new features using multiple views (stereo or monocular with motion).

**Stereo**: Disparity (pixel offset between left/right images) → depth
**Monocular**: Triangulate from two frames at different poses (requires motion)

**Output**: 3D points in world frame (map landmarks)

---

**Stage 5: Local Mapping**

Optimize recent camera poses and map points using **Bundle Adjustment**:
- Minimize reprojection error: Σ ||observed_pixel - projected_3D_point||²
- Non-linear optimization (Levenberg-Marquardt, Gauss-Newton)

**Keyframe Selection**: Only optimize subset of frames (keyframes) to save computation.

---

**Stage 6: Loop Closure**

Detect when robot revisits previously mapped area, correct accumulated drift.

**Method**:
1. Compute image similarity (Bag of Words, DBoW2 library)
2. If similarity high, verify with feature matching
3. If verified, compute pose correction (loop constraint)
4. Run global optimization (Pose Graph Optimization) to distribute correction

**Impact**: Without loop closure, drift accumulates (position error grows unbounded). With loop closure, drift bounded.

---

#### Monocular vs. Stereo vs. RGB-D VSLAM

| Type | Input | Scale | Depth Accuracy | CPU | Best Use Case |
|------|-------|-------|----------------|-----|---------------|
| **Monocular** | Single camera | ❌ Unknown (up to scale) | ⚠️ Triangulation (poor at close range) | ★★☆☆☆ Low | Drones, small robots |
| **Stereo** | Two cameras | ✅ Known (from baseline) | ★★★★☆ Good | ★★★★☆ High | Outdoor robots, autonomous vehicles |
| **RGB-D** | Camera + depth sensor | ✅ Known (from depth) | ★★★★★ Excellent | ★★★☆☆ Moderate | Indoor robots, manipulation |

**Monocular Scale Problem**: Trajectory shape is correct, but absolute distances unknown (cannot distinguish "1m forward" from "2m forward" without other sensors).

**Solution**: Fuse with IMU (Visual-Inertial Odometry, VIO) or wheel odometry.

---

### 5. Domain Randomization for Sim-to-Real Transfer

Domain randomization trains policies/models on diverse simulated environments, improving robustness to real-world variations.

#### Randomization Strategies

**1. Lighting Randomization**
- **Sun position**: Randomize azimuth (0-360°), elevation (30-80°)
- **Brightness**: Scale diffuse intensity (0.5-1.5×)
- **Shadows**: On/off, shadow quality (hard vs. soft)

**Example**:
```python
# In Gazebo world spawn script
import random
sun_azimuth = random.uniform(0, 2*np.pi)
sun_elevation = random.uniform(np.pi/6, np.pi/2.5)
brightness = random.uniform(0.5, 1.5)
```

---

**2. Texture Randomization**
- Replace object textures with random images
- Vary surface materials (metal, wood, plastic)
- Add procedural noise (Perlin noise for natural variation)

**Unity Advantage**: Asset Store has thousands of textures. Gazebo requires manual texture swapping.

---

**3. Sensor Parameter Randomization**
- **Camera**: Noise stddev (0.005-0.02), exposure (±1 stop), white balance
- **LiDAR**: Range noise (0.02-0.05 m), dropout probability (0-10%)
- **IMU**: Bias drift (±0.1 m/s² per minute)

**Example** (Gazebo launch file with randomization):
```python
noise_stddev = random.uniform(0.005, 0.02)
urdf_with_noise = urdf_template.replace("{{NOISE}}", str(noise_stddev))
```

---

**4. Dynamics Randomization**
- **Mass**: ±20% of nominal value
- **Friction**: Vary coefficient (0.3-0.9 for surfaces)
- **Motor torque**: ±15% to simulate wear

**Result**: Policy robust to model uncertainty, transfers better to real hardware.

---

#### When Does Domain Randomization Work?

**Successes**:
- ✅ Object detection (YOLO, Mask R-CNN trained on synthetic data)
- ✅ Grasping (OpenAI Dactyl: 100% sim-trained, grasped Rubik's cube in reality)
- ✅ Navigation (avoiding obstacles under varied lighting)

**Failures**:
- ❌ Contact-rich manipulation (friction/stiffness too hard to randomize accurately)
- ❌ Deformable objects (cloth, liquids—poorly modeled in simulation)
- ❌ Long-horizon tasks (compounding errors over 100+ steps)

**Best Practice**: Combine domain randomization (train in sim) + fine-tuning (adapt on real robot with small dataset).

---

## Practical Example: Running ORB-SLAM3 in Gazebo

### Overview

We'll run ORB-SLAM3 (state-of-the-art visual SLAM) in Gazebo with a simulated RGB-D camera, demonstrating complete VSLAM pipeline.

### Prerequisites

- Software: ROS 2 Humble, Gazebo Classic, ORB-SLAM3 (build from source)
- Hardware: GPU recommended (ORB feature extraction is CPU-intensive)

### Implementation

**Step 1: Install ORB-SLAM3**

```bash
# Install dependencies
sudo apt install libeigen3-dev libopencv-dev libpangolin-dev

# Clone ORB-SLAM3
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3

# Build
chmod +x build.sh
./build.sh

# Build ROS 2 wrapper (if available, otherwise use ROS 1 with ros1_bridge)
```

---

**Step 2: Create Gazebo World with Texture**

`worlds/textured_room.world`:
```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="textured_room">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <include><uri>model://sun</uri></include>

    <!-- Ground with texture -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><plane><size>20 20</size></plane></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>  <!-- Textured floor -->
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add boxes with textures for features -->
    <include><uri>model://coke_can</uri><pose>2 0 0.5 0 0 0</pose></include>
    <include><uri>model://bookshelf</uri><pose>3 2 0 0 0 0</pose></include>
    <include><uri>model://table</uri><pose>-1 -1 0 0 0 0</pose></include>
  </world>
</sdf>
```

---

**Step 3: Spawn Robot with RGB-D Camera**

(Use mobile robot URDF from Chapter 2.2, add depth camera plugin)

```xml
<gazebo reference="camera_link">
  <sensor name="rgbd_camera" type="depth">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60° -->
      <image><width>640</width><height>480</height></image>
      <clip><near>0.3</near><far>10.0</far></clip>
    </camera>
    <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>image_raw:=camera/rgb/image</remapping>
        <remapping>depth/image_raw:=camera/depth/image</remapping>
        <remapping>camera_info:=camera/rgb/camera_info</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

---

**Step 4: Configure ORB-SLAM3**

Create `config/rgbd_gazebo.yaml`:
```yaml
%YAML:1.0

# Camera Parameters
Camera.fx: 320.0  # Focal length X (pixels)
Camera.fy: 320.0  # Focal length Y
Camera.cx: 320.0  # Principal point X
Camera.cy: 240.0  # Principal point Y

Camera.k1: 0.0  # Radial distortion
Camera.k2: 0.0
Camera.p1: 0.0  # Tangential distortion
Camera.p2: 0.0

Camera.width: 640
Camera.height: 480
Camera.fps: 30

# Depth Parameters
ThDepth: 40.0  # Max depth (meters)
DepthMapFactor: 1000.0  # Depth scale (1000 for mm → m)

# ORB Features
ORBextractor.nFeatures: 1000  # Features per frame
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8

# Viewer
Viewer.KeyFrameSize: 0.05
Viewer.CameraSize: 0.08
```

---

**Step 5: Launch Simulation**

```bash
# Terminal 1: Gazebo
ros2 launch my_robot_description gazebo_sim.launch.py world:=textured_room.world

# Terminal 2: Teleoperation (drive robot around)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: ORB-SLAM3
cd ~/ORB_SLAM3
./Examples/ROS/ORB_SLAM3/RGBD_ROS \
  Vocabulary/ORBvoc.txt \
  config/rgbd_gazebo.yaml
```

---

**Step 6: Observe VSLAM Output**

ORB-SLAM3 GUI shows:
- **Top-left**: Current frame with detected ORB features (green circles)
- **Top-right**: Feature matches between frames (lines connecting keypoints)
- **Bottom**: 3D map (blue points = map landmarks, red line = robot trajectory)

As robot moves, trajectory grows and map expands. Return to starting location to trigger loop closure (map snaps to correct position).

### Expected Output

- ORB-SLAM3 tracks camera pose at 30 Hz
- Map contains 500-2000 landmarks (depends on texture richness)
- Drift accumulates during exploration, corrected when loop closure detected
- Final trajectory should match ground truth (compare to Gazebo `/odom` topic)

### Troubleshooting

- **Issue**: "Not enough features detected"
  **Solution**: Add more textured objects to world, increase `ORBextractor.nFeatures` to 1500

- **Issue**: Tracking lost frequently
  **Solution**: Drive slower (reduce `/cmd_vel` velocities), ensure lighting adequate

- **Issue**: Loop closure not triggered
  **Solution**: Return to starting location, stop for 2-3 seconds (gives SLAM time to detect similarity)

### Further Exploration

- Compare ORB-SLAM3 trajectory to ground truth: `ros2 topic echo /odom`
- Add noise to camera (`<stddev>0.01</stddev>`), observe SLAM robustness
- Try monocular SLAM (remove depth), observe scale drift

---

## Figures & Diagrams

### Figure 2.4-1: VSLAM Pipeline

*(See separate diagram file `fig2.4-vslam-pipeline.md` for 6-stage flowchart)*

**Caption**: Visual SLAM pipeline showing feature detection, matching, pose estimation, triangulation, local mapping, and loop closure. Illustrates how camera images are processed to estimate robot trajectory and build 3D map.

**Reference**: This figure supports Section 4 (VSLAM Fundamentals) by visualizing the complete processing pipeline.

---

## Exercises

### Exercise 1: Sensor Noise Tuning (Difficulty: Easy)

**Objective**: Match simulated IMU to real hardware datasheet

**Task**: Configure Gazebo IMU plugin for **Bosch BMI088** with these specs:
- Gyro noise density: 0.014°/s/√Hz (at 200 Hz → 0.198°/s)
- Accel noise density: 150 μg/√Hz (at 200 Hz → 0.0212 m/s²)

**Requirements**:
- Convert noise density to stddev for Gazebo `<noise>` tag
- Write complete IMU sensor configuration

**Expected Outcome**: IMU publishes `/imu/data` with realistic noise matching BMI088 datasheet

**Hints**:
- Noise stddev = noise_density × √(sample_rate)
- 1 g = 9.81 m/s²
- 1° = 0.01745 rad

**Estimated Time**: 20 minutes

---

### Exercise 2: Domain Randomization Implementation (Difficulty: Medium)

**Objective**: Randomize lighting and textures in Gazebo for vision ML training

**Task**: Write Python script that:
1. Spawns Gazebo world with random sun position (azimuth 0-360°, elevation 30-70°)
2. Spawns 10 boxes with random textures (select from 5 texture options)
3. Spawns robot with camera, captures 100 images with varied lighting
4. Saves images to dataset folder with labels

**Requirements**:
- Use `gazebo_msgs/srv/SetLightProperties` to change sun
- Use `gazebo_msgs/srv/SetModelState` to randomize object poses
- Capture images via `/camera/image` subscription

**Expected Outcome**: Dataset of 100 images with diverse lighting/textures suitable for training object detector

**Estimated Time**: 60 minutes

---

### Exercise 3: VSLAM Evaluation (Difficulty: Hard)

**Objective**: Quantify VSLAM accuracy by comparing to ground truth

**Task**: Run ORB-SLAM3 in Gazebo, compute trajectory error metrics:
1. Record SLAM trajectory (published on `/orbslam3/pose`)
2. Record ground truth trajectory (Gazebo `/odom`)
3. Compute **Absolute Trajectory Error (ATE)**: RMSE of position differences
4. Compute **Relative Pose Error (RPE)**: Frame-to-frame odometry error
5. Plot trajectories (SLAM vs. ground truth) in 3D

**Requirements**:
- Use `evo` package for trajectory evaluation: `pip install evo`
- Convert ROS bag to TUM format (timestamp x y z qx qy qz qw)
- Run: `evo_ape tum ground_truth.txt slam_trajectory.txt --plot`

**Expected Outcome**: Report showing ATE < 0.05 m and RPE < 0.01 m for good VSLAM performance

**Hints**:
- Record topics: `ros2 bag record /orbslam3/pose /odom`
- Extract to text: `ros2 bag convert bag_file/ --format tum`
- Lower ATE/RPE = better SLAM accuracy

**Estimated Time**: 90 minutes

---

## Summary & Key Takeaways

In this chapter, you learned:

- **Realistic sensor models** require noise (Gaussian for cameras/IMU, dropout for LiDAR) matching real hardware datasheets
- **Allan variance** characterizes IMU noise across time scales (white noise, bias instability, random walk)
- **VSLAM pipeline** consists of feature detection, matching, pose estimation, triangulation, mapping, and loop closure—enabling camera-only localization
- **Domain randomization** (lighting, textures, sensor noise) improves sim-to-real transfer by training on diverse environments
- **ORB-SLAM3** is a state-of-the-art VSLAM system supporting monocular, stereo, and RGB-D cameras with real-time performance

**Connection to Module 3**: Module 2 covered simulation environments (Gazebo, Unity) and sensor simulation. Module 3 (Isaac AI Brain) will integrate GPU-accelerated perception pipelines (TensorRT), VSLAM, and reinforcement learning on NVIDIA hardware—building on the digital twin foundation established in this module.

---

## Additional Resources

### Official Documentation
- ORB-SLAM3 Paper: Campos et al. (2021). "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM." IEEE T-RO.
- Gazebo Sensors Tutorial: http://classic.gazebosim.org/tutorials?tut=ros_gzplugins#Sensor - Complete sensor plugin reference

### Recommended Reading
- Tobin et al. (2017). "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World." IROS 2017.
- Allan Variance Explained: https://www.nist.gov/pml/time-frequency-division/frequency-stability/allan-variance - NIST guide

### Community Resources
- ORB-SLAM3 GitHub Issues: https://github.com/UZ-SLAMLab/ORB_SLAM3/issues - Troubleshooting Q&A
- ROS Discourse (SLAM): https://discourse.ros.org/ - Search "VSLAM" or "ORB-SLAM"

---

## Notes for Instructors

**Teaching Tips**:
- Start with visual demo: Show perfect simulated camera vs. noisy camera side-by-side, run object detector on both, show accuracy drop without noise
- Common misconception: "Simulation is deterministic"—emphasize need for noise to prevent overfitting
- Live demo: Run ORB-SLAM3 in Gazebo, show loop closure in action (trajectory snaps to correct position)

**Lab Exercise Ideas**:
- **Lab 1**: Sensor characterization—students record IMU data, compute Allan variance plot, identify noise components
- **Lab 2**: Domain randomization—students train YOLOv8 on 100% synthetic images, test on real images, compare to baseline
- **Lab 3**: VSLAM challenge—students run ORB-SLAM3 in maze world, compete for lowest ATE vs. ground truth

**Assessment Suggestions**:
- **Sensor configuration quiz**: Given hardware datasheet, write complete Gazebo sensor XML with accurate noise parameters
- **VSLAM conceptual exam**: Explain each VSLAM stage (detection → matching → pose estimation → mapping), identify failure modes (low texture, motion blur, dynamic objects)
- **Sim-to-real project**: Train vision model in Gazebo with domain randomization, deploy to real robot, report accuracy gap and mitigation strategies

---

**Chapter Metadata**:
- **Word Count**: 3,600 words (core concepts)
- **Figures**: 1 (VSLAM pipeline)
- **Code Examples**: 6 (noise models, randomization, ORB-SLAM3 config)
- **Exercises**: 3 (easy, medium, hard)
- **Glossary Terms**: 8
- **Cross-References**: Chapter 2.1, 2.2, forward to Module 3
