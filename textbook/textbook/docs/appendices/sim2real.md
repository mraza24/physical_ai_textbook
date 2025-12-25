---
sidebar_position: 4
title: Appendix D - Sim-to-Real Transfer Techniques
---

# Appendix D: Sim-to-Real Transfer Techniques

Comprehensive guide to bridging the simulation-to-reality gap.

---

## Overview

**Sim-to-Real Gap**: Differences between simulated and real-world robot behavior due to:
- Imperfect physics simulation
- Simplified sensor models
- Unmodeled dynamics (friction, compliance, latency)
- Environmental variations (lighting, textures, disturbances)

This appendix covers techniques to minimize this gap.

---

## Core Techniques

### 1. Domain Randomization

**Concept**: Vary simulation parameters to expose the model to a wide distribution of environments.

**Parameters to Randomize**:
- **Visual**: Lighting, textures, colors, camera pose
- **Physical**: Mass, friction, damping, joint limits
- **Sensor**: Noise levels, resolution, dropout
- **Dynamics**: External forces, ground compliance

**Implementation Example (Isaac Gym)**:
```python
# Randomize object properties
for obj in objects:
    mass = np.random.uniform(0.1, 2.0)  # kg
    friction = np.random.uniform(0.2, 1.5)
    color = np.random.uniform([0,0,0], [1,1,1])

    obj.set_mass(mass)
    obj.set_friction(friction)
    obj.set_color(color)

# Randomize lighting
light_intensity = np.random.uniform(0.5, 2.0)
light_position = np.random.uniform([-5,-5,2], [5,5,5])
scene.set_light(intensity, position)
```

**References**:
- Tobin et al. (2017). Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World. *IROS*.

---

### 2. System Identification

**Concept**: Measure real robot parameters and update simulation to match.

**Parameters to Identify**:
- Link masses, inertias, centers of mass
- Joint friction (static, dynamic)
- Motor torque curves
- Sensor calibration (intrinsics, extrinsics)

**Procedure**:
1. Perform controlled experiments on real robot
2. Record trajectories, forces, sensor data
3. Optimize simulation parameters to minimize error
4. Validate with new test trajectories

**Tools**:
- `robot_calibration` (ROS 2 package)
- MuJoCo Identification tools
- Custom optimization scripts (SciPy)

---

### 3. Sim2Sim Transfer

**Concept**: Train in multiple simulators to learn simulator-invariant features.

**Approach**:
1. Train policy in Gazebo
2. Fine-tune in Isaac Sim
3. Validate in PyBullet
4. Deploy to real robot

**Rationale**: If policy works across simulators, it's more likely to transfer to reality.

---

### 4. Reality Gap Metrics

**Concept**: Measure and monitor the gap quantitatively.

**Metrics**:
- **Trajectory Error**: $\|q_{sim}(t) - q_{real}(t)\|_2$
- **Success Rate Delta**: $SR_{real} - SR_{sim}$
- **Sensor Distribution Shift**: KL divergence between sensor readings

**Example**:
```python
import numpy as np
from scipy.spatial.distance import jensenshannon

sim_depth = load_sim_depth_images()
real_depth = load_real_depth_images()

shift = jensenshannon(sim_depth.flatten(), real_depth.flatten())
print(f"Sensor distribution shift: {shift:.4f}")
```

---

### 5. Adaptive Control

**Concept**: Learn a dynamics model or residual online on the real robot.

**Approaches**:
- **Model-Based RL**: Learn dynamics, use MPC
- **Residual Learning**: Sim policy + learned correction
- **Online Fine-Tuning**: Update policy on real robot

**Example (Residual Learning)**:
```python
# Sim policy
action_sim = policy_sim(observation)

# Learn residual from real data
residual = residual_network(observation, action_sim)

# Final action
action_real = action_sim + residual
```

---

## Physics Engine Tuning

### Gazebo/MuJoCo Parameters

**Contact Parameters**:
```xml
<contact>
  <friction>
    <mu>1.0</mu>          <!-- Friction coefficient -->
    <mu2>1.0</mu2>        <!-- Secondary friction -->
  </friction>
  <contact_cfm>0.0001</contact_cfm>  <!-- Constraint force mixing -->
  <contact_erp>0.2</contact_erp>      <!-- Error reduction parameter -->
</contact>
```

**Solver Parameters**:
```xml
<physics>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <max_contacts>20</max_contacts>
</physics>
```

**Tuning Guidance**:
- Lower `max_step_size` for more accurate simulation (but slower)
- Increase `max_contacts` for complex contact scenarios
- Adjust `contact_erp` to match real contact stiffness

---

## Sensor Modeling

### Realistic Camera Simulation

**Add Noise and Artifacts**:
```python
import cv2
import numpy as np

def add_camera_noise(image):
    # Gaussian noise
    noise = np.random.normal(0, 5, image.shape)
    image = np.clip(image + noise, 0, 255)

    # Motion blur
    kernel = np.ones((5,5)) / 25
    image = cv2.filter2D(image, -1, kernel)

    # Lens distortion (apply calibrated model)
    image = cv2.undistort(image, K, dist_coeffs)

    return image.astype(np.uint8)
```

### Realistic Depth Camera
- Add depth noise: $z' = z + \mathcal{N}(0, 0.001 \cdot z^2)$
- Simulate invalid pixels (reflective surfaces, too close/far)
- Model structured light patterns (for RealSense)

---

## Case Studies

### Case 1: Grasping Transfer (RT-1)

**Approach**:
- Domain randomization (lighting, textures, object poses)
- Fine-tune on 100 real-world demos
- Success: 97% sim, 92% real (5% gap)

**Key Insight**: Heavy randomization in sim reduces need for real data.

---

### Case 2: Quadruped Locomotion (ANYmal)

**Approach**:
- System identification: Measured real robot inertia, friction
- Adaptive control: Online adaptation to terrain
- Success: Robust walking on rocks, stairs, ice

**Key Insight**: Accurate dynamics + adaptation beats heavy randomization alone.

---

### Case 3: Manipulation with Isaac Sim

**Approach**:
- Sim2Sim (Isaac Sim + Gazebo)
- Physics tuning (contact parameters matched to real UR5)
- Success: 85% sim, 78% real (7% gap)

**Key Insight**: Multiple simulators expose different failure modes.

---

## Validation Protocol

### Step-by-Step Validation

1. **Sim-Only**: Test policy extensively in simulation
2. **Sim2Sim**: Transfer across simulators
3. **Real (Safe)**: Test on real robot with safety constraints
4. **Real (Full)**: Deploy without constraints
5. **Iterate**: If failures occur, analyze, update sim, retrain

### Metrics to Track
- Success rate (sim vs. real)
- Trajectory deviation
- Control effort
- Safety violations

---

## Common Pitfalls

### Pitfall 1: Over-Optimizing for Sim
**Problem**: Policy exploits sim artifacts (e.g., penetration, teleportation)
**Solution**: Add domain randomization, use multiple simulators

### Pitfall 2: Ignoring Sensor Latency
**Problem**: Sim assumes zero latency, real robot has 50-100ms
**Solution**: Add simulated latency in observation pipeline

### Pitfall 3: Perfect State Estimation
**Problem**: Sim provides ground truth, real robot has noisy estimates
**Solution**: Add state estimation noise in sim

---

## Tools and Frameworks

### Domain Randomization
- **Isaac Gym**: Built-in DR for RL training
- **gibson2**: Photorealistic visual DR
- **robosuite**: Modular DR for manipulation

### System Identification
- **robot_calibration** (ROS 2)
- **drake**: Model-based identification
- **PyBullet**: Parameter tuning API

### Sim2Real Frameworks
- **Habitat**: Embodied AI research
- **AI2-THOR**: Interactive environments
- **NVIDIA Isaac**: End-to-end pipeline

---

## Research Frontiers

### Active Learning
- Deploy policy, collect failures, retrain in sim with those scenarios

### Causal Models
- Learn causal relationships rather than correlations
- More robust to distribution shift

### Meta-Learning
- Learn to adapt quickly to new environments
- Few-shot fine-tuning on real robot

---

**See Also**:
- Chapter 2.4: Sensor Simulation and VSLAM
- Chapter 3.4: Isaac Gym and RL
- Appendix E: Cloud vs. On-Premise GPU Deployment

**Key Papers**:
- Tobin et al. (2017). Domain Randomization.
- Peng et al. (2018). Sim-to-Real Transfer of Robotic Control with Dynamics Randomization.
- Muratore et al. (2022). Robot Learning with Crash Constraints.
