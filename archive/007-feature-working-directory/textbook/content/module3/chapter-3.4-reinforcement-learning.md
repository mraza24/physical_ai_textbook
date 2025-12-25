# Chapter 3.4: Reinforcement Learning with Isaac Gym & Lab

> **Module**: 3 - AI-Robot Brain (NVIDIA Isaac)
> **Week**: 10
> **Estimated Reading Time**: 26 minutes

---

## Summary

Isaac Gym provides GPU-accelerated physics simulation with 1,000-10,000 parallel environments, enabling reinforcement learning (RL) training in hours instead of weeks. This chapter covers RL fundamentals (PPO algorithm), Isaac Gym setup for quadruped locomotion, Isaac Lab workflows, and sim-to-real transfer strategies.

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Explain** reinforcement learning fundamentals (reward functions, policy networks, PPO algorithm)
2. **Configure** Isaac Gym with 1,000+ parallel environments for robot training
3. **Train** quadruped locomotion policies using Isaac Lab RL workflows
4. **Implement** domain randomization for robust sim-to-real transfer
5. **Deploy** trained policies to real robots (Unitree Go1, Boston Dynamics Spot)

**Prerequisite Knowledge**: Chapter 3.1 (Isaac Sim), Chapter 3.2 (Perception), Python, basic neural networks

---

## Key Terms

- **Reinforcement Learning (RL)**: Agent learns optimal actions through trial-and-error with reward signals
- **Policy**: Neural network mapping observations (robot state) to actions (joint torques)
- **Reward Function**: Scalar signal indicating task success (e.g., +1 for forward progress, -1 for falling)
- **PPO**: Proximal Policy Optimization algorithm (stable, sample-efficient RL)
- **Parallel Environments**: Multiple simulation instances running simultaneously (GPU)
- **Domain Randomization**: Varying physics parameters (mass, friction) to improve sim-to-real transfer
- **Isaac Gym**: GPU-accelerated simulator for parallel RL training (deprecated → Isaac Lab)
- **Isaac Lab**: Next-generation RL framework built on Isaac Sim (2024+)

---

## Core Concepts

### 1. Reinforcement Learning Fundamentals

**RL Problem Setup**:
```
Agent (robot) observes state s_t
       ↓
Policy π(a_t | s_t) selects action a_t
       ↓
Environment executes action, returns reward r_t
       ↓
Agent observes new state s_{t+1}
       ↓
Goal: Maximize cumulative reward Σ r_t
```

**Example: Quadruped Walking**
- **State** s_t: Joint angles (12D), velocities (12D), orientation (4D) = 28D vector
- **Action** a_t: Target joint positions (12D)
- **Reward** r_t: +forward_velocity -0.5×energy_cost -10×(if fallen)

**Neural Network Policy**:
```
Observations (28D) → MLP (256→128→64) → Actions (12D)
                    ↑
              PPO Training
```

**Why GPU Parallelism Matters**:
- **1 CPU environment**: 10M steps = 10 days
- **1000 GPU environments**: 10M steps = 3.6 hours (66× speedup)

---

### 2. PPO Algorithm (Proximal Policy Optimization)

**Why PPO?**
- ✅ **Stable**: Avoids catastrophic policy collapse (unlike TRPO)
- ✅ **Sample-efficient**: Reuses past experiences (unlike A3C)
- ✅ **Scalable**: Works with 1000+ parallel envs

**PPO Update Rule**:
```
Collect rollout: (s_t, a_t, r_t) for N steps
Compute advantage: A_t = r_t + γ V(s_{t+1}) - V(s_t)
Update policy: maximize L(θ) = min(ratio × A_t, clip(ratio, 1±ε) × A_t)
where ratio = π_new(a_t|s_t) / π_old(a_t|s_t)
```

**Key Hyperparameters**:
- `clip_range`: 0.2 (limit policy change per update)
- `learning_rate`: 3e-4 (typical)
- `num_envs`: 4096 (more = faster, but diminishing returns)
- `num_steps`: 24 (rollout length)

---

### 3. Isaac Gym Architecture

**Legacy System (Deprecated 2024)**:
```
┌──────────────────────────────────────┐
│         Isaac Gym (PhysX GPU)        │
├──────────────────────────────────────┤
│ 4096 parallel environments           │
│ Each env: Ant robot (8 joints)       │
│ Physics step: 60 Hz                  │
│ Total: 245,760 bodies/sec            │
├──────────────────────────────────────┤
│ Training: PPO via rl_games           │
│ Backend: PyTorch                     │
└──────────────────────────────────────┘
```

**Isaac Lab (2024+, Recommended)**:
```
┌──────────────────────────────────────┐
│    Isaac Sim 4.0+ (Omniverse Core)   │
├──────────────────────────────────────┤
│ Isaac Lab (Python API)               │
│  - Task definitions (quadruped_env)  │
│  - Domain randomization              │
│  - PPO/SAC/TD3 trainers              │
├──────────────────────────────────────┤
│ 1000-8192 parallel environments      │
│ Realistic rendering (sim-to-real)    │
└──────────────────────────────────────┘
```

**Migration**: Isaac Gym → Isaac Lab (API largely compatible)

---

### 4. Isaac Lab Workflow: Quadruped Locomotion

**Task**: Train Unitree Go1 robot to walk forward at 1 m/s

**Step 1: Install Isaac Lab**

```bash
# Prerequisites: Isaac Sim 4.5.0+
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
./isaaclab.sh --install  # Install dependencies

# Verify
python source/standalone/workflows/rl_games/train.py --task Isaac-Velocity-Flat-Anymal-C-v0 --headless
```

**Step 2: Define Task Environment**

```python
# quadruped_env.py
import isaaclab.envs.mdp as mdp
from isaaclab.envs import ManagerBasedRLEnv, ManagerBasedRLEnvCfg

class QuadrupedEnvCfg(ManagerBasedRLEnvCfg):
    def __post_init__(self):
        self.scene.robot = ArticulationCfg(
            prim_path="/World/Robot",
            spawn=UsdFileCfg(usd_path="go1/go1.usd"),  # Unitree Go1
        )

        # Observation space (44D)
        self.observations.policy = ObservationGroupCfg(
            base_lin_vel=mdp.base_lin_vel,       # 3D
            base_ang_vel=mdp.base_ang_vel,       # 3D
            projected_gravity=mdp.projected_gravity,  # 3D
            joint_pos=mdp.joint_pos,             # 12D
            joint_vel=mdp.joint_vel,             # 12D
            actions=mdp.last_action,             # 12D (previous action)
        )

        # Action space (12D): Target joint positions
        self.actions.joint_pos = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[".*"], scale=0.5
        )

        # Reward function (weighted sum)
        self.rewards = {
            "lin_vel_xy": mdp.lin_vel_reward(weight=1.0, target_vel=1.0),  # Forward speed
            "ang_vel_z": mdp.ang_vel_reward(weight=-0.05),  # Penalize spinning
            "action_rate": mdp.action_rate_l2(weight=-0.01),  # Smooth actions
            "dof_acc": mdp.joint_acc_l2(weight=-2.5e-7),  # Smooth joints
            "orientation": mdp.flat_orientation(weight=-5.0),  # Stay upright
        }

        # Termination: Fall down (z < 0.3m)
        self.terminations.base_contact = mdp.illegal_contact(
            sensor_cfg=ContactSensorCfg(prim_path="/World/Robot/base"),
            threshold=1.0
        )

# Register task
gym.register("Isaac-Velocity-Unitree-Go1-v0", entry_point=QuadrupedEnvCfg)
```

**Step 3: Train with PPO**

```bash
# Train 10M steps with 4096 parallel envs (RTX 4090: 2 hours)
python scripts/rl_games/train.py \
  --task Isaac-Velocity-Unitree-Go1-v0 \
  --num_envs 4096 \
  --headless \
  --max_iterations 10000

# Monitor training
tensorboard --logdir logs/rl_games
```

**Step 4: Evaluate Policy**

```bash
# Load trained checkpoint, visualize
python scripts/rl_games/play.py \
  --task Isaac-Velocity-Unitree-Go1-v0 \
  --num_envs 16 \
  --checkpoint runs/Velocity_Go1/nn/Velocity_Go1.pth
```

**Expected Results**:
- **Episode reward**: 300-400 (after 5M steps)
- **Forward velocity**: 0.9-1.1 m/s
- **Training time**: 2 hours (RTX 4090), 6 hours (RTX 3080)

---

### 5. Domain Randomization for Sim-to-Real

**Problem**: Sim-trained policy fails on real robot due to:
- Physics mismatch (friction, contact dynamics)
- Sensor noise (IMU drift, joint encoder errors)
- Actuator delays (10-20ms latency)

**Solution**: Randomize simulation parameters during training

**Randomization Strategy**:

```python
# domain_rand_cfg.py
class DomainRandCfg:
    # Physics randomization (per episode reset)
    robot_mass = (8.0, 14.0)  # kg (Go1 nominal: 11 kg)
    friction_coef = (0.3, 1.5)  # Ground friction
    restitution = (0.0, 0.4)  # Bounciness

    # Actuator randomization
    motor_strength = (0.8, 1.2)  # Torque multiplier
    motor_damping = (0.5, 2.0)  # Joint damping

    # Sensor noise
    imu_noise = 0.05  # m/s² (Gaussian)
    joint_pos_noise = 0.01  # rad

    # Environmental randomization
    terrain_roughness = (0.0, 0.05)  # m (height variation)
    push_force = (0.0, 100.0)  # N (random external force)
```

**Implementation**:

```python
# Apply randomization at episode reset
def reset(self):
    # Randomize mass
    mass = np.random.uniform(8.0, 14.0)
    self.robot.set_mass(mass)

    # Randomize friction
    friction = np.random.uniform(0.3, 1.5)
    self.ground.set_friction(friction)

    # Add IMU noise to observations
    imu_reading = true_imu + np.random.normal(0, 0.05, size=3)

    return obs
```

**Ablation Study**:
| Configuration | Sim Success | Real Success |
|---------------|-------------|--------------|
| No randomization | 95% | 20% |
| Physics only | 95% | 60% |
| Physics + Sensors | 92% | 75% |
| **Full randomization** | **88%** | **85%** |

**Trade-off**: Slight sim performance drop (95% → 88%), but 4× better real-world transfer

---

### 6. Deployment to Real Robot

**Hardware**: Unitree Go1 ($2,700, 12 DOF)

**Deployment Workflow**:

```python
# 1. Export ONNX model (from PyTorch)
import torch.onnx

dummy_input = torch.randn(1, 44)  # Observation dim
torch.onnx.export(
    policy_model,
    dummy_input,
    "go1_policy.onnx",
    input_names=['obs'],
    output_names=['actions']
)

# 2. Convert to TensorRT (for Jetson Orin NX onboard)
trtexec --onnx=go1_policy.onnx --saveEngine=go1_policy.engine --fp16

# 3. Load on robot controller
import tensorrt as trt
engine = trt.Runtime().deserialize_cuda_engine(open("go1_policy.engine", "rb").read())

# 4. Inference loop (50 Hz control)
while True:
    obs = read_robot_state()  # IMU, joint positions/velocities
    actions = engine.infer(obs)  # Neural network forward pass (2 ms)
    send_joint_commands(actions)  # PD controller
    time.sleep(0.02)  # 50 Hz
```

**Control Frequency**:
- **Policy inference**: 2 ms (50 Hz)
- **Low-level PD control**: 500 Hz (motor controller)
- **Physics simulation during training**: 60 Hz

**Latency Budget**:
```
Read sensors: 1 ms
Policy inference: 2 ms
Send commands: 1 ms
Total: 4 ms (250 Hz max possible)
```

---

### 7. Advanced: Hierarchical RL

**Problem**: Single policy struggles with complex tasks (navigation + manipulation)

**Solution**: Hierarchical policies

```
High-level policy (1 Hz): "Walk to table" → waypoint (x, y)
       ↓
Low-level policy (50 Hz): Given waypoint → joint torques
```

**Example: Mobile Manipulation**

```python
class HierarchicalPolicy:
    def __init__(self):
        self.high_level = load_policy("navigation_policy.pth")  # MLP 128D
        self.low_level = load_policy("locomotion_policy.pth")  # MLP 256D

    def step(self, obs):
        # High-level: Decide subgoal every 20 steps (1 Hz)
        if self.step_count % 20 == 0:
            subgoal = self.high_level(obs['lidar'], obs['goal'])

        # Low-level: Execute locomotion to reach subgoal
        obs_low = {**obs['proprioception'], 'subgoal': subgoal}
        actions = self.low_level(obs_low)

        return actions
```

**Training**: Train low-level first (10M steps), then train high-level (5M steps) with frozen low-level

---

## Practical Example: Quadruped Stair Climbing

### Overview

Train Unitree Go1 to climb stairs (15 cm step height) using domain randomization.

### Implementation

**Step 1: Create Stair Environment**

```python
# stairs_env.py
class StairsEnvCfg(QuadrupedEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Add stairs terrain
        self.scene.terrain = TerrainImporterCfg(
            prim_path="/World/Terrain",
            terrain_type="stairs",
            stairs_height=0.15,  # 15 cm
            stairs_width=0.30,   # 30 cm
        )

        # Harder reward: penalize foot slipping
        self.rewards.foot_slip = mdp.foot_slip_penalty(weight=-0.1)

        # Curriculum: start with 5 cm, increase to 15 cm
        self.curriculum = {
            'stairs_height': [(0, 0.05), (3e6, 0.15)]  # steps → height
        }
```

**Step 2: Train with Curriculum**

```bash
python train.py --task Isaac-Stairs-Unitree-Go1-v0 --num_envs 4096
```

**Training Progress**:
- **0-2M steps**: 5 cm stairs, reward = 200 (baseline walking)
- **2-5M steps**: 10 cm stairs, reward = 150 (learns to lift feet)
- **5-10M steps**: 15 cm stairs, reward = 250 (successful climbing)

**Step 3: Domain Randomization**

```python
# Randomize stairs during training
stairs_height = np.random.uniform(0.10, 0.20)  # 10-20 cm
stairs_width = np.random.uniform(0.25, 0.35)   # 25-35 cm
surface_friction = np.random.uniform(0.5, 1.2)
```

**Step 4: Test on Real Robot**

```bash
# Deploy to Go1 via ROS 2
ros2 launch go1_control policy_control.launch.py policy:=stairs_policy.engine

# Send navigation command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"
```

### Expected Outcome

- **Sim success rate**: 85% (climbs 15 cm stairs)
- **Real success rate**: 70% (with domain randomization)
- **Failure modes**: Slips on smooth stairs (friction < 0.6), misjudges step height (vision needed)

### Troubleshooting

- **Issue**: Robot falls backward on stairs
  **Solution**: Increase `orientation` reward weight (-5.0 → -10.0)

- **Issue**: Foot slips on steps
  **Solution**: Add foot contact sensors, penalize slip velocity

- **Issue**: Sim-to-real gap (works in sim, fails on real)
  **Solution**: Increase friction randomization range (0.3-1.5 → 0.2-2.0)

---

## Exercises

### Exercise 1: Reward Function Design (Difficulty: Easy)

**Objective**: Modify reward function to optimize for energy efficiency

**Task**: Add power consumption penalty to quadruped walking task

**Requirements**:
- Compute power: P = Σ |τ_i × ω_i| (torque × velocity)
- Add reward term: `-0.001 × P`
- Compare: Original policy vs energy-efficient policy (forward speed vs power)

**Expected Outcome**: 15-20% power reduction, 5-10% speed reduction

**Estimated Time**: 30 minutes

---

### Exercise 2: Train Jumping Policy (Difficulty: Medium)

**Objective**: Train quadruped to jump over 20 cm obstacle

**Task**: Design reward function, train PPO policy

**Requirements**:
- Reward: +10 if robot clears obstacle, -1 if collision
- Train 5M steps with 2048 envs
- Evaluate: Success rate over 20 trials

**Expected Outcome**: 60-80% success rate

**Estimated Time**: 90 minutes (including training time)

---

### Exercise 3: Sim-to-Real Transfer (Difficulty: Hard)

**Objective**: Deploy trained policy to real quadruped robot

**Task**: Export policy, deploy to Unitree Go1, measure performance

**Requirements**:
- Convert PyTorch → ONNX → TensorRT
- Deploy to Jetson Orin NX (Go1 onboard computer)
- Test: Forward walking, turning, rough terrain
- Measure: Success rate, latency, power consumption

**Expected Outcome**:
- Latency: < 5 ms (200 Hz policy)
- Success: > 70% (if domain randomization used)
- Power: 80-120W (locomotion)

**Estimated Time**: 4 hours (hardware setup + testing)

---

## Summary & Key Takeaways

- **Isaac Gym/Lab** enables RL training with 1,000-10,000 parallel GPU environments (66× faster than CPU)
- **PPO algorithm** is stable, sample-efficient choice for robot learning (clip_range=0.2 key)
- **Domain randomization** (physics, sensors, environment) improves sim-to-real transfer 4× (20% → 85%)
- **Quadruped locomotion** achieves 1 m/s walking in 2 hours training (RTX 4090, 4096 envs)
- **Deployment** uses TensorRT for real-time inference (2 ms, 50 Hz policy) on Jetson Orin

**Connection to Module 4**: While RL learns low-level skills (walking, grasping), Vision-Language-Action models (Module 4) learn high-level reasoning ("go to the kitchen, pick up the cup")—VLAs often use RL policies as action primitives.

---

## Additional Resources

- Isaac Lab Documentation: https://isaac-sim.github.io/IsaacLab/
- RL Games (PPO implementation): https://github.com/Denys88/rl_games
- Learning to Walk in Minutes (paper): https://arxiv.org/abs/2109.11978

---

## Notes for Instructors

**Teaching Tips**:
- Demo: Show 1 env vs 4096 envs side-by-side (training speed difference)
- Visualization: Render all 4096 envs simultaneously (impressive for students)

**Lab Ideas**:
- Lab 1: Modify reward function, observe policy changes
- Lab 2: Train ant/humanoid locomotion
- Lab 3: Sim-to-real transfer (if hardware available)

**Assessment**:
- Quiz: Explain PPO clip_range purpose
- Project: Train custom task (ball pushing, box stacking)

**Common Student Struggles**:
- Reward function design (too sparse → no learning; too dense → reward hacking)
- Hyperparameter tuning (learning_rate, num_steps)
- Debugging sim-to-real gap

---

**Chapter Metadata**:
- **Word Count**: 3,200 words
- **Code Examples**: 8
- **Exercises**: 3
- **Glossary Terms**: 8
