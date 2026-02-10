---
sidebar_position: 5
title: Chapter 3.4 - Isaac Gym and RL (Optional)
---

# Chapter 3.4: Isaac Gym and Reinforcement Learning

**Module**: 3 - The AI-Robot Brain
**Optional Advanced Topic**
**Estimated Reading Time**: 45 minutes

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. Understand massively parallel RL training with Isaac Gym
2. Set up Isaac Gym environments for custom tasks
3. Train RL policies using PPO, SAC, or other algorithms
4. Transfer learned policies to Isaac Sim and real robots
5. Evaluate policy performance and robustness

---

## Prerequisites

- Completed Chapters 3.1, 3.2, and 3.3
- Understanding of reinforcement learning concepts (MDP, policy, value function)
- Familiarity with PyTorch

---

## Introduction

**Isaac Gym** revolutionizes reinforcement learning by running **thousands of parallel simulations** on a single GPU. Traditional RL training runs 1-8 environments on CPUs, taking days or weeks. Isaac Gym achieves 10,000+ environments simultaneously, reducing training time from **weeks to hours**.

**Key Breakthrough**:
- **Traditional RL**: 8 CPU cores × 30 FPS = 240 samples/sec → 7 days for 150M samples
- **Isaac Gym**: 4096 GPU envs × 60 FPS = 245,760 samples/sec → **7 hours** for 150M samples

**Use Cases**:
- Humanoid locomotion (ANYmal, Cassie, Digit)
- Robotic manipulation (pick-and-place, assembly)
- Drone control and navigation
- Multi-agent coordination

---

## Key Terms

:::info Glossary Terms
- **Isaac Gym**: GPU-accelerated physics simulation for RL training
- **PPO**: Proximal Policy Optimization algorithm (on-policy)
- **SAC**: Soft Actor-Critic algorithm (off-policy)
- **Sim-to-Real Transfer**: Transferring policies from simulation to hardware
- **Domain Randomization**: Varying environment parameters for robustness
- **Reward Shaping**: Designing reward functions to guide learning
- **Tensorboard**: Visualization tool for training metrics
:::

---

## Core Concepts

### 1. Massively Parallel Simulation

#### Architecture

**Isaac Gym** uses NVIDIA PhysX 5 for GPU-accelerated physics.

```
GPU Memory
├── Physics (PhysX 5)
│   ├── 4096 environments (parallel)
│   ├── Rigid body dynamics
│   └── Collision detection
├── RL Policy (PyTorch)
│   ├── Actor network
│   └── Critic network
└── Observation Buffers
    ├── Joint states (4096 × 18 DOF)
    └── Contact forces
```

**Performance**:
- **4096 environments** @ 60 Hz = 245,760 steps/sec
- **Memory**: 8-12 GB VRAM for 4096 envs
- **Training Time**: 150M samples in 7 hours (vs 7 days on CPU)

#### Installation

```bash
# Download Isaac Gym (requires NVIDIA Developer account)
# https://developer.nvidia.com/isaac-gym

# Extract
cd ~/Downloads
tar -xvf IsaacGym_Preview_4_Package.tar.gz
cd isaacgym

# Install Python package
cd python
pip install -e .

# Verify installation
cd examples
python joint_monkey.py
```

**Expected Output**:
```
Isaac Gym Environment loaded
Num environments: 64
Num actors per environment: 1
Simulation step rate: 60 Hz
```

### 2. RL Training Pipeline

#### Environment Setup

**Example: ANYmal Quadruped Locomotion**

```python
from isaacgym import gymapi
from isaacgym import gymtorch
import torch

class ANYmalEnv:
    def __init__(self, num_envs=4096):
        self.num_envs = num_envs
        self.num_obs = 48  # 12 joint pos + 12 joint vel + 12 contact + 12 orientation
        self.num_actions = 12  # 4 legs × 3 joints

        # Create Isaac Gym instance
        self.gym = gymapi.acquire_gym()

        # Simulation parameters
        sim_params = gymapi.SimParams()
        sim_params.dt = 1.0 / 60.0  # 60 Hz
        sim_params.substeps = 2
        sim_params.physx.solver_type = 1  # TGS (faster)
        sim_params.physx.num_position_iterations = 4
        sim_params.physx.num_velocity_iterations = 0
        sim_params.physx.contact_offset = 0.02
        sim_params.physx.rest_offset = 0.001
        sim_params.use_gpu_pipeline = True

        # Create sim
        self.sim = self.gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

        # Load robot asset
        asset_root = "assets"
        asset_file = "anymal_c/urdf/anymal_c.urdf"
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = False
        self.anymal_asset = self.gym.load_asset(self.sim, asset_root, asset_file, asset_options)

        # Create environments
        self.envs = []
        spacing = 2.0
        lower = gymapi.Vec3(-spacing, -spacing, 0.0)
        upper = gymapi.Vec3(spacing, spacing, spacing)

        for i in range(num_envs):
            env = self.gym.create_env(self.sim, lower, upper, int(num_envs**0.5))
            pose = gymapi.Transform()
            pose.p = gymapi.Vec3(0, 0, 0.6)  # Start 60cm above ground

            actor_handle = self.gym.create_actor(env, self.anymal_asset, pose, "anymal", i, 1)
            self.envs.append(env)

        # Prepare simulation tensors
        self.gym.prepare_sim(self.sim)

        # Get state tensors (GPU)
        self.root_states = gymtorch.wrap_tensor(self.gym.acquire_actor_root_state_tensor(self.sim))
        self.dof_states = gymtorch.wrap_tensor(self.gym.acquire_dof_state_tensor(self.sim))

        print(f"Created {num_envs} environments")

    def reset(self):
        """Reset all environments"""
        # Reset robot pose
        self.root_states[:, 0:3] = torch.tensor([0, 0, 0.6])  # Position
        self.root_states[:, 3:7] = torch.tensor([0, 0, 0, 1])  # Quaternion (identity)
        self.root_states[:, 7:13] = 0  # Linear/angular velocities

        # Reset joint positions to default
        self.dof_states[:, 0] = 0  # Joint positions
        self.dof_states[:, 1] = 0  # Joint velocities

        # Apply resets
        self.gym.set_actor_root_state_tensor(self.sim, gymtorch.unwrap_tensor(self.root_states))
        self.gym.set_dof_state_tensor(self.sim, gymtorch.unwrap_tensor(self.dof_states))

        return self.get_observations()

    def step(self, actions):
        """Step simulation with actions"""
        # Apply actions (target joint positions)
        self.gym.set_dof_position_target_tensor(self.sim, gymtorch.unwrap_tensor(actions))

        # Step physics
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)

        # Get new observations
        obs = self.get_observations()
        rewards = self.compute_rewards()
        dones = self.check_termination()

        return obs, rewards, dones

    def get_observations(self):
        """Get observations for all environments"""
        # Joint positions and velocities
        joint_pos = self.dof_states[:, 0].view(self.num_envs, 12)
        joint_vel = self.dof_states[:, 1].view(self.num_envs, 12)

        # Base orientation (roll, pitch)
        base_quat = self.root_states[:, 3:7]

        # Contact forces (feet touching ground)
        contact_forces = self.get_contact_forces()

        # Concatenate all observations
        obs = torch.cat([joint_pos, joint_vel, base_quat[:, :2], contact_forces], dim=-1)
        return obs

    def compute_rewards(self):
        """Compute rewards for all environments"""
        # Forward velocity reward
        forward_vel = self.root_states[:, 7]  # x-velocity
        forward_reward = torch.clamp(forward_vel, 0, 2.0)

        # Orientation penalty (penalize tilting)
        base_quat = self.root_states[:, 3:7]
        up_vec = torch.tensor([0, 0, 1]).to(base_quat.device)
        projected_gravity = quat_rotate(base_quat, up_vec)
        orientation_penalty = -torch.abs(projected_gravity[:, 0]) - torch.abs(projected_gravity[:, 1])

        # Energy penalty (penalize large joint torques)
        torques = self.gym.acquire_dof_force_tensor(self.sim)
        energy_penalty = -0.01 * torch.sum(torques**2, dim=-1)

        # Total reward
        reward = forward_reward + 0.5 * orientation_penalty + energy_penalty
        return reward

    def check_termination(self):
        """Check if episodes should terminate"""
        # Terminate if robot falls (base too low)
        base_height = self.root_states[:, 2]
        fallen = base_height < 0.3

        return fallen
```

#### Training Loop with PPO

**PPO Implementation**:
```python
import torch
import torch.nn as nn
from torch.optim import Adam

class ActorCritic(nn.Module):
    def __init__(self, num_obs=48, num_actions=12):
        super().__init__()
        self.actor = nn.Sequential(
            nn.Linear(num_obs, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, num_actions)
        )
        self.critic = nn.Sequential(
            nn.Linear(num_obs, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 1)
        )

    def forward(self, obs):
        return self.actor(obs), self.critic(obs)

class PPOTrainer:
    def __init__(self, env, policy, lr=3e-4, gamma=0.99, epsilon=0.2):
        self.env = env
        self.policy = policy.cuda()
        self.optimizer = Adam(policy.parameters(), lr=lr)
        self.gamma = gamma
        self.epsilon = epsilon

    def train(self, num_iterations=10000, steps_per_iteration=24):
        obs = self.env.reset()

        for iteration in range(num_iterations):
            # Collect rollouts
            obs_buf = []
            action_buf = []
            reward_buf = []
            value_buf = []
            logprob_buf = []

            for step in range(steps_per_iteration):
                with torch.no_grad():
                    actions, values = self.policy(obs)
                    action_logprobs = compute_logprobs(actions)

                obs_buf.append(obs)
                action_buf.append(actions)
                value_buf.append(values)
                logprob_buf.append(action_logprobs)

                # Step environment
                obs, rewards, dones = self.env.step(actions)
                reward_buf.append(rewards)

                # Reset terminated environments
                if dones.any():
                    obs[dones] = self.env.reset()[dones]

            # Compute advantages using GAE
            advantages = self.compute_gae(reward_buf, value_buf)

            # PPO update
            loss = self.ppo_update(obs_buf, action_buf, logprob_buf, advantages)

            # Logging
            if iteration % 100 == 0:
                avg_reward = torch.mean(torch.stack(reward_buf)).item()
                print(f"Iteration {iteration}: Avg Reward = {avg_reward:.2f}, Loss = {loss:.4f}")

    def compute_gae(self, rewards, values, lambda_=0.95):
        """Generalized Advantage Estimation"""
        advantages = []
        gae = 0

        for t in reversed(range(len(rewards))):
            delta = rewards[t] + self.gamma * values[t+1] - values[t]
            gae = delta + self.gamma * lambda_ * gae
            advantages.insert(0, gae)

        return torch.stack(advantages)

    def ppo_update(self, obs, actions, old_logprobs, advantages):
        """PPO clipped objective update"""
        # Flatten buffers
        obs = torch.cat(obs)
        actions = torch.cat(actions)
        old_logprobs = torch.cat(old_logprobs)
        advantages = advantages.flatten()

        # Normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        # Compute policy ratio
        new_actions, values = self.policy(obs)
        new_logprobs = compute_logprobs(new_actions)
        ratio = torch.exp(new_logprobs - old_logprobs)

        # Clipped surrogate objective
        surr1 = ratio * advantages
        surr2 = torch.clamp(ratio, 1 - self.epsilon, 1 + self.epsilon) * advantages
        policy_loss = -torch.min(surr1, surr2).mean()

        # Value loss
        value_loss = nn.MSELoss()(values.squeeze(), advantages)

        # Total loss
        loss = policy_loss + 0.5 * value_loss

        # Backprop
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        return loss.item()

# Train
env = ANYmalEnv(num_envs=4096)
policy = ActorCritic(num_obs=48, num_actions=12)
trainer = PPOTrainer(env, policy)
trainer.train(num_iterations=10000)
```

**Training Performance**:
- **Iteration 0**: Avg Reward = -2.5 (random policy)
- **Iteration 1000**: Avg Reward = 5.2 (walking emerges)
- **Iteration 5000**: Avg Reward = 12.8 (robust locomotion)
- **Iteration 10000**: Avg Reward = 18.5 (near-optimal)
- **Total Time**: 6-8 hours on RTX 3090

### 3. Sim-to-Real Transfer

**Challenge**: Policies trained in perfect simulation often fail on real hardware due to:
1. **Unmodeled dynamics**: Friction, backlash, motor delays
2. **Sensor noise**: IMU drift, encoder quantization
3. **Latency**: 10-20ms control loop delay

#### Domain Randomization

**Randomize physics parameters during training**:

```python
class RandomizedANYmalEnv(ANYmalEnv):
    def reset(self):
        # Randomize mass
        base_mass = 30.0  # kg (nominal)
        mass_noise = torch.rand(self.num_envs) * 5.0 - 2.5  # ±2.5 kg
        self.set_actor_mass(base_mass + mass_noise)

        # Randomize friction
        friction = torch.rand(self.num_envs) * 0.5 + 0.8  # [0.8, 1.3]
        self.set_friction(friction)

        # Randomize motor strength
        motor_strength = torch.rand(self.num_envs) * 0.2 + 0.9  # [0.9, 1.1]
        self.motor_gains *= motor_strength

        # Add observation noise
        self.obs_noise_scale = 0.05

        return super().reset()

    def get_observations(self):
        obs = super().get_observations()
        # Add Gaussian noise to observations
        noise = torch.randn_like(obs) * self.obs_noise_scale
        return obs + noise
```

**Randomization Ranges** (based on ANYmal C):
| Parameter | Nominal | Randomization Range |
|-----------|---------|---------------------|
| Base mass | 30 kg | ±10% (27-33 kg) |
| Link mass | Varies | ±20% |
| Friction | 1.0 | [0.5, 1.5] |
| Motor gains | 1.0 | ±10% |
| Joint damping | 0.5 | [0.2, 0.8] |
| Control latency | 0ms | [0, 20ms] |

#### System Identification

**Measure real robot parameters**:
```python
# Real robot test
def calibrate_robot():
    # 1. Mass estimation (lift robot, measure actuator torques)
    measured_mass = estimate_mass()

    # 2. Friction estimation (move joints slowly, measure torques)
    measured_friction = estimate_friction()

    # 3. Motor delay (step input, measure response time)
    measured_delay = estimate_motor_delay()

    return {
        'mass': measured_mass,
        'friction': measured_friction,
        'delay': measured_delay
    }

# Narrow randomization around measured values
real_params = calibrate_robot()
# Use real_params as mean, smaller std dev
```

### 4. Policy Deployment

#### Export Policy for Real Robot

```python
# 1. Export trained policy to ONNX
dummy_input = torch.randn(1, 48).cuda()
torch.onnx.export(policy.actor, dummy_input, "anymal_policy.onnx")

# 2. Convert to TensorRT for fast inference
import tensorrt as trt
trtexec --onnx=anymal_policy.onnx --saveEngine=anymal_policy.engine --fp16

# 3. ROS 2 node for deployment
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tensorrt as trt

class PolicyNode(Node):
    def __init__(self):
        super().__init__('anymal_policy_node')
        self.engine = self.load_tensorrt_engine('anymal_policy.engine')

        self.sub = self.create_subscription(JointState, '/joint_states', self.callback, 10)
        self.pub = self.create_publisher(JointState, '/joint_commands', 10)

    def callback(self, msg):
        # Get observations from robot
        obs = self.extract_observations(msg)

        # Run policy
        actions = self.engine.infer(obs)

        # Publish commands
        cmd = JointState()
        cmd.position = actions.tolist()
        self.pub.publish(cmd)
```

**Deployment Checklist**:
1. Test in Isaac Sim first (digital twin)
2. Verify policy runs at 50+ Hz
3. Add safety limits (joint, velocity, torque)
4. Implement emergency stop
5. Start with low motor gains, gradually increase

---

## Practical Example: Quadruped Locomotion

**Task**: Train ANYmal to walk forward at 1.5 m/s.

### Step 1: Define Reward Function

```python
def compute_rewards(self):
    # 1. Forward velocity (primary objective)
    target_vel = 1.5  # m/s
    actual_vel = self.root_states[:, 7]
    vel_reward = -torch.abs(actual_vel - target_vel)

    # 2. Orientation penalty (stay upright)
    orientation_reward = -torch.sum(self.root_states[:, 3:5]**2, dim=-1)

    # 3. Joint acceleration penalty (smooth motion)
    joint_acc = (self.dof_states[:, 1] - self.prev_dof_vel) / self.dt
    acc_penalty = -0.01 * torch.sum(joint_acc**2, dim=-1)

    # 4. Foot contact reward (minimize air time)
    contact_reward = torch.sum(self.contact_forces > 1.0, dim=-1).float()

    # 5. Energy penalty
    torques = self.dof_forces
    energy_penalty = -0.001 * torch.sum(torques**2, dim=-1)

    return vel_reward + 0.5 * orientation_reward + acc_penalty + 0.1 * contact_reward + energy_penalty
```

### Step 2: Train

```bash
python train.py --task=ANYmal --num_envs=4096 --max_iterations=10000
```

**Expected Training Curve**:
```
Iteration 0:    Reward = -3.2 (random flailing)
Iteration 500:  Reward = 2.1 (standing stable)
Iteration 1500: Reward = 8.5 (slow walking)
Iteration 3000: Reward = 14.2 (target speed reached)
Iteration 10000: Reward = 18.9 (robust locomotion)
```

### Step 3: Evaluate

```python
# Evaluate in test environments
def evaluate_policy(policy, num_episodes=100):
    env = ANYmalEnv(num_envs=100)
    obs = env.reset()

    total_rewards = []
    success_count = 0

    for episode in range(num_episodes):
        episode_reward = 0
        for step in range(1000):  # 1000 steps = 16.7 seconds
            actions = policy(obs)
            obs, rewards, dones = env.step(actions)
            episode_reward += rewards.mean()

            if dones.any():
                break

        # Success = walked 10+ meters without falling
        distance_traveled = env.root_states[:, 0].mean()
        if distance_traveled > 10.0:
            success_count += 1

        total_rewards.append(episode_reward)

    print(f"Success Rate: {success_count}/{num_episodes} = {100*success_count/num_episodes:.1f}%")
    print(f"Avg Reward: {torch.mean(torch.tensor(total_rewards)):.2f}")

evaluate_policy(trained_policy)
# Output: Success Rate: 94/100 = 94.0%, Avg Reward: 17.8
```

---

## Advanced Topics

### Multi-Skill Learning

**Train multiple gaits** (walk, trot, gallop):

```python
class MultiGaitEnv(ANYmalEnv):
    def reset(self):
        # Randomly assign gait to each environment
        self.target_gait = torch.randint(0, 3, (self.num_envs,))
        # 0 = walk (0.5 m/s), 1 = trot (1.5 m/s), 2 = gallop (3.0 m/s)
        return super().reset()

    def compute_rewards(self):
        target_speeds = torch.tensor([0.5, 1.5, 3.0])[self.target_gait]
        vel_reward = -torch.abs(self.root_states[:, 7] - target_speeds)
        # ... other rewards
```

### Distillation for Real-Time Inference

**Teacher-Student**: Train large policy (512 hidden), distill to small (128 hidden) for 1ms inference.

```python
# Train student to mimic teacher
def distill(teacher, student, env, iterations=1000):
    optimizer = Adam(student.parameters(), lr=1e-3)

    for i in range(iterations):
        obs = env.reset()
        teacher_actions = teacher(obs)
        student_actions = student(obs)

        loss = nn.MSELoss()(student_actions, teacher_actions)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    print(f"Distillation complete. Inference time: {measure_inference_time(student):.2f}ms")
```

---

## Summary

This chapter covered **Isaac Gym for Reinforcement Learning**:

1. **Massively Parallel Training**: 4096 environments at 60 Hz = 245k samples/sec
2. **PPO Algorithm**: On-policy RL with clipped objective
3. **Domain Randomization**: Randomize physics for sim-to-real transfer
4. **Policy Deployment**: Export to ONNX/TensorRT for real robots

**Key Takeaways**:
- Isaac Gym reduces RL training from weeks to hours
- Domain randomization critical for real-world transfer
- 4096 environments fit in 8-12 GB VRAM
- Quadruped locomotion: 10k iterations in ~7 hours

**Next Module**: Vision-Language-Action models for robot learning from human demonstrations.

---

## End-of-Chapter Exercises

### Exercise 1: Train Custom Task (Difficulty: Advanced)

**Tasks**:
1. Create custom environment (e.g., box pushing, ball balancing)
2. Design reward function
3. Train PPO policy for 5000 iterations
4. Evaluate success rate (>80%)
5. Visualize learned behavior

**Success Criteria**: Policy achieves task objective in >80% of test episodes

### Exercise 2: Sim-to-Real Transfer (Difficulty: Expert)

**Tasks**:
1. Train policy with domain randomization
2. Test in Isaac Sim with realistic parameters
3. Deploy to real robot (or hardware-in-loop sim)
4. Measure performance degradation (<20%)
5. Fine-tune with real data

**Success Criteria**: Real robot performance within 80% of sim performance

---

## Further Reading

1. Isaac Gym Documentation: https://developer.nvidia.com/isaac-gym
2. Isaac Gym Paper (Makoviychuk et al., 2021): https://arxiv.org/abs/2108.10470
3. Proximal Policy Optimization (Schulman et al., 2017): https://arxiv.org/abs/1707.06347
4. Learning Dexterous In-Hand Manipulation (OpenAI, 2019): https://arxiv.org/abs/1808.00177
5. Domain Randomization (Tobin et al., 2017): https://arxiv.org/abs/1703.06907

---

## Next Module

Continue to **[Module 4: Vision-Language-Action](../module4/intro)**

**Module 3 Complete!** You can now train and deploy RL policies at scale with GPU acceleration.
