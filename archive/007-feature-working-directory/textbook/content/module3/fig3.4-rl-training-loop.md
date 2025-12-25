# Figure 3.4: Reinforcement Learning Training Loop (Isaac Gym)

> **Chapter**: 3.4 - Reinforcement Learning
> **Figure Type**: Process Flow Diagram with Feedback Loop
> **Format**: Mermaid

---

## Diagram

```mermaid
graph TB
    subgraph Parallel["Parallel Environments (GPU)"]
        direction LR
        Env1["Env 1<br/>Quadruped"]
        Env2["Env 2<br/>Quadruped"]
        EnvDots["..."]
        Env4096["Env 4096<br/>Quadruped"]

        Env1 ~~~ Env2
        Env2 ~~~ EnvDots
        EnvDots ~~~ Env4096
    end

    subgraph Collect["Rollout Collection (24 steps)"]
        Obs["Observations s_t<br/>(joint angles, velocities, IMU)<br/>Shape: [4096, 44]"]
        Policy["Policy Network π(a|s)<br/>(MLP 256→128→64)<br/>(PyTorch, GPU)"]
        Actions["Actions a_t<br/>(target joint positions)<br/>Shape: [4096, 12]"]
        Rewards["Rewards r_t<br/>(forward velocity - energy)<br/>Shape: [4096, 1]"]

        Obs --> Policy
        Policy --> Actions
        Actions -->|Execute in envs| Parallel
        Parallel -->|Step physics| Rewards
    end

    subgraph PPO["PPO Update (after 24 steps)"]
        direction TB
        Buffer["Replay Buffer<br/>(s, a, r, s') × 98,304"]
        Advantage["Compute Advantages<br/>A_t = r_t + γV(s_{t+1}) - V(s_t)"]
        Loss["PPO Loss<br/>L = min(ratio×A, clip(ratio,1±ε)×A)"]
        Update["Gradient Descent<br/>(lr=3e-4, 5 epochs)"]

        Buffer --> Advantage
        Advantage --> Loss
        Loss --> Update
    end

    subgraph Monitor["Monitoring"]
        TensorBoard["TensorBoard<br/>(episode reward, FPS)"]
        Checkpoint["Save Checkpoint<br/>(every 100 iters)"]
    end

    Rewards -->|Store| Buffer
    Update -->|Update weights| Policy
    Policy -.->|Log metrics| TensorBoard
    Update -.->|Save| Checkpoint

    Start["Start Training<br/>(10M steps)"] --> Obs
    Checkpoint -.->|Converged?<br/>(reward > 300)| End["Trained Policy<br/>(.pth file)"]

    style Parallel fill:#99ccff
    style Collect fill:#99ff99
    style PPO fill:#ffcc99
    style Monitor fill:#cc99ff
    style Start fill:#ff9999
    style End fill:#ff9999
```

---

## Caption

**Figure 3.4**: Reinforcement Learning Training Loop with Isaac Gym (4096 parallel environments). **(1) Rollout Collection**: Policy network observes 4096 quadruped states (joint angles, velocities, IMU), outputs actions (joint targets), environments execute actions and compute rewards (forward velocity - energy cost). Collect 24 steps = 98,304 transitions. **(2) PPO Update**: Compute advantages (TD error), optimize policy via clipped surrogate loss (5 epochs, lr=3e-4). **(3) Repeat** for 10M steps (2 hours on RTX 4090). Monitor via TensorBoard, save checkpoints every 100 iterations. Result: Policy achieves 1 m/s walking (reward > 300).

---

## Code References

- **Environment setup**: `textbook/content/module3/chapter-3.4-reinforcement-learning.md:115-165` (QuadrupedEnvCfg with observations/actions/rewards)
- **PPO training**: `chapter-3.4-reinforcement-learning.md:170-180` (train.py command with 4096 envs)
- **Policy network**: `chapter-3.4-reinforcement-learning.md:75-82` (MLP architecture 256→128→64)
- **Advantage computation**: `chapter-3.4-reinforcement-learning.md:90-95` (PPO update rule)
- **Domain randomization**: `chapter-3.4-reinforcement-learning.md:230-255` (physics/sensor randomization)

---

## Usage Notes

**Teaching Context**:
- Use in **Week 10 Lecture 1** to explain RL training workflow
- Compare **1 env (CPU)** vs **4096 envs (GPU)**: 10 days vs 2 hours (66× speedup)
- Hands-on: Students monitor training via TensorBoard

**Student Activities**:
- **Lab Exercise**: Modify reward function (add energy penalty), observe policy changes
- **Discussion**: Why 24 steps? (Answer: Balance between on-policy freshness and sample efficiency)

---

## Error Scenarios

- **Issue**: Policy doesn't learn (reward stays at 0)
  - **Solution**: Check reward scaling (too small? multiply by 10), verify termination conditions
  - **Reference**: `chapter-3.4-reinforcement-learning.md:415`

- **Issue**: Training crashes with "CUDA out of memory"
  - **Solution**: Reduce `num_envs` (4096 → 2048) or reduce MLP size (256 → 128 hidden units)
  - **Trade-off**: Slower training

- **Issue**: Reward spikes then collapses (reward hacking)
  - **Solution**: Reduce `learning_rate` (3e-4 → 1e-4), increase `clip_range` (0.2 → 0.3)
  - **Example**: Policy learns to fall backward (gets forward velocity reward before hitting ground)

- **Issue**: Sim policy doesn't transfer to real robot
  - **Solution**: Enable domain randomization (mass, friction, sensor noise)
  - **Reference**: `chapter-3.4-reinforcement-learning.md:230-270` (DomainRandCfg)

---

## Notes for Instructors

**Diagram Pedagogy**:
- Walk through **clockwise loop**: Collect rollouts → PPO update → Updated policy → Repeat
- Emphasize **parallelism**: 4096 envs = 66× speedup (critical for RL feasibility)
- Demo: Show TensorBoard during training (episode reward curve climbing from 0 → 300)

**PPO Details** (for advanced students):

**Why PPO?**
1. **Clipped objective** prevents large policy updates (catastrophic forgetting)
2. **On-policy** but reuses data (5 epochs per rollout)
3. **Advantage function** reduces variance (vs raw rewards)

**Hyperparameter Effects**:
- `clip_range` (ε): 0.2 typical
  - Too small (0.1): Slow learning (policy barely changes)
  - Too large (0.5): Unstable (policy oscillates)
- `learning_rate`: 3e-4 typical
  - Too small (1e-5): Takes 10× longer
  - Too large (1e-3): Diverges (reward → -inf)
- `num_envs`: More = faster, but diminishing returns
  - 1024 envs: 30× speedup
  - 4096 envs: 66× speedup
  - 16384 envs: 100× speedup (not 264×, due to GPU saturation)

**Advantage Computation**:
```
A_t = r_t + γ r_{t+1} + γ² r_{t+2} + ... + γ^k V(s_{t+k}) - V(s_t)
    = "actual returns" - "predicted value"
```
- Positive advantage: Action better than expected → increase probability
- Negative advantage: Action worse than expected → decrease probability

**Loss Function**:
```python
ratio = π_new(a_t | s_t) / π_old(a_t | s_t)  # Importance sampling
L_CLIP = min(ratio × A_t, clip(ratio, 1-ε, 1+ε) × A_t)
```
- If A_t > 0 (good action): Increase π_new, but clip at 1+ε (don't overshoot)
- If A_t < 0 (bad action): Decrease π_new, but clip at 1-ε (don't kill probability)

**Extensions**:
- Show **curriculum learning**: Start with flat terrain, increase roughness over time
- Discuss **hierarchical RL**: High-level policy (waypoints) + low-level policy (locomotion)
- Add **visual observations**: Camera images (84×84×3) → CNN encoder → MLP policy

**Timing Breakdown** (RTX 4090, 4096 envs):
```
Physics step (4096 envs):  15 ms  (GPU PhysX)
Policy inference:           3 ms  (MLP forward pass)
Reward computation:         2 ms  (vectorized Python)
PPO update (every 24 steps): 50 ms  (backprop + optimizer)

Throughput: 4096 × 60 = 245,760 steps/sec
10M steps: 10,000,000 / 245,760 = 40.6 minutes (ideal)
Actual: 2 hours (including PPO updates, logging, checkpointing)
```

**Assessment**:
- Quiz: "Why use multiple environments? (Answer: Faster data collection, 66× speedup)"
- Project: Train ant/humanoid locomotion, achieve reward > 5000

**Common Student Mistakes**:
- Not normalizing observations (joint angles in [-π, +π] vs velocities in [-10, +10])
  - Solution: Use `ObservationNormalizer` (mean=0, std=1)
- Forgetting to reset environments after termination
  - Symptom: Reward plateaus (robots stuck in fallen state)
- Setting reward too sparse (e.g., +1 only at goal, 0 otherwise)
  - Solution: Add dense shaping rewards (distance to goal, forward velocity)

**Real-World Transfer Tips**:
1. **Domain randomization** (mandatory): Vary mass, friction, latency
2. **System identification**: Measure real robot parameters (mass, CoM, inertia)
3. **Safety**: Train with constraints (joint limits, torque limits)
4. **Latency compensation**: Train with observation delay (20ms → 50ms)

---

**Diagram Metadata**:
- **Lines of Mermaid Code**: 58
- **Nodes**: 18
- **Complexity**: High (nested subgraphs, feedback loop, performance annotations)
