# Exercise Template

**Task**: 029
**Purpose**: Standard format for end-of-chapter exercises

---

## Exercise Structure

````markdown
## End-of-Chapter Exercises

### Exercise [Number]: [Title] (Difficulty: [Easy/Medium/Hard/Advanced])

**Objective**: [What the student should learn or accomplish]

**Prerequisites**:
- [Concept 1 from chapter]
- [Concept 2 from chapter]
- [Software/hardware requirement]

**Instructions**:
1. [Clear, numbered step]
2. [Another step]
3. [Final step]

**Expected Outcome**:
- [What success looks like]
- [Specific outputs or behaviors]

**Hints** (Optional):
<details>
<summary>Click to reveal hint</summary>

- [Hint 1 for stuck students]
- [Hint 2 if needed]
</details>

**Solution** (Optional, in instructor guide):
```[language]
// Solution code or approach
```
````

---

## Exercise Types

### Type 1: Knowledge Check (Easy)
**Purpose**: Verify understanding of concepts

**Example**:
```markdown
### Exercise 1: ROS 2 Topic Types (Difficulty: Easy)

**Objective**: Identify appropriate message types for robot sensors.

**Instructions**:
1. For each sensor below, choose the correct ROS 2 message type:
   - RGB Camera: _______
   - Depth Camera: _______
   - IMU: _______
   - Laser Scanner: _______

2. Options:
   - `sensor_msgs/Image`
   - `sensor_msgs/Imu`
   - `sensor_msgs/LaserScan`
   - `sensor_msgs/PointCloud2`

**Expected Outcome**:
All sensors correctly matched with message types.

**Answer Key**:
- RGB Camera: sensor_msgs/Image
- Depth Camera: sensor_msgs/Image or sensor_msgs/PointCloud2
- IMU: sensor_msgs/Imu
- Laser Scanner: sensor_msgs/LaserScan
```

---

### Type 2: Coding Exercise (Medium)
**Purpose**: Practice implementing concepts

**Example**:
```markdown
### Exercise 2: Create a Simple Publisher (Difficulty: Medium)

**Objective**: Write a ROS 2 node that publishes robot velocity commands.

**Prerequisites**:
- ROS 2 Humble installed
- Understanding of publishers (Chapter 1.1)
- Python basics

**Instructions**:
1. Create a new file `velocity_publisher.py`
2. Import necessary ROS 2 modules (`rclpy`, `geometry_msgs/Twist`)
3. Create a node class that:
   - Publishes to `/cmd_vel` topic
   - Sends velocity commands at 10 Hz
   - Alternates between forward (0.5 m/s) and stop (0.0 m/s) every 2 seconds
4. Implement proper initialization and shutdown
5. Test with: `ros2 topic echo /cmd_vel`

**Expected Outcome**:
Terminal shows alternating Twist messages with linear.x values of 0.5 and 0.0.

**Hints**:
<details>
<summary>Need help?</summary>

- Use `create_timer()` for 10 Hz publishing
- Use a boolean flag to toggle between states
- Remember to initialize `linear` and `angular` components
</details>
```

---

### Type 3: System Integration (Hard)
**Purpose**: Combine multiple concepts

**Example**:
```markdown
### Exercise 3: Multi-Node Communication System (Difficulty: Hard)

**Objective**: Build a system with publisher, subscriber, and service.

**Prerequisites**:
- Chapters 1.1, 1.2 complete
- ROS 2 workspace set up
- Understanding of topics and services

**Instructions**:
1. **Node 1 (Sensor Simulator)**:
   - Publish simulated temperature data to `/temperature` (Float32)
   - Publish at 1 Hz

2. **Node 2 (Monitor)**:
   - Subscribe to `/temperature`
   - Log each value
   - Call `/reset_stats` service when temperature > 100

3. **Node 3 (Stats Service)**:
   - Provide `/reset_stats` service (Empty)
   - Track min/max/avg temperature
   - Reset stats when service is called

4. Create launch file to start all nodes

**Expected Outcome**:
- All three nodes running
- Temperature values logged
- Stats reset when threshold exceeded
- Launch file starts system correctly

**Deliverables**:
- `sensor_simulator.py`
- `monitor_node.py`
- `stats_service.py`
- `multi_node.launch.py`

**Rubric** (10 points):
- Sensor publishes correctly (2 pts)
- Monitor subscribes and logs (2 pts)
- Service implementation (3 pts)
- Service call triggered correctly (2 pts)
- Launch file works (1 pt)
```

---

### Type 4: Challenge / Project (Advanced)
**Purpose**: Open-ended application of knowledge

**Example**:
```markdown
### Challenge Exercise: Autonomous Object Follower (Difficulty: Advanced)

**Objective**: Create a system where a simulated robot follows detected objects.

**Requirements**:
- Must use Gazebo simulation
- Implement object detection (simple or ML-based)
- Control robot velocity based on object position
- Safe operation (collision avoidance)

**Suggested Approach**:
1. Set up Gazebo world with robot and objects
2. Use camera topic for vision input
3. Detect object (color-based or YOLO)
4. Calculate velocity commands to approach object
5. Stop when within threshold distance

**Assessment Criteria**:
- [ ] Robot correctly identifies target object
- [ ] Smooth velocity control (no jittering)
- [ ] Maintains safe distance
- [ ] Handles object loss (stops or searches)
- [ ] Code is well-documented

**Extensions** (Optional):
- Add multiple object types
- Implement priority system
- Use Isaac ROS for GPU-accelerated detection
- Deploy on physical robot (Jetson + RealSense)
```

---

## Exercise Difficulty Guidelines

### Easy (Knowledge Check)
- **Time**: 5-10 minutes
- **Focus**: Recall and comprehension
- **Example**: Multiple choice, fill-in-blank, matching
- **Prerequisites**: Chapter reading only

### Medium (Coding)
- **Time**: 15-30 minutes
- **Focus**: Application and implementation
- **Example**: Write function, modify code, debug program
- **Prerequisites**: Chapter examples completed

### Hard (Integration)
- **Time**: 30-60 minutes
- **Focus**: Synthesis and multi-step problems
- **Example**: Multi-file project, system integration
- **Prerequisites**: Multiple chapters, labs completed

### Advanced (Challenge)
- **Time**: 1-3 hours (or project-length)
- **Focus**: Evaluation, creation, open-ended
- **Example**: Original project, research implementation
- **Prerequisites**: Module complete, strong understanding

---

## Exercise Distribution per Chapter

**Recommended**:
- 1-2 Easy exercises (quick checks)
- 2-3 Medium exercises (core practice)
- 1 Hard exercise (integration)
- 0-1 Advanced challenge (optional)

**Total**: 4-7 exercises per chapter

---

## Auto-Grading (Optional)

For programming exercises, include test cases:

```python
# test_velocity_publisher.py
def test_publisher_frequency():
    """Test that node publishes at 10 Hz."""
    node = VelocityPublisher()
    # ... test implementation
    assert measured_frequency == pytest.approx(10.0, abs=0.5)

def test_velocity_values():
    """Test that velocity alternates correctly."""
    # ... test implementation
    assert all(v in [0.0, 0.5] for v in velocities)
```

---

## Instructor Resources

For each exercise, provide:

### Solutions Manual
- Complete working code
- Explanation of approach
- Common student mistakes
- Grading rubric

### Common Issues & Fixes
- Error message → Solution mapping
- Debugging tips
- Extension ideas for advanced students

---

**Status**: ✅ Exercise template ready for all chapters
