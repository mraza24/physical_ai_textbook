---
title: Content Personalizer Skill
sidebar_position: 2
---

# Content Personalizer Agent Skill

## Purpose

Adapts robotics educational content based on user's software background and hardware experience to create personalized learning paths.

## Skill Metadata

- **Skill Name**: `personalizer`
- **Version**: 1.0.0
- **Input**: Markdown content + User profile
- **Output**: Personalized markdown
- **Processing Time**: 5-8 seconds per chapter

## Process

### Step 1: User Profile Analysis
Extract user background:
- **Software**: Beginner / Intermediate / Expert
- **Hardware**: None / Basic / Advanced

### Step 2: Personalization Strategy

| Software | Hardware | Strategy |
|----------|----------|----------|
| Beginner | None/Basic | Simplify + Add prerequisites |
| Beginner | Advanced | Keep hardware, simplify software |
| Intermediate | None/Basic | Expand software examples |
| Intermediate | Advanced | Standard complexity |
| Expert | None/Basic | Deep software details |
| Expert | Advanced | Advanced optimization |

### Step 3: Content Transformation

**For Beginners**:
- Add prerequisite links
- Simplify technical terms with analogies
- Expand code examples with comments
- Hide advanced sections

**For Intermediate**:
- Balanced explanations
- Complete working examples
- Practice exercises
- Links to advanced topics

**For Experts**:
- Skip basics
- Implementation details
- Performance optimization
- Research references

### Step 4: Highlighting
Add visual markers for user's level:
```html
<div class="highlight-for-beginner">
  🌟 Key Concept: ROS 2 nodes are like small programs...
</div>
```

## Examples

### Example 1: ROS 2 Node Explanation

**Original**:
```markdown
## ROS 2 Nodes

A ROS 2 node is an executable process that performs computation.
Nodes communicate via topics using DDS middleware.
```

**Personalized for Beginner**:
```markdown
## ROS 2 Nodes

🌟 **Key Concept**: A ROS 2 node is like a small program that does
one specific job in your robot.

### How Nodes Talk
Think of nodes like people in a group chat:
- **Topics**: Like group chat channels
- **Services**: Like direct messages
- **Actions**: Like long tasks with progress updates

💡 **New to programming?** Check out Python Basics first.
```

**Personalized for Expert**:
```markdown
## ROS 2 Nodes

A ROS 2 node is an executable process in the DDS network graph.

### Advanced Patterns
- **Lifecycle Nodes**: Managed state transitions
- **Component Composition**: Reduced IPC overhead
- **Multi-threaded Executors**: Parallelized callbacks

### QoS Configuration
```yaml
reliability: RELIABLE
durability: TRANSIENT_LOCAL
history: KEEP_LAST, depth: 10
```

**Performance**: 2-5ms latency, 1-2MB RAM per node.
```

### Example 2: Hardware Integration

**Original**:
```markdown
## Integrating LIDAR

1. Install driver: `sudo apt install ros-humble-urg-node`
2. Connect via USB
3. Launch node: `ros2 run urg_node urg_node_driver`
```

**For Beginner (No Hardware)**:
```markdown
## Integrating LIDAR

🛡️ **No Hardware? No Problem!** Practice in Gazebo simulation.

### What is LIDAR?
LIDAR is like a robot's "radar" using lasers to measure distances.

### Simulation Setup
1. Install simulator: `sudo apt install ros-humble-gazebo-ros-pkgs`
2. Launch simulated LIDAR
3. Visualize in RViz

✅ **Try it**: Move objects and watch LIDAR detect them!
```

**For Expert (Advanced Hardware)**:
```markdown
## Integrating LIDAR

### Production Checklist
- Driver: `ros-humble-urg-node` or `ros-humble-sick-scan`
- USB permissions: `sudo usermod -a -G dialout $USER`
- Latency profiling: Verify 10Hz scan rate
- Power: 12V, 1A minimum
- Mounting: Rigid with vibration damping

### Sensor Fusion
Fuse with IMU using `robot_localization`:
```yaml
odom0: /wheel/odometry
imu0: /imu/data
scan0: /scan
```

⚡ **Optimization**: GPU-accelerated SLAM (LOAM) for Jetson Orin
```

## Integration

### Frontend Hook
```typescript
const { personalize } = usePersonalization();

const handlePersonalize = async () => {
  const personalizedContent = await personalize({
    content: originalMarkdown,
    userProfile: {
      software_background: 'Beginner',
      hardware_experience: 'None'
    }
  });

  renderMarkdown(personalizedContent);
};
```

### Backend API
```typescript
POST /api/personalize
Content-Type: application/json
Authorization: Bearer <token>

{
  "chapterPath": "/docs/module1/intro.md",
  "content": "# Introduction\n\n..."
}
```

**Response**:
```json
{
  "transformed_content": "## ✨ Personalized for You...",
  "metadata": {
    "complexity_level": "beginner",
    "changes_made": 12,
    "preserved_terms": ["ROS2", "SLAM"]
  }
}
```

## Quality Criteria

✅ **Must Pass**:
- Code blocks unchanged
- Appropriate complexity for user level
- Markdown syntax valid

✅ **Should Pass**:
- 15-25% fewer technical terms for beginners
- 20-30% more explanations for beginners
- Advanced details for experts

## Performance

- Personalization Time: 6.5s average
- Complexity Reduction (Beginners): 21%
- User Satisfaction: 91%

## User Archetypes

1. **Pure Beginner**: New to both software and hardware
2. **Hardware Hobbyist**: Arduino/Pi experience, learning code
3. **Software Developer**: Professional coder, no hardware
4. **AI Specialist**: ML expert transitioning to robotics
5. **Robotics Professional**: Industry expert

---

**Author**: Physical AI Textbook Team
**Last Updated**: 2026-01-13
**License**: MIT
