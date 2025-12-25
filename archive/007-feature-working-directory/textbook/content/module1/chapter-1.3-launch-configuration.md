# Chapter 1.3: Launch Files & Configuration

> **Module**: 1 - Robotic Nervous System (ROS 2)
> **Week**: 5 (Part 1)
> **Estimated Reading Time**: 45 minutes

---

## Summary

This chapter introduces ROS 2 launch files—Python scripts that automate the startup of multi-node robot systems with parameter configuration, topic remapping, and conditional logic—eliminating the need to manually start dozens of nodes in separate terminals. Mastering launch files is essential for deploying complex robot applications efficiently and reproducibly.

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Create** Python launch files that start multiple nodes with configurable parameters
2. **Configure** node parameters using YAML files for environment-specific settings
3. **Implement** topic and service remapping to integrate nodes with incompatible naming conventions
4. **Apply** conditional logic in launch files for different deployment scenarios (simulation vs. hardware)
5. **Organize** launch file hierarchies with includes for modular system composition

**Prerequisite Knowledge**: Chapter 1.1 (ROS 2 Fundamentals), Chapter 1.2 (Nodes & Communication), basic Python programming

---

## Key Terms

This chapter introduces the following technical terms (see Glossary for detailed definitions):

- **Launch File**: Python script that describes how to start and configure multiple nodes
- **LaunchDescription**: ROS 2 object containing all launch actions (nodes, parameters, includes)
- **Node Launch Action**: Declaration to start a ROS 2 node with specified parameters
- **Parameter**: Named configuration value accessible to nodes at runtime (e.g., `camera_fps: 30`)
- **YAML Parameter File**: Human-readable configuration file mapping parameter names to values
- **Remapping**: Redirecting topic/service names (e.g., `/camera/image` → `/my_camera/image`)
- **Composition**: Running multiple nodes in a single process for efficiency
- **Launch Argument**: Command-line parameter that customizes launch file behavior

---

## Core Concepts

### 1. Why Launch Files? The Multi-Node Problem

As robot systems grow, manually starting nodes becomes unmanageable. Consider a typical autonomous robot:

```bash
# Terminal 1: Camera driver
ros2 run camera_driver camera_node

# Terminal 2: Object detector
ros2 run perception detector_node

# Terminal 3: Motion planner
ros2 run planning planner_node

# Terminal 4: Navigation
ros2 run navigation navigator_node

# Terminal 5: Visualization
ros2 run rviz2 rviz2

# ...and 20+ more nodes
```

Problems with manual startup:
- **Time-consuming**: 5-10 minutes to start a full system
- **Error-prone**: Forget to start a critical node, system fails mysteriously
- **Non-reproducible**: Different team members start nodes in different orders with different parameters
- **Tedious**: Shutting down requires closing 25 terminals individually

**Launch files solve this**: One command starts the entire system with consistent configuration.

```bash
ros2 launch my_robot robot.launch.py
# Starts all 25 nodes, configured correctly, in 2 seconds
```

### 2. Anatomy of a Python Launch File

ROS 2 uses **Python launch files** (unlike ROS 1's XML). Python provides:
- Conditional logic (if/else for simulation vs. hardware)
- Loops (start N copies of a node)
- Dynamic computation (calculate parameters from environment variables)

**Basic structure**:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='custom_node_name',
            parameters=[{'param1': 'value1', 'param2': 42}],
            remappings=[('/old_topic', '/new_topic')],
            output='screen'  # Print logs to terminal
        ),
        # Add more nodes...
    ])
```

**Key components**:
- `generate_launch_description()`: Required function that returns LaunchDescription
- `LaunchDescription([...])`: Container for launch actions
- `Node(...)`: Declares a node to start, with package, executable, and configuration
- `parameters=[...]`: Dictionary of parameter name-value pairs
- `remappings=[...]`: List of (old, new) topic/service name tuples
- `output='screen'`: Display node output in terminal (default: log file only)

### 3. Multi-Node Launch File Example

Let's create a launch file for a simple camera → detector pipeline:

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define nodes
    camera_node = Node(
        package='image_tools',
        executable='cam2image',
        name='camera',
        parameters=[{
            'frequency': 30.0,  # Publish at 30 Hz
            'width': 640,
            'height': 480
        }],
        remappings=[
            ('/image', '/camera/image_raw')  # Remap default topic
        ],
        output='screen'
    )

    detector_node = Node(
        package='perception',
        executable='object_detector',
        name='detector',
        parameters=[{
            'model_path': '/models/yolo.onnx',
            'confidence_threshold': 0.5
        }],
        output='screen'
    )

    visualizer_node = Node(
        package='image_tools',
        executable='showimage',
        name='visualizer',
        remappings=[
            ('/image', '/detector/image_detections')
        ],
        output='screen'
    )

    return LaunchDescription([
        camera_node,
        detector_node,
        visualizer_node
    ])
```

**Run this launch file**:
```bash
ros2 launch my_package camera_pipeline.launch.py
```

All three nodes start simultaneously, configured correctly. Press Ctrl+C once to stop all nodes.

**Key Points**:
- Each node is configured with its own parameters
- Remappings ensure topic name compatibility
- All nodes start/stop together as a unit
- `output='screen'` shows logs interleaved in one terminal

### 4. Parameters and YAML Configuration Files

Hardcoding parameters in launch files is inflexible. What if you want different configurations for development vs. production, or simulation vs. hardware?

**YAML parameter files** provide external configuration:

**File**: `config/camera_params.yaml`
```yaml
camera:
  ros__parameters:
    frequency: 30.0
    width: 1920
    height: 1080
    auto_exposure: true
    gain: 1.5
```

**File**: `config/detector_params.yaml`
```yaml
detector:
  ros__parameters:
    model_path: "/models/yolo_v8.onnx"
    confidence_threshold: 0.7
    nms_threshold: 0.45
    input_size: [640, 640]
    classes:
      - person
      - car
      - bicycle
```

**Load parameters in launch file**:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_package')

    # Load YAML files
    camera_params = os.path.join(pkg_dir, 'config', 'camera_params.yaml')
    detector_params = os.path.join(pkg_dir, 'config', 'detector_params.yaml')

    camera_node = Node(
        package='image_tools',
        executable='cam2image',
        name='camera',
        parameters=[camera_params],  # Load from file
        output='screen'
    )

    detector_node = Node(
        package='perception',
        executable='object_detector',
        name='detector',
        parameters=[detector_params],
        output='screen'
    )

    return LaunchDescription([camera_node, detector_node])
```

**Benefits**:
- **Separation of concerns**: Code vs. configuration
- **Environment-specific configs**: `dev_params.yaml`, `prod_params.yaml`, `sim_params.yaml`
- **Version control**: Track parameter changes over time
- **Non-programmers can tune**: Roboticists edit YAML without touching code

### 5. Launch Arguments for Runtime Customization

Launch arguments allow passing values at runtime:

```bash
ros2 launch my_package robot.launch.py use_sim:=true camera_fps:=60
```

**Declare and use arguments**:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation instead of real hardware'
    )

    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='30',
        description='Camera frame rate (Hz)'
    )

    # Use arguments in node configuration
    camera_node = Node(
        package='camera_driver',
        executable='camera_node',
        name='camera',
        parameters=[{
            'fps': LaunchConfiguration('camera_fps'),  # Use argument value
            'simulate': LaunchConfiguration('use_sim')
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_arg,
        camera_fps_arg,
        camera_node
    ])
```

**Arguments with conditional logic**:

```python
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='false')

    # Start real camera only if NOT in simulation
    real_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        condition=UnlessCondition(LaunchConfiguration('use_sim'))
    )

    # Start simulated camera only if IN simulation
    sim_camera = Node(
        package='gazebo_ros',
        executable='spawn_camera',
        name='camera',
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    return LaunchDescription([
        use_sim_arg,
        real_camera,
        sim_camera
    ])
```

Now `ros2 launch my_package robot.launch.py use_sim:=true` starts the sim camera, while `use_sim:=false` starts the real camera.

### 6. Topic and Service Remapping

**Problem**: You want to use a third-party node that publishes to `/camera/image`, but your detector expects images on `/front_camera/rgb`.

**Solution**: Remapping redirects names without modifying node code.

```python
camera_node = Node(
    package='camera_driver',
    executable='camera',
    name='front_camera',
    remappings=[
        ('/camera/image', '/front_camera/rgb'),  # Old → New
        ('/camera/info', '/front_camera/camera_info')
    ]
)

detector_node = Node(
    package='perception',
    executable='detector',
    name='detector',
    remappings=[
        ('/image_raw', '/front_camera/rgb')  # Subscribes to remapped topic
    ]
)
```

**Remapping works for**:
- Topics (pub/sub)
- Services (request/response)
- Actions (goal/feedback/result)

**Use cases**:
- Integrating nodes with incompatible naming conventions
- Running multiple instances of the same node (e.g., two cameras with different namespaces)
- Testing nodes in isolation (remap to test topics)

### 7. Composable Nodes for Performance

**Problem**: Each ROS 2 node is a separate process. Starting 50 nodes creates 50 processes with overhead (memory, context switching, serialization/deserialization).

**Solution**: **Node composition** runs multiple nodes in a single process as shared-library components.

**Benefits**:
- **Lower latency**: Intra-process communication (shared memory) instead of inter-process (DDS)
- **Reduced CPU**: No serialization overhead for messages passed between composed nodes
- **Less memory**: Single process memory footprint instead of N processes

**Composition example**:

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Define container (single process)
    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Node 1: Camera driver (as component)
            ComposableNode(
                package='camera_driver',
                plugin='camera::CameraNode',  # Class name
                name='camera',
                parameters=[{'fps': 30}]
            ),
            # Node 2: Detector (as component)
            ComposableNode(
                package='perception',
                plugin='perception::DetectorNode',
                name='detector',
                parameters=[{'confidence': 0.7}]
            ),
        ],
        output='screen'
    )

    return LaunchDescription([container])
```

**Key differences from regular nodes**:
- Use `ComposableNodeContainer` instead of multiple `Node` actions
- Nodes must be implemented as **component plugins** (requires C++ changes)
- Specify `plugin='namespace::ClassName'` instead of `executable`
- All nodes in one container run in one process

**When to use composition**:
- High-frequency communication (>100 Hz) between nodes
- Latency-critical applications (real-time control)
- Resource-constrained platforms (embedded systems, Jetson)

**When NOT to use**:
- Nodes in different programming languages (composition requires same language, typically C++)
- Nodes that crash frequently (one crash kills entire container)
- Development phase (separate processes are easier to debug)

---

## Practical Example: Multi-Node Robot System Launch File

### Overview

This example demonstrates a complete launch file for a mobile robot with camera, perception, navigation, and logging—showing parameter loading, remapping, conditional logic, and launch includes.

### Prerequisites

- Software: ROS 2 Humble, Python 3.10+
- Knowledge: Chapters 1.1-1.2 (nodes, topics, services, actions)

### Implementation

**File**: `robot_bringup.launch.py`

```python
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('my_robot')

    # Launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Launch in simulation mode'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error'],
        description='Logging level'
    )

    # Parameter files
    camera_params = os.path.join(pkg_dir, 'config', 'camera.yaml')
    nav_params = os.path.join(pkg_dir, 'config', 'navigation.yaml')

    # Nodes
    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        parameters=[camera_params],
        remappings=[
            ('/camera/color/image_raw', '/front_camera/rgb'),
            ('/camera/depth/image_rect_raw', '/front_camera/depth')
        ],
        condition=UnlessCondition(LaunchConfiguration('use_sim')),
        output='screen'
    )

    detector_node = Node(
        package='perception',
        executable='object_detector',
        name='detector',
        parameters=[{
            'model_path': os.path.join(pkg_dir, 'models', 'yolo.onnx'),
            'confidence_threshold': 0.7
        }],
        remappings=[
            ('/image', '/front_camera/rgb')
        ],
        output='screen'
    )

    # Include navigation launch file (modular)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'navigation.launch.py')
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'params_file': nav_params
        }.items()
    )

    # Data logger (conditional - only if NOT simulating)
    logger_node = Node(
        package='rosbag2',
        executable='rosbag2',
        name='logger',
        arguments=['record', '-a'],  # Record all topics
        condition=UnlessCondition(LaunchConfiguration('use_sim')),
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        use_sim_arg,
        log_level_arg,
        # Nodes
        camera_node,
        detector_node,
        # Includes
        nav_launch,
        # Conditional nodes
        logger_node
    ])
```

### Running the Launch File

**Real hardware**:
```bash
ros2 launch my_robot robot_bringup.launch.py
```

**Simulation**:
```bash
ros2 launch my_robot robot_bringup.launch.py use_sim:=true
```

**With debug logging**:
```bash
ros2 launch my_robot robot_bringup.launch.py log_level:=debug
```

### Expected Output

```
[INFO] [launch]: Starting robot system...
[INFO] [camera]: RealSense camera initialized at 30 fps
[INFO] [detector]: Loaded YOLO model, confidence=0.7
[INFO] [navigation]: Navigation stack ready
[INFO] [logger]: Recording to rosbag2_2025_12_11-10_30_45
```

All nodes start, configured correctly, and shut down together with Ctrl+C.

---

## Figures & Diagrams

### Figure 1.3-1: Launch File Execution Flow

![Launch File Flow](../../diagrams/module1/fig1.3-launch-flow.svg)

**Caption**: Flowchart showing launch file execution: parse launch file → process arguments → load parameters → apply conditions → start nodes → monitor lifecycle. Launch system handles dependencies, ensuring nodes start in correct order.

---

## Exercises

### Exercise 1: Create a Two-Node Launch File (Difficulty: Easy)

**Objective**: Practice basic launch file syntax

**Task**: Create a launch file that starts a talker and listener node (from Chapter 1.1 examples) simultaneously.

**Requirements**:
- Start both nodes with one command
- Configure talker to publish at 2 Hz (use parameter)
- Remap `/chatter` to `/robot/comms`
- Display output from both nodes

**Expected Outcome**:
```bash
$ ros2 launch my_package chat.launch.py
[talker]: Publishing at 2 Hz...
[listener]: Received: "Hello ROS 2! Count: 0"
```

**Estimated Time**: 15 minutes

---

### Exercise 2: Parameter File Configuration (Difficulty: Medium)

**Objective**: Separate configuration from code using YAML files

**Task**: Move hardcoded parameters from Exercise 1's launch file into a YAML config file.

**Requirements**:
- Create `config/chat_params.yaml` with talker frequency, topic name
- Modify launch file to load parameters from YAML
- Test with different YAML files for 1 Hz, 5 Hz, 10 Hz configurations

**Expected Outcome**: Same behavior as Exercise 1, but configuration is external and easily swappable.

**Estimated Time**: 20 minutes

---

### Exercise 3: Conditional Simulation/Hardware Launch (Difficulty: Hard)

**Objective**: Use launch arguments and conditions for deployment flexibility

**Task**: Create a launch file that starts different nodes based on `use_sim` argument—simulated camera in simulation mode, real camera otherwise.

**Requirements**:
- `DeclareLaunchArgument` for `use_sim` (default false)
- Conditional node launching using `IfCondition`/`UnlessCondition`
- Simulated camera publishes dummy data at `/sim_camera/image`
- Real camera (or placeholder) publishes at `/real_camera/image`
- Both remap to common topic `/robot/camera`

**Expected Outcome**:
```bash
$ ros2 launch my_package camera.launch.py use_sim:=true
[sim_camera]: Publishing simulated images...

$ ros2 launch my_package camera.launch.py use_sim:=false
[real_camera]: Connecting to hardware...
```

**Estimated Time**: 30 minutes

---

## Summary & Key Takeaways

In this chapter, you learned:

- **Launch files** automate multi-node startup, eliminating manual terminal management
- **Python launch files** provide flexibility with conditional logic, loops, and dynamic configuration
- **YAML parameter files** separate configuration from code for environment-specific deployments
- **Launch arguments** enable runtime customization (simulation mode, logging level, parameters)
- **Remapping** resolves naming conflicts between nodes without code changes
- **Composable nodes** improve performance by running multiple nodes in a single process
- **Launch file hierarchies** with includes support modular, reusable system composition

**Connection to Chapter 1.4**: Now that you can launch and configure nodes, Chapter 1.4 introduces the ROS 2 build system (colcon) and package structure—how to organize your code, compile it, and distribute it as reusable packages with proper dependency management.

---

## Additional Resources

### Official Documentation
- **ROS 2 Launch**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html
- **Launch File Architecture**: https://design.ros2.org/articles/roslaunch.html
- **Composable Nodes**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html

### Recommended Reading
- **Launch System Design**: Design rationale for Python-based launch
- **Parameter Best Practices**: Guidelines for organizing YAML configs

### Community Resources
- **Launch File Examples**: ros2/launch repository contains extensive examples
- **ROS Discourse**: Community Q&A for launch file debugging

---

## Notes for Instructors

**Teaching Tips**:
- **Live demo**: Start nodes manually (5 terminals), then show same system launching with one command—immediate value demonstration
- **Common mistake**: Students forget `generate_launch_description()` function name (must be exact) or forget to return `LaunchDescription`

**Lab Exercise Ideas**:
- **Debug challenge**: Provide broken launch file with syntax errors (missing commas, wrong function names)—students fix and run
- **Multi-environment deployment**: Students create `dev.yaml`, `prod.yaml`, `sim.yaml` parameter files for same robot

**Assessment Suggestions**:
- **Code review**: Check proper use of launch arguments (defaults provided, descriptions clear)
- **Parameter organization**: YAML files should group related parameters logically
- **Conditional logic**: Verify conditions work correctly (test both branches)

---

**Chapter Metadata**:
- **Word Count**: ~2700 words (core concepts)
- **Figures**: 1 (launch execution flow)
- **Code Examples**: 6 (basic launch, multi-node, YAML params, arguments, conditionals, composition)
- **Exercises**: 3 (easy, medium, hard)
- **Glossary Terms**: 8
