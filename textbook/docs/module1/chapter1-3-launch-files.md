---
sidebar_position: 4
title: Chapter 1.3 - Launch Files and Configuration
---

# Chapter 1.3: Launch Files and Configuration

**Module**: 1 - The Robotic Nervous System
**Week**: 3
**Estimated Reading Time**: 30 minutes

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. Write Python launch files to start complex multi-node systems
2. Configure node parameters using YAML files
3. Use namespaces and remapping for multi-robot systems
4. Implement node composition for improved performance
5. Debug launch file issues

---

## Prerequisites

- Completed Chapters 1.1 and 1.2
- Understanding of ROS 2 nodes, topics, services, and actions
- Basic Python programming

---

## Introduction

As your robotics projects grow in complexity, manually starting each node becomes impractical. Imagine a mobile robot with:
- A camera node
- A LIDAR node
- A localization node
- A path planning node
- A motor controller node

Starting these individually, configuring parameters, and managing dependencies would be tedious and error-prone. This is where **launch files** come in.

Launch files allow you to:
- Start multiple nodes with a single command
- Configure parameters programmatically
- Set up namespaces for multi-robot systems
- Manage node lifecycle and dependencies
- Create reusable robot configurations

---

## Key Terms

:::info Glossary Terms
- **Launch File**: Python script that automates starting multiple nodes with configuration
- **Parameter**: Runtime configuration value for nodes (e.g., topic names, sensor IDs)
- **Namespace**: Hierarchical grouping of nodes and topics (e.g., `/robot1/camera`)
- **Node Composition**: Running multiple nodes in a single process for efficiency
- **Launch Description**: Python object representing the launch configuration
:::

---

## Core Concepts

### 1. Python Launch Files

ROS 2 uses Python launch files (not XML like ROS 1) for flexibility and programmability.

#### Basic Launch File Structure

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener'
        ),
    ])
```

**Key Components**:
- `LaunchDescription`: Container for all launch actions
- `Node`: Action to start a ROS 2 node
- `generate_launch_description()`: Function that returns the launch configuration

#### Running a Launch File

```bash
# Run launch file
ros2 launch <package_name> <launch_file_name>

# Example
ros2 launch my_robot_bringup robot.launch.py
```

---

### 2. Node Configuration with Parameters

#### Inline Parameters

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',
            executable='camera_node',
            name='camera',
            parameters=[{
                'frame_rate': 30,
                'resolution': '1920x1080',
                'camera_id': 0
            }]
        ),
    ])
```

#### YAML Parameter Files

**parameters.yaml**:
```yaml
camera_node:
  ros__parameters:
    frame_rate: 30
    resolution: "1920x1080"
    camera_id: 0
    auto_exposure: true
```

**Launch file using YAML**:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get path to parameters file
    config = os.path.join(
        get_package_share_directory('my_robot'),
        'config',
        'parameters.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_robot',
            executable='camera_node',
            name='camera',
            parameters=[config]
        ),
    ])
```

---

### 3. Namespaces and Remapping

#### Namespaces for Multi-Robot Systems

```python
def generate_launch_description():
    return LaunchDescription([
        # Robot 1
        Node(
            package='my_robot',
            executable='controller',
            name='controller',
            namespace='robot1'  # Topics become /robot1/cmd_vel, /robot1/odom, etc.
        ),
        # Robot 2
        Node(
            package='my_robot',
            executable='controller',
            name='controller',
            namespace='robot2'  # Topics become /robot2/cmd_vel, /robot2/odom, etc.
        ),
    ])
```

#### Topic Remapping

```python
Node(
    package='my_robot',
    executable='camera_node',
    name='camera',
    remappings=[
        ('image_raw', 'camera/image'),  # Remap /image_raw → /camera/image
        ('camera_info', 'camera/info')
    ]
)
```

---

### 4. Node Composition

Node composition allows multiple nodes to run in a single process, reducing overhead.

#### Component Node

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_robot',
                plugin='my_robot::CameraComponent',
                name='camera'
            ),
            ComposableNode(
                package='my_robot',
                plugin='my_robot::ProcessorComponent',
                name='processor'
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
```

**Benefits**:
- Reduced inter-process communication overhead
- Lower memory footprint
- Better performance for high-frequency data pipelines

---

### 5. Launch File Arguments

Make launch files configurable with command-line arguments:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera device ID'
    )

    return LaunchDescription([
        use_sim_time_arg,
        camera_id_arg,
        Node(
            package='my_robot',
            executable='camera_node',
            name='camera',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'camera_id': LaunchConfiguration('camera_id')
            }]
        ),
    ])
```

**Usage**:
```bash
# Use defaults
ros2 launch my_robot camera.launch.py

# Override arguments
ros2 launch my_robot camera.launch.py use_sim_time:=true camera_id:=1
```

---

### 6. Conditional Launch Logic

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    return LaunchDescription([
        use_rviz_arg,

        # Always launch robot nodes
        Node(
            package='my_robot',
            executable='controller',
            name='controller'
        ),

        # Conditionally launch RViz
        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/path/to/config.rviz']
        ),
    ])
```

---

## Practical Examples

### Example 1: Simple Robot Bringup

**robot_bringup.launch.py**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Motor controller
        Node(
            package='my_robot',
            executable='motor_controller',
            name='motor_controller',
            parameters=[{'wheel_radius': 0.05, 'wheel_base': 0.3}],
            output='screen'
        ),

        # Sensor fusion
        Node(
            package='my_robot',
            executable='sensor_fusion',
            name='sensor_fusion',
            remappings=[
                ('imu_raw', 'sensors/imu'),
                ('odom_raw', 'sensors/wheel_odom')
            ],
            output='screen'
        ),

        # State publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen'
        ),
    ])
```

---

### Example 2: Multi-Robot Simulation

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robots = []

    # Launch 3 robots with different namespaces
    for i in range(3):
        robot_namespace = f'robot_{i}'

        robots.append(
            Node(
                package='turtlebot3_gazebo',
                executable='turtlebot3_drive',
                name='turtlebot3_drive',
                namespace=robot_namespace,
                parameters=[{
                    'initial_x': float(i * 2),
                    'initial_y': 0.0
                }]
            )
        )

    return LaunchDescription(robots)
```

---

### Example 3: Including Other Launch Files

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Path to another launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': 'my_world.world'}.items()
    )

    return LaunchDescription([
        gazebo_launch,
        # Add your nodes here
    ])
```

---

## Debugging Launch Files

### Common Issues and Solutions

1. **Node not starting**:
   ```bash
   # Check if package/executable exists
   ros2 pkg list | grep my_robot
   ros2 pkg executables my_robot
   ```

2. **Parameter not loading**:
   ```bash
   # Check parameter file path
   ros2 param list
   ros2 param get /node_name parameter_name
   ```

3. **Topic remapping not working**:
   ```bash
   # Verify remapped topics
   ros2 topic list
   ros2 node info /node_name
   ```

### Launch File Debugging Flags

```bash
# Verbose output
ros2 launch --debug my_robot robot.launch.py

# Show launch file structure
ros2 launch --show-dependencies my_robot robot.launch.py
```

---

## Best Practices

1. **Use Descriptive Names**: Name nodes and parameters clearly
   ```python
   name='front_camera'  # Good
   name='node1'         # Bad
   ```

2. **Parameterize Everything**: Use arguments for flexibility
   ```python
   DeclareLaunchArgument('robot_model', default_value='turtlebot3_burger')
   ```

3. **Organize Config Files**: Keep YAML parameters in `/config` directory

4. **Use Namespaces**: Avoid topic/service name collisions in multi-robot systems

5. **Add Comments**: Document complex launch logic
   ```python
   # Launch SLAM only if mapping mode is enabled
   ```

6. **Handle Errors Gracefully**: Use try/except for file operations
   ```python
   try:
       config = os.path.join(get_package_share_directory('my_robot'), 'config', 'params.yaml')
   except Exception as e:
       print(f'Error loading config: {e}')
   ```

---

## Summary

In this chapter, you learned:

- ✅ Python launch files automate multi-node startup
- ✅ Parameters can be configured inline or via YAML files
- ✅ Namespaces enable multi-robot coordination
- ✅ Node composition improves performance
- ✅ Launch arguments make configurations reusable
- ✅ Conditional logic adapts launches to different scenarios

**Key Takeaway**: Launch files are the orchestration layer of ROS 2, enabling you to manage complex robotic systems efficiently.

---

## End-of-Chapter Exercises

### Exercise 1: Write a Multi-Node Launch File (Difficulty: Medium)

Create a launch file that starts:
1. A `talker` node publishing to `/chatter`
2. A `listener` node subscribing to `/chatter`
3. A `rqt_graph` node for visualization

**Bonus**: Add a launch argument to control the talker's publish rate.

---

### Exercise 2: Multi-Robot System (Difficulty: Hard)

Write a launch file that starts two robots (`robot_a` and `robot_b`) with:
- Separate namespaces
- Different starting positions
- Shared map topic (`/map`)

**Hint**: Use topic remapping for the shared map.

---

### Exercise 3: Parameter Configuration (Difficulty: Easy)

Create a YAML parameter file for a camera node with:
- Frame rate: 30 FPS
- Resolution: 1920x1080
- Auto-exposure: enabled

Write a launch file that loads these parameters.

---

## Further Reading

### Required
1. [ROS 2 Launch Files Tutorial](https://docs.ros.org/en/humble/Tutorials/Launch-Files.html)
2. [ROS 2 Parameters Guide](https://docs.ros.org/en/humble/Tutorials/Parameters.html)
3. [Node Composition](https://docs.ros.org/en/humble/Tutorials/Composition.html)

### Optional
- [Launch System Architecture](https://design.ros2.org/articles/roslaunch.html)
- [Advanced Launch Features](https://docs.ros.org/en/humble/Tutorials/Launch/Launch-Main.html)

---

## Next Chapter

Continue to **[Chapter 1.4: Building ROS 2 Packages](./chapter1-4-packages)** to learn how to organize your code into reusable ROS 2 packages.

---

**Pro Tip**: Start simple with basic launch files, then gradually add parameters, namespaces, and composition as your project grows in complexity.
