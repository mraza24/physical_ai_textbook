---
sidebar_position: 3
title: Appendix C - Launch File Reference
---

# Appendix C: Launch File Reference

Comprehensive reference for Python launch files in ROS 2.

---

## Overview

Launch files in ROS 2 use Python (unlike XML in ROS 1), providing full programming flexibility.

---

## Basic Launch File Template

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node_name',
            output='screen',
            parameters=[{
                'param1': 'value1',
                'param2': 42
            }]
        )
    ])
```

---

## Common Launch Actions

### 1. Node Action
```python
Node(
    package='package_name',
    executable='executable_name',
    name='node_name',  # Optional, overrides default
    namespace='robot1',  # Optional namespace
    output='screen',  # Or 'log'
    parameters=[{  # Optional parameters
        'param_name': param_value
    }],
    arguments=['arg1', 'arg2'],  # Command-line arguments
    remappings=[  # Topic/service remapping
        ('old_topic', 'new_topic')
    ]
)
```

### 2. IncludeLaunchDescription (Nested Launch)
```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('other_package'),
            'launch',
            'other_launch.py'
        )
    ),
    launch_arguments={
        'arg1': 'value1'
    }.items()
)
```

### 3. DeclareLaunchArgument
```python
from launch.actions import DeclareLaunchArgument

DeclareLaunchArgument(
    'robot_name',
    default_value='robot1',
    description='Name of the robot'
)
```

Access in launch file:
```python
from launch.substitutions import LaunchConfiguration

robot_name = LaunchConfiguration('robot_name')
```

---

## Parameter Files

### YAML Parameter File
**config/params.yaml**:
```yaml
my_node:
  ros__parameters:
    param1: "value1"
    param2: 42
    param3: 3.14
```

### Load in Launch File
```python
import os
from ament_index_python.packages import get_package_share_directory

config_file = os.path.join(
    get_package_share_directory('my_package'),
    'config',
    'params.yaml'
)

Node(
    package='my_package',
    executable='my_node',
    parameters=[config_file]
)
```

---

## Conditional Execution

### ExecuteProcess (Run Shell Command)
```python
from launch.actions import ExecuteProcess

ExecuteProcess(
    cmd=['ros2', 'topic', 'list'],
    output='screen'
)
```

### GroupAction with Condition
```python
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

GroupAction(
    actions=[
        Node(...)
    ],
    condition=IfCondition(LaunchConfiguration('use_sim'))
)
```

---

## Event Handlers

### RegisterEventHandler
```python
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

RegisterEventHandler(
    OnProcessStart(
        target_action=node1,
        on_start=[
            LogInfo(msg='Node 1 started, launching Node 2'),
            node2
        ]
    )
)
```

---

## Multi-Robot Launch

### Namespacing Multiple Robots
```python
def generate_launch_description():
    robots = ['robot1', 'robot2', 'robot3']

    nodes = []
    for robot_name in robots:
        nodes.append(
            Node(
                package='my_package',
                executable='my_node',
                namespace=robot_name,
                parameters=[{
                    'robot_id': robot_name
                }]
            )
        )

    return LaunchDescription(nodes)
```

---

## Advanced: Composable Nodes

### Using Component Container
```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

ComposableNodeContainer(
    name='my_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='my_package',
            plugin='my_package::MyComponent',
            name='my_component',
            parameters=[{...}]
        )
    ],
    output='screen'
)
```

---

## Complete Example: Robot Simulation Launch

**launch/robot_sim.launch.py**:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    pkg_gazebo = get_package_share_directory('gazebo_ros')
    pkg_robot = get_package_share_directory('my_robot_package')

    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    world_file = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_robot, 'worlds', 'empty.world'),
        description='Gazebo world file'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': open(os.path.join(
                pkg_robot, 'urdf', 'robot.urdf'
            )).read()
        }]
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_robot, 'config', 'robot.rviz')],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        world_file,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        rviz
    ])
```

**Run**:
```bash
ros2 launch my_robot_package robot_sim.launch.py
ros2 launch my_robot_package robot_sim.launch.py world:=/path/to/my_world.sdf
```

---

## Debugging Launch Files

### Enable Verbose Output
```bash
ros2 launch --debug my_package my_launch.py
```

### Print Launch Configuration
```python
from launch.actions import LogInfo

LogInfo(msg=['Robot name: ', LaunchConfiguration('robot_name')])
```

### Check Effective Parameters
```bash
ros2 param list
ros2 param get /my_node my_param
```

---

## Best Practices

1. **Use DeclareLaunchArgument** for configurable values
2. **Load parameters from YAML** for maintainability
3. **Namespace nodes** for multi-robot systems
4. **Use IncludeLaunchDescription** for modularity
5. **Add descriptions** to all arguments
6. **Test launch files incrementally** (add one node at a time)

---

**See Also**:
- Chapter 1.3: Launch Files and Configuration
- ROS 2 Launch Documentation: https://docs.ros.org/en/humble/Tutorials/Launch-Files.html
