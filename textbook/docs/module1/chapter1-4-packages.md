---
sidebar_position: 5
title: Chapter 1.4 - Building ROS 2 Packages
---

# Chapter 1.4: Building ROS 2 Packages

**Module**: 1 - The Robotic Nervous System
**Week**: 4
**Estimated Reading Time**: 35 minutes

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. Create custom ROS 2 packages in Python and C++
2. Write package.xml manifests and CMakeLists.txt files
3. Build workspaces using colcon
4. Manage package dependencies
5. Organize code for maintainability and reuse

---

## Prerequisites

- Completed Chapters 1.1, 1.2, and 1.3
- Understanding of ROS 2 nodes, launch files, and configuration
- Familiarity with C++ or Python

---

## Introduction

As your robotics projects grow, you'll need to organize your code into **ROS 2 packages**. A package is a self-contained unit that bundles:
- Executable nodes (Python/C++)
- Launch files
- Configuration files (YAML, URDF)
- Dependencies
- Documentation

Think of packages like Python modules or npm packages—they make code reusable, maintainable, and shareable across projects.

**Why Packages Matter**:
- **Modularity**: Separate concerns (e.g., vision, navigation, manipulation)
- **Reusability**: Share code across robots and teams
- **Dependency Management**: Automatic installation of required libraries
- **Distribution**: Publish to ROS ecosystem for others to use

---

## Key Terms

:::info Glossary Terms
- **Package**: Collection of related ROS 2 nodes, libraries, and configuration
- **Workspace**: Directory containing one or more ROS 2 packages
- **colcon**: Build tool for ROS 2 packages
- **package.xml**: Manifest file declaring package metadata and dependencies
- **CMakeLists.txt**: Build instructions for C++ packages
- **setup.py**: Build instructions for Python packages
:::

---

## Core Concepts

### 1. Package Structure

#### Python Package Layout
```
my_robot_pkg/
├── package.xml              # Package metadata
├── setup.py                 # Python build config
├── setup.cfg                # Entry points
├── my_robot_pkg/            # Python module
│   ├── __init__.py
│   ├── my_node.py           # Node implementations
│   └── utils.py             # Helper functions
├── launch/                  # Launch files
│   └── robot.launch.py
├── config/                  # YAML configs
│   └── params.yaml
└── resource/                # Resource marker
    └── my_robot_pkg
```

#### C++ Package Layout
```
my_robot_cpp/
├── package.xml              # Package metadata
├── CMakeLists.txt           # CMake build config
├── include/                 # Header files
│   └── my_robot_cpp/
│       └── my_node.hpp
├── src/                     # Source files
│   └── my_node.cpp
├── launch/
│   └── robot.launch.py
└── config/
    └── params.yaml
```

---

### 2. Creating Python Packages

#### Step 1: Generate Package Template
```bash
# Navigate to workspace src directory
cd ~/ros2_ws/src

# Create Python package
ros2 pkg create --build-type ament_python my_robot_pkg \
  --dependencies rclpy std_msgs geometry_msgs

# Package structure created automatically
```

**What this does**:
- Creates directory structure
- Generates `package.xml` with dependencies
- Creates `setup.py` and `setup.cfg`
- Adds resource marker file

#### Step 2: Implement a Node

**my_robot_pkg/my_robot_pkg/talker_node.py**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker_node')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 3: Configure Entry Points

**setup.py**:
```python
from setuptools import setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Example ROS 2 Python package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_robot_pkg.talker_node:main',
        ],
    },
)
```

**Key Points**:
- `packages`: List of Python modules
- `data_files`: Non-Python files (launch, config)
- `entry_points`: Create executable commands

---

### 3. Creating C++ Packages

#### Step 1: Generate C++ Package
```bash
cd ~/ros2_ws/src

ros2 pkg create --build-type ament_cmake my_robot_cpp \
  --dependencies rclcpp std_msgs geometry_msgs

# Creates C++ package structure
```

#### Step 2: Implement Node Header

**include/my_robot_cpp/talker_node.hpp**:
```cpp
#ifndef MY_ROBOT_CPP__TALKER_NODE_HPP_
#define MY_ROBOT_CPP__TALKER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace my_robot_cpp
{

class TalkerNode : public rclcpp::Node
{
public:
  TalkerNode();

private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  int counter_;
};

}  // namespace my_robot_cpp

#endif  // MY_ROBOT_CPP__TALKER_NODE_HPP_
```

#### Step 3: Implement Node Source

**src/talker_node.cpp**:
```cpp
#include "my_robot_cpp/talker_node.hpp"

namespace my_robot_cpp
{

TalkerNode::TalkerNode() : Node("talker_node"), counter_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&TalkerNode::timer_callback, this));
}

void TalkerNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello ROS 2: " + std::to_string(counter_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

}  // namespace my_robot_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<my_robot_cpp::TalkerNode>());
  rclcpp::shutdown();
  return 0;
}
```

#### Step 4: Configure CMakeLists.txt

**CMakeLists.txt**:
```cmake
cmake_minimum_required(VERSION 3.5)
project(my_robot_cpp)

# Compiler settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build executable
add_executable(talker src/talker_node.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# Install executable
install(TARGETS talker
  DESTINATION lib/${PROJECT_NAME})

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# Export dependencies
ament_package()
```

**Key CMake Commands**:
- `find_package`: Locate dependencies
- `add_executable`: Compile source to binary
- `ament_target_dependencies`: Link libraries
- `install`: Copy files to install space

---

### 4. Building with colcon

#### Workspace Setup
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Place packages in src/
# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

#### Build Options
```bash
# Build specific package
colcon build --packages-select my_robot_pkg

# Build with verbose output
colcon build --event-handlers console_direct+

# Build in parallel (4 jobs)
colcon build --parallel-workers 4

# Clean build
rm -rf build install log
colcon build
```

#### Testing Packages
```bash
# Run tests
colcon test

# View test results
colcon test-result --all
```

---

### 5. Package Dependencies

#### package.xml Structure
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_pkg</name>
  <version>0.0.1</version>
  <description>Example ROS 2 package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <!-- Build tool -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Dependency Types**:
- `buildtool_depend`: Required to build (e.g., cmake, ament_python)
- `depend`: Required at build and runtime
- `build_depend`: Only needed at build time
- `exec_depend`: Only needed at runtime
- `test_depend`: Required for testing

---

## Practical Examples

### Example 1: Complete Python Package

**Create Package**:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_controller \
  --dependencies rclpy geometry_msgs sensor_msgs
```

**Add Controller Node** (`robot_controller/controller.py`):
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.get_logger().info('Robot controller started')

    def scan_callback(self, msg):
        # Simple obstacle avoidance
        cmd = Twist()
        min_distance = min(msg.ranges)

        if min_distance < 0.5:  # Obstacle detected
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn
        else:
            cmd.linear.x = 0.2  # Move forward
            cmd.angular.z = 0.0

        self.cmd_publisher.publish(cmd)

def main():
    rclpy.init()
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
```

**Update setup.py entry_points**:
```python
entry_points={
    'console_scripts': [
        'controller = robot_controller.controller:main',
    ],
},
```

**Build and Run**:
```bash
cd ~/ros2_ws
colcon build --packages-select robot_controller
source install/setup.bash
ros2 run robot_controller controller
```

---

### Example 2: Multi-Package Workspace

**Workspace Structure**:
```
ros2_ws/
├── src/
│   ├── my_robot_description/   # URDF models
│   ├── my_robot_bringup/       # Launch files
│   ├── my_robot_control/       # Control nodes
│   └── my_robot_perception/    # Vision nodes
├── build/                      # Build artifacts
├── install/                    # Installed files
└── log/                        # Build logs
```

**Inter-Package Dependencies** (in `package.xml`):
```xml
<!-- my_robot_bringup depends on other packages -->
<depend>my_robot_description</depend>
<depend>my_robot_control</depend>
<depend>my_robot_perception</depend>
```

**Build Order**:
```bash
# colcon automatically resolves dependency order
colcon build

# Manually specify order if needed
colcon build --packages-up-to my_robot_bringup
```

---

### Example 3: Installing Launch and Config Files

**setup.py data_files**:
```python
import os
from glob import glob

data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # Install all launch files
    (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
    # Install all config files
    (os.path.join('share', package_name, 'config'),
        glob('config/*.yaml')),
    # Install URDF files
    (os.path.join('share', package_name, 'urdf'),
        glob('urdf/*.urdf')),
]
```

---

## Best Practices

### 1. Package Naming
```bash
# Good
my_robot_navigation
my_robot_perception

# Bad
MyRobotNavigation   # CamelCase not recommended
my-robot-nav        # Hyphens not allowed
```

### 2. One Package, One Purpose
```bash
# Good: Separate packages
camera_driver/       # Hardware interface
image_processing/    # Vision algorithms
object_detection/    # ML inference

# Bad: Everything in one package
robot_vision/        # Too broad
```

### 3. Dependency Management
```xml
<!-- Use specific versions when needed -->
<depend version_gte="2.0.0">sensor_msgs</depend>

<!-- Prefer <depend> over separate build/exec_depend -->
<depend>rclpy</depend>
```

### 4. Workspace Hygiene
```bash
# Add to .gitignore
build/
install/
log/
*.pyc
__pycache__/
```

---

## Debugging Package Issues

### 1. Package Not Found
```bash
# Check if package is built
colcon list

# Source workspace
source install/setup.bash

# Verify package is in ROS_PACKAGE_PATH
ros2 pkg list | grep my_package
```

### 2. Dependency Errors
```bash
# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Check package.xml syntax
xmllint --noout package.xml
```

### 3. Build Failures
```bash
# Clean and rebuild
rm -rf build install log
colcon build --packages-select my_package --event-handlers console_direct+

# Check compiler errors in log/
cat log/latest_build/my_package/stdout.log
```

---

## Summary

In this chapter, you learned:

- ✅ ROS 2 packages organize code into reusable modules
- ✅ Python packages use `setup.py`, C++ uses `CMakeLists.txt`
- ✅ `package.xml` declares metadata and dependencies
- ✅ `colcon` builds workspaces with multiple packages
- ✅ Entry points make nodes executable with `ros2 run`
- ✅ Proper structure enables code sharing and collaboration

**Key Takeaway**: Packages are the fundamental unit of ROS 2 software organization. Mastering package creation is essential for building scalable robotic systems.

---

## End-of-Chapter Exercises

### Exercise 1: Create Temperature Monitor Package (Difficulty: Easy)

Create a Python package `temperature_monitor` with:
1. Node that publishes random temperature values (20-30°C) every second
2. Another node that subscribes and warns if temperature > 28°C
3. Launch file to start both nodes

**Hints**:
- Use `std_msgs/Float32` for temperature
- Use `random.uniform()` for temperature generation

---

### Exercise 2: Multi-Language Package (Difficulty: Medium)

Create a workspace with:
1. C++ package `sensor_driver` with a publisher node
2. Python package `data_processor` with a subscriber node
3. Make the Python package depend on the C++ package

**Challenge**: Ensure the Python node can only start after the C++ node.

---

### Exercise 3: Package with Tests (Difficulty: Hard)

Extend the temperature monitor with:
1. Unit tests for temperature validation logic
2. Integration tests for pub/sub communication
3. Run tests with `colcon test` and view results

**Bonus**: Add CI/CD with GitHub Actions to run tests on every commit.

---

## Further Reading

### Required
1. [ROS 2 Packages Tutorial](https://docs.ros.org/en/humble/Tutorials/Creating-Your-First-ROS2-Package.html)
2. [colcon Documentation](https://colcon.readthedocs.io/)
3. [ament_cmake User Guide](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html)

### Optional
- [Python setup.py Best Practices](https://setuptools.pypa.io/en/latest/userguide/quickstart.html)
- [CMake Tutorial](https://cmake.org/cmake/help/latest/guide/tutorial/index.html)
- [ROS 2 Package Quality Guidelines](https://docs.ros.org/en/humble/Contributing/Developer-Guide.html)

---

## Next Chapter

Continue to **[Module 2: The Digital Twin](../module2/intro)** to learn about simulation environments.

**🎓 Module 1 Complete!** You now understand the complete ROS 2 development workflow from fundamentals to package management.

---

**Pro Tip**: Use `ros2 pkg create` templates to bootstrap new packages quickly, then customize as needed. Consistent package structure makes collaboration easier!
