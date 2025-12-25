# Chapter 1.4: Building Packages & Workspaces

> **Module**: 1 - Robotic Nervous System (ROS 2)
> **Week**: 5 (Part 2)
> **Estimated Reading Time**: 45 minutes

---

## Summary

This chapter introduces the ROS 2 build system—workspaces for organizing code, packages for modular distribution, and colcon for compilation—providing the foundation for creating, building, and sharing reusable robot software. Understanding these concepts is essential for moving from single-file scripts to professional, maintainable robot systems.

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Understand** the four-directory workspace structure (src/, build/, install/, log/)
2. **Create** ROS 2 packages with proper manifests (package.xml) and build configurations
3. **Build** workspaces using colcon with appropriate flags and options
4. **Manage** package dependencies and ensure reproducible builds
5. **Use** overlay workspaces to extend functionality without modifying base installations

**Prerequisite Knowledge**: Chapter 1.1-1.3 (ROS 2 fundamentals, nodes, launch files), basic Linux command-line skills

---

## Key Terms

This chapter introduces the following technical terms (see Glossary for detailed definitions):

- **Workspace**: Top-level directory containing source code, build artifacts, and installed packages
- **Package**: Smallest unit of ROS 2 software distribution, containing nodes, launch files, and configuration
- **colcon**: Command-line build tool for compiling ROS 2 workspaces
- **package.xml**: Manifest file declaring package metadata, dependencies, and licensing
- **Overlay**: Workspace layered on top of another (underlay), allowing extensions without modification
- **Underlay**: Base workspace (e.g., `/opt/ros/humble/`) that provides foundational packages
- **Source Space (src/)**: Directory containing package source code
- **Install Space (install/)**: Directory containing compiled executables and resources ready for use

---

## Core Concepts

### 1. The Workspace: Organizing Robot Software

A **ROS 2 workspace** is a directory containing everything needed to build, install, and run robot software. Unlike throwing Python files in arbitrary locations, workspaces provide structure, reproducibility, and isolation.

**Why workspaces matter**:
- **Isolation**: Different projects don't interfere with each other
- **Reproducibility**: Same source + same dependencies = same build
- **Distribution**: Package source code separately from binaries
- **Development workflow**: Edit source → build → test → iterate

### 2. Workspace Anatomy: The Four Directories

Every ROS 2 workspace follows a standard structure:

```
my_robot_ws/
├── src/              # SOURCE SPACE (your code)
│   ├── package_a/
│   ├── package_b/
│   └── package_c/
├── build/            # BUILD SPACE (intermediate files)
│   ├── package_a/
│   ├── package_b/
│   └── package_c/
├── install/          # INSTALL SPACE (compiled binaries)
│   ├── setup.bash    # Source this to use workspace
│   ├── package_a/
│   ├── package_b/
│   └── package_c/
└── log/              # LOG SPACE (build logs)
    └── build_2025-12-11_10-30-45/
```

**Directory roles**:

#### src/ (Source Space)
- **Contains**: Your package source code (Python, C++, launch files, config)
- **Version controlled**: Commit `src/` to Git, ignore other directories
- **Human-editable**: You write and edit files here

#### build/ (Build Space)
- **Contains**: Intermediate build artifacts (object files, CMake cache)
- **Generated**: Created by colcon, never edit manually
- **Gitignored**: Don't commit—regenerated on each build

#### install/ (Install Space)
- **Contains**: Compiled executables, libraries, resources
- **Used at runtime**: Source `install/setup.bash` to run nodes
- **Gitignored**: Binary artifacts, not source code

#### log/ (Log Space)
- **Contains**: Build logs, test results, timestamps
- **Debugging**: Check logs when builds fail
- **Gitignored**: Logs are ephemeral

**Key insight**: Only `src/` is source-controlled. Everything else is generated from `src/` and can be recreated with `colcon build`.

### 3. Packages: The Building Blocks

A **package** is the smallest redistributable unit of ROS 2 software. Each package contains:
- **Nodes**: Executable programs (Python scripts or C++ binaries)
- **Launch files**: System configuration
- **Config files**: Parameters, URDF models, etc.
- **Dependencies**: Declared in `package.xml`

**Package directory structure** (Python package example):

```
my_package/
├── package.xml          # Manifest (required)
├── setup.py             # Python build config (required for Python)
├── setup.cfg            # Python entry points
├── resource/            # Empty marker directory
│   └── my_package
├── my_package/          # Python source code
│   ├── __init__.py
│   ├── my_node.py       # Node implementation
│   └── submodule.py
├── launch/              # Launch files
│   └── my_launch.py
├── config/              # Parameter files
│   └── params.yaml
└── test/                # Unit tests
    └── test_my_node.py
```

**Two types of packages**:
1. **Python packages**: Use `setup.py`, easier for rapid development
2. **C++ packages**: Use `CMakeLists.txt`, better performance for compute-intensive nodes

This chapter focuses on Python packages (simpler for learning); C++ follows similar principles.

### 4. The package.xml Manifest

Every package requires `package.xml`—an XML file declaring metadata and dependencies.

**Minimal package.xml**:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_driver</name>
  <version>1.0.0</version>
  <description>Driver for my custom robot hardware</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <!-- Build tool (required) -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Dependencies (libraries this package needs) -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

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

**Key sections**:
- `<name>`: Package name (must match directory name)
- `<version>`: Semantic versioning (MAJOR.MINOR.PATCH)
- `<description>`: One-sentence summary
- `<maintainer>`: Contact for bug reports
- `<license>`: Software license (MIT, Apache 2.0, BSD, GPL, etc.)
- `<buildtool_depend>`: Build system (ament_python or ament_cmake)
- `<depend>`: Runtime dependencies (packages this package imports/uses)
- `<test_depend>`: Testing libraries (pytest, linters)

**Dependency types**:
- `<depend>`: Both build and runtime dependency (most common)
- `<build_depend>`: Only needed during compilation
- `<exec_depend>`: Only needed at runtime
- `<test_depend>`: Only needed for testing

### 5. Creating a Package from Scratch

**Step 1: Create workspace**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

**Step 2: Create package**

```bash
ros2 pkg create --build-type ament_python my_robot_controller \
  --dependencies rclpy std_msgs sensor_msgs \
  --node-name controller_node
```

This command:
- Creates package directory `my_robot_controller/`
- Generates `package.xml` with specified dependencies
- Creates `setup.py`, `setup.cfg`, and directory structure
- Adds a starter node `controller_node.py`

**Generated structure**:

```
my_robot_controller/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── my_robot_controller
├── my_robot_controller/
│   ├── __init__.py
│   └── controller_node.py
└── test/
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

**Step 3: Edit node code**

Open `my_robot_controller/controller_node.py` and implement your node (see Chapter 1.1 for node structure).

**Step 4: Declare entry points**

Edit `setup.py` to make your node executable:

```python
from setuptools import setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch', ['launch/my_launch.py']),
        # Install config files
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Robot controller package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = my_robot_controller.controller_node:main',
        ],
    },
)
```

**Key sections**:
- `entry_points`: Maps executable name (`controller_node`) to Python function (`main()` in `controller_node.py`)
- `data_files`: Installs launch files and configs to install space

### 6. Building with colcon

**colcon** is ROS 2's build tool, replacing ROS 1's catkin.

**Basic build**:

```bash
cd ~/ros2_ws
colcon build
```

This:
1. Discovers all packages in `src/`
2. Resolves dependency order (builds dependencies first)
3. Compiles each package (Python or C++)
4. Installs to `install/`
5. Generates setup scripts (`install/setup.bash`)

**Common colcon options**:

```bash
# Build specific package
colcon build --packages-select my_robot_controller

# Build with more verbose output
colcon build --event-handlers console_direct+

# Build in parallel (faster on multi-core)
colcon build --parallel-workers 4

# Build in debug mode (for C++ packages)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Build and run tests
colcon build && colcon test

# Clean build (start fresh)
rm -rf build/ install/ log/
colcon build
```

**After building, source the workspace**:

```bash
source ~/ros2_ws/install/setup.bash
```

Now your nodes are available:

```bash
ros2 run my_robot_controller controller_node
ros2 launch my_robot_controller my_launch.py
```

**Key Points**:
- Always source `install/setup.bash` after building
- Add `source ~/ros2_ws/install/setup.bash` to `~/.bashrc` to avoid repeating
- Rebuild after editing Python code (for entry points)—or use `--symlink-install` for development

**Development tip**: Use `--symlink-install` to avoid rebuilding after every Python edit:

```bash
colcon build --symlink-install
```

This creates symlinks from `install/` to `src/`, so changes take effect immediately without rebuilding (Python only, C++ still requires rebuild).

### 7. Dependency Management

**Problem**: Your package uses `sensor_msgs/Image`. How does the build system know to find it?

**Solution**: Declare dependencies in `package.xml`:

```xml
<depend>sensor_msgs</depend>
```

When you run `colcon build`, the system:
1. Checks that `sensor_msgs` is available (from underlay or workspace)
2. Builds your package after `sensor_msgs` is built (if in same workspace)
3. Configures import paths so `from sensor_msgs.msg import Image` works

**Dependency resolution order**:
1. Packages in current workspace (`src/`)
2. Underlay workspace (e.g., parent workspace)
3. ROS 2 installation (`/opt/ros/humble/`)

**Finding missing dependencies**:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

This command:
- Reads all `package.xml` files in `src/`
- Finds system dependencies (apt packages, pip packages)
- Installs missing dependencies automatically

**Example**: If your package depends on `opencv`:

```xml
<!-- In package.xml -->
<depend>cv_bridge</depend>
<exec_depend>python3-opencv</exec_depend>
```

Running `rosdep install` will `apt install ros-humble-cv-bridge python3-opencv`.

### 8. Overlay Workspaces: Extending Without Modifying

**Scenario**: You want to modify the `turtlesim` package (from ROS 2 installation) without reinstalling ROS 2.

**Solution**: Create an **overlay workspace**:

```bash
# Create overlay workspace
mkdir -p ~/turtlesim_ws/src
cd ~/turtlesim_ws/src

# Clone turtlesim source
git clone https://github.com/ros/ros_tutorials.git -b humble

# Build overlay
cd ~/turtlesim_ws
colcon build --packages-select turtlesim

# Source overlay (this "overlays" the ROS 2 installation)
source ~/turtlesim_ws/install/setup.bash
```

Now when you run `ros2 run turtlesim turtlesim_node`, it uses **your modified version** from the overlay, not the system version.

**Overlay chain**:

```
Your overlay         ~/turtlesim_ws/install/setup.bash
       ↓ extends
ROS 2 installation   /opt/ros/humble/setup.bash
       ↓ extends
System libraries     /usr/lib/...
```

**Sourcing order matters**:

```bash
# CORRECT: Source underlay first, then overlay
source /opt/ros/humble/setup.bash
source ~/my_ws/install/setup.bash

# WRONG: Overlay won't extend underlay
source ~/my_ws/install/setup.bash
source /opt/ros/humble/setup.bash  # This overwrites, doesn't extend
```

**Use cases for overlays**:
- Modifying upstream packages without forking
- Testing patches before contributing upstream
- Project-specific customizations of standard packages
- Multi-project development (each project is an overlay)

---

## Practical Example: Creating a Multi-Package Workspace

### Overview

This example demonstrates creating a workspace with two packages: a driver package (publishes sensor data) and a processor package (subscribes and processes data), showing inter-package dependencies.

### Prerequisites

- Software: ROS 2 Humble, Python 3.10+
- Knowledge: Chapters 1.1-1.3 (nodes, topics, launch files)

### Implementation

**Step 1: Create workspace and packages**

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Package 1: Sensor driver
ros2 pkg create --build-type ament_python robot_driver \
  --dependencies rclpy sensor_msgs \
  --node-name driver_node

# Package 2: Data processor (depends on robot_driver messages)
ros2 pkg create --build-type ament_python robot_processor \
  --dependencies rclpy sensor_msgs \
  --node-name processor_node
```

**Step 2: Implement driver node**

Edit `robot_driver/robot_driver/driver_node.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import random

class DriverNode(Node):
    def __init__(self):
        super().__init__('driver_node')
        self.publisher = self.create_publisher(Temperature, 'sensor/temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)
        self.get_logger().info('Driver node started')

    def publish_temperature(self):
        msg = Temperature()
        msg.temperature = 20.0 + random.uniform(-2.0, 2.0)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.temperature:.2f}°C')

def main():
    rclpy.init()
    node = DriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Step 3: Implement processor node**

Edit `robot_processor/robot_processor/processor_node.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature

class ProcessorNode(Node):
    def __init__(self):
        super().__init__('processor_node')
        self.subscription = self.create_subscription(
            Temperature,
            'sensor/temperature',
            self.temperature_callback,
            10
        )
        self.readings = []
        self.get_logger().info('Processor node started')

    def temperature_callback(self, msg):
        temp = msg.temperature
        self.readings.append(temp)
        if len(self.readings) > 10:
            self.readings.pop(0)
        avg = sum(self.readings) / len(self.readings)
        self.get_logger().info(f'Received: {temp:.2f}°C | Avg: {avg:.2f}°C')

def main():
    rclpy.init()
    node = ProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Step 4: Update entry points**

Ensure `setup.py` in both packages has correct entry points (auto-generated if you used `--node-name` flag).

**Step 5: Build workspace**

```bash
cd ~/robot_ws
colcon build --symlink-install
```

**Step 6: Source and run**

```bash
source ~/robot_ws/install/setup.bash

# Terminal 1
ros2 run robot_driver driver_node

# Terminal 2
ros2 run robot_processor processor_node
```

### Expected Output

**Terminal 1 (driver)**:
```
[driver_node]: Driver node started
[driver_node]: Published: 21.34°C
[driver_node]: Published: 19.87°C
...
```

**Terminal 2 (processor)**:
```
[processor_node]: Processor node started
[processor_node]: Received: 21.34°C | Avg: 21.34°C
[processor_node]: Received: 19.87°C | Avg: 20.61°C
...
```

---

## Figures & Diagrams

### Figure 1.4-1: ROS 2 Workspace Structure

![Workspace Structure](../../diagrams/module1/fig1.4-workspace-structure.svg)

**Caption**: Directory tree showing the four-directory workspace structure. Source code lives in `src/`, intermediate build artifacts in `build/`, compiled binaries in `install/`, and logs in `log/`. Only `src/` is version-controlled; other directories are generated by `colcon build`.

---

## Exercises

### Exercise 1: Create Your First Package (Difficulty: Easy)

**Objective**: Practice package creation and building

**Task**: Create a package called `hello_robot` with a node that prints "Hello, Robot World!" every 2 seconds.

**Requirements**:
- Use `ros2 pkg create` with Python build type
- Implement timer-based printing node
- Build workspace and run node
- Verify output

**Expected Outcome**:
```bash
$ ros2 run hello_robot hello_node
[hello_node]: Hello, Robot World!
[hello_node]: Hello, Robot World!
...
```

**Estimated Time**: 20 minutes

---

### Exercise 2: Multi-Package Dependency (Difficulty: Medium)

**Objective**: Understand inter-package dependencies

**Task**: Create two packages where package B depends on package A:
- **Package A**: Defines a custom message type (or uses existing type) and publishes data
- **Package B**: Subscribes to data from package A and processes it

**Requirements**:
- Declare dependency in package B's `package.xml`
- Build both packages in correct order
- Verify package B finds package A's messages

**Expected Outcome**: Package B successfully imports and subscribes to package A's topics.

**Estimated Time**: 35 minutes

---

### Exercise 3: Overlay Workspace Modification (Difficulty: Hard)

**Objective**: Learn overlay workspace mechanics

**Task**: Create an overlay that modifies a standard ROS 2 package (e.g., `demo_nodes_cpp`'s talker to publish at different frequency).

**Requirements**:
- Clone source of upstream package into overlay workspace
- Modify source code (change timer period)
- Build overlay
- Verify your modified version is used when sourcing overlay
- Verify original version is used without sourcing overlay

**Expected Outcome**: Understand how overlays extend underlays without modifying system installation.

**Estimated Time**: 40 minutes

---

## Summary & Key Takeaways

In this chapter, you learned:

- **Workspaces** organize ROS 2 projects with four directories (src/, build/, install/, log/)
- **Packages** are the fundamental unit of ROS 2 software, containing nodes, launch files, and configuration
- **package.xml** declares metadata and dependencies for reproducible builds
- **colcon** is the build tool that compiles workspaces and resolves dependencies
- **Overlay workspaces** allow extending functionality without modifying base installations
- **Dependency management** via `package.xml` and `rosdep` ensures all requirements are met

**Connection to Module 2**: With ROS 2 fundamentals complete (nodes, communication, launch files, packages), Module 2 introduces **Digital Twin Simulation**—testing your ROS 2 systems in virtual environments (Gazebo, Unity) before deploying to physical hardware.

---

## Additional Resources

### Official Documentation
- **colcon Documentation**: https://colcon.readthedocs.io/
- **ROS 2 Package Creation**: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
- **Workspace Tutorial**: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html

### Recommended Reading
- **ament Build System**: ROS 2 build system architecture
- **rosdep Documentation**: Dependency installation tool

### Community Resources
- **colcon GitHub**: Source code and issue tracker
- **ROS 2 Workspace Examples**: Community-contributed workspace templates

---

## Notes for Instructors

**Teaching Tips**:
- **Live demo**: Create package from scratch, build it, run node—shows complete workflow in 5 minutes
- **Common mistake**: Students forget to source `install/setup.bash` after building, nodes aren't found—emphasize this step

**Lab Exercise Ideas**:
- **Dependency chain**: Create 3 packages (A→B→C) where each depends on the previous, verify build order
- **Overlay experiment**: Modify `turtlesim` color, build overlay, compare to original

**Assessment Suggestions**:
- **Package quality**: Check `package.xml` completeness (description, maintainer, license, dependencies)
- **Build reproducibility**: Can another student clone repo and `colcon build` successfully?

---

**Chapter Metadata**:
- **Word Count**: ~2800 words (core concepts)
- **Figures**: 1 (workspace structure diagram)
- **Code Examples**: 5 (package.xml, setup.py, driver/processor nodes, colcon commands)
- **Exercises**: 3 (easy, medium, hard)
- **Glossary Terms**: 8
