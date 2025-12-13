# Code Example Template

**Task**: 028
**Purpose**: Standard format for all code examples in textbook

---

## Code Block Structure

````markdown
### Example [Number]: [Title]

**Scenario**: [Brief description of what this code demonstrates]

**File**: `[filename.ext]` (optional, if this is a complete file)

```[language]
# [Language: Python/C++/YAML/Bash/etc.]
# Purpose: [What this code does]
# Requirements: [Dependencies, packages needed]

# [Section 1 Description]
import necessary_module

# [Section 2 Description - explain each major block]
def example_function():
    """
    Docstring explaining function purpose, parameters, returns.
    """
    # Implementation with inline comments
    pass

# [Section 3 Description]
if __name__ == '__main__':
    # Entry point explanation
    example_function()
```

**Explanation**:
1. [Step-by-step explanation of code flow]
2. [Key concepts demonstrated]
3. [Common pitfalls to avoid]

**Expected Output**:
```
[Show what the user should see when running this code]
```

**Try It Yourself**:
- [Modification 1 students can try]
- [Modification 2 students can try]
- [Expected outcome of modifications]
````

---

## Language-Specific Guidelines

### Python

````python
#!/usr/bin/env python3
"""
Module docstring: Brief description of file purpose.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ExampleNode(Node):
    """Class docstring explaining the node's purpose."""

    def __init__(self):
        super().__init__('example_node')
        self.publisher = self.create_publisher(String, 'example_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Node initialized')

    def timer_callback(self):
        """Callback function documentation."""
        msg = String()
        msg.data = 'Hello from ROS 2'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
````

### C++

````cpp
/**
 * @file example_node.cpp
 * @brief Brief description of file purpose
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ExampleNode : public rclcpp::Node
{
public:
    ExampleNode() : Node("example_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("example_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ExampleNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Node initialized");
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello from ROS 2";
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExampleNode>());
    rclcpp::shutdown();
    return 0;
}
````

### YAML Configuration

````yaml
# launch/example_config.yaml
# Purpose: Configuration file for example node

/**:
  ros__parameters:
    # Node-specific parameters
    publish_rate: 10.0  # Hz
    topic_name: "example_topic"

    # QoS settings
    qos_reliability: "reliable"  # Options: reliable, best_effort
    qos_history_depth: 10

    # Optional parameters with defaults
    enable_logging: true
    log_level: "info"  # Options: debug, info, warn, error
````

### Bash Scripts

````bash
#!/bin/bash
# Script: example_setup.sh
# Purpose: Set up environment for example

# Exit on error
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build workspace
cd ~/ros2_ws
colcon build --packages-select example_package

# Source workspace
source install/setup.bash

# Run example node
ros2 run example_package example_node
````

---

## Code Testing Requirements

All code examples MUST:
1. ✅ Be syntactically correct
2. ✅ Run without errors on target platform (Ubuntu 22.04, ROS 2 Humble)
3. ✅ Include all necessary imports/includes
4. ✅ Have clear comments explaining each section
5. ✅ Include expected output or behavior
6. ✅ Follow ROS 2 coding standards (for ROS code)
7. ✅ Be self-contained (or clearly state dependencies)

---

## Common Code Example Types

### Type 1: Minimal Example
- **Purpose**: Demonstrate single concept
- **Length**: 10-30 lines
- **Complexity**: Beginner
- **Example**: Simple publisher node

### Type 2: Practical Example
- **Purpose**: Real-world use case
- **Length**: 30-100 lines
- **Complexity**: Intermediate
- **Example**: Camera subscriber with image processing

### Type 3: Full Application
- **Purpose**: Complete system demonstration
- **Length**: 100-300 lines (or multiple files)
- **Complexity**: Advanced
- **Example**: Full VLA pipeline

---

## Repository Structure for Code Examples

```
textbook/
├── code-examples/
│   ├── module1/
│   │   ├── chapter1-1/
│   │   │   ├── simple_publisher.py
│   │   │   ├── simple_subscriber.py
│   │   │   └── README.md
│   │   ├── chapter1-2/
│   │   └── ...
│   ├── module2/
│   ├── module3/
│   └── module4/
└── docs/
```

---

## Code Formatting Standards

### Python
- **Style Guide**: PEP 8
- **Line Length**: 88 characters (Black formatter)
- **Indentation**: 4 spaces
- **Imports**: Grouped (standard lib, third-party, local)

### C++
- **Style Guide**: ROS 2 C++ Style Guide
- **Line Length**: 100 characters
- **Indentation**: 2 spaces
- **Naming**: snake_case for variables, PascalCase for classes

### YAML
- **Indentation**: 2 spaces
- **No tabs**: Use spaces only
- **Comments**: Explain non-obvious parameters

---

**Status**: ✅ Template ready for all code examples
