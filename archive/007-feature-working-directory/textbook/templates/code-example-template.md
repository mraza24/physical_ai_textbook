# Code Example Template

## Overview

**Purpose**: [1-2 sentence description of what this code demonstrates]

**Concepts Demonstrated**:
- [Concept 1]
- [Concept 2]
- [Concept 3]

---

## Prerequisites

### Software Requirements
- **Language**: Python 3.10+ (or C++ with ROS 2)
- **ROS 2**: Humble Hawksbill
- **Dependencies**:
  ```bash
  pip install rclpy [other-packages]
  ```

### Hardware Requirements (if applicable)
- GPU: [e.g., RTX 4070 Ti+ for Isaac examples]
- Sensors: [e.g., RealSense D435 for vision examples]

---

## Code

### File: `[filename].py`

```python
#!/usr/bin/env python3
"""
[Brief module docstring explaining what this script does]
"""

import rclpy
from rclpy.node import Node
# [Other imports]

class ExampleNode(Node):
    """
    [Class docstring explaining the node's purpose]
    """

    def __init__(self):
        super().__init__('example_node')
        # [Initialization code with inline comments]

    def callback_function(self, msg):
        """
        [Function docstring]
        """
        # [Implementation with clear comments]
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Step-by-Step Explanation

### Step 1: [First major step]

**Code**:
```python
[Relevant code snippet]
```

**Explanation**: [What this code does and why]

### Step 2: [Second step]

**Code**:
```python
[Next code snippet]
```

**Explanation**: [Explanation]

[Continue for all major steps]

---

## Running the Example

### Terminal 1: Start ROS 2 Core (if needed)
```bash
ros2 launch [package] [launch_file]
```

### Terminal 2: Run the Example
```bash
python3 example_node.py
```

### Expected Output

```
[INFO] [timestamp]: Node started
[INFO] [timestamp]: Received message: ...
```

---

## Troubleshooting

**Issue**: [Common problem]
**Solution**: [How to fix]

**Issue**: [Another problem]
**Solution**: [Resolution]

---

## Further Exploration

- Try modifying [parameter] to [expected effect]
- Extend the example by [suggestion]
- Combine with [other example] to [achieve something more complex]
