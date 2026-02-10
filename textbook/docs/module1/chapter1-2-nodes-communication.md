---
sidebar_position: 3
title: Chapter 1.2 - ROS 2 Nodes and Communication
---

# Chapter 1.2: ROS 2 Nodes and Communication

**Module**: 1 - The Robotic Nervous System
**Week**: 2
**Estimated Reading Time**: 35 minutes

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. Implement services for request-response communication patterns
2. Create action servers for long-running tasks with feedback
3. Understand and manage node lifecycle states
4. Build multi-node distributed systems with proper coordination
5. Debug communication issues using ROS 2 tools

---

## Prerequisites

- Completed Chapter 1.1 (ROS 2 Fundamentals)
- Understanding of publishers and subscribers
- Familiarity with ROS 2 command-line tools

---

## Introduction

In Chapter 1.1, you learned about topics for continuous data streaming. But robots need more than just publish-subscribe patterns. Consider these scenarios:

- **Robot Arm**: "Move to position (x, y, z)" → Wait for completion confirmation
- **Path Planner**: "Calculate route from A to B" → Receive the computed path
- **Battery Monitor**: Request current charge level → Get immediate response

These require **synchronous request-response** (services) and **goal-based execution with feedback** (actions). This chapter covers advanced communication patterns that enable coordinated multi-node systems.

---

## Key Terms

:::info Glossary Terms
- **Service**: Synchronous request-response communication pattern
- **Action**: Asynchronous goal-based communication with feedback
- **Lifecycle Node**: Managed node with explicit state transitions
- **Node Composition**: Running multiple nodes in a single process
- **Service Server**: Node that provides a service (handles requests)
- **Service Client**: Node that calls a service (sends requests)
:::

---

## Core Concepts

### 1. Services for Request-Response

Services enable one-to-one synchronous communication. Unlike topics (one-to-many, continuous), services are **called only when needed** and **block until response**.

#### Service Architecture
```
Client Node                    Server Node
    |                              |
    |------- Request ------→|
    |                              |
    |                         [Process]
    |                              |
    |←------ Response ------|
```

#### Example: Add Two Integers Service

**Server (Python)**:
```python
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        self.get_logger().info('Add Two Ints Server ready')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response

def main():
    rclpy.init()
    node = AddTwoIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Client (Python)**:
```python
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result: {future.result().sum}')
            return future.result().sum
        else:
            self.get_logger().error('Service call failed')
            return None

def main():
    rclpy.init()
    client = AddTwoIntsClient()
    result = client.send_request(5, 7)
    client.destroy_node()
    rclpy.shutdown()
```

#### Running Services
```bash
# Terminal 1: Start server
ros2 run my_package add_two_ints_server

# Terminal 2: Start client
ros2 run my_package add_two_ints_client

# Terminal 3: Call service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 7}"
```

#### Service Introspection
```bash
# List all services
ros2 service list

# Get service type
ros2 service type /add_two_ints

# View service interface
ros2 interface show example_interfaces/srv/AddTwoInts
```

---

### 2. Actions for Long-Running Tasks

Actions are for **asynchronous** tasks that:
- Take time to complete (e.g., navigation, grasping)
- Provide **periodic feedback** (e.g., progress updates)
- Can be **canceled** mid-execution

#### Action Architecture
```
Client Node                    Server Node
    |                              |
    |------- Goal -------→|
    |←---- Goal Accepted --|
    |                              |
    |←----- Feedback -------|  (periodic updates)
    |←----- Feedback -------|
    |                              |
    |←----- Result ---------|  (final outcome)
```

#### Example: Fibonacci Action

**Action Definition** (`Fibonacci.action`):
```
# Goal: Generate Fibonacci sequence up to order N
int32 order
---
# Result: The full sequence
int32[] sequence
---
# Feedback: Partial sequence during computation
int32[] partial_sequence
```

**Action Server (Python)**:
```python
import time
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        self.get_logger().info('Fibonacci Action Server ready')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: order={goal_handle.request.order}')

        # Initialize feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Generate Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Compute next number
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1]
            )

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)  # Simulate work

        # Mark as succeeded and return result
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info(f'Result: {result.sequence}')
        return result

def main():
    rclpy.init()
    node = FibonacciActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Action Client (Python)**:
```python
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Final result: {result.sequence}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.partial_sequence}')

def main():
    rclpy.init()
    client = FibonacciActionClient()
    client.send_goal(10)
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()
```

#### Running Actions
```bash
# Terminal 1: Start action server
ros2 run my_package fibonacci_action_server

# Terminal 2: Start action client
ros2 run my_package fibonacci_action_client

# Terminal 3: Monitor action
ros2 action list
ros2 action info /fibonacci
```

---

### 3. Node Lifecycle Management

Lifecycle nodes have **explicit state transitions**, enabling coordinated startup/shutdown and error handling.

#### Lifecycle States
```
┌─────────────────────────────────────┐
│         Unconfigured                │
│      (initial state)                │
└─────────────────────────────────────┘
           ↓ configure()
┌─────────────────────────────────────┐
│          Inactive                   │
│  (configured but not running)       │
└─────────────────────────────────────┘
           ↓ activate()
┌─────────────────────────────────────┐
│           Active                    │
│      (fully operational)            │
└─────────────────────────────────────┘
           ↓ deactivate()
           (back to Inactive)
           ↓ cleanup()
           (back to Unconfigured)
```

#### Example: Lifecycle Node (Python)
```python
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_lifecycle_node')

    def on_configure(self, state):
        self.get_logger().info('Configuring...')
        # Initialize resources (e.g., open files, connect to hardware)
        self.publisher = self.create_lifecycle_publisher(String, 'output', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating...')
        # Start processing (e.g., begin publishing)
        self.timer = self.create_timer(1.0, self.timer_callback)
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating...')
        # Pause processing
        self.timer.cancel()
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up...')
        # Release resources
        self.destroy_timer(self.timer)
        self.destroy_publisher(self.publisher)
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from lifecycle node'
        self.publisher.publish(msg)
```

#### Managing Lifecycle Nodes
```bash
# Get current state
ros2 lifecycle get /my_lifecycle_node

# Transition to configured
ros2 lifecycle set /my_lifecycle_node configure

# Transition to active
ros2 lifecycle set /my_lifecycle_node activate

# Transition to inactive
ros2 lifecycle set /my_lifecycle_node deactivate

# Clean up and return to unconfigured
ros2 lifecycle set /my_lifecycle_node cleanup
```

---

### 4. Multi-Node Distributed Systems

Real robots run **multiple nodes** coordinating via services, actions, and topics.

#### Example: Robot Patrol System

**Architecture**:
```
┌──────────────┐
│ Patrol Manager│ (Action Server)
└──────┬───────┘
       │ calls
┌──────▼───────┐      publishes     ┌────────────┐
│Path Planner  │─────────────────→│ cmd_vel    │
│ (Service)    │                   │ (Topic)    │
└──────────────┘                   └────────────┘
       ↑ requests                         │
       │                                  ↓
┌──────┴───────┐                   ┌────────────┐
│   Navigator  │←─────subscribes───│  Odometry  │
│              │                   │  (Topic)   │
└──────────────┘                   └────────────┘
```

**Patrol Manager Node**:
```python
class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')

        # Action server for patrol mission
        self._action_server = ActionServer(
            self, Patrol, 'patrol', self.execute_patrol
        )

        # Service client for path planning
        self.planner_client = self.create_client(
            ComputePathToPose, 'compute_path'
        )

        # Action client for navigation
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

    def execute_patrol(self, goal_handle):
        waypoints = goal_handle.request.waypoints

        for i, waypoint in enumerate(waypoints):
            # Request path from planner
            path = self.call_planner(waypoint)

            # Execute navigation
            self.navigate_to(waypoint)

            # Publish feedback
            feedback = Patrol.Feedback()
            feedback.current_waypoint = i
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        return Patrol.Result(success=True)
```

---

## Practical Examples

### Example 1: Image Processing Service

**Service Definition** (`ProcessImage.srv`):
```
sensor_msgs/Image input_image
string processing_type  # "blur", "edge_detect", "grayscale"
---
sensor_msgs/Image output_image
bool success
string message
```

**Server Implementation**:
```python
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from my_interfaces.srv import ProcessImage

class ImageProcessingServer(Node):
    def __init__(self):
        super().__init__('image_processing_server')
        self.srv = self.create_service(
            ProcessImage,
            'process_image',
            self.process_callback
        )
        self.bridge = CvBridge()

    def process_callback(self, request, response):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(request.input_image, 'bgr8')

        # Apply processing
        if request.processing_type == 'blur':
            processed = cv2.GaussianBlur(cv_image, (15, 15), 0)
        elif request.processing_type == 'edge_detect':
            processed = cv2.Canny(cv_image, 50, 150)
        elif request.processing_type == 'grayscale':
            processed = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        else:
            response.success = False
            response.message = f'Unknown processing type: {request.processing_type}'
            return response

        # Convert back to ROS Image
        response.output_image = self.bridge.cv2_to_imgmsg(processed, 'bgr8')
        response.success = True
        response.message = 'Processing complete'
        return response
```

---

### Example 2: Battery Monitor Action

**Action Definition** (`ChargeBattery.action`):
```
# Goal
float32 target_charge  # Target charge percentage
---
# Result
bool success
float32 final_charge
---
# Feedback
float32 current_charge
float32 time_remaining  # seconds
```

**Action Server**:
```python
class BatteryChargerServer(Node):
    def __init__(self):
        super().__init__('battery_charger_server')
        self._action_server = ActionServer(
            self, ChargeBattery, 'charge_battery', self.execute_callback
        )
        self.current_charge = 20.0  # Start at 20%

    def execute_callback(self, goal_handle):
        feedback_msg = ChargeBattery.Feedback()

        while self.current_charge < goal_handle.request.target_charge:
            # Check for cancel
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return ChargeBattery.Result(success=False, final_charge=self.current_charge)

            # Simulate charging
            self.current_charge += 2.0
            time_remaining = (goal_handle.request.target_charge - self.current_charge) / 2.0

            # Publish feedback
            feedback_msg.current_charge = self.current_charge
            feedback_msg.time_remaining = time_remaining
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1.0)

        goal_handle.succeed()
        return ChargeBattery.Result(success=True, final_charge=self.current_charge)
```

---

## Debugging Communication Issues

### Common Problems and Solutions

#### 1. Service Not Available
```bash
# Check if service exists
ros2 service list

# Manually call service to test
ros2 service call /my_service my_interfaces/srv/MyService "{field: value}"

# Check service server logs
ros2 node info /my_server_node
```

#### 2. Action Feedback Not Received
```bash
# Monitor action feedback
ros2 action send_goal /my_action my_interfaces/action/MyAction "{goal_data}" --feedback

# Check action server status
ros2 action info /my_action
```

#### 3. Lifecycle Node Stuck in Transition
```bash
# Check current state
ros2 lifecycle get /my_node

# Force transition
ros2 lifecycle set /my_node configure
ros2 lifecycle set /my_node activate
```

---

## Best Practices

### 1. Service Design
```python
# ✅ Good: Short, synchronous operations
class PoseEstimationService:
    def callback(self, request, response):
        response.pose = self.estimate_pose(request.image)
        return response  # Returns in <100ms

# ❌ Bad: Long-running operations (use actions instead)
class TrainModelService:
    def callback(self, request, response):
        self.train_model(epochs=1000)  # Takes hours!
        return response
```

### 2. Action Feedback
```python
# ✅ Good: Meaningful progress updates
feedback.percent_complete = (i / total) * 100
feedback.current_waypoint = waypoint_name
goal_handle.publish_feedback(feedback)

# ❌ Bad: No feedback
# User has no idea what's happening
```

### 3. Error Handling
```python
# ✅ Good: Handle failures gracefully
try:
    result = self.do_work()
    response.success = True
    response.data = result
except Exception as e:
    response.success = False
    response.error_message = str(e)
    self.get_logger().error(f'Service failed: {e}')
```

---

## Summary

In this chapter, you learned:

- ✅ **Services** enable synchronous request-response communication
- ✅ **Actions** handle long-running tasks with feedback and cancellation
- ✅ **Lifecycle nodes** provide explicit state management
- ✅ **Multi-node systems** coordinate via services, actions, and topics
- ✅ **Debugging tools** help diagnose communication issues

**Key Takeaway**: Choose the right communication pattern for your use case:
- **Topics**: Continuous data streams (sensor readings, odometry)
- **Services**: Quick request-response (pose estimation, path lookup)
- **Actions**: Long-running tasks with feedback (navigation, manipulation)

---

## End-of-Chapter Exercises

### Exercise 1: Create a Calculator Service (Difficulty: Easy)

Create a service that performs basic arithmetic operations:
```
Operation.srv:
---
float64 a
float64 b
string operation  # "add", "subtract", "multiply", "divide"
---
float64 result
bool success
string message
```

**Bonus**: Add error handling for division by zero.

---

### Exercise 2: LED Blink Action (Difficulty: Medium)

Create an action that blinks an LED a specified number of times:
- **Goal**: Number of blinks
- **Feedback**: Current blink count
- **Result**: Success status

Simulate the LED with `print()` statements.

---

### Exercise 3: Multi-Node Coordination (Difficulty: Hard)

Build a system with 3 nodes:
1. **Sensor Node**: Publishes random temperature readings
2. **Monitor Node**: Subscribes to temperature, calls alert service if >30°C
3. **Alert Server**: Service that logs alerts and returns acknowledgment

**Bonus**: Add a lifecycle manager node to coordinate startup.

---

## Further Reading

### Required
1. [ROS 2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Services.html)
2. [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Actions.html)
3. [Lifecycle Nodes](https://design.ros2.org/articles/node_lifecycle.html)

### Optional
- [Service Interface Design](https://docs.ros.org/en/humble/Concepts/About-Interfaces.html)
- [Action Interface Design](https://design.ros2.org/articles/actions.html)
- [Node Composition](https://docs.ros.org/en/humble/Tutorials/Composition.html)

---

## Next Chapter

Continue to **[Chapter 1.3: Launch Files and Configuration](./chapter1-3-launch-files)** to learn how to start multi-node systems efficiently.

---

**Pro Tip**: Start with topics for continuous data, add services for quick queries, and use actions for long-running tasks. This layered approach creates robust robotic systems!
