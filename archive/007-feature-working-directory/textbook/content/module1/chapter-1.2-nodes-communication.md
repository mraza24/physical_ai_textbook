# Chapter 1.2: Nodes & Communication Patterns

> **Module**: 1 - Robotic Nervous System (ROS 2)
> **Week**: 4
> **Estimated Reading Time**: 50 minutes

---

## Summary

This chapter explores advanced ROS 2 communication patterns—services for synchronous request-response interactions and actions for long-running goal-oriented tasks with feedback—providing the tools to implement complex robot behaviors beyond simple pub-sub data streaming. Mastering these patterns enables you to build robots that respond to commands, report progress, and handle failures gracefully.

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Implement** service servers and clients in Python for request-response communication
2. **Create** action servers with goal acceptance, feedback publishing, and result reporting
3. **Configure** Quality of Service (QoS) policies to optimize reliability, latency, and bandwidth
4. **Debug** communication issues using ROS 2 introspection tools (`ros2 service`, `ros2 action`, `ros2 topic info`)
5. **Design** robot architectures that appropriately combine topics, services, and actions for complex behaviors

**Prerequisite Knowledge**: Chapter 1.1 (ROS 2 Fundamentals), basic understanding of computational graphs and pub-sub communication

---

## Key Terms

This chapter introduces the following technical terms (see Glossary for detailed definitions):

- **Service Server**: Node component that provides a service, processing requests and returning responses
- **Service Client**: Node component that calls a service, sending requests and awaiting responses
- **Action Server**: Node component that accepts goals, executes long-running tasks, publishes feedback, and returns results
- **Action Client**: Node component that sends goals to action servers, monitors feedback, and handles results
- **Goal State**: Current status of an action goal (accepted, executing, succeeded, aborted, canceled)
- **Callback**: Function invoked automatically when events occur (message received, service requested, goal accepted)
- **Synchronous Call**: Blocking operation that waits for completion before continuing (services)
- **Asynchronous Call**: Non-blocking operation that returns immediately, allowing other work to proceed (topics, actions)

---

## Core Concepts

### 1. Services: Synchronous Request-Response

Services implement the classic **client-server pattern**: a client sends a request and blocks until receiving a response from the server. This synchronous communication is ideal for queries ("What's my battery level?") and commands that complete quickly ("Switch to manual mode").

#### Service Anatomy

Every ROS 2 service has three components:

1. **Service Type**: Defines request and response message structures (e.g., `std_srvs/srv/Trigger`, `example_interfaces/srv/AddTwoInts`)
2. **Service Name**: The endpoint clients call (e.g., `/get_battery_level`, `/reset_odometry`)
3. **Service Server**: The node that implements the service logic

**Service Type Example** (`example_interfaces/srv/AddTwoInts`):
```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

The `---` separator divides request (what client sends) from response (what server returns).

#### When to Use Services

✅ **Use services when**:
- Operation completes in <1 second
- You need confirmation that the operation succeeded
- Blocking the client while waiting is acceptable
- Request-response semantics are clear (query battery → get percentage)

❌ **Don't use services when**:
- Data is continuous (use topics instead)
- Operation takes >1 second (use actions instead)
- You don't need a response (use topics instead)
- Multiple clients need simultaneous responses (services are one-to-one)

**Key Points**:
- Service clients **block** during the call—the calling node cannot do other work until the response arrives or timeout occurs
- Services have **no history**—if the call fails, the client must retry
- Service servers should be **stateless** when possible—each request is independent
- Use **timeouts** to prevent indefinite blocking if the server is unresponsive

### 2. Implementing Service Servers

Let's build a service that returns the robot's battery level—a common query in mobile robotics.

#### Step 1: Choose or Create a Service Type

For simple queries, use `std_srvs/srv/Trigger`:
```
# Request (empty)
---
# Response
bool success
string message
```

For our battery service, we could use `Trigger` and encode the percentage in the `message` field, or create a custom type. For simplicity, we'll use `Trigger`.

#### Step 2: Implement the Server

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import random

class BatteryServerNode(Node):
    def __init__(self):
        super().__init__('battery_server')
        # Create service: name, type, callback
        self.srv = self.create_service(
            Trigger,
            'get_battery_level',
            self.battery_callback
        )
        self.battery_level = 100.0  # Simulated battery state
        self.get_logger().info('Battery server ready at /get_battery_level')

    def battery_callback(self, request, response):
        """Called automatically when a client calls the service"""
        # Simulate battery drain (for demo purposes)
        self.battery_level -= random.uniform(0, 2.0)
        self.battery_level = max(self.battery_level, 0.0)

        # Populate response
        response.success = True
        response.message = f'{self.battery_level:.1f}%'

        self.get_logger().info(f'Battery query: returning {response.message}')
        return response  # MUST return response object

def main():
    rclpy.init()
    node = BatteryServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key concepts**:
- `create_service(Type, name, callback)` registers the service
- `battery_callback(request, response)` is invoked for each client call
- The callback **must return the response object** (common mistake: forgetting this)
- Server runs continuously via `rclpy.spin()`, processing requests as they arrive

### 3. Implementing Service Clients

Now create a client to call our battery service:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class BatteryClientNode(Node):
    def __init__(self):
        super().__init__('battery_client')
        # Create client: type, service name
        self.client = self.create_client(Trigger, 'get_battery_level')

        # Wait for service to become available (important!)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for battery service...')

        self.get_logger().info('Battery service available!')

    def call_service(self):
        """Synchronously call the battery service"""
        request = Trigger.Request()  # Empty request for Trigger

        # Synchronous call (blocks until response or timeout)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Battery level: {response.message}')
            else:
                self.get_logger().warn(f'Service failed: {response.message}')
        else:
            self.get_logger().error('Service call timed out or failed')

def main():
    rclpy.init()
    node = BatteryClientNode()

    # Call service 3 times
    for i in range(3):
        node.get_logger().info(f'--- Call {i+1} ---')
        node.call_service()
        rclpy.spin_once(node, timeout_sec=1.0)  # Allow time between calls

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key concepts**:
- `wait_for_service()` blocks until server is available (prevents race conditions)
- `call_async()` returns a `Future` object—the actual call happens asynchronously
- `spin_until_future_complete()` blocks the current thread until the response arrives
- Always check `future.result() is not None` before accessing the response

### 4. Actions: Goals, Feedback, and Results

**Actions** are ROS 2's solution for long-running tasks that benefit from:
1. **Progress feedback**: "50% complete, ETA 10 seconds"
2. **Cancellation**: Abort the task mid-execution
3. **Result reporting**: Final status and output data

Actions are implemented as **three topics** under the hood:
- `/action_name/goal`: Client sends goals, server sends goal IDs
- `/action_name/feedback`: Server publishes progress updates
- `/action_name/result`: Server publishes final result

But the ROS 2 API abstracts this complexity—you work with actions as single entities.

#### Action Anatomy

Action types have three parts:

```
# Goal (what the client requests)
float64 target_distance  # Example: navigate forward X meters
---
# Result (what the server returns upon completion)
float64 actual_distance  # How far the robot actually moved
string message          # "Success", "Aborted: obstacle detected", etc.
---
# Feedback (periodic progress updates)
float64 current_distance  # Distance traveled so far
float64 percent_complete  # 0.0 to 100.0
```

**Common action types**:
- `nav2_msgs/action/NavigateToPose`: Navigate to a 2D pose
- `control_msgs/action/FollowJointTrajectory`: Execute arm trajectory
- Custom actions for domain-specific tasks

#### Action Lifecycle

```
Client sends GOAL → Server ACCEPTS/REJECTS
                  ↓ (if accepted)
              EXECUTING → Server publishes FEEDBACK (repeatedly)
                  ↓
              SUCCEEDED / ABORTED / CANCELED → Server sends RESULT
```

Clients can **cancel** at any time during execution, and servers should handle cancellation gracefully.

### 5. Implementing Action Servers

Let's create an action server for a "navigate forward" task that simulates a robot moving and provides feedback:

**First, define the action** (in a real project, you'd create a package with this `.action` file, but we'll use a built-in type for simplicity). For demonstration, assume we have:

```
# FibonacciAction (using a ROS 2 example action)
int32 order  # Goal: compute Fibonacci sequence up to order N
---
int32[] sequence  # Result: the computed sequence
---
int32[] partial_sequence  # Feedback: sequence computed so far
```

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci
import time

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        self.get_logger().info('Fibonacci action server started')

    def execute_callback(self, goal_handle):
        """Execute the Fibonacci computation goal"""
        self.get_logger().info(f'Executing goal: compute Fibonacci up to order {goal_handle.request.order}')

        # Initialize feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Compute Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result = Fibonacci.Result()
                result.sequence = feedback_msg.partial_sequence
                return result

            # Compute next Fibonacci number
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1]
            )

            # Publish feedback
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1)  # Simulate computation time

        # Mark goal as succeeded
        goal_handle.succeed()

        # Return result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info(f'Goal succeeded! Result: {result.sequence}')
        return result

def main():
    rclpy.init()
    server = FibonacciActionServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key concepts**:
- `ActionServer(node, type, name, callback)` creates the action server
- `execute_callback(goal_handle)` is invoked when a new goal arrives
- `goal_handle.publish_feedback(msg)` sends progress updates to the client
- `goal_handle.succeed()` or `goal_handle.canceled()` marks goal completion
- Always check `goal_handle.is_cancel_requested` in long loops
- The callback **must return a result object**

### 6. Implementing Action Clients

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self.get_logger().info('Fibonacci action client started')

    def send_goal(self, order):
        """Send a goal to compute Fibonacci sequence"""
        # Wait for action server
        self._action_client.wait_for_server()

        # Create goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Sending goal: order={order}')

        # Send goal asynchronously
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted, waiting for result...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

    def feedback_callback(self, feedback_msg):
        """Called automatically when server publishes feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.partial_sequence}')

def main():
    rclpy.init()
    client = FibonacciActionClient()
    client.send_goal(10)  # Compute Fibonacci up to order 10
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key concepts**:
- `ActionClient(node, type, name)` creates the client
- `send_goal_async(goal, feedback_callback)` sends the goal and registers feedback handler
- `feedback_callback(msg)` is invoked automatically for each feedback message
- `goal_handle.get_result_async()` retrieves the final result

---

## Practical Example: Battery Monitor with Service and Action

### Overview

This example combines services and actions to create a realistic robot battery monitoring system:
- **Service**: Query current battery level (quick request-response)
- **Action**: Initiate charging sequence (long-running task with progress feedback)

### Prerequisites

- Software: ROS 2 Humble, Python 3.10+
- Knowledge: Chapter 1.1 concepts, basic service/action understanding

### Implementation

**Complete code omitted for brevity—see GitHub repository for full implementation**

The system demonstrates:
1. Service server returning real-time battery percentage
2. Action server simulating 60-second charging with feedback every 5 seconds
3. Client that queries battery, decides if charging needed, initiates charging action
4. Cancellation handling if battery reaches 100% early

### Expected Output

```
[battery_monitor]: Battery at 45.2% - charging needed
[battery_monitor]: Starting charge action...
[charging_station]: Feedback: 55.0% charged
[charging_station]: Feedback: 65.0% charged
[charging_station]: Feedback: 75.0% charged
[charging_station]: Goal succeeded: Battery fully charged to 100.0%
```

---

## Figures & Diagrams

### Figure 1.2-1: Service Call Sequence Diagram

![Service Sequence](../../diagrams/module1/fig1.2-service-sequence.svg)

**Caption**: UML sequence diagram showing service client-server interaction. Client sends request and blocks until receiving response or timeout. Note the synchronous nature—client cannot proceed until response arrives.

---

### Figure 1.2-2: Action Lifecycle State Machine

![Action Lifecycle](../../diagrams/module1/fig1.2-action-lifecycle.svg)

**Caption**: State machine diagram for action goal lifecycle. Goals progress from PENDING → ACTIVE → terminal state (SUCCEEDED/ABORTED/CANCELED). Clients can request cancellation at any time during ACTIVE state.

---

## Exercises

### Exercise 1: Build a Mode Switcher Service (Difficulty: Medium)

**Objective**: Implement a service that switches robot operating modes

**Task**: Create a service server that accepts mode names ("manual", "autonomous", "estop") and returns success/failure.

**Requirements**:
- Use `std_srvs/srv/SetBool` or create custom service type
- Validate mode names (reject invalid modes)
- Log mode changes
- Service client that switches modes based on keyboard input

**Expected Outcome**:
```bash
$ python3 mode_switcher_client.py autonomous
[mode_server]: Switched to AUTONOMOUS mode
$ python3 mode_switcher_client.py invalid_mode
[mode_server]: ERROR: Invalid mode 'invalid_mode'
```

**Estimated Time**: 25 minutes

---

### Exercise 2: Countdown Action (Difficulty: Medium)

**Objective**: Create an action server that counts down from N to 0

**Task**: Implement an action that takes an integer goal (e.g., 10) and counts down to 0, publishing feedback every second.

**Requirements**:
- Define custom action type or use existing integer action
- Publish feedback with current count
- Support cancellation (stop counting if requested)
- Result includes "Reached 0!" or "Canceled at X"

**Expected Outcome**:
```
[countdown]: Goal: count from 10
[countdown]: Feedback: 10
[countdown]: Feedback: 9
[countdown]: Feedback: 8
...
[countdown]: Succeeded: Reached 0!
```

**Estimated Time**: 35 minutes

---

## Summary & Key Takeaways

In this chapter, you learned:

- **Services** provide synchronous request-response communication, ideal for queries and quick commands
- **Service servers** process requests via callbacks and must return response objects
- **Service clients** block until receiving responses, requiring timeout handling
- **Actions** support long-running tasks with goal acceptance, periodic feedback, and result reporting
- **Action servers** must handle cancellation gracefully and publish feedback regularly
- **Action clients** can monitor progress via feedback callbacks and cancel goals mid-execution

**Connection to Chapter 1.3**: With topics, services, and actions mastered, Chapter 1.3 introduces launch files—Python scripts that start multiple nodes simultaneously with parameter configuration, enabling complex multi-node robot systems.

---

## Additional Resources

### Official Documentation
- **ROS 2 Services**: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html
- **ROS 2 Actions**: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html
- **Creating Services**: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html

### Community Resources
- **ROS 2 Design**: Action design rationale and implementation details
- **GitHub Examples**: ros2/examples repository contains service/action samples

---

## Notes for Instructors

**Teaching Tips**:
- **Demonstrate blocking**: Run service client with `time.sleep(10)` in server callback—students see client freeze, reinforcing synchronous nature
- **Live cancellation**: Launch action server, send goal, cancel mid-execution with Ctrl+C—shows graceful handling

**Lab Exercise Ideas**:
- Multi-step action: "Make coffee" action with feedback ("Grinding beans", "Brewing", "Pouring")
- Service chain: Client calls service A, which internally calls service B—demonstrates composition

**Assessment Suggestions**:
- Evaluate action server for: feedback frequency (≥1 Hz), cancellation handling, result accuracy
- Check service timeouts: Does client handle unresponsive servers gracefully?

---

**Chapter Metadata**:
- **Word Count**: ~2600 words (core concepts)
- **Figures**: 2 (sequence diagram, state machine)
- **Code Examples**: 4 (service server, service client, action server, action client)
- **Exercises**: 2 (medium difficulty)
- **Glossary Terms**: 8
