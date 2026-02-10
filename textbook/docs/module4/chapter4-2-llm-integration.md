---
sidebar_position: 3
title: Chapter 4.2 - LLM Integration with ROS 2
---

# Chapter 4.2: LLM Integration with ROS 2

**Module**: 4 - Vision-Language-Action
**Week**: 12
**Estimated Reading Time**: 30 minutes

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. Set up OpenAI or Anthropic API for LLM access
2. Create ROS 2 service node for LLM task planning
3. Design effective prompts for robot control
4. Parse LLM outputs into executable robot actions
5. Handle errors and invalid LLM responses

---

## Prerequisites

- Completed Chapter 4.1 (VLA Introduction)
- OpenAI or Anthropic API key
- Understanding of ROS 2 services

---

## Introduction

**Large Language Models (LLMs)** like GPT-4, Claude, and Llama can decompose high-level commands into step-by-step robot actions. Instead of hardcoding every possible task, we can use LLMs as a **task planner** that generates action sequences on-the-fly.

**Example**:
```
User: "Prepare coffee"
LLM Output:
1. Navigate to kitchen counter
2. Locate coffee machine
3. Pick up coffee cup
4. Place cup under spout
5. Press brew button
```

**Why LLMs for Robotics?**
- **Flexible**: Handle arbitrary natural language commands
- **Generalizable**: No need to pre-program every task
- **Common Sense**: Leverage world knowledge (e.g., coffee cups go under spouts)
- **Rapid Prototyping**: Test new tasks without recompiling code

---

## Key Terms

:::info Glossary Terms
- **LLM**: Large Language Model (GPT-4, Claude, Llama, etc.)
- **Prompt Engineering**: Designing inputs to guide LLM outputs
- **API Latency**: Time from request to response (typically 1-5 seconds)
- **Function Calling**: Structured LLM outputs as JSON function calls
- **Few-Shot Learning**: Providing example inputs/outputs to guide LLM
- **System Prompt**: Instructions that set LLM behavior and constraints
:::

---

## Core Concepts

### 1. LLM APIs

#### OpenAI API (GPT-4)

**Best For**: Highest reasoning quality, function calling
**Cost**: $30 per 1M input tokens, $60 per 1M output tokens (GPT-4o)
**Latency**: 1-3 seconds for typical robot commands

**Installation**:
```bash
pip install openai
```

**Basic Usage**:
```python
from openai import OpenAI

client = OpenAI(api_key='sk-...')

response = client.chat.completions.create(
    model="gpt-4o",
    messages=[
        {"role": "system", "content": "You are a robot task planner."},
        {"role": "user", "content": "Pick up the red cup"}
    ]
)

plan = response.choices[0].message.content
print(plan)
# Output: "1. Locate red cup\n2. Move arm to cup\n3. Close gripper\n4. Lift cup"
```

#### Anthropic API (Claude)

**Best For**: Long context (200k tokens), safety, instruction following
**Cost**: $3 per 1M input tokens, $15 per 1M output tokens (Claude Sonnet)
**Latency**: 1-2 seconds

**Installation**:
```bash
pip install anthropic
```

**Basic Usage**:
```python
from anthropic import Anthropic

client = Anthropic(api_key='sk-ant-...')

response = client.messages.create(
    model="claude-sonnet-4-20250514",
    max_tokens=1024,
    messages=[
        {"role": "user", "content": "Plan how to pick up a red cup"}
    ]
)

plan = response.content[0].text
print(plan)
```

#### Local Models (Ollama)

**Best For**: Privacy, zero cost, offline operation
**Cost**: Free (just GPU/CPU compute)
**Latency**: 5-10 seconds (depends on hardware)

**Installation**:
```bash
# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Download Llama 3.1 (8B)
ollama pull llama3.1:8b
```

**Usage**:
```python
import requests

def query_ollama(prompt):
    response = requests.post('http://localhost:11434/api/generate', json={
        'model': 'llama3.1:8b',
        'prompt': prompt,
        'stream': False
    })
    return response.json()['response']

plan = query_ollama("Plan how to pick up a red cup")
print(plan)
```

### 2. Prompt Design for Robotics

#### System Prompt Template

**Good System Prompt**:
```python
SYSTEM_PROMPT = """You are a robot task planner. Your job is to break down high-level commands into step-by-step actions.

Available Actions:
- navigate(x, y, theta): Move robot to position
- pick_object(object_name): Pick up object with gripper
- place_object(x, y, z): Place held object at location
- open_gripper(): Open gripper
- close_gripper(): Close gripper

Constraints:
- Always check if gripper is empty before picking
- Always navigate before manipulating objects
- Keep responses concise (max 5 steps)

Output Format:
Return a numbered list of actions. Example:
1. navigate(1.5, 2.0, 0)
2. pick_object("red_cup")
3. navigate(0.5, 1.0, 1.57)
4. place_object(0.5, 1.0, 0.8)
"""
```

**Key Elements**:
1. **Role**: "You are a robot task planner"
2. **Available Actions**: List valid functions
3. **Constraints**: Safety rules, physical limits
4. **Output Format**: Structured (numbered list, JSON, etc.)
5. **Examples**: Show desired output format

#### Few-Shot Examples

**Why Few-Shot?**: LLMs learn from examples better than pure instructions.

```python
FEW_SHOT_EXAMPLES = [
    {
        "user": "Pick up the blue block",
        "assistant": """1. navigate(1.0, 0.5, 0)
2. pick_object("blue_block")"""
    },
    {
        "user": "Move the cup to the table",
        "assistant": """1. pick_object("cup")
2. navigate(2.0, 1.0, 0)
3. place_object(2.0, 1.0, 0.7)"""
    }
]
```

**Usage**:
```python
messages = [{"role": "system", "content": SYSTEM_PROMPT}]

# Add few-shot examples
for example in FEW_SHOT_EXAMPLES:
    messages.append({"role": "user", "content": example["user"]})
    messages.append({"role": "assistant", "content": example["assistant"]})

# Add user query
messages.append({"role": "user", "content": "Throw away the banana"})

response = client.chat.completions.create(model="gpt-4o", messages=messages)
```

#### Function Calling (Structured Outputs)

**Problem**: Parsing free-text LLM outputs is error-prone.
**Solution**: Use OpenAI function calling for guaranteed JSON format.

```python
tools = [
    {
        "type": "function",
        "function": {
            "name": "navigate",
            "description": "Move robot to a position",
            "parameters": {
                "type": "object",
                "properties": {
                    "x": {"type": "number", "description": "X coordinate in meters"},
                    "y": {"type": "number"},
                    "theta": {"type": "number", "description": "Orientation in radians"}
                },
                "required": ["x", "y", "theta"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "pick_object",
            "description": "Pick up an object",
            "parameters": {
                "type": "object",
                "properties": {
                    "object_name": {"type": "string"}
                },
                "required": ["object_name"]
            }
        }
    }
]

response = client.chat.completions.create(
    model="gpt-4o",
    messages=[{"role": "user", "content": "Pick up the red cup at (1, 2)"}],
    tools=tools,
    tool_choice="auto"
)

# Extract function calls
tool_calls = response.choices[0].message.tool_calls
for call in tool_calls:
    function_name = call.function.name
    arguments = json.loads(call.function.arguments)
    print(f"{function_name}({arguments})")

# Output:
# navigate({'x': 1, 'y': 2, 'theta': 0})
# pick_object({'object_name': 'red_cup'})
```

### 3. ROS 2 Service Integration

#### Define Custom Service

**File**: `llm_planner_interfaces/srv/GetPlan.srv`
```
string command
---
string[] actions
bool success
string message
```

**Build**:
```bash
cd ~/ros2_ws
colcon build --packages-select llm_planner_interfaces
source install/setup.bash
```

#### LLM Planner Node

**File**: `llm_planner_node.py`
```python
import rclpy
from rclpy.node import Node
from llm_planner_interfaces.srv import GetPlan
from openai import OpenAI
import json

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')

        # Create service
        self.srv = self.create_service(GetPlan, 'get_plan', self.plan_callback)

        # Initialize OpenAI client
        self.client = OpenAI(api_key=self.get_parameter_or('api_key', 'sk-...').value)

        # System prompt
        self.system_prompt = """You are a robot task planner.

Available actions:
- navigate(x, y, theta)
- pick_object(name)
- place_object(x, y, z)
- open_gripper()
- close_gripper()

Output a numbered list of actions."""

        self.get_logger().info('LLM Planner Node ready')

    def plan_callback(self, request, response):
        try:
            self.get_logger().info(f'Received command: {request.command}')

            # Query LLM
            llm_response = self.client.chat.completions.create(
                model='gpt-4o',
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": request.command}
                ],
                temperature=0.3,  # Low temperature for consistency
                max_tokens=256
            )

            # Parse response
            plan_text = llm_response.choices[0].message.content
            actions = self.parse_plan(plan_text)

            response.actions = actions
            response.success = True
            response.message = f"Generated {len(actions)} actions"

            self.get_logger().info(f'Plan: {actions}')

        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(f'Planning failed: {e}')

        return response

    def parse_plan(self, plan_text):
        """Extract actions from LLM output"""
        actions = []
        for line in plan_text.split('\n'):
            line = line.strip()
            # Match lines like "1. navigate(1, 2, 0)"
            if line and line[0].isdigit():
                # Remove numbering
                action = line.split('.', 1)[1].strip()
                actions.append(action)
        return actions

def main():
    rclpy.init()
    node = LLMPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run**:
```bash
ros2 run llm_planner llm_planner_node --ros-args -p api_key:=sk-...
```

#### Client Node (Test)

```python
import rclpy
from rclpy.node import Node
from llm_planner_interfaces.srv import GetPlan

class PlannerClient(Node):
    def __init__(self):
        super().__init__('planner_client')
        self.client = self.create_client(GetPlan, 'get_plan')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_command(self, command):
        request = GetPlan.Request()
        request.command = command

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            self.get_logger().info(f'Plan: {response.actions}')
            return response.actions
        else:
            self.get_logger().error(f'Failed: {response.message}')
            return None

def main():
    rclpy.init()
    client = PlannerClient()

    # Test commands
    client.send_command("Pick up the red block")
    client.send_command("Clean the table")
    client.send_command("Bring me a water bottle")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Output Parsing and Validation

#### Validation Checks

```python
import re

def validate_action(action):
    """Validate action string format"""
    # Check if action matches function call pattern
    pattern = r'^(\w+)\((.*)\)$'
    match = re.match(pattern, action)

    if not match:
        return False, "Invalid function call format"

    function_name = match.group(1)
    args_str = match.group(2)

    # Check if function is allowed
    allowed_functions = ['navigate', 'pick_object', 'place_object', 'open_gripper', 'close_gripper']
    if function_name not in allowed_functions:
        return False, f"Unknown function: {function_name}"

    # Validate arguments
    try:
        args = eval(f"({args_str},)")  # Safely parse as tuple
    except:
        return False, "Invalid arguments"

    # Function-specific validation
    if function_name == 'navigate':
        if len(args) != 3:
            return False, "navigate requires 3 arguments (x, y, theta)"
        if not all(isinstance(arg, (int, float)) for arg in args):
            return False, "navigate arguments must be numbers"

    elif function_name == 'pick_object':
        if len(args) != 1:
            return False, "pick_object requires 1 argument (object_name)"
        if not isinstance(args[0], str):
            return False, "pick_object argument must be string"

    return True, "Valid"

# Test
print(validate_action("navigate(1.5, 2.0, 0)"))  # (True, "Valid")
print(validate_action("pick_object(red_cup)"))  # (False, "Invalid arguments")
print(validate_action("hack_robot()"))  # (False, "Unknown function: hack_robot")
```

#### Error Handling

```python
def execute_plan_safely(actions, robot):
    """Execute plan with error handling"""
    for i, action in enumerate(actions):
        # Validate
        valid, message = validate_action(action)
        if not valid:
            print(f"Action {i+1} invalid: {message}")
            return False

        # Execute
        try:
            success = robot.execute(action)
            if not success:
                print(f"Action {i+1} failed: {action}")
                return False
        except Exception as e:
            print(f"Error executing action {i+1}: {e}")
            return False

        print(f"✓ Completed action {i+1}: {action}")

    print("✓ Plan completed successfully")
    return True
```

---

## Practical Example: Coffee Machine Control

**Task**: Use LLM to plan coffee-making task.

### Step 1: Define Robot Actions

```python
class CoffeeRobot:
    def navigate(self, x, y, theta):
        print(f"Navigating to ({x}, {y}, {theta})")
        # ROS 2 navigation code here
        return True

    def pick_object(self, object_name):
        print(f"Picking up {object_name}")
        # ROS 2 manipulation code here
        return True

    def place_object(self, x, y, z):
        print(f"Placing object at ({x}, {y}, {z})")
        return True

    def press_button(self, button_name):
        print(f"Pressing {button_name} button")
        return True
```

### Step 2: Query LLM

```python
from openai import OpenAI

client = OpenAI(api_key='sk-...')

system_prompt = """You are controlling a coffee-making robot.

Available actions:
- navigate(x, y, theta): Move to position
- pick_object(name): Pick up object (e.g., "cup", "coffee_pod")
- place_object(x, y, z): Place held object
- press_button(name): Press machine button (e.g., "brew", "power")

Environment:
- Coffee machine is at (2.0, 1.0)
- Cups are at (0.5, 0.5)
- Coffee pods are at (0.5, 0.8)

Task: Make a cup of coffee
Output: Numbered list of actions"""

response = client.chat.completions.create(
    model='gpt-4o',
    messages=[
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": "Make a cup of coffee"}
    ]
)

plan = response.choices[0].message.content
print(plan)
```

**LLM Output**:
```
1. navigate(0.5, 0.5, 0)
2. pick_object("cup")
3. navigate(2.0, 1.0, 0)
4. place_object(2.0, 1.0, 0.85)
5. navigate(0.5, 0.8, 0)
6. pick_object("coffee_pod")
7. navigate(2.0, 1.0, 0)
8. place_object(2.0, 1.0, 0.95)
9. press_button("brew")
```

### Step 3: Execute Plan

```python
def execute_coffee_plan(plan_text, robot):
    actions = []
    for line in plan_text.split('\n'):
        line = line.strip()
        if line and line[0].isdigit():
            action = line.split('.', 1)[1].strip()
            actions.append(action)

    for action in actions:
        # Parse action
        match = re.match(r'(\w+)\((.*)\)', action)
        if not match:
            print(f"Invalid action: {action}")
            continue

        function_name = match.group(1)
        args_str = match.group(2)
        args = eval(f"({args_str},)")

        # Execute
        method = getattr(robot, function_name)
        method(*args)

robot = CoffeeRobot()
execute_coffee_plan(plan, robot)
```

**Output**:
```
Navigating to (0.5, 0.5, 0)
Picking up cup
Navigating to (2.0, 1.0, 0)
Placing object at (2.0, 1.0, 0.85)
...
Pressing brew button
```

---

## Advanced Topics

### Chain-of-Thought Prompting

**Problem**: LLM skips steps or makes logical errors.
**Solution**: Ask LLM to "think step by step".

```python
prompt = """Task: Make coffee

Think through this step-by-step:
1. What objects do I need?
2. Where are they located?
3. In what order should I collect them?
4. What's the final step?

Now generate the action plan:"""

response = client.chat.completions.create(
    model='gpt-4o',
    messages=[
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": prompt}
    ]
)
```

**Result**: More complete and logical plans.

### Multi-Turn Dialogues

**Use Case**: Clarify ambiguous commands.

```python
messages = [
    {"role": "system", "content": SYSTEM_PROMPT},
    {"role": "user", "content": "Bring me something to drink"}
]

response = client.chat.completions.create(model='gpt-4o', messages=messages)
# LLM: "What would you like to drink? (water, coffee, soda)"

messages.append({"role": "assistant", "content": response.choices[0].message.content})
messages.append({"role": "user", "content": "Water"})

response = client.chat.completions.create(model='gpt-4o', messages=messages)
# LLM: "1. navigate(1.0, 2.0, 0)\n2. pick_object('water_bottle')\n..."
```

---

## Summary

This chapter covered **LLM Integration with ROS 2**:

1. **LLM APIs**: OpenAI (GPT-4), Anthropic (Claude), Ollama (local)
2. **Prompt Design**: System prompts, few-shot examples, function calling
3. **ROS 2 Integration**: Custom service nodes for task planning
4. **Validation**: Parse and validate LLM outputs before execution

**Key Takeaways**:
- LLMs enable flexible task planning from natural language
- Prompt engineering critical for reliable outputs
- Always validate LLM outputs before robot execution
- Function calling provides structured JSON outputs
- Latency (1-5s) limits real-time use; best for high-level planning

**Next Chapter**: Whisper for voice command integration with robots.

---

## End-of-Chapter Exercises

### Exercise 1: Build LLM Planning Service (Difficulty: Medium)

**Tasks**:
1. Create ROS 2 service node with OpenAI or Claude API
2. Implement 5 robot actions (navigate, pick, place, etc.)
3. Design system prompt with constraints and examples
4. Test with 10 different natural language commands
5. Measure success rate and average latency

**Success Criteria**: 80%+ success rate, <3s average latency

### Exercise 2: Function Calling for Robustness (Difficulty: Hard)

**Tasks**:
1. Define OpenAI function schemas for robot actions
2. Modify LLM planner to use function calling
3. Compare free-text vs. function calling:
   - Parsing errors
   - Invalid actions
   - Execution success rate
4. Document findings in a table

**Success Criteria**: Function calling reduces errors by 50%+

---

## Further Reading

1. **OpenAI API Documentation**: https://platform.openai.com/docs
2. **Anthropic Claude API**: https://docs.anthropic.com/
3. **Prompt Engineering Guide**: https://www.promptingguide.ai/
4. **SayCan (Google)**: Language model task planning for robots
   - https://say-can.github.io/
5. **Code as Policies**: LLMs generate Python code for robot control
   - https://code-as-policies.github.io/

---

## Next Chapter

Continue to **[Chapter 4.3: Whisper Voice Commands](./chapter4-3-whisper-voice)** to add speech-to-text capabilities for hands-free robot control.
