# Chapter 4.3: LLM Task Planning with Claude & GPT-4

> **Module**: 4 - Voice-Language-Action Brain
> **Week**: 12
> **Estimated Reading Time**: 28 minutes

---

## Summary

Large Language Models (LLMs) like Claude 3.5 Sonnet and GPT-4 provide high-level task planning for robots—decomposing complex goals ("make breakfast") into executable steps. This chapter covers prompt engineering for robotics, reasoning patterns (ReAct, Chain-of-Thought), LLM-VLA integration, error handling, and hybrid systems combining symbolic planning with learned policies.

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Apply** prompt engineering techniques for robot task decomposition (few-shot examples, system prompts, constraints)
2. **Compare** Claude vs. GPT-4 for robotics (latency, cost, context window, tool use)
3. **Implement** ReAct (Reasoning + Acting) pattern for iterative task execution with feedback
4. **Integrate** LLM planner with VLA executor via ROS 2 action server
5. **Handle** execution failures through LLM replanning and error recovery

**Prerequisite Knowledge**: Chapter 4.1 (VLA), Chapter 4.2 (Whisper), Module 1 (ROS 2 actions), basic LLM API usage

---

## Key Terms

- **Task Planning**: Decomposing high-level goals into ordered sequence of executable actions
- **Prompt Engineering**: Crafting LLM inputs (system prompt, user prompt, examples) to elicit desired outputs
- **ReAct**: Reasoning and Acting—LLM pattern interleaving thought, action, observation in loop
- **Chain-of-Thought (CoT)**: Prompting technique encouraging step-by-step reasoning before final answer
- **Tool Use**: LLM capability to call external functions (e.g., robot actions, perception queries)
- **System Prompt**: Instructions defining LLM role/constraints/output format (unchanging across requests)
- **Few-Shot Learning**: Providing 2-5 example input-output pairs to guide LLM behavior
- **Grounding**: Connecting LLM symbolic reasoning to physical world state (via perception/sensors)

---

## Core Concepts

### 1. Why LLMs for Robotics?

**VLA Limitations** (Chapter 4.1):
- ❌ **Single-Step**: VLAs excel at reactive tasks ("pick red cup") but struggle with multi-step plans ("make coffee: fill pot, add grounds, brew")
- ❌ **No Common Sense**: VLAs learn from demos, lack world knowledge (e.g., "water boils at 100°C", "fragile objects break if dropped")
- ❌ **Fixed Horizon**: Trained on ~10-step episodes, cannot handle 50+ step tasks

**LLM Advantages**:
- ✅ **Compositional**: Decompose complex goals into sub-tasks
- ✅ **World Knowledge**: Pre-trained on internet text (recipes, physics, social norms)
- ✅ **Flexible**: Adapt to new tasks without retraining
- ✅ **Explainable**: Generate human-readable plans ("First, I'll pick up the cup...")

**Hybrid Approach** (Best of Both):
```
User: "Make me breakfast"
       ↓
LLM Planner: ["boil_water", "toast_bread", "fry_egg", "plate_food"]
       ↓
VLA Executor: Executes each action with visual feedback
```

---

### 2. Claude vs. GPT-4 for Robotics

| Feature | Claude 3.5 Sonnet | GPT-4 Turbo | Recommendation |
|---------|-------------------|-------------|----------------|
| **Latency** | 2-5s | 3-8s | Claude (faster) |
| **Cost** | $3/$15 per 1M tokens | $10/$30 per 1M tokens | Claude (3× cheaper) |
| **Context Window** | 200k tokens | 128k tokens | Claude (can fit long task history) |
| **Tool Use** | Native (function calling) | Native (function calling) | Tie |
| **Reasoning** | Strong (constitutional AI) | Strong (RLHF) | Tie |
| **Vision** | Yes (Claude 3.5) | Yes (GPT-4V) | Tie |
| **Availability** | API + Bedrock | API + Azure | Tie |

**Robotics Use Cases**:
- **Real-Time Tasks** (<5s latency): Claude 3.5 Haiku (0.5-1s, $0.25/$1.25 per 1M) or GPT-4 Mini
- **Complex Planning** (multi-step): Claude 3.5 Sonnet (best balance)
- **Budget-Constrained**: Claude Haiku (5-10× cheaper than GPT-4)

---

### 3. Prompt Engineering for Robotics

**System Prompt** (defines robot capabilities, constraints, output format):

```python
SYSTEM_PROMPT = """You are a task planner for a 7-DOF robotic arm in a kitchen environment.

CAPABILITIES:
- pick(object_name): Grasp object (e.g., pick("red_cup"))
- place(object_name, location): Place object at location (e.g., place("red_cup", "table"))
- pour(source, target): Pour liquid (e.g., pour("kettle", "cup"))
- press(button_name): Press button (e.g., press("coffee_maker_button"))
- wait(seconds): Wait for duration (e.g., wait(30))

CONSTRAINTS:
- Robot can only hold 1 object at a time (must place before picking another)
- Heavy objects (>2kg) cannot be lifted
- Fragile objects must be handled gently (slow speed)
- Hot surfaces should be avoided

OUTPUT FORMAT:
Return JSON array of actions:
[
  {"action": "pick", "args": ["red_cup"], "reasoning": "Need cup for coffee"},
  {"action": "place", "args": ["red_cup", "coffee_maker"], "reasoning": "Position under dispenser"}
]

Be concise. Only output valid actions. If task impossible, return empty array with explanation.
"""
```

**User Prompt** (high-level goal + context):

```python
USER_PROMPT = """Task: Make a cup of coffee

Current Scene:
- coffee_maker: ready, has water and grounds
- red_cup: on table
- blue_cup: on shelf
- milk: in fridge

Generate action plan."""
```

**LLM Output**:

```json
[
  {"action": "pick", "args": ["red_cup"], "reasoning": "Nearest cup to coffee maker"},
  {"action": "place", "args": ["red_cup", "coffee_maker"], "reasoning": "Position under dispenser"},
  {"action": "press", "args": ["brew_button"], "reasoning": "Start brewing"},
  {"action": "wait", "args": [45], "reasoning": "Coffee takes ~45 seconds"},
  {"action": "pick", "args": ["red_cup"], "reasoning": "Retrieve filled cup"},
  {"action": "place", "args": ["red_cup", "table"], "reasoning": "Deliver to user"}
]
```

**Few-Shot Examples** (improves accuracy):

```python
EXAMPLES = [
    {
        "user": "Task: Water the plant. Scene: watering_can on shelf, plant on table.",
        "assistant": '[{"action": "pick", "args": ["watering_can"]}, {"action": "pour", "args": ["watering_can", "plant"]}, {"action": "place", "args": ["watering_can", "shelf"]}]'
    },
    {
        "user": "Task: Clean spill. Scene: sponge in sink, spill on counter.",
        "assistant": '[{"action": "pick", "args": ["sponge"]}, {"action": "wipe", "args": ["counter"]}, {"action": "place", "args": ["sponge", "sink"]}]'
    }
]
```

**API Call** (Claude):

```python
import anthropic

client = anthropic.Anthropic(api_key="your-api-key")

messages = EXAMPLES + [{"role": "user", "content": USER_PROMPT}]

response = client.messages.create(
    model="claude-3-5-sonnet-20241022",
    max_tokens=1024,
    system=SYSTEM_PROMPT,
    messages=messages
)

plan = json.loads(response.content[0].text)
print(f"Generated {len(plan)} actions")
```

---

### 4. ReAct: Reasoning + Acting Pattern

**Problem**: LLMs plan without feedback → fail if world state changes (e.g., cup moved)

**Solution**: ReAct loop—interleave Thought, Action, Observation

**Pattern**:
```
1. Thought: "I need to pick up the cup"
2. Action: execute pick("red_cup")
3. Observation: "Success: red_cup grasped" OR "Failure: cup not found"
4. Thought: "Cup not found, maybe it's the blue cup instead?"
5. Action: execute pick("blue_cup")
6. Observation: "Success: blue_cup grasped"
...
```

**Implementation**:

```python
def react_loop(task, max_steps=10):
    state = get_robot_state()  # Current scene perception

    for step in range(max_steps):
        # THOUGHT: LLM reasons about next action
        prompt = f"""Task: {task}
Current State: {state}
Previous Actions: {history}

Think step-by-step:
1. What is the next action?
2. Why is it needed?
3. What could go wrong?

Output JSON: {{"thought": "...", "action": "...", "args": [...]}}
"""
        response = llm.generate(prompt)
        thought, action, args = parse_response(response)

        print(f"[THOUGHT] {thought}")

        # ACTION: Execute via VLA or hardcoded controller
        observation = execute_action(action, args)

        print(f"[OBSERVATION] {observation}")

        # Update history and state
        history.append({"action": action, "result": observation})
        state = get_robot_state()  # Re-perceive scene

        # Check if goal achieved
        if is_goal_satisfied(task, state):
            print(f"✅ Task completed in {step+1} steps")
            return True

    print(f"❌ Task failed after {max_steps} steps")
    return False
```

**Example Execution**:
```
User: "Bring me the cup"

[THOUGHT] I see a red cup and blue cup. User said "the cup" (ambiguous). I'll try the nearest one (red cup).
[ACTION] pick("red_cup")
[OBSERVATION] Error: Object not found at expected location

[THOUGHT] Red cup moved or misidentified. Let me try blue cup.
[ACTION] pick("blue_cup")
[OBSERVATION] Success: Grasped blue_cup

[THOUGHT] Now navigate to user location.
[ACTION] navigate_to("user")
[OBSERVATION] Success: Arrived at user

[THOUGHT] Place cup within reach.
[ACTION] place("blue_cup", "user_table")
[OBSERVATION] Success: blue_cup placed

✅ Task completed in 4 steps
```

---

### 5. LLM-VLA Integration

**Architecture**:
```
┌─────────────────────────────────────┐
│   LLM Planner (Claude 3.5 Sonnet)   │
│   Input: Task + Scene                │
│   Output: [action1, action2, ...]   │
└──────────────┬──────────────────────┘
               ↓
┌──────────────────────────────────────┐
│   ROS 2 Action Server                │
│   /execute_plan (custom action)      │
└──────────────┬───────────────────────┘
               ↓
┌──────────────────────────────────────┐
│   VLA Executor (OpenVLA from Ch 4.1) │
│   Executes each action with vision   │
└──────────────┬───────────────────────┘
               ↓
┌──────────────────────────────────────┐
│   Robot Hardware (UR5, Franka, etc.) │
└──────────────────────────────────────┘
```

**ROS 2 Action Definition**:

```python
# ExecutePlan.action
string task_description
---
bool success
string final_state
---
string current_action
float32 progress
```

**LLM Planner Node**:

```python
# llm_planner_node.py
import rclpy
from rclpy.action import ActionServer
from robot_interfaces.action import ExecutePlan
import anthropic

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')
        self.llm = anthropic.Anthropic(api_key=os.getenv("ANTHROPIC_API_KEY"))
        self.action_server = ActionServer(
            self,
            ExecutePlan,
            '/execute_plan',
            self.execute_plan_callback
        )

    def execute_plan_callback(self, goal_handle):
        task = goal_handle.request.task_description
        scene = self.get_scene_description()  # From perception

        # Generate plan
        plan = self.generate_plan(task, scene)

        # Execute each action
        for i, step in enumerate(plan):
            feedback = ExecutePlan.Feedback()
            feedback.current_action = step['action']
            feedback.progress = (i + 1) / len(plan)
            goal_handle.publish_feedback(feedback)

            # Call VLA executor
            success = self.execute_vla_action(step)

            if not success:
                # Replan on failure
                remaining_plan = self.replan(task, plan[i:])
                plan = plan[:i] + remaining_plan

        goal_handle.succeed()
        result = ExecutePlan.Result()
        result.success = True
        return result

    def generate_plan(self, task, scene):
        response = self.llm.messages.create(
            model="claude-3-5-sonnet-20241022",
            system=ROBOT_SYSTEM_PROMPT,
            messages=[{"role": "user", "content": f"Task: {task}\nScene: {scene}"}]
        )
        return json.loads(response.content[0].text)
```

---

### 6. Error Handling & Replanning

**Failure Types**:
1. **Perception Failure**: Object not found (moved, occluded, misidentified)
2. **Execution Failure**: Grasp failed (slipped, collision, out of reach)
3. **Precondition Failure**: Action invalid (e.g., place without holding object)
4. **Goal Failure**: Task unachievable (missing object, broken tool)

**Replanning Strategies**:

**A. Local Replan** (fix single step):
```python
if action_failed:
    prompt = f"""Action {action} failed: {error_message}
Current State: {state}
Original Goal: {task}

Suggest alternative action to recover.
Output JSON: {{"action": "...", "args": [...], "reasoning": "..."}}
"""
    alternative = llm.generate(prompt)
    execute(alternative)
```

**B. Global Replan** (regenerate entire plan):
```python
if action_failed:
    prompt = f"""Original plan failed at step {i}: {action}
Error: {error_message}

Current State: {state}
Goal: {task}

Generate NEW complete plan from current state.
"""
    new_plan = llm.generate(prompt)
    execute_plan(new_plan)
```

**C. Human-in-the-Loop** (ask for help):
```python
if replan_attempts > 3:
    request_human_intervention(task, error_history)
```

---

### 7. Chain-of-Thought (CoT) Prompting

**Technique**: Encourage LLM to "think aloud" before answering

**Without CoT**:
```
User: Can the robot make coffee if the coffee maker is unplugged?
LLM: Yes.  # WRONG (missed prerequisite)
```

**With CoT**:
```
User: Can the robot make coffee if the coffee maker is unplugged?
Let's think step by step:
LLM:
1. Making coffee requires the coffee maker to heat water.
2. Heating requires electrical power.
3. If unplugged, there is no power.
4. Therefore, the coffee maker cannot heat water.
5. Therefore, the robot CANNOT make coffee.

Answer: No, the robot cannot make coffee if the coffee maker is unplugged.
```

**Implementation**:
```python
USER_PROMPT = f"""Task: {task}

Let's think step by step:
1. What are the preconditions?
2. What tools/objects are needed?
3. What is the sequence of actions?
4. What could go wrong?

Then output the plan."""
```

---

## Practical Example: LLM-Guided Table Setting

### Overview

Robot sets dinner table for 2 people using LLM planner (Claude 3.5) + VLA executor (OpenVLA).

### Implementation

**Step 1: Define Task & Scene**

```python
TASK = "Set the table for 2 people with plates, forks, and cups"

SCENE = {
    "plates": {"location": "cabinet", "count": 4},
    "forks": {"location": "drawer", "count": 6},
    "cups": {"location": "shelf", "count": 3},
    "table": {"location": "dining_room", "occupied": False}
}
```

**Step 2: Generate Plan (Claude)**

```python
response = llm.messages.create(
    model="claude-3-5-sonnet-20241022",
    system=ROBOT_SYSTEM_PROMPT,
    messages=[{
        "role": "user",
        "content": f"Task: {TASK}\nScene: {json.dumps(SCENE)}\nGenerate plan."
    }]
)

plan = json.loads(response.content[0].text)
print(f"Plan: {len(plan)} steps")
# Output:
# [
#   {"action": "open", "args": ["cabinet"], "reasoning": "Access plates"},
#   {"action": "pick", "args": ["plate"], ...},
#   {"action": "place", "args": ["plate", "table_position_1"], ...},
#   ... (12 steps total: 2 plates + 2 forks + 2 cups = 6 objects × 2 actions)
# ]
```

**Step 3: Execute via VLA**

```bash
# Terminal 1: Launch LLM planner
ros2 run llm_control llm_planner_node

# Terminal 2: Launch VLA executor
ros2 run vla_control vla_executor_node

# Terminal 3: Send goal
ros2 action send_goal /execute_plan robot_interfaces/action/ExecutePlan "{task_description: 'Set the table for 2 people'}"
```

### Expected Outcome

- **Plan Generation**: 3 seconds (Claude API call)
- **Execution**: 90 seconds (12 steps × 7.5s avg per action)
- **Success Rate**: 80% (fails if object misidentified or grasp fails)
- **Error Recovery**: Replan triggered 1-2 times (e.g., "fork not in drawer" → check alternative location)

### Troubleshooting

- **Issue**: LLM generates invalid actions (e.g., "fly_to_shelf")
  **Solution**: Improve system prompt with explicit action whitelist, provide more few-shot examples

- **Issue**: Plan ignores physics (e.g., places plate mid-air)
  **Solution**: Add constraint checking: validate each action before execution

- **Issue**: High latency (10s+ for plan generation)
  **Solution**: Use Claude 3.5 Haiku (1s latency) or cache system prompt (reduces tokens)

---

## Exercises

### Exercise 1: Prompt Engineering (Difficulty: Easy)

**Objective**: Optimize system prompt for meal prep tasks

**Task**: Write system prompt defining robot capabilities for kitchen tasks (chop, stir, boil)

**Requirements**:
- Define 5 actions with args and constraints
- Test with 3 tasks: "make salad", "boil pasta", "scramble eggs"
- Measure: plan validity (all actions executable?)

**Expected Outcome**: 90%+ valid plans

**Estimated Time**: 60 minutes

---

### Exercise 2: ReAct Loop Implementation (Difficulty: Medium)

**Objective**: Implement ReAct pattern with error recovery

**Task**: Build React loop executing "clean kitchen" with re-planning on failures

**Requirements**:
- Simulate 30% action failure rate
- LLM replans after each failure
- Max 3 replan attempts before giving up
- Log thought/action/observation for each step

**Expected Outcome**: 70% task success rate (vs. 0% without replanning)

**Estimated Time**: 3 hours

---

### Exercise 3: LLM-VLA Hybrid System (Difficulty: Hard)

**Objective**: Build end-to-end voice-controlled robot

**Task**: Integrate Whisper (Ch 4.2) → LLM planner → VLA executor (Ch 4.1)

**Requirements**:
- Voice command → Whisper transcript
- LLM generates 5-10 step plan
- VLA executes each step with visual feedback
- Replan if action fails
- Test: 10 voice commands, measure end-to-end success

**Expected Outcome**: 65% voice-to-execution success

**Estimated Time**: 6 hours

---

## Summary & Key Takeaways

- **LLMs for Planning**: Claude/GPT-4 decompose high-level goals into executable sub-tasks, leveraging world knowledge
- **Prompt Engineering**: System prompts define robot capabilities/constraints, few-shot examples improve accuracy
- **ReAct Pattern**: Interleave reasoning, action, observation for adaptive execution with feedback
- **LLM-VLA Integration**: LLM plans symbolically, VLA executes with vision-language grounding
- **Error Handling**: Local replan (fix single step) vs. global replan (regenerate plan) vs. human-in-the-loop
- **Claude vs. GPT-4**: Claude 3.5 Sonnet recommended (2-5s latency, 3× cheaper, 200k context)

**Connection to Chapter 4.4**: With voice (Whisper), planning (LLM), and execution (VLA) components covered, Chapter 4.4 integrates all three into deployable systems—edge deployment (Jetson), hybrid architectures (System 1 + System 2), and real-world case studies.

---

## Additional Resources

- Claude API Documentation: https://docs.anthropic.com/claude/docs
- OpenAI GPT-4 API: https://platform.openai.com/docs/guides/gpt
- ReAct Paper: https://arxiv.org/abs/2210.03629 (Yao et al., 2022)
- Chain-of-Thought Paper: https://arxiv.org/abs/2201.11903 (Wei et al., 2022)

---

## Notes for Instructors

**Teaching Tips**:
- Live demo: Send same task to Claude vs. GPT-4, compare latency/cost/quality
- Show prompt evolution: bad prompt → good prompt (with constraints + examples)

**Lab Ideas**:
- Lab 1: Prompt engineering competition (best system prompt for kitchen robot)
- Lab 2: Implement ReAct loop in simulation
- Lab 3: Voice-to-VLA full pipeline (Whisper → LLM → VLA)

**Assessment**:
- Quiz: Explain ReAct vs. standard planning (Answer: ReAct uses feedback, adapts to world changes)
- Project: LLM-guided robot completing 5 household tasks (>70% success)

**Common Student Mistakes**:
- Over-engineering system prompts (too long = slower, more expensive)
- Not validating LLM outputs (executing invalid actions crashes robot)
- Ignoring latency (5s LLM call acceptable for planning, NOT for reactive control)

---

**Chapter Metadata**:
- **Word Count**: 3,200 words
- **Code Examples**: 8
- **Exercises**: 3
- **Glossary Terms**: 8
