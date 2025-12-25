# Exercise Template

## Exercise [NUMBER]: [TITLE]

**Difficulty**: [Easy / Medium / Hard]
**Estimated Time**: [X] minutes
**Module**: [MODULE_NUMBER] - [MODULE_NAME]
**Chapter**: [CHAPTER_NUMBER]

---

## Learning Objective

This exercise tests your understanding of: [Specific learning objective from chapter]

---

## Task Description

[Clear, detailed instructions for what the student should do. Be specific about:
- What to create/build/implement
- What technologies to use
- What outcomes to achieve]

---

## Requirements

### Functional Requirements
- [ ] [Specific requirement 1]
- [ ] [Specific requirement 2]
- [ ] [Specific requirement 3]

### Technical Requirements
- **Language**: Python 3.10+ / C++
- **Framework**: ROS 2 Humble
- **Tools**: [List specific tools/packages needed]

---

## Expected Outcome

**When the exercise is complete, you should be able to**:
1. [Specific observable outcome 1]
2. [Specific observable outcome 2]
3. [Specific observable outcome 3]

**Example Output**:
```
[Show expected terminal output or describe expected behavior]
```

---

## Starter Code (Optional)

```python
# [Provide skeleton code if appropriate for the exercise]
# Students fill in the TODO sections

import rclpy
from rclpy.node import Node

class ExerciseNode(Node):
    def __init__(self):
        super().__init__('exercise_node')
        # TODO: Initialize your node

    def callback(self, msg):
        # TODO: Implement callback logic
        pass

def main(args=None):
    # TODO: Complete the main function
    pass

if __name__ == '__main__':
    main()
```

---

## Hints

**Hint 1**: [Provide a hint for students who get stuck early]

**Hint 2**: [Another hint for a different common sticking point]

**Hint 3**: [Final hint that almost gives away the solution]

---

## Validation

### How to Know You've Succeeded

1. **Test 1**: [Specific test case]
   - Run: `[command]`
   - Expected: `[result]`

2. **Test 2**: [Another test case]
   - Run: `[command]`
   - Expected: `[result]`

### Self-Assessment Questions

- [ ] Can you explain why [concept] works this way?
- [ ] What would happen if you changed [parameter] to [value]?
- [ ] How would you extend this to [more complex scenario]?

---

## Sample Solution (For Instructor Reference Only)

**Note**: Students should attempt the exercise before looking at solutions.

```python
[Complete working solution]
```

**Explanation**: [Key points about the solution approach]

---

## Extensions (Optional Challenges)

For students who complete the exercise early:

1. **Easy Extension**: [Simpler add-on]
2. **Medium Extension**: [More complex modification]
3. **Hard Extension**: [Challenging integration with other concepts]

---

## Resources

- **Chapter Reference**: See Chapter [X.Y] for [concept]
- **Documentation**: [Link to relevant official docs]
- **Related Examples**: [Link to code examples in textbook]
