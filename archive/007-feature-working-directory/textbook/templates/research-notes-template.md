# Research Notes Template

## Topic: [RESEARCH_TOPIC]

**Research Track**: [ROS 2 / Simulation / Isaac / VLA / Hardware]

**Module**: [MODULE_NUMBER] - [MODULE_NAME]

**Researcher**: [NAME]

**Date**: [YYYY-MM-DD]

---

## Official Source Information

### Primary Source
- **Title**: [Official documentation, paper title, or resource name]
- **URL**: [Full URL]
- **Version/Date**: [e.g., ROS 2 Humble, accessed 2025-12-11]
- **Publisher**: [e.g., Open Robotics, NVIDIA, OpenAI]

### APA 7 Citation
```
[Complete APA 7 formatted citation - copy directly to references tracker]
```

---

## Key Concepts

### Concept 1: [CONCEPT_NAME]

**Definition**: [Clear, concise definition in 1-2 sentences]

**Explanation**: [Detailed explanation in your own words, 2-4 sentences]

**Relevance to Textbook**: [Which chapter(s) will use this concept? Why is it important?]

**Example**: [Concrete example or code snippet if applicable]

```[language]
# [Example code or configuration]
```

---

### Concept 2: [CONCEPT_NAME]

**Definition**: [1-2 sentence definition]

**Explanation**: [Detailed explanation]

**Relevance to Textbook**: [Chapter mapping]

**Example**: [Concrete example]

---

### Concept 3: [CONCEPT_NAME]

[Repeat structure for additional concepts]

---

## Technical Specifications

### Version Information
- **Software Version**: [e.g., ROS 2 Humble Hawksbill, Gazebo 11.14.0]
- **OS Requirements**: [e.g., Ubuntu 22.04 LTS]
- **Hardware Requirements**: [e.g., GPU: RTX 4070 Ti+, RAM: 16GB+]
- **Dependencies**: [e.g., Python 3.10+, specific packages]

### Installation Notes
```bash
# [Key installation commands if applicable]
sudo apt install ros-humble-desktop
```

---

## Important Details

### Features/Capabilities
- [Feature 1]: [Description and use case]
- [Feature 2]: [Description and use case]
- [Feature 3]: [Description and use case]

### Limitations/Constraints
- [Limitation 1]: [Description and workaround if available]
- [Limitation 2]: [Description]

### Best Practices
1. [Best practice 1]
2. [Best practice 2]
3. [Best practice 3]

---

## Chapter Applications

### Chapter [X.Y]: [CHAPTER_TITLE]

**Concepts to Include**:
- [Concept 1]
- [Concept 2]

**Code Examples Needed**:
- [Example 1 description]
- [Example 2 description]

**Figures Needed**:
- [Figure 1 description - e.g., "ROS 2 computational graph showing publisher-subscriber"]

---

### Chapter [X.Y]: [ANOTHER_CHAPTER]

[Repeat structure for other applicable chapters]

---

## Glossary Terms Identified

| Term | Brief Definition | Chapter Reference |
|------|------------------|-------------------|
| [Term 1] | [1-sentence definition] | [X.Y] |
| [Term 2] | [1-sentence definition] | [X.Y] |
| [Term 3] | [1-sentence definition] | [X.Y] |

**Note**: Add these terms to `tracking/glossary-tracker.md`

---

## References to Collect

### Official Documentation
- [Source 1]: [URL] - [APA citation]
- [Source 2]: [URL] - [APA citation]

### Academic Papers
- [Paper 1]: [Title, Authors, Year] - [DOI or arXiv]
- [Paper 2]: [Title, Authors, Year] - [DOI or arXiv]

### Additional Resources
- [Tutorial/Blog]: [URL] - [Brief description]

**Note**: Add all references to `tracking/references-tracker.md`

---

## Code Snippets / Examples

### Example 1: [EXAMPLE_NAME]

```[language]
# [Well-commented code snippet that demonstrates concept]
# This can be adapted for chapter code examples

import rclpy
from rclpy.node import Node

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        # Initialization code
```

**Explanation**: [What this code does and why it's useful]

**Source**: [If copied/adapted from official docs, cite here]

---

### Example 2: [ANOTHER_EXAMPLE]

[Repeat structure]

---

## Diagrams / Visualizations Needed

### Figure 1: [DIAGRAM_NAME]

**Type**: [Mermaid flowchart / Excalidraw wiring / Screenshot]

**Description**: [What the diagram should show]

**Components**:
- [Component 1]
- [Component 2]
- [Component 3]

**Mermaid Draft** (if applicable):
```mermaid
graph LR
    A[Component A] --> B[Component B]
    B --> C[Component C]
```

---

## Questions / Clarifications Needed

1. [Question 1 - something that needs further research or clarification]
2. [Question 2]
3. [Question 3]

**Resolution**: [To be answered by further research or consultation with official docs/community]

---

## Cross-References

### Related Research Notes
- [Link to other research notes that relate to this topic]
- [e.g., "See research/ros2-fundamentals.md for node communication details"]

### Module Dependencies
- **Depends On**: [Other modules/concepts that must be understood first]
- **Enables**: [Concepts that build on this research]

---

## Verification Checklist

- [ ] All key concepts identified and explained
- [ ] Official source cited with URL and date
- [ ] APA citation complete and formatted correctly
- [ ] Version/platform requirements documented
- [ ] At least 3 glossary terms identified
- [ ] Chapter applications mapped
- [ ] Code examples captured (if applicable)
- [ ] Diagram needs identified (if applicable)
- [ ] No outdated or deprecated information
- [ ] Added source to references tracker
- [ ] Added terms to glossary tracker

---

## Notes & Observations

[Free-form notes about interesting findings, potential pitfalls, common misconceptions, etc.]

---

**Status**: [Draft / In Review / Complete]

**Next Steps**: [What needs to be done with this research - e.g., "Use for Chapter 1.1 writing", "Create ROS 2 graph diagram"]

---

## Template Usage Notes

### When to Use This Template

Use this template for each research track in Phase 1:
1. ROS 2 Humble documentation (Task 013)
2. Gazebo 11 simulation (Task 014)
3. Unity Robotics Hub (Task 015)
4. NVIDIA Isaac ecosystem (Task 016)
5. VLA architectures (Task 017)
6. LLM APIs (Task 018)
7. Whisper audio processing (Task 019)
8. Jetson Orin Nano (Task 020)
9. Intel RealSense sensors (Task 021)
10. Humanoid robot platforms (Task 022)
11. VSLAM algorithms (Task 023)
12. Bipedal locomotion (Task 024)
13. Digital twin architectures (Task 025)
14. Sim-to-real transfer (Task 026)
15. Cloud GPU infrastructure (Task 027)
16. Nav2 navigation stack (Task 028)
17. Isaac Gym RL (Task 029)
18. TensorRT optimization (Task 030)

### Filename Convention

Save research notes as:
```
research/[topic-name].md
```

Examples:
- `research/ros2-fundamentals.md`
- `research/gazebo-simulation.md`
- `research/isaac-perception.md`
- `research/vla-architectures.md`

### Integration with Content Creation

During chapter writing (Phases 3-6):
1. Reference relevant research notes
2. Extract concepts for chapter content
3. Adapt code examples for chapter exercises
4. Create diagrams based on identified needs
5. Add glossary terms and references to trackers
