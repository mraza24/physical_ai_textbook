# Reference Management System - Physical AI Textbook

**Purpose**: Standards for managing citations and references using APA 7th Edition format.

---

## Citation Style: APA 7th Edition

All references in this textbook follow **APA 7th Edition** format.

### Why APA 7?
- Standard in psychology, education, and robotics research
- Clear format for digital sources (websites, APIs, software)
- Widely recognized in academic and industry publications

---

## Reference Types and Formats

### 1. Books

**Format**:
```
Author, A. A., & Author, B. B. (Year). Title of book (Edition). Publisher. https://doi.org/xxxxx
```

**Example**:
```
Thrun, S., Burgard, W., & Fox, D. (2005). Probabilistic robotics. MIT Press.
```

### 2. Journal Articles

**Format**:
```
Author, A. A., & Author, B. B. (Year). Title of article. Journal Name, Volume(Issue), pages. https://doi.org/xxxxx
```

**Example**:
```
Brohan, A., Brown, N., Carbajal, J., et al. (2023). RT-1: Robotics Transformer for real-world control at scale. arXiv preprint arXiv:2212.06817. https://arxiv.org/abs/2212.06817
```

### 3. Conference Papers

**Format**:
```
Author, A. A. (Year). Title of paper. In Proceedings of Conference Name (pp. pages). Publisher. https://doi.org/xxxxx
```

**Example**:
```
Mur-Artal, R., Montiel, J. M. M., & Tardos, J. D. (2015). ORB-SLAM: A versatile and accurate monocular SLAM system. IEEE Transactions on Robotics, 31(5), 1147-1163. https://doi.org/10.1109/TRO.2015.2463671
```

### 4. Websites and Documentation

**Format**:
```
Organization/Author. (Year, Month Day). Page title. Site Name. Retrieved Month Day, Year, from URL
```

**Examples**:
```
Open Robotics. (2024). ROS 2 Documentation: Humble. ROS 2 Docs. Retrieved December 10, 2025, from https://docs.ros.org/en/humble/

NVIDIA. (2024). Isaac SDK Documentation. NVIDIA Developer. Retrieved December 10, 2025, from https://developer.nvidia.com/isaac-sdk
```

### 5. Software and APIs

**Format**:
```
Developer/Organization. (Year). Software name (Version) [Software]. URL
```

**Examples**:
```
Open Robotics. (2023). ROS 2 Humble Hawksbill (v2.0) [Software]. https://github.com/ros2/ros2

OpenAI. (2024). Whisper API [API]. https://platform.openai.com/docs/guides/speech-to-text
```

### 6. GitHub Repositories

**Format**:
```
Username/Organization. (Year). Repository name [Source code]. GitHub. URL
```

**Example**:
```
Unity Technologies. (2024). Unity-Robotics-Hub [Source code]. GitHub. https://github.com/Unity-Technologies/Unity-Robotics-Hub
```

### 7. Technical Reports and White Papers

**Format**:
```
Author, A. A. (Year). Title of report (Report No. xxx). Organization. URL
```

**Example**:
```
NVIDIA. (2023). NVIDIA Jetson Orin technical specification (Technical Report). NVIDIA Corporation. https://developer.nvidia.com/embedded/jetson-orin
```

### 8. YouTube Videos and Tutorials

**Format**:
```
Creator. (Year, Month Day). Video title [Video]. YouTube. URL
```

**Example**:
```
Articulated Robotics. (2023, June 15). ROS 2 tutorial: Creating a simple robot [Video]. YouTube. https://youtube.com/watch?v=xxxxx
```

---

## In-Text Citations

### Single Author
- (Thrun, 2005)
- Thrun (2005) demonstrated...

### Multiple Authors
- **Two authors**: (Thrun & Burgard, 2005)
- **Three or more**: (Brohan et al., 2023)

### Direct Quotes
- (Thrun, 2005, p. 42)
- According to Thrun (2005), "quote here" (p. 42)

### Multiple Works
- (Thrun, 2005; Mur-Artal et al., 2015)

---

## Reference List Template

Use this template for the final `references.md` file:

```markdown
# References

All citations in this textbook follow APA 7th Edition format.

---

## Books

[Alphabetical by first author's last name]

---

## Journal Articles & Conference Papers

[Alphabetical by first author's last name]

---

## Technical Documentation

[Alphabetical by organization]

---

## Software & APIs

[Alphabetical by name]

---

## Online Resources

[Alphabetical by organization/title]

---

## Additional Reading

### Research Papers
[Recent papers for advanced study]

### Community Resources
[Forums, blogs, tutorials]

### Video Content
[YouTube channels, video courses]
```

---

## Module-Specific Reference Tracking

### Module 1: ROS 2
**Core References**:
- ROS 2 Official Documentation
- ROS 2 Design Documentation
- DDS Specifications

**Example Entry**:
```
Open Robotics. (2024). ROS 2 Humble documentation. Retrieved December 10, 2025, from https://docs.ros.org/en/humble/
```

### Module 2: Digital Twin
**Core References**:
- Gazebo Documentation
- Unity Robotics Hub
- URDF Specifications

**Example Entry**:
```
Open Robotics. (2024). Gazebo simulation documentation. Retrieved December 10, 2025, from https://gazebosim.org/docs
```

### Module 3: NVIDIA Isaac
**Core References**:
- Isaac SDK Documentation
- Isaac Sim Documentation
- TensorRT Documentation

**Example Entry**:
```
NVIDIA. (2024). NVIDIA Isaac SDK documentation. NVIDIA Developer. Retrieved December 10, 2025, from https://developer.nvidia.com/isaac-sdk
```

### Module 4: VLA
**Core References**:
- RT-1 Paper (Google Research)
- RT-2 Paper (Google DeepMind)
- OpenVLA Paper
- OpenAI Whisper Documentation
- Claude/GPT API Documentation

**Example Entries**:
```
Brohan, A., et al. (2023). RT-2: Vision-language-action models transfer web knowledge to robotic control. arXiv preprint arXiv:2307.15818.

OpenAI. (2024). Whisper API documentation. Retrieved December 10, 2025, from https://platform.openai.com/docs/guides/speech-to-text
```

---

## Reference Tracking File

Create a tracking file to collect references as chapters are written:

**File**: `reference-tracker.md`

```markdown
# Reference Tracker

## Module 1: ROS 2
- [ ] ROS 2 Official Docs
- [ ] DDS Specification
- [ ] ...

## Module 2: Digital Twin
- [ ] Gazebo Docs
- [ ] Unity Robotics Hub
- [ ] ...

## Module 3: Isaac
- [ ] Isaac SDK Docs
- [ ] TensorRT Docs
- [ ] ...

## Module 4: VLA
- [ ] RT-1 Paper
- [ ] RT-2 Paper
- [ ] Whisper Docs
- [ ] ...
```

---

## Citation Tools

### Recommended Tools
1. **Zotero**: Free, open-source reference manager with browser extension
2. **Mendeley**: Reference manager with PDF annotation
3. **Citation Generator**: https://www.scribbr.com/apa-citation-generator/

### Manual APA Formatting
- Use the templates above
- Double-check author names, dates, titles
- Verify URLs are accessible
- Include retrieval dates for web sources

---

## Quality Checklist

Before finalizing references:

- [ ] All references follow APA 7th Edition format
- [ ] References are alphabetized by first author's last name
- [ ] All in-text citations have corresponding reference entries
- [ ] All URLs are valid and accessible
- [ ] Retrieval dates included for web sources
- [ ] DOIs included where available
- [ ] Software versions specified
- [ ] Minimum 20+ total references across all modules
- [ ] Each module has 5+ specific references

---

## Common Mistakes to Avoid

### 1. Inconsistent Formatting
❌ Wrong: `Thrun, S. (2005) Probabilistic Robotics`
✅ Correct: `Thrun, S., Burgard, W., & Fox, D. (2005). Probabilistic robotics. MIT Press.`

### 2. Missing Retrieval Dates
❌ Wrong: `ROS 2 Docs. https://docs.ros.org`
✅ Correct: `Open Robotics. (2024). ROS 2 documentation. Retrieved December 10, 2025, from https://docs.ros.org`

### 3. Incomplete Author Lists
❌ Wrong: `Brohan, A. (2023)...`
✅ Correct: `Brohan, A., et al. (2023)...` or list all authors

### 4. No DOI When Available
❌ Wrong: Paper with DOI but no link
✅ Correct: Always include DOI as `https://doi.org/xxxxx`

---

## Example Complete Reference List

```markdown
# References

## Books

Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic robotics*. MIT Press.

## Journal Articles

Mur-Artal, R., Montiel, J. M. M., & Tardos, J. D. (2015). ORB-SLAM: A versatile and accurate monocular SLAM system. *IEEE Transactions on Robotics*, *31*(5), 1147-1163. https://doi.org/10.1109/TRO.2015.2463671

## Technical Documentation

NVIDIA. (2024). *Isaac SDK documentation*. NVIDIA Developer. Retrieved December 10, 2025, from https://developer.nvidia.com/isaac-sdk

Open Robotics. (2024). *ROS 2 Humble documentation*. Retrieved December 10, 2025, from https://docs.ros.org/en/humble/

## Software

OpenAI. (2024). *Whisper API* [API]. https://platform.openai.com/docs/guides/speech-to-text

Unity Technologies. (2024). *Unity-Robotics-Hub* [Source code]. GitHub. https://github.com/Unity-Technologies/Unity-Robotics-Hub
```

---

**Need Help?** See the [APA Style Guide](https://apastyle.apa.org/) for detailed examples.
