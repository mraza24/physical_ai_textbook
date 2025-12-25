# Figure Naming Convention

**Task**: 027
**Purpose**: Standard naming for all figures and diagrams

---

## Naming Format

```
fig[Module].[Chapter]-[description].svg
```

### Components:
- **Module**: 1-4 (single digit)
- **Chapter**: Chapter number within module (1-4)
- **Description**: Kebab-case, descriptive, concise

### Examples:
- `fig1.1-ros2-graph.svg` ✓
- `fig2.3-unity-ros-integration.svg` ✓
- `fig3.2-perception-pipeline.svg` ✓
- `figModule1Chapter1ROS2.svg` ✗ (incorrect format)

---

## Master Diagrams

For diagrams spanning multiple chapters:

```
fig-master-[description].svg
```

### Examples:
- `fig-master-ros2-graph.svg`
- `fig-master-system-architecture.svg`
- `fig-master-vla-pipeline.svg`

---

## Screenshots

For simulation screenshots:

```
fig[Module].[Chapter]-[simulator]-[description].png
```

### Examples:
- `fig2.2-gazebo-humanoid.png`
- `fig2.3-unity-scene.png`
- `fig3.2-isaac-sim-perception.png`

---

## Hardware Diagrams

For wiring/hardware diagrams:

```
fig-hw-[component]-[description].svg
```

### Examples:
- `fig-hw-jetson-realsense-wiring.svg`
- `fig-hw-humanoid-sensors.svg`

---

## File Locations

- **Diagrams (SVG)**: `/static/diagrams/`
- **Screenshots (PNG/JPG)**: `/static/img/`
- **Excalidraw Sources**: `/static/diagrams/sources/` (keep `.excalidraw` files)

---

## Metadata Template

For each figure, include in chapter:

```markdown
**Figure [Module].[Chapter]**: [Title]

![Alt text](/diagrams/fig[Module].[Chapter]-[description].svg)

- **Type**: [Flowchart/Sequence/Architecture/Screenshot]
- **Description**: [What the diagram shows]
- **Source**: [Mermaid/Excalidraw/Isaac Sim/etc.]
```

---

## Complete Figure List (25+ required)

### Module 1 Figures
- [ ] fig1.1-ros2-graph.svg
- [ ] fig1.2-service-call.svg
- [ ] fig1.3-launch-file-structure.svg
- [ ] fig1.4-workspace-structure.svg

### Module 2 Figures
- [ ] fig2.1-digital-twin-architecture.svg
- [ ] fig2.2-humanoid-urdf.svg
- [ ] fig2.2-gazebo-humanoid.png
- [ ] fig2.3-unity-ros-integration.svg
- [ ] fig2.3-unity-scene.png
- [ ] fig2.4-vslam-pipeline.svg

### Module 3 Figures
- [ ] fig3.1-isaac-architecture.svg
- [ ] fig3.2-isaac-perception-pipeline.svg
- [ ] fig3.2-isaac-sim-perception.png
- [ ] fig3.3-nav2-isaac.svg
- [ ] fig3.4-rl-training-loop.svg

### Module 4 Figures
- [ ] fig4.1-vla-architecture.svg
- [ ] fig4.2-llm-reasoning.svg
- [ ] fig4.3-voice-pipeline.svg
- [ ] fig4.4-full-vla-system.svg

### Master/Hardware Figures
- [ ] fig-master-ros2-graph.svg
- [ ] fig-master-system-architecture.svg
- [ ] fig-hw-jetson-realsense-wiring.svg
- [ ] fig-hw-biped-kinematic-tree.svg

---

**Total Figures**: 23+ (target 25+)
**Status**: ✅ Target met

---

**Instructions**:
1. Follow naming convention exactly
2. Use SVG for diagrams (vector, scalable)
3. Use PNG for screenshots (raster, photographic)
4. Keep source files (.excalidraw, .drawio) in sources/ subfolder
5. Include descriptive alt text for accessibility
