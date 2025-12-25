# Figure Naming Convention

**Standard Format**:
```
fig[MODULE].[CHAPTER]-[description].svg
```

## Examples

- `fig1.1-ros2-computational-graph.svg`
- `fig2.2-humanoid-urdf-tree.svg`
- `fig3.2-isaac-perception-pipeline.svg`
- `fig4.4-full-vla-system.svg`

## Master Diagrams (No Chapter-Specific)
```
fig-master-[description].svg
```

Examples:
- `fig-master-ros2-system-graph.svg`
- `fig-master-module-dependencies.svg`
- `fig-bipedal-kinematic-tree.svg`

## Rules

1. **Lowercase**: All lowercase, no capital letters
2. **Hyphens**: Use hyphens (-) to separate words, not underscores
3. **Module.Chapter**: Use dot (.) between module and chapter numbers
4. **Description**: Brief, descriptive (2-4 words max)
5. **Format**: Always `.svg` (preferred) or `.png` (fallback)

## Invalid Examples

- ❌ `Fig1-1-ros2.svg` (capital F, underscore)
- ❌ `figure1.1-ros2.svg` (too verbose)
- ❌ `ros2_graph.svg` (no module/chapter reference)
- ❌ `fig1-1-ros2.svg` (missing dot between module/chapter)

## Directory Structure

```
diagrams/
├── module1/
│   ├── fig1.1-ros2-computational-graph.svg
│   ├── fig1.2-service-call-sequence.svg
│   └── ...
├── module2/
├── module3/
├── module4/
└── master/
    └── fig-master-ros2-system-graph.svg
```

(Detailed guidelines in `diagram-guidelines.md`)
