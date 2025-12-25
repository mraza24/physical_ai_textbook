# Figure 1.3-1: Launch File Execution Flow

**Chapter**: 1.3 - Launch Files & Configuration
**Type**: Mermaid Flowchart
**Purpose**: Visualize launch file processing and node startup sequence

---

## Diagram

```mermaid
flowchart TD
    Start([User runs:<br/>ros2 launch pkg file.launch.py])

    Start --> ParseArgs[Parse Command-Line<br/>Arguments]
    ParseArgs --> LoadFile[Load Python<br/>Launch File]
    LoadFile --> Execute[Execute:<br/>generate_launch_description()]

    Execute --> ProcessArgs[Process Launch<br/>Arguments<br/>use_sim, log_level, etc.]
    ProcessArgs --> LoadParams[Load Parameter<br/>Files YAML]
    LoadParams --> EvalConditions[Evaluate<br/>Conditions<br/>IfCondition, UnlessCondition]

    EvalConditions --> FilterNodes{Filter Nodes<br/>by Conditions}

    FilterNodes -->|Condition True| IncludeNode[Include Node<br/>in Launch]
    FilterNodes -->|Condition False| SkipNode[Skip Node]

    IncludeNode --> ApplyRemap[Apply<br/>Remappings]
    ApplyRemap --> SetParams[Set Node<br/>Parameters]

    SetParams --> CheckDeps{Dependencies<br/>Resolved?}
    CheckDeps -->|No| WaitDeps[Wait for<br/>Dependencies]
    WaitDeps --> CheckDeps
    CheckDeps -->|Yes| StartNode[Start Node<br/>Process]

    StartNode --> Monitor[Monitor Node<br/>Lifecycle]
    Monitor --> AllStarted{All Nodes<br/>Started?}

    AllStarted -->|No| FilterNodes
    AllStarted -->|Yes| Running([System Running])

    Running --> UserStop{User<br/>Ctrl+C?}
    UserStop -->|No| Running
    UserStop -->|Yes| Shutdown[Shutdown All<br/>Nodes]

    Shutdown --> Cleanup[Cleanup<br/>Resources]
    Cleanup --> End([Exit])

    SkipNode --> AllStarted

    %% Styling
    classDef startEnd fill:#e1f5ff,stroke:#0066cc,stroke-width:2px
    classDef process fill:#fff9e6,stroke:#ff9800,stroke-width:1px
    classDef decision fill:#ffe0b2,stroke:#ff6f00,stroke-width:2px
    classDef action fill:#e8f5e9,stroke:#4caf50,stroke-width:1px

    class Start,End,Running startEnd
    class ParseArgs,LoadFile,Execute,ProcessArgs,LoadParams,EvalConditions,ApplyRemap,SetParams,StartNode,Monitor,Shutdown,Cleanup process
    class FilterNodes,CheckDeps,AllStarted,UserStop decision
    class IncludeNode,SkipNode,WaitDeps action
```

---

## Description

This flowchart illustrates the complete lifecycle of a ROS 2 launch file from invocation to shutdown.

### Execution Phases

#### 1. Initialization (Start → Execute)
1. User runs `ros2 launch my_pkg file.launch.py arg1:=val1`
2. Launch system parses command-line arguments
3. Python launch file is loaded and executed
4. `generate_launch_description()` function is called

#### 2. Configuration Processing (ProcessArgs → EvalConditions)
1. Launch arguments are processed (defaults applied if not provided)
2. Parameter YAML files are loaded from disk
3. Conditional expressions are evaluated (IfCondition, UnlessCondition)

#### 3. Node Filtering (FilterNodes)
- Nodes with conditions are included/excluded based on evaluation
- Example: `condition=IfCondition('use_sim')` → only included if `use_sim==true`

#### 4. Node Configuration (ApplyRemap → SetParams)
1. Topic/service remappings are applied to each node
2. Parameters are set (from YAML files, inline dicts, or launch arguments)
3. Node-specific configurations are prepared

#### 5. Dependency Resolution (CheckDeps)
- Launch system checks if node dependencies are satisfied
- Example: Node B depends on service from Node A → wait for A to start first
- Prevents race conditions and ensures correct startup order

#### 6. Node Startup (StartNode)
- Each node process is spawned with configured parameters
- Nodes begin initialization and ROS 2 setup

#### 7. Runtime Monitoring (Monitor → Running)
- Launch system monitors node health
- System enters "Running" state when all nodes have started
- User sees aggregated logs from all nodes

#### 8. Shutdown (UserStop → End)
- User presses Ctrl+C
- Launch system sends SIGINT to all nodes
- Waits for graceful shutdown (timeout: 5-10 seconds)
- Cleans up resources (closes file handles, network sockets)
- Exits

---

## Key Features

### Conditional Node Launching

```python
# In launch file
camera_node = Node(
    ...,
    condition=IfCondition(LaunchConfiguration('use_sim'))
)
```

**Flow**:
```
EvalConditions → FilterNodes
  ├─ use_sim==true  → IncludeNode → ApplyRemap → ...
  └─ use_sim==false → SkipNode → AllStarted?
```

---

### Dependency Management

**Example**: Navigation depends on map server

```python
nav_node = Node(
    ...,
    on_exit=Shutdown()  # If navigation crashes, shut down system
)
```

**Flow**:
```
SetParams → CheckDeps
  ├─ Map server running? Yes → StartNode(nav)
  └─ Map server running? No  → WaitDeps → CheckDeps (retry)
```

---

### Parameter Loading Priority

```
1. Command-line args:  ros2 launch ... param:=value
2. Launch file inline: parameters=[{'param': value}]
3. YAML file:          parameters=[yaml_file]
4. Node default:       (if not overridden)
```

All resolved during **LoadParams** and **SetParams** phases.

---

## Error Handling

### Common Failure Points

**1. Launch File Syntax Error** (LoadFile/Execute phase)
```
ERROR: Failed to load launch file
  File "file.launch.py", line 15
    return LaunchDescription([
         ^
SyntaxError: unexpected EOF
```
**Solution**: Check Python syntax, matching brackets

---

**2. Missing Parameter File** (LoadParams phase)
```
ERROR: Parameter file not found: config/params.yaml
```
**Solution**: Verify file path, check `get_package_share_directory()`

---

**3. Node Executable Not Found** (StartNode phase)
```
ERROR: Executable 'my_node' not found in package 'my_package'
```
**Solution**: Verify package name, executable name, colcon build completed

---

**4. Dependency Timeout** (CheckDeps phase)
```
WARN: Waiting for dependency 'map_server'... (10 seconds)
ERROR: Dependency timeout, aborting launch
```
**Solution**: Ensure all dependencies are included in launch file

---

## Performance Considerations

### Launch Time Optimization

**Sequential launch** (default):
```
Node A starts → waits 1s → Node B starts → waits 1s → Node C starts
Total: 3+ seconds
```

**Parallel launch** (when dependencies allow):
```
Node A, B, C start simultaneously
Total: ~1 second
```

**How to enable**: Remove unnecessary dependencies, let launch system parallelize

---

### Memory Footprint

**Separate processes** (default):
- Each node: 20-50 MB base overhead
- 20 nodes: ~400-1000 MB total

**Composed nodes** (Chapter 1.3, Section 7):
- All nodes in one container: ~50-100 MB total
- Suitable for resource-constrained systems

---

## Usage in Chapter

Referenced in **Sections 2-7** to show how launch files are processed and executed. Students should understand:
- Launch files are Python scripts executed by the launch system
- Nodes are not started until all configuration is resolved
- Shutdown is coordinated across all nodes (no orphaned processes)

**Pedagogical Note**: Instructors can add `print()` statements in launch files to show execution flow:

```python
def generate_launch_description():
    print("1. Generating launch description...")
    # ... configure nodes ...
    print("2. Launch description ready")
    return LaunchDescription([...])
```

Students observe that `generate_launch_description()` runs once before any nodes start.

---

## Extension: Launch File Debugging

**Dry run** (show what would launch without actually starting):
```bash
ros2 launch --debug my_pkg file.launch.py
```

**Show parsed launch tree**:
```bash
ros2 launch --show-all-subprocesses my_pkg file.launch.py
```

**Verbose logging**:
```bash
ros2 launch --debug my_pkg file.launch.py
```

These tools help students understand the flow diagram by observing it in action.

---

**File**: `fig1.3-launch-flow.md` (Mermaid source)
**Export**: Render to SVG for textbook inclusion
**Dimensions**: Recommended 1000×1200px (tall flowchart)
