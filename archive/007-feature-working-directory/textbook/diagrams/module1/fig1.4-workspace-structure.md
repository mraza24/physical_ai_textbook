# Figure 1.4-1: ROS 2 Workspace Structure

**Chapter**: 1.4 - Building Packages & Workspaces
**Type**: Directory Tree Diagram
**Purpose**: Visualize the four-directory workspace structure

---

## Diagram

```
my_robot_ws/                    ← Workspace root
│
├── src/                        ← SOURCE SPACE (version controlled)
│   ├── robot_driver/           │  Package 1
│   │   ├── package.xml         │  ✓ Commit to Git
│   │   ├── setup.py            │  ✓ Commit to Git
│   │   ├── setup.cfg           │  ✓ Commit to Git
│   │   ├── resource/
│   │   │   └── robot_driver
│   │   ├── robot_driver/       │  Python source code
│   │   │   ├── __init__.py
│   │   │   └── driver_node.py
│   │   ├── launch/
│   │   │   └── driver.launch.py
│   │   ├── config/
│   │   │   └── params.yaml
│   │   └── test/
│   │       └── test_driver.py
│   │
│   ├── robot_perception/       │  Package 2
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── robot_perception/
│   │   │   ├── __init__.py
│   │   │   ├── detector.py
│   │   │   └── tracker.py
│   │   └── ...
│   │
│   └── robot_navigation/       │  Package 3
│       ├── package.xml
│       ├── CMakeLists.txt      │  (C++ package)
│       ├── src/
│       │   └── navigator.cpp
│       └── include/
│           └── navigator.hpp
│
├── build/                      ← BUILD SPACE (generated, gitignored)
│   ├── robot_driver/           │  ❌ Don't commit
│   │   ├── CMakeCache.txt      │  Intermediate files
│   │   ├── build/
│   │   └── ...
│   ├── robot_perception/
│   └── robot_navigation/
│
├── install/                    ← INSTALL SPACE (generated, gitignored)
│   ├── setup.bash              │  ❌ Don't commit
│   ├── setup.zsh               │  Source these to use workspace
│   ├── setup.sh                │
│   ├── local_setup.bash
│   ├── robot_driver/           │  Installed package 1
│   │   ├── lib/
│   │   │   └── robot_driver/
│   │   │       └── driver_node      ← Executable
│   │   └── share/
│   │       └── robot_driver/
│   │           ├── launch/
│   │           │   └── driver.launch.py
│   │           └── config/
│   │               └── params.yaml
│   ├── robot_perception/       │  Installed package 2
│   └── robot_navigation/       │  Installed package 3
│
└── log/                        ← LOG SPACE (generated, gitignored)
    ├── build_2025-12-11_10-30-45/    ❌ Don't commit
    │   ├── robot_driver/
    │   │   ├── stdout.log
    │   │   └── stderr.log
    │   ├── robot_perception/
    │   └── robot_navigation/
    ├── latest_build -> build_2025-12-11_10-30-45/
    └── test_2025-12-11_11-00-12/
```

---

## Directory Descriptions

### src/ - Source Space

**Purpose**: Contains all package source code that you write and edit

**Contents**:
- Package directories (one per package)
- Each package has `package.xml` (manifest)
- Python packages have `setup.py` and Python modules
- C++ packages have `CMakeLists.txt` and C++ source

**Version Control**:
- ✅ **Commit `src/` to Git**
- Track all source code, manifests, launch files, config files
- This is the only directory that should be version controlled

**Commands**:
```bash
cd ~/my_robot_ws/src
git init
git add .
git commit -m "Initial commit"
```

---

### build/ - Build Space

**Purpose**: Contains intermediate build artifacts during compilation

**Contents**:
- CMake cache files
- Object files (.o)
- Dependency information
- Temporary build files

**Characteristics**:
- ❌ **Never commit to Git** (add to `.gitignore`)
- ❌ **Never edit manually**
- Generated automatically by `colcon build`
- Can be safely deleted (will be regenerated)

**Regenerating**:
```bash
rm -rf build/
colcon build  # Recreates build/
```

---

### install/ - Install Space

**Purpose**: Contains compiled binaries and resources ready to run

**Contents**:
- **Executables**: `lib/<package>/<node_name>`
- **Libraries**: Shared objects (.so), Python packages
- **Resources**: Launch files, config files, URDF models
- **Setup scripts**: `setup.bash`, `setup.zsh`, `setup.sh`

**Usage**:
```bash
source ~/my_robot_ws/install/setup.bash
ros2 run robot_driver driver_node  # Runs from install/
```

**Characteristics**:
- ❌ **Never commit to Git** (binary artifacts)
- Generated from `src/` via `colcon build`
- Contains **only** what's needed at runtime (not source code)

**What gets installed**:
- Python nodes → `lib/<package>/<executable>`
- Launch files → `share/<package>/launch/`
- Config files → `share/<package>/config/`
- URDF models → `share/<package>/urdf/`

---

### log/ - Log Space

**Purpose**: Records build output and test results

**Contents**:
- Build logs (`stdout.log`, `stderr.log` per package)
- Test results
- Timestamps for each build

**Characteristics**:
- ❌ **Never commit to Git** (ephemeral logs)
- Useful for debugging build failures
- `latest_build` symlink points to most recent build

**Viewing logs**:
```bash
# View build logs for specific package
cat log/latest_build/robot_driver/stdout.log

# View all errors
grep -r "error:" log/latest_build/
```

---

## Workspace Workflow

### 1. Initial Setup

```bash
mkdir -p ~/my_robot_ws/src
cd ~/my_robot_ws/src
# Create or clone packages here
```

### 2. Development Cycle

```
Edit code in src/
       ↓
colcon build
       ↓
Generates build/, install/, log/
       ↓
source install/setup.bash
       ↓
ros2 run <package> <node>
       ↓
Test and iterate
```

### 3. Version Control

```bash
# In workspace root
echo "build/" >> .gitignore
echo "install/" >> .gitignore
echo "log/" >> .gitignore

# Commit only source
git add src/
git commit -m "Add robot packages"
```

---

## File Size Comparison

Typical workspace after building:

```
src/         2-10 MB     (your code)
build/       50-200 MB   (intermediate files)
install/     20-100 MB   (binaries + resources)
log/         1-5 MB      (text logs)
```

**Why not commit build/install/log?**
- Large file sizes bloat Git repository
- Binary files (build/install) change on every build
- Platform-specific (Linux build won't work on Windows)
- **Reproducible from source**: Anyone can clone `src/` and run `colcon build`

---

## Multiple Packages

**Scale**: Real robot systems have 10-50+ packages

```
my_robot_ws/src/
├── drivers/              (Hardware interfaces)
│   ├── camera_driver/
│   ├── lidar_driver/
│   └── motor_controller/
├── perception/           (Vision & sensing)
│   ├── object_detection/
│   ├── tracking/
│   └── slam/
├── planning/             (Path planning)
│   ├── global_planner/
│   └── local_planner/
├── control/              (Execution)
│   ├── velocity_controller/
│   └── arm_controller/
└── interfaces/           (Custom messages)
    └── robot_msgs/
```

Each package builds independently, installs to shared `install/` space.

---

## Development vs Production

### Development Workspace

```bash
# Use symlink install for Python (no rebuild needed)
colcon build --symlink-install

# Changes to Python files take effect immediately
# Changes to launch/config files take effect immediately
# No need to rebuild after editing Python code
```

**Trade-off**: Symlinks mean `install/` depends on `src/`—if you delete `src/`, `install/` breaks.

---

### Production Workspace

```bash
# Regular install (copies files)
colcon build

# install/ is self-contained
# Can delete src/ and build/, install/ still works
# Suitable for deployment to robots
```

**Use case**: Build on development machine, copy `install/` to robot, run directly.

---

## Overlay Workspace Example

```
User overlay:      ~/my_robot_ws/install/setup.bash
                          ↓ extends
ROS 2 underlay:    /opt/ros/humble/setup.bash
                          ↓ extends
System libraries:  /usr/lib/...
```

**Sourcing order**:
```bash
source /opt/ros/humble/setup.bash  # Underlay first
source ~/my_robot_ws/install/setup.bash  # Overlay second
```

Now `~/my_robot_ws/install/` packages override `/opt/ros/humble/` if names match.

---

## Usage in Chapter

Referenced in **Sections 2, 5, 6, 8** to visualize workspace organization. Students should understand:
- Only `src/` is source code (version control this)
- `build/`, `install/`, `log/` are generated (can be deleted and recreated)
- `install/setup.bash` must be sourced to use workspace
- Each package in `src/` gets its own subdirectory in `build/` and `install/`

**Pedagogical Note**: Instructors can demonstrate filesystem before/after build:

```bash
# Before colcon build
ls ~/my_robot_ws/
# Output: src/

# After colcon build
ls ~/my_robot_ws/
# Output: src/ build/ install/ log/
```

This reinforces that three directories are generated from one source directory.

---

**File**: `fig1.4-workspace-structure.md` (Text diagram)
**Format**: ASCII tree for clarity (can convert to visual diagram if needed)
**Dimensions**: Full-page diagram showing complete workspace structure
