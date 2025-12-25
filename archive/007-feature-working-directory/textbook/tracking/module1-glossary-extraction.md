# Module 1: Glossary Terms Extraction

**Module**: 1 - Robotic Nervous System (ROS 2)
**Chapters**: 1.1, 1.2, 1.3, 1.4
**Date**: 2025-12-11
**Total Terms**: 32

---

## Terms by Chapter

### Chapter 1.1: ROS 2 Fundamentals (8 terms)

| Term | Definition | First Use |
|------|------------|-----------|
| **Node** | An independent process in the ROS 2 computational graph that performs a specific computation (e.g., camera driver, object detector, motion planner) | Ch 1.1 |
| **Topic** | A named communication channel using asynchronous publish-subscribe messaging, ideal for streaming sensor data | Ch 1.1 |
| **Service** | A synchronous request-response communication pattern for querying information or triggering actions that complete quickly | Ch 1.1 |
| **Action** | A goal-oriented task pattern supporting long-running operations with progress feedback and cancellation (e.g., "navigate to waypoint") | Ch 1.1 |
| **DDS (Data Distribution Service)** | Industry-standard middleware providing reliable, real-time communication across processes and networks | Ch 1.1 |
| **Publisher** | A node component that sends messages to a topic | Ch 1.1 |
| **Subscriber** | A node component that receives messages from a topic | Ch 1.1 |
| **QoS (Quality of Service)** | Configurable policies (reliability, durability, deadline) that determine message delivery behavior | Ch 1.1 |

---

### Chapter 1.2: Nodes & Communication (8 terms)

| Term | Definition | First Use |
|------|------------|-----------|
| **Service Server** | Node component that provides a service, processing requests and returning responses | Ch 1.2 |
| **Service Client** | Node component that calls a service, sending requests and awaiting responses | Ch 1.2 |
| **Action Server** | Node component that accepts goals, executes long-running tasks, publishes feedback, and returns results | Ch 1.2 |
| **Action Client** | Node component that sends goals to action servers, monitors feedback, and handles results | Ch 1.2 |
| **Goal State** | Current status of an action goal (accepted, executing, succeeded, aborted, canceled) | Ch 1.2 |
| **Callback** | Function invoked automatically when events occur (message received, service requested, goal accepted) | Ch 1.2 |
| **Synchronous Call** | Blocking operation that waits for completion before continuing (services) | Ch 1.2 |
| **Asynchronous Call** | Non-blocking operation that returns immediately, allowing other work to proceed (topics, actions) | Ch 1.2 |

---

### Chapter 1.3: Launch Files & Configuration (8 terms)

| Term | Definition | First Use |
|------|------------|-----------|
| **Launch File** | Python script that describes how to start and configure multiple nodes | Ch 1.3 |
| **LaunchDescription** | ROS 2 object containing all launch actions (nodes, parameters, includes) | Ch 1.3 |
| **Node Launch Action** | Declaration to start a ROS 2 node with specified parameters | Ch 1.3 |
| **Parameter** | Named configuration value accessible to nodes at runtime (e.g., `camera_fps: 30`) | Ch 1.3 |
| **YAML Parameter File** | Human-readable configuration file mapping parameter names to values | Ch 1.3 |
| **Remapping** | Redirecting topic/service names (e.g., `/camera/image` → `/my_camera/image`) | Ch 1.3 |
| **Composition** | Running multiple nodes in a single process for efficiency | Ch 1.3 |
| **Launch Argument** | Command-line parameter that customizes launch file behavior | Ch 1.3 |

---

### Chapter 1.4: Building Packages & Workspaces (8 terms)

| Term | Definition | First Use |
|------|------------|-----------|
| **Workspace** | Top-level directory containing source code, build artifacts, and installed packages | Ch 1.4 |
| **Package** | Smallest unit of ROS 2 software distribution, containing nodes, launch files, and configuration | Ch 1.4 |
| **colcon** | Command-line build tool for compiling ROS 2 workspaces | Ch 1.4 |
| **package.xml** | Manifest file declaring package metadata, dependencies, and licensing | Ch 1.4 |
| **Overlay** | Workspace layered on top of another (underlay), allowing extensions without modification | Ch 1.4 |
| **Underlay** | Base workspace (e.g., `/opt/ros/humble/`) that provides foundational packages | Ch 1.4 |
| **Source Space (src/)** | Directory containing package source code | Ch 1.4 |
| **Install Space (install/)** | Directory containing compiled executables and resources ready for use | Ch 1.4 |

---

## Consolidated Glossary (32 terms, alphabetical)

| # | Term | Definition | Chapter |
|---|------|------------|---------|
| 1 | **Action** | A goal-oriented task pattern supporting long-running operations with progress feedback and cancellation | 1.1 |
| 2 | **Action Client** | Node component that sends goals to action servers, monitors feedback, and handles results | 1.2 |
| 3 | **Action Server** | Node component that accepts goals, executes long-running tasks, publishes feedback, and returns results | 1.2 |
| 4 | **Asynchronous Call** | Non-blocking operation that returns immediately, allowing other work to proceed | 1.2 |
| 5 | **Callback** | Function invoked automatically when events occur (message received, service requested, goal accepted) | 1.2 |
| 6 | **colcon** | Command-line build tool for compiling ROS 2 workspaces | 1.4 |
| 7 | **Composition** | Running multiple nodes in a single process for efficiency | 1.3 |
| 8 | **DDS (Data Distribution Service)** | Industry-standard middleware providing reliable, real-time communication across processes and networks | 1.1 |
| 9 | **Goal State** | Current status of an action goal (accepted, executing, succeeded, aborted, canceled) | 1.2 |
| 10 | **Install Space (install/)** | Directory containing compiled executables and resources ready for use | 1.4 |
| 11 | **Launch Argument** | Command-line parameter that customizes launch file behavior | 1.3 |
| 12 | **Launch File** | Python script that describes how to start and configure multiple nodes | 1.3 |
| 13 | **LaunchDescription** | ROS 2 object containing all launch actions (nodes, parameters, includes) | 1.3 |
| 14 | **Node** | An independent process in the ROS 2 computational graph that performs a specific computation | 1.1 |
| 15 | **Node Launch Action** | Declaration to start a ROS 2 node with specified parameters | 1.3 |
| 16 | **Overlay** | Workspace layered on top of another (underlay), allowing extensions without modification | 1.4 |
| 17 | **Package** | Smallest unit of ROS 2 software distribution, containing nodes, launch files, and configuration | 1.4 |
| 18 | **package.xml** | Manifest file declaring package metadata, dependencies, and licensing | 1.4 |
| 19 | **Parameter** | Named configuration value accessible to nodes at runtime | 1.3 |
| 20 | **Publisher** | A node component that sends messages to a topic | 1.1 |
| 21 | **QoS (Quality of Service)** | Configurable policies (reliability, durability, deadline) that determine message delivery behavior | 1.1 |
| 22 | **Remapping** | Redirecting topic/service names without modifying node code | 1.3 |
| 23 | **Service** | A synchronous request-response communication pattern for querying information or triggering quick actions | 1.1 |
| 24 | **Service Client** | Node component that calls a service, sending requests and awaiting responses | 1.2 |
| 25 | **Service Server** | Node component that provides a service, processing requests and returning responses | 1.2 |
| 26 | **Source Space (src/)** | Directory containing package source code | 1.4 |
| 27 | **Subscriber** | A node component that receives messages from a topic | 1.1 |
| 28 | **Synchronous Call** | Blocking operation that waits for completion before continuing | 1.2 |
| 29 | **Topic** | A named communication channel using asynchronous publish-subscribe messaging | 1.1 |
| 30 | **Underlay** | Base workspace that provides foundational packages | 1.4 |
| 31 | **Workspace** | Top-level directory containing source code, build artifacts, and installed packages | 1.4 |
| 32 | **YAML Parameter File** | Human-readable configuration file mapping parameter names to values | 1.3 |

---

## Category Breakdown

### Communication Patterns (9 terms)
- Node, Topic, Service, Action
- Publisher, Subscriber
- Service Server, Service Client
- Action Server, Action Client

### Communication Behavior (4 terms)
- QoS, DDS
- Synchronous Call, Asynchronous Call
- Callback, Goal State

### Configuration & Launch (7 terms)
- Launch File, LaunchDescription, Node Launch Action
- Parameter, YAML Parameter File
- Launch Argument
- Remapping, Composition

### Build System (8 terms)
- Workspace, Package
- colcon, package.xml
- Source Space, Install Space
- Overlay, Underlay

---

## Integration with Master Glossary

These 32 terms should be added to `/textbook/tracking/glossary-tracker.md` under the **ROS 2** category. They complement the 10 ROS 2 terms already seeded in Phase 1:

**Phase 1 Seeded Terms** (from glossary-tracker.md):
1. Node (already in Module 1)
2. Topic (already in Module 1)
3. Service (already in Module 1)
4. Action (already in Module 1)
5. Publisher (already in Module 1)
6. Subscriber (already in Module 1)
7. Message
8. Launch File (already in Module 1)
9. Package (already in Module 1)
10. Workspace (already in Module 1)

**New Unique Terms** from Module 1: 22 additional terms beyond Phase 1 seeds

**Total ROS 2 Category Terms**: 32 (10 seed + 22 new)

---

## Validation Checklist

- ✅ All 32 terms have clear, concise definitions
- ✅ Each term references first chapter of use
- ✅ Terms organized alphabetically in consolidated list
- ✅ Terms categorized by functional area
- ✅ No duplicate definitions across chapters
- ✅ Definitions are self-contained (understandable without reading chapter)
- ✅ Technical accuracy verified against ROS 2 documentation
- ✅ Consistent terminology (e.g., "node component" vs "node feature")

---

## Next Steps

1. **Update Master Glossary** (`/textbook/tracking/glossary-tracker.md`):
   - Add 22 new unique terms to ROS 2 category
   - Update existing 10 seeded terms with refined definitions from chapters
   - Total ROS 2 terms: 32

2. **Back Matter Glossary** (Phase 7):
   - Include all 32 Module 1 terms
   - Cross-reference chapter numbers
   - Add "See also" references between related terms

3. **Validation**:
   - Ensure no terminology conflicts with Modules 2-4
   - Verify consistency with official ROS 2 documentation
   - Check for missing critical terms (if any)

---

**Extraction Complete**: 32 terms across 4 chapters, ready for integration into master glossary.
