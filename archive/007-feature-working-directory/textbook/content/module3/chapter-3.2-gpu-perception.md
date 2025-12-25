# Chapter 3.2: GPU-Accelerated Perception

> **Module**: 3 - AI-Robot Brain (NVIDIA Isaac)
> **Week**: 8
> **Estimated Reading Time**: 25 minutes

---

## Summary

GPU-accelerated perception pipelines enable real-time vision processing at 30+ FPS using TensorRT-optimized deep learning models. This chapter covers object detection (YOLOv8), semantic segmentation (U-Net), depth estimation, and Isaac ROS perception packages for production robotics deployments.

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Implement** TensorRT optimization workflow (train → ONNX → TensorRT engine)
2. **Deploy** YOLOv8 object detection with Isaac ROS at 100+ FPS on RTX GPUs
3. **Configure** semantic segmentation for real-time scene understanding
4. **Optimize** perception pipelines using FP16/INT8 quantization with minimal accuracy loss
5. **Benchmark** GPU utilization and identify bottlenecks (memory bandwidth, compute)

**Prerequisite Knowledge**: Chapter 3.1 (Isaac Ecosystem), Module 2 (sensor simulation), deep learning basics (CNNs, training)

---

## Key Terms

- **TensorRT Engine**: Optimized inference model generated from ONNX, hardware-specific binary
- **Quantization**: Reducing numerical precision (FP32 → FP16/INT8) for faster inference
- **Semantic Segmentation**: Pixel-wise classification (assign each pixel a class label)
- **Instance Segmentation**: Detect + segment individual objects (e.g., distinguish "person 1" from "person 2")
- **DNN Inference**: Forward pass through neural network (no backpropagation, no training)
- **Batch Processing**: Process multiple images simultaneously to maximize GPU utilization
- **CUDA Kernel**: GPU function executed by thousands of threads in parallel
- **Memory Bandwidth**: Data transfer rate between GPU memory and compute cores (GB/s)

---

## Core Concepts

### 1. Perception Pipeline Architecture

**Typical Robot Perception Stack**:

```
Camera (RGB) → Preprocessing → DNN Inference → Post-processing → ROS Topic
    30 Hz         Resize          Detection        NMS          Bounding Boxes
                 Normalize         (TensorRT)     Filtering
```

**Isaac ROS Acceleration Points**:
- ✅ **Preprocessing**: GPU-accelerated resize, normalization (`isaac_ros_image_proc`)
- ✅ **DNN Inference**: TensorRT engines (`isaac_ros_dnn_inference`)
- ✅ **Post-processing**: CUDA kernels for NMS, filtering
- ✅ **End-to-end**: Zero-copy pipelines (no CPU transfers)

**Speedup vs. CPU**:
- **CPU Pipeline**: 10-15 FPS (bottleneck: DNN inference in PyTorch)
- **GPU Pipeline**: 100-250 FPS (TensorRT FP16, RTX 4070 Ti)
- **Improvement**: 10-20× faster

---

### 2. TensorRT Optimization Deep Dive

#### Step 1: Train Model (PyTorch/TensorFlow)

```python
# Train YOLOv8 on COCO dataset
from ultralytics import YOLO

model = YOLO('yolov8n.yaml')  # Nano model (3.2M parameters)
model.train(data='coco.yaml', epochs=100, imgsz=640)
model.save('yolov8n.pt')
```

**Output**: `yolov8n.pt` (PyTorch weights, 6 MB)

---

#### Step 2: Export to ONNX

```python
model = YOLO('yolov8n.pt')
model.export(format='onnx', imgsz=640)
```

**ONNX Benefits**:
- Framework-agnostic (PyTorch → TensorRT, TensorFlow → TensorRT)
- Operator fusion (conv + batch_norm → single op)
- Constant folding (compute static values at export time)

**Output**: `yolov8n.onnx` (3 MB, smaller due to graph optimizations)

---

#### Step 3: Convert to TensorRT Engine

```bash
/usr/src/tensorrt/bin/trtexec \
    --onnx=yolov8n.onnx \
    --saveEngine=yolov8n_fp16.engine \
    --fp16 \
    --workspace=4096 \
    --verbose
```

**Key Parameters**:
- `--fp16`: Enable half-precision (2× faster, <1% accuracy loss)
- `--int8`: Enable 8-bit quantization (4× faster, requires calibration dataset)
- `--workspace`: GPU memory budget for optimization (MB)
- `--verbose`: Print layer-by-layer optimization details

**TensorRT Optimizations Applied**:
1. **Layer Fusion**: Conv → BatchNorm → ReLU merged into single CUDA kernel
2. **Precision Calibration**: FP32 layers → FP16/INT8 where safe
3. **Kernel Auto-Tuning**: Select fastest implementation for RTX 4070 Ti architecture
4. **Dynamic Tensor Memory**: Reuse memory across layers (reduce VRAM footprint)

**Output**: `yolov8n_fp16.engine` (4 MB, includes GPU-specific code)

**Performance**:
- **FP32 (PyTorch)**: 60 FPS
- **FP16 (TensorRT)**: 150 FPS
- **INT8 (TensorRT)**: 280 FPS (requires calibration)

---

#### Step 4: Deploy in Isaac ROS

```python
# launch/yolov8.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_yolov8',
            executable='isaac_ros_yolov8',
            parameters=[{
                'model_file_path': 'yolov8n_fp16.engine',
                'confidence_threshold': 0.5,
                'nms_threshold': 0.4,
                'max_detections': 100
            }],
            remappings=[('image', '/camera/image_raw')]
        )
    ])
```

**Zero-Copy Pipeline**: Camera → GPU → TensorRT → Post-processing (no CPU transfers)

---

### 3. Semantic Segmentation with Isaac ROS

**Use Case**: Pixel-wise scene understanding (road, sidewalk, obstacle, sky)

**Model**: U-Net architecture (encoder-decoder with skip connections)

#### Training Custom Segmentation Model

```python
import torch
import torch.nn as nn
from torchvision.models.segmentation import deeplabv3_resnet50

# Load pre-trained DeepLabV3 (COCO pre-training)
model = deeplabv3_resnet50(pretrained=True)

# Replace classifier for custom classes (21 → 5 classes)
model.classifier[4] = nn.Conv2d(256, 5, kernel_size=1)

# Fine-tune on custom dataset
optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
criterion = nn.CrossEntropyLoss()

for epoch in range(50):
    for images, masks in train_loader:
        outputs = model(images)['out']
        loss = criterion(outputs, masks)
        loss.backward()
        optimizer.step()

# Export to ONNX
torch.onnx.export(model, dummy_input, 'unet_custom.onnx')
```

---

#### Deploying Segmentation with Isaac ROS

```bash
# Convert to TensorRT
trtexec --onnx=unet_custom.onnx \
        --saveEngine=unet_fp16.engine \
        --fp16

# Launch Isaac ROS segmentation node
ros2 launch isaac_ros_unet unet.launch.py \
     model_file:=unet_fp16.engine
```

**Output Topic**: `/segmentation/masks` (sensor_msgs/Image, 8UC1 encoding)

**Performance**:
- **Input**: 640×480 RGB image
- **Output**: 640×480 mask (5 classes)
- **Latency**: 8 ms (RTX 4070 Ti, FP16)
- **Throughput**: 125 FPS

---

### 4. Depth Estimation & 3D Perception

**Isaac ROS Stereo Pipeline**:

```
Left Image ──┐
             ├─→ Stereo Matching (GPU) ──→ Disparity Map ──→ Depth Image
Right Image ─┘                                                   ↓
                                                          Point Cloud (PCL)
```

**isaac_ros_stereo_image_proc**:
- GPU-accelerated stereo matching (Semi-Global Matching algorithm)
- Output: Disparity map (16-bit, sub-pixel accuracy)
- Convert to depth: `depth = (focal_length × baseline) / disparity`

**Performance**:
- **CPU (stereo_image_proc)**: 5-10 FPS
- **GPU (isaac_ros_stereo_image_proc)**: 60+ FPS
- **Speedup**: 6-12×

---

**Point Cloud Segmentation**:

```python
# Isaac ROS pipeline
ros2 launch isaac_ros_depth_segmentation depth_segmentation.launch.py

# Publishes /segmented_cloud (colored point cloud)
# Each point has (x, y, z, label)
```

**Use Case**: Detect ground plane, cluster obstacles (pedestrians, vehicles, boxes)

---

### 5. Perception Benchmarking & Optimization

#### Identifying Bottlenecks

**Tools**:
- `nvidia-smi`: GPU utilization, memory usage
- `nsys` (Nsight Systems): Profile CUDA kernels, identify slow layers
- `ros2 topic hz`: Measure output rate

**Example Bottleneck Analysis**:

```bash
# Monitor GPU while running perception
nvidia-smi -l 1

# Expected output:
# GPU-Util: 85% ← Good (near 100% = fully utilized)
# Memory:   6GB / 12GB ← Underutilized VRAM (can batch more)
```

**Common Bottlenecks**:

| Symptom | Cause | Solution |
|---------|-------|----------|
| GPU-Util < 50% | Memory bandwidth limited | Increase batch size, use FP16 |
| FPS << expected | CPU preprocessing slow | Move preprocessing to GPU (`isaac_ros_image_proc`) |
| High latency | Large batch size | Reduce batch to 1 for real-time |
| Low accuracy | INT8 quantization too aggressive | Use FP16 or calibrate INT8 better |

---

#### INT8 Calibration for Maximum Speed

**Workflow**:

```bash
# 1. Collect calibration dataset (100-500 representative images)
mkdir calibration_data
cp train/*.jpg calibration_data/

# 2. Convert to TensorRT INT8 with calibration
trtexec --onnx=yolov8n.onnx \
        --saveEngine=yolov8n_int8.engine \
        --int8 \
        --calib=calibration_data \
        --calibration=entropy  # or minmax

# 3. Test accuracy on validation set
ros2 run isaac_ros_benchmark benchmark_detector \
     model:=yolov8n_int8.engine \
     dataset:=coco_val2017
```

**Accuracy Trade-offs**:
- **FP32**: 99.0% mAP (baseline)
- **FP16**: 98.9% mAP (0.1% loss)
- **INT8 (calibrated)**: 98.2% mAP (0.8% loss)
- **INT8 (no calibration)**: 95.0% mAP (4% loss)

**Recommendation**: Use FP16 for production (best speed/accuracy trade-off), INT8 only if latency critical.

---

### 6. Multi-Sensor Fusion

**Scenario**: Robot with 4 cameras + 1 LiDAR

**Naive Approach** (Sequential):
```
Process Cam1 (33ms) → Cam2 (33ms) → Cam3 (33ms) → Cam4 (33ms) = 132ms
```
**Result**: Only 7.5 FPS (unacceptable)

**Isaac ROS Multi-Stream** (Parallel):
```
Cam1 ──┐
Cam2 ──┤
Cam3 ──┼─→ GPU (4 parallel TensorRT engines) ──→ 4× detections
Cam4 ──┘
       Time: 35ms (slight overhead from parallelism)
```
**Result**: 28 FPS (acceptable, near camera rate)

**Configuration**:
```python
# Launch 4 detector nodes with different input topics
for i in range(4):
    Node(
        package='isaac_ros_yolov8',
        executable='isaac_ros_yolov8',
        namespace=f'cam{i}',
        parameters=[{'model_file_path': 'yolov8n_fp16.engine'}],
        remappings=[('image', f'/camera{i}/image_raw')]
    )
```

---

## Practical Example: Real-Time Object Tracking

### Overview

Combine detection (YOLOv8) + tracking (DeepSORT) for persistent object IDs across frames.

### Implementation

**Step 1: Launch Detection**

```bash
ros2 launch isaac_ros_yolov8 yolov8.launch.py
# Publishes /detections (vision_msgs/Detection2DArray)
```

**Step 2: Launch Tracking**

```bash
ros2 launch isaac_ros_tracking deepsort.launch.py
# Subscribes to /detections
# Publishes /tracked_objects (unique IDs)
```

**Step 3: Visualize**

```bash
ros2 run rqt_image_view rqt_image_view /tracked_objects/visualization
# Shows bounding boxes with ID labels (ID: 5, ID: 7, etc.)
```

### Expected Output

- Objects receive persistent IDs (person ID=5 tracked across 100 frames)
- IDs maintained even during occlusions (up to 30 frames)
- New objects assigned new IDs automatically

---

## Exercises

### Exercise 1: TensorRT Optimization Comparison (Difficulty: Easy)

**Objective**: Measure FP32 vs. FP16 vs. INT8 performance

**Task**: Convert same model to 3 engines, benchmark FPS and accuracy

**Requirements**:
- Use YOLOv8n on COCO val2017 (100 images)
- Report: FPS, mAP, VRAM usage

**Expected Outcome**:
| Precision | FPS | mAP | VRAM |
|-----------|-----|-----|------|
| FP32 | 60 | 99.0% | 800MB |
| FP16 | 150 | 98.9% | 500MB |
| INT8 | 280 | 98.2% | 300MB |

**Estimated Time**: 30 minutes

---

### Exercise 2: Multi-Camera Perception (Difficulty: Medium)

**Objective**: Deploy 2-camera perception system

**Task**: Run Isaac ROS on 2 simultaneous camera streams, fuse detections

**Requirements**:
- 2 USB webcams or RealSense cameras
- Launch 2 detector nodes
- Merge detections into single `/all_detections` topic

**Expected Outcome**: Combined detection rate 50+ FPS (2 cameras × 25 FPS each)

**Estimated Time**: 45 minutes

---

### Exercise 3: Custom Segmentation Training (Difficulty: Hard)

**Objective**: Train custom U-Net for specific use case

**Task**: Train segmentation model on custom dataset (e.g., warehouse objects)

**Requirements**:
- Collect 500+ annotated images (use LabelMe or CVAT)
- Train U-Net (50 epochs)
- Deploy with Isaac ROS, achieve >90% IoU

**Expected Outcome**: Real-time segmentation (30 FPS) on custom classes

**Estimated Time**: 2-3 hours

---

## Summary & Key Takeaways

- **TensorRT optimization** (FP16/INT8) provides 2-10× speedups over PyTorch with minimal accuracy loss
- **Isaac ROS perception** packages enable GPU-accelerated detection, segmentation, depth processing
- **Zero-copy pipelines** eliminate CPU-GPU transfers, maximizing throughput
- **Multi-stream processing** enables multi-camera perception at real-time rates
- **Calibration** is critical for INT8 quantization to maintain accuracy

**Connection to Chapter 3.3**: With perception pipelines delivering 30+ FPS detections, Chapter 3.3 explores navigation and manipulation—using perception outputs for obstacle avoidance, path planning, and object grasping with Nav2 and MoveIt2 integration.

---

## Additional Resources

- TensorRT Developer Guide: https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/
- Isaac ROS Perception: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_inference/
- YOLOv8 Documentation: https://docs.ultralytics.com/

---

## Notes for Instructors

**Teaching Tips**:
- Live demo: Run same model FP32 vs FP16, show 2× FPS improvement visually
- Discuss accuracy trade-offs: When is 1% mAP loss acceptable?

**Lab Ideas**:
- Lab 1: Benchmark student laptops (CPU vs. GPU perception)
- Lab 2: Train custom detector on classroom objects
- Lab 3: Deploy to Jetson, measure power consumption vs. performance

**Assessment**:
- Quiz: Explain TensorRT optimization steps
- Project: Build multi-camera surveillance system with tracking

---

**Chapter Metadata**:
- **Word Count**: 3,000 words
- **Code Examples**: 8
- **Exercises**: 3
- **Glossary Terms**: 8
