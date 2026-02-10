---
sidebar_position: 3
title: Chapter 3.2 - Isaac Perception Pipeline
---

# Chapter 3.2: Isaac Perception Pipeline

**Module**: 3 - The AI-Robot Brain
**Week**: 10
**Estimated Reading Time**: 30 minutes

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. Deploy TensorRT-optimized object detection models (YOLO, EfficientDet)
2. Convert PyTorch/ONNX models to TensorRT engines
3. Run real-time object detection at 30+ FPS
4. Integrate Isaac perception with ROS 2 pipelines
5. Measure and optimize inference performance

---

## Prerequisites

- Completed Chapter 3.1 (Isaac Overview)
- Isaac ROS packages installed
- Understanding of deep learning inference

---

## Introduction

**GPU-accelerated perception** is critical for real-time robotics. This chapter covers deploying object detection models with Isaac ROS, achieving 30-120 FPS inference speeds using TensorRT optimization.

---

## Key Terms

:::info Glossary Terms
- **DNN Inference**: Running a trained neural network on new data
- **ONNX**: Open Neural Network Exchange format
- **INT8 Quantization**: Reducing model precision for speedup
- **Bounding Box**: Rectangle enclosing detected object
- **Non-Maximum Suppression (NMS)**: Removing duplicate detections
:::

---

## Core Concepts

### 1. Object Detection Models

#### YOLOv8 (Recommended)

**Architecture**: Single-stage detector (fast)
**Variants**:
- YOLOv8n (nano): 3.2MB, 120 FPS
- YOLOv8s (small): 11MB, 85 FPS
- YOLOv8m (medium): 26MB, 50 FPS

**Installation**:
```bash
pip install ultralytics
```

**Training**:
```python
from ultralytics import YOLO

# Load pretrained model
model = YOLO('yolov8n.pt')

# Train on custom dataset
results = model.train(
    data='custom_data.yaml',
    epochs=100,
    imgsz=640,
    batch=16
)

# Export to ONNX
model.export(format='onnx')
```

#### EfficientDet (High Accuracy)

**Use Case**: When accuracy > speed
**Performance**: 45 mAP @ 20 FPS (D0 variant)

### 2. Model Optimization with TensorRT

#### Conversion Pipeline

```
PyTorch (.pt) → ONNX (.onnx) → TensorRT (.engine)
```

**Step 1: Export to ONNX**
```python
import torch
model = torch.load('yolov8n.pt')
dummy_input = torch.randn(1, 3, 640, 640)

torch.onnx.export(
    model,
    dummy_input,
    'yolov8n.onnx',
    opset_version=17,
    input_names=['images'],
    output_names=['output0']
)
```

**Step 2: Build TensorRT Engine**
```bash
trtexec --onnx=yolov8n.onnx \
        --saveEngine=yolov8n.engine \
        --fp16 \
        --workspace=4096  # 4GB workspace
```

**Optimization Techniques**:

1. **FP16 Precision** (2× speedup):
```bash
trtexec --onnx=model.onnx --saveEngine=model_fp16.engine --fp16
```

2. **INT8 Quantization** (4× speedup):
```bash
trtexec --onnx=model.onnx --saveEngine=model_int8.engine --int8 \
        --calib=calibration_data/
```

3. **Dynamic Shapes**:
```bash
trtexec --onnx=model.onnx --saveEngine=model.engine \
        --minShapes=images:1x3x320x320 \
        --optShapes=images:1x3x640x640 \
        --maxShapes=images:1x3x1280x1280
```

**Performance Comparison**:
| Precision | Inference Time | Accuracy Loss |
|-----------|----------------|---------------|
| FP32 | 22ms | 0% (baseline) |
| FP16 | 11ms | <0.5% |
| INT8 | 6ms | 1-2% |

### 3. Isaac ROS DNN Inference

#### Pipeline Architecture

```
Camera → DNN Encoder → TensorRT Node → Decoder → Detections
  (ROS)      (GPU)         (GPU)         (GPU)       (ROS)
```

**Launch File**:
```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='detection_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Preprocess images
                ComposableNode(
                    package='isaac_ros_dnn_image_encoder',
                    plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
                    name='dnn_encoder',
                    parameters=[{
                        'input_image_width': 640,
                        'input_image_height': 640,
                        'network_image_width': 640,
                        'network_image_height': 640,
                    }]
                ),

                # TensorRT inference
                ComposableNode(
                    package='isaac_ros_tensor_rt',
                    plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                    name='tensor_rt',
                    parameters=[{
                        'engine_file_path': '/models/yolov8n_fp16.engine',
                    }]
                ),

                # Decode detections
                ComposableNode(
                    package='isaac_ros_yolov8',
                    plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
                    name='yolov8_decoder',
                    parameters=[{
                        'confidence_threshold': 0.5,
                        'nms_threshold': 0.45,
                    }]
                )
            ]
        )
    ])
```

**Run**:
```bash
ros2 launch my_package yolov8_detection.launch.py
```

### 4. Real-Time Performance

#### Achieving 30+ FPS

**Optimization Checklist**:

1. **Use Composable Nodes** (avoid inter-process communication):
```python
# Good: Composable nodes in same container
ComposableNodeContainer(...)

# Bad: Separate nodes (IPC overhead)
Node(package='pkg1', ...)
Node(package='pkg2', ...)
```

2. **Minimize Data Copying**:
```python
# Use zero-copy transport
parameters=[{'use_intra_process_comms': True}]
```

3. **GPU Memory Management**:
```python
# Preallocate buffers
parameters=[{
    'dla_core': -1,  # Use GPU, not DLA
    'enable_fp16': True,
}]
```

4. **Profiling**:
```bash
ros2 run ros2_tracing trace
```

**Expected Performance** (RTX 3060):
| Model | Resolution | FPS | Latency |
|-------|------------|-----|---------|
| YOLOv8n FP16 | 640×640 | 120 | 8ms |
| YOLOv8s FP16 | 640×640 | 85 | 12ms |
| YOLOv8m FP16 | 640×640 | 50 | 20ms |

---

## Practical Example: Warehouse Object Detection

**Scenario**: Detect boxes on conveyor belt

**Step 1: Prepare Dataset**

```yaml
# custom_data.yaml
path: /data/warehouse
train: images/train
val: images/val

names:
  0: cardboard_box
  1: wooden_pallet
  2: plastic_crate
```

**Step 2: Train YOLOv8**

```bash
yolo train model=yolov8n.pt data=custom_data.yaml epochs=100 imgsz=640
yolo export model=runs/train/exp/weights/best.pt format=onnx
```

**Step 3: Convert to TensorRT**

```bash
trtexec --onnx=best.onnx --saveEngine=warehouse_yolov8.engine --fp16
```

**Step 4: Deploy with Isaac ROS**

```python
# warehouse_detection_node.py
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

class WarehouseDetector(Node):
    def __init__(self):
        super().__init__('warehouse_detector')
        self.sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )
        self.box_count = 0

    def detection_callback(self, msg):
        for det in msg.detections:
            class_id = det.results[0].hypothesis.class_id
            confidence = det.results[0].hypothesis.score

            if class_id == 'cardboard_box' and confidence > 0.8:
                self.box_count += 1
                self.get_logger().info(f'Box #{self.box_count} detected!')
```

**Expected Results**:
- Detection Rate: 95%+ accuracy
- FPS: 90+ on RTX 3060
- Latency: <15ms end-to-end

---

## Performance Benchmarking

### GPU Utilization

```bash
# Monitor GPU
nvitop

# Expected during inference:
#   GPU Util: 85-95%
#   Memory: 2-4 GB
#   Power: 80-120W
```

### Latency Breakdown

```python
# Enable profiling in TensorRT node
parameters=[{
    'enable_profiling': True,
    'profiling_verbosity': 'detailed'
}]
```

**Typical Latency** (640×640 input):
- Image preprocessing: 1-2ms
- TensorRT inference: 6-10ms
- Postprocessing (NMS): 1-2ms
- **Total**: 8-14ms (60-120 FPS)

---

## Common Issues

### Issue 1: Low FPS Despite GPU

**Symptom**: FPS < 30, GPU util < 50%

**Cause**: CPU preprocessing bottleneck

**Solution**: Use GPU preprocessing (DNN Encoder)
```python
ComposableNode(
    package='isaac_ros_dnn_image_encoder',  # GPU-accelerated
    ...
)
```

### Issue 2: Out of Memory

**Symptom**: `CUDA out of memory`

**Solution**: Reduce batch size or use smaller model
```bash
# Use YOLOv8n instead of YOLOv8m
trtexec --onnx=yolov8n.onnx ...
```

### Issue 3: Low Accuracy

**Symptom**: Many false positives/negatives

**Solution**:
1. Increase training data (1000+ images per class)
2. Use data augmentation
3. Tune confidence threshold:
```python
parameters=[{'confidence_threshold': 0.7}]  # Increase from 0.5
```

---

## Summary

This chapter covered **Isaac Perception for GPU-accelerated object detection**:

1. **YOLOv8**: State-of-the-art real-time object detection
2. **TensorRT**: 3-5× speedup with FP16/INT8 quantization
3. **Isaac ROS**: Seamless integration with ROS 2 ecosystem
4. **Performance**: 30-120 FPS on NVIDIA GPUs

**Key Takeaways**:
- Always use TensorRT for production deployment
- FP16 provides best speed/accuracy tradeoff
- Composable nodes reduce latency
- Profile with nvitop to verify GPU utilization

**Next Chapter**: Isaac Manipulation & Navigation

---

## End-of-Chapter Exercises

### Exercise 1: Deploy YOLOv8 (Difficulty: Medium)

**Tasks**:
1. Convert pretrained YOLOv8n to TensorRT
2. Launch Isaac ROS detection pipeline
3. Run on webcam feed
4. Measure FPS

**Success Criteria**: FPS > 60

### Exercise 2: Train Custom Detector (Difficulty: Hard)

**Tasks**:
1. Collect 500+ images of 3 custom objects
2. Annotate with CVAT
3. Train YOLOv8
4. Deploy with Isaac ROS

**Success Criteria**: mAP > 80%

---

## Further Reading

1. Isaac ROS Object Detection: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_object_detection/
2. YOLOv8 Documentation: https://docs.ultralytics.com/
3. TensorRT Best Practices: https://docs.nvidia.com/deeplearning/tensorrt/best-practices/

---

## Next Chapter

Continue to **[Chapter 3.3: Isaac Manipulation and Navigation](./chapter3-3-isaac-manipulation-nav)** to integrate perception with robot control.
