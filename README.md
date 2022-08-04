# PointPillars-ROS
A 3D detection Pointpillars ROS deployment on Nvidia Jetson TX1/TX2

This repo implements https://github.com/hova88/PointPillars_MultiHead_40FPS into Autoware lidar_point_pillars framework https://github.com/autowarefoundation/autoware_ai_perception/tree/master/lidar_point_pillars

# Requirements (My Environment)

```
Ubuntu 18.04
ROS Melodic

jetson_release
 - NVIDIA Jetson TX1
   * Jetpack 4.5.1 [L4T 32.5.1]
   * NV Power Mode: MAXN - Type: 0
   * jetson_stats.service: active
 - Libraries:
   * CUDA: 10.2.89
   * cuDNN: 8.0.0.180
   * TensorRT: 7.1.3.0
   * Visionworks: 1.6.0.501
   * OpenCV: 3.4.5 compiled CUDA: YES
   * VPI: ii libnvvpi1 1.0.15 arm64 NVIDIA Vision Programming Interface library
   * Vulkan: 1.2.70
```

# Usage
## How to compile

Simply, use catkin_make build up the whole project.

## How to launch
Launch file (cuDNN and TensorRT support): 

`pfe_onnx_file, rpn_onnx_file, pp_config, input_topic` are required

```
roslaunch lidar_point_pillars lidar_point_pillars.launch pfe_onnx_file:=/PATH/TO/FILE.onnx rpn_onnx_file:=/PATH/TO/FILE.onnx pp_config:=/PATH/TO/pp_multihead.yaml input_topic:=/points_raw 
```

## Test launch

```
roslaunch test_point_pillars test_point_pillars.launch
```
nuscenes test data download: [nuscenes_10sweeps_points.txt](https://drive.google.com/file/d/1KD0LT0kzcpGUysUu__dfnfYnHUW62iwN/view?usp=sharing)

## Models Files:
From: https://github.com/hova88/PointPillars_MultiHead_40FPS

[cbgs_pp_multihead_pfe.onnx](https://drive.google.com/file/d/1gQWtBZ4vfrSmv2nToSIarr-d7KkEWqxw/view?usp=sharing)

[cbgs_pp_multihead_backbone.onnx](https://drive.google.com/file/d/1dvUkjvhE0GEWvf6GchSGg8-lwukk7bTw/view?usp=sharing)
