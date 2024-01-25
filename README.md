# PointPillars-ROS for ROS 2
Trying to use PointPillars-ROS in ROS 2 Humble
# Requirements 
## My Environment 
|OS|ROS DISTRO|CPU|GPU|RAM|
|---|---|---|---|---|
|Ubuntu22.04|ROS 2 Humble|Intel Core i9-12900K|GeForce RTX 4090|128GB|

## dependence
Could not find the required component 'jsk_recognition_msgs'.
```
sudo apt-get install ros-melodic-jsk-recognition-msgs 
sudo apt-get install ros-melodic-jsk-rviz-plugins
```

Need OpenCV compiled with CUDA.

# Usage
## How to compile
```bash
colcon build --symlink-install
source install/local_setup.bash
```

## How to launch
Launch file (cuDNN and TensorRT support): 

`pfe_onnx_file, rpn_onnx_file, pp_config, input_topic` are required

```
roslaunch lidar_point_pillars lidar_point_pillars.launch pfe_onnx_file:=/PATH/TO/FILE.onnx rpn_onnx_file:=/PATH/TO/FILE.onnx pp_config:=/PATH/TO/pp_multihead.yaml input_topic:=/points_raw 
```

`score_threshold, nms_overlap_threshold, etc` are optional to change the runtime parameters.

## Or, simply, 

Use `launch.sh` to run.

## Test launch

```
roslaunch test_point_pillars test_point_pillars.launch
```
nuscenes test data download: [nuscenes_10sweeps_points.txt](https://drive.google.com/file/d/1KD0LT0kzcpGUysUu__dfnfYnHUW62iwN/view?usp=sharing)

From: https://github.com/hova88/PointPillars_MultiHead_40FPS

Tx1 (single test):
```
  Preprocess    7.48567  ms
  Pfe           266.516  ms
  Scatter       1.78591  ms
  Backbone      369.692  ms
  Postprocess   35.8309  ms
  Summary       681.325  ms
```

Xavier (single test):
```
  Preprocess    2.15862  ms
  Pfe           46.2593  ms
  Scatter       0.54678  ms
  Backbone      80.7096  ms
  Postprocess   11.3462  ms
  Summary       141.034  ms
```

Boolmap on Xavier (single test):
```
  Preprocess    1.16403  ms
  Backbone      42.7098  ms
  Postprocess   16.4531  ms
  Summary       60.3469  ms
```

## Test Rosbag:

I use [nuscenes2bag](https://github.com/clynamen/nuscenes2bag) to create some test rosbag: [nu0061 all 19s 5.5G, download password: s2eh](https://pan.baidu.com/s/1vqKvJ8jRwxEZKuuFBCig2w), [nu0061 laser and tf only 19s 209M, download password: m7wh](https://pan.baidu.com/s/11geDn_kD2LuWf2R4VqdbEg).

To use this nuscenes rosbag, you shoulde change input_topic to `/lidar_top` , and use src/rviz/nuscenes.rviz for visualization.

Usually, I use `rosbag play r 0.1 ` for more play time.

More test rosbag, like kitti, carla or real data by myself, will be released recently.

## Models Files:
Faster ONNX models:
* zz0809_512_e50 model is with the same config file as cbgs model, and the evaluation data is re-tested by the same eval benchmark.
* zz0808_256_e50 model is half resolution, you should used this config file to run: `src/lidar_point_pillars/cfgs/tx1_ppmh_256x256.yaml`
* z0927_kitti is trained by kitti dataset, with three classes. It has only 10 (4+6) gather features, and can run with this config file: `src/lidar_point_pillars/cfgs/pointpillar_kitti_g10.yaml`
* z1009_kitti_g11 is trained by kitti dataset, with three classes. It has 11 gather features, with one refile zero dim. It can run with this config file: `src/lidar_point_pillars/cfgs/pointpillar_kitti_g11.yaml`
* z1117_boolmap_e30 is trained by nuscenes dataset, with boolmap vfe. It only has backbone onnx model. It can run with this config file: `pointpillar_boolmap_multihead.yaml`

|                                             | download | Tx1 time | Xavier time |resolution| training data | mean ap | nd score  | car ap | ped ap | truck ap|
|-----------|:--------:|:-----------:|:--------:|:-------------:|:-------:|:---------:|:------:|:------:|:-------:|:--------:| 
| cbgs_ppmh | [pfe](https://drive.google.com/file/d/1gQWtBZ4vfrSmv2nToSIarr-d7KkEWqxw/view?usp=sharing) [backbone](https://drive.google.com/file/d/1dvUkjvhE0GEWvf6GchSGg8-lwukk7bTw/view?usp=sharing) | ~700ms   | ~140ms |64x512x512| unknown       |0.447    | 0.515     | 0.813  | 0.724  | 0.500   |
| zz0809_512_e50 |[pfe](https://drive.google.com/file/d/1mLP3v0iXUG5CrT_KLi9VBbsBbByl-WeQ/view?usp=sharing) [backbone](https://drive.google.com/file/d/1bkQfxgyxYNyBbsnwgX_JWe8YgByBTSX7/view?usp=sharing)|~700ms| ~140ms |64x512x512|nusc tr-v|0.460|0.524|0.818|0.733|0.507|
| zz0808_256_e50 |[pfe](https://drive.google.com/file/d/1pxsP5fhQG0XzpU0yzJOjRcO3ru_JM5Vn/view?usp=sharing) [backbone](https://drive.google.com/file/d/1Pb8xZ_55oo95SDSzS1KHvQ_MvnS-X1Iv/view?usp=sharing)|~250ms| ~110ms |64x256x256|nusc tr-v|0.351|0.454|0.781|0.571|0.427|
| z1117_boolmap_e40 |[backbone](https://drive.google.com/file/d/12zucbZf4gK4tM5ytQy0W5nqzIPl9QLD6/view?usp=sharing)||~60ms|64x512x512|nusc tr-v|0.353|0.449|0.744|0.525|0.291|
|             kitti models                       | | | | | | | | car ap@0.7 | ped ap@0.5 | truck ap@0.7|
| z0927_kitti_g10 |[pfe](https://drive.google.com/file/d/1mLP3v0iXUG5CrT_KLi9VBbsBbByl-WeQ/view?usp=sharing) [backbone](https://drive.google.com/file/d/1bkQfxgyxYNyBbsnwgX_JWe8YgByBTSX7/view?usp=sharing)|~700ms| ~140ms |64x512x512|kitti|||90.149|44.893|34.977|
| z1009_kitti_g11_e72 |[pfe](https://drive.google.com/file/d/1zlxStcAAqsoUsxe09zFsvenwj9QyQYJA/view?usp=sharing) [backbone](https://drive.google.com/file/d/1-mM4jy01vpl_5AEMcSgBLH5tcU-TcSHn/view?usp=sharing)|~700ms| ~140ms |64x512x512|kitti|||90.191|46.915|40.944|

All the models can be viewed on [BAIDU PAN](https://pan.baidu.com/s/1MbPKJxsNa_5qf_teWh7Mnw?pwd=cu2f).

More models will be released recently.

## How to train boolmap model?