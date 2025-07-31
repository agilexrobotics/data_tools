
<h3 align="center">
    <p style="font-size: 70px;">ARIO Dataset Tools</p>
</h3>
<div align="center">

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://github.com/ario-dataset/ARIO-dataset-agilex/blob/master/LICENSE)
![ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)
![ROS1](https://img.shields.io/badge/ROS-noetic-blue.svg)

</div>

---

## Introduction

These are ARIO dataset tools. Data collection, data saving, and data publishing codes are included. The data is saved in a fixed format. For details on the format, see Directory structure translation.

## Testing environment

![build](https://img.shields.io/badge/build-catkin_make-blue.svg)
[![PCL](https://img.shields.io/badge/PCL-1.10.0-blue.svg)](https://github.com/PointCloudLibrary/pcl/tree/pcl-1.10.0)
[![rapidyaml](https://img.shields.io/badge/rapidyaml-v0.5.0-blue.svg)](https://github.com/biojppm/rapidyaml/tree/b35ccb150282760cf5c2d316895cb86bd161ac89)
[![rapidjson](https://img.shields.io/badge/rapidjson-v1.1.0-blue.svg)](https://github.com/Tencent/rapidjson/tree/f54b0e47a08782a6131cc3d60f94d038fa6e0a51)

## Installation

Download our source code, and place this package in the workspace {YOUR_WS}/src.

## Build

```shell
cd {YOUR_WS}/src
git clone https://github.com/agilexrobotics/data_tools.git
git clone https://github.com/agilexrobotics/data_msgs.git
cd {YOUR_WS}
catkin_make -DCATKIN_WHITELIST_PACKAGES="data_msgs"
source devel/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```
## Run

Run the data collection code.

```bash
source ~/{YOUR_WS}/devel/setup.sh
# aloha
roslaunch data_tools run_data_capture.launch type:=aloha datasetDir:={data_path} episodeIndex:=0
# single pika
roslaunch data_tools run_data_capture.launch type:=single_pika datasetDir:={data_path} episodeIndex:=0
# double pika
roslaunch data_tools run_data_capture.launch type:=multi_pika datasetDir:={data_path} episodeIndex:=0
# single pika teleop
roslaunch data_tools run_data_capture.launch type:=single_pika_teleop datasetDir:={data_path} episodeIndex:=0
# double pika teleop
roslaunch data_tools run_data_capture.launch type:=multi_pika_teleop datasetDir:={data_path} episodeIndex:=0
```
If successful, the following status will be displayed.

```shell
path: /home/agilex/data/episode0
total time: 7.0014
topic: frame in 1 second / total frame
/camera/color/image_raw: 0 / 165
/camera_fisheye/color/image_raw: 0 / 0
/camera/depth/image_rect_raw: 0 / 165
/camera/depth/color/points: 0 / 165
/vive_pose: 0 / 0
/gripper/data: 0 / 367
/imu/data: 0 / 367
sum total frame: 1229

config topic: total frame

press ENTER to stop capture:
```

To stop data collection, press ENTER to stop. Writing to the file takes time, so you can try waiting or pressing ENTER several times.

If "Done" is displayed, it means the data just collected has been written to the file.

```shell
Done
[data_tools_dataCapture-1] process has finished cleanly
log file: /home/noetic/.ros/log/21114750-1995-11ef-b6f1-578b5ce9ba2e/data_tools_dataCapture-1*.log
all processes on machine have died, roslaunch will exit
shutting down processing monitor...
... shutting down processing monitor complete
done
```

**Note: If there is no data in the topic, the terminal output will be stuck, you need to use Ctrl+C to exit.**

## Directory structure translation

For example, dataset number 0 collected:

```shell
episode0
.
├── arm
│   ├── endPose
│   │   ├── masterLeft
│   │   │   └── 1714373556.360405.json
│   │   ├── masterRight
│   │   │   └── 1714373556.360135.json
│   │   ├── puppetLeft
│   │   │   └── 1714373556.393465.json
│   │   └── puppetRight
│   │       └── 1714373556.386106.json
│   └── jointState
│       ├── masterLeft
│       │   └── 1714373556.360460.json
│       ├── masterRight
│       │   └── 1714373556.360205.json
│       ├── puppetLeft
│       │   └── 1714373556.363036.json
│       └── puppetRight
│           └── 1714373556.363178.json
├── camera
│   ├── color
│   │   ├── front
│   │   │   ├── 1714373556.409885.png
│   │   │   └── config.json
│   │   ├── left
│   │   │   ├── 1714373556.370113.png
│   │   │   └── config.json
│   │   └── right
│   │       ├── 1714373556.358616.png
│   │       └── config.json
│   ├── depth
│   │   ├── front
│   │   │   ├── 1714373556.388914.png
│   │   │   └── config.json
│   │   ├── left
│   │   │   ├── 1714373556.353924.png
│   │   │   └── config.json
│   │   └── right
│   │       ├── 1714373556.376457.png
│   │       └── config.json
│   └── pointCloud
│       ├── front
│       │   ├── 1714373556.248885.pcd
│       │   └── config.json
│       ├── left
│       │   ├── 1714373556.247312.pcd
│       │   └── config.json
│       └── right
│           ├── 1714373556.273297.pcd
│           └── config.json
├── instructions.json
└── robotBase
    └── vel
```

[Example data](https://huggingface.co/datasets/agilexrobotics/ARIO)

## Synchronize the datasets
```shell
source ~/{YOUR_WS}/devel/setup.sh
# aloha
roslaunch data_tools run_data_sync.launch type:=aloha datasetDir:={data_path}
# single pika
roslaunch data_tools run_data_sync.launch type:=single_pika datasetDir:={data_path} 
# double pika
roslaunch data_tools run_data_sync.launch type:=multi_pika datasetDir:={data_path}
# single pika teleop
roslaunch data_tools run_data_sync.launch type:=single_pika_teleop datasetDir:={data_path}
# double pika teleop
roslaunch data_tools run_data_sync.launch type:=multi_pika_teleop datasetDir:={data_path}
```
or
```shell
source ~/{YOUR_WS}/devel/setup.sh
# aloha
python3 data_sync.py --type aloha --datasetDir {data_path}
# single pika
python3 data_sync.py --type single_pika --datasetDir {data_path}
# double pika
python3 data_sync.py --type multi_pika --datasetDir {data_path}
# single pika teleop
python3 data_sync.py --type single_pika_teleop --datasetDir {data_path}
# double pika teleop
python3 data_sync.py --type multi_pika_teleop --datasetDir {data_path}
```

After execution, a `sync.txt` file will be generated in the path of each specific dataset. For example, the image data synchronization index file path: `{data_path}/episode0/camera/color/left/sync.txt`.

## create point cloud(optional)
```shell
source ~/{YOUR_WS}/devel/setup.sh
# aloha
python3 camera_point_cloud_filter.py --type aloha --datasetDir {data_path}
# single pika
python3 camera_point_cloud_filter.py --type single_pika --datasetDir {data_path}
# double pika
python3 camera_point_cloud_filter.py --type multi_pika --datasetDir {data_path}
# single pika teleop
python3 camera_point_cloud_filter.py --type single_pika_teleop --datasetDir {data_path}
# double pika teleop
python3 camera_point_cloud_filter.py --type multi_pika_teleop --datasetDir {data_path}
```

## Convert raw data to HDF5 format

Run the following code to generate a data.hdf5 file in the path of the task0 task, which includes all datasets (episode0, episode1, ..., episodeX). This file contains the synchronized joint information and the synchronized index file of the image data.

use point cloud
```shell
source ~/{YOUR_WS}/devel/setup.sh
# aloha
python3 data_to_hdf5.py --type aloha --useCameraPointCloud true --datasetDir {data_path}
# single pika
python3 data_to_hdf5.py --type single_pika --useCameraPointCloud true --datasetDir {data_path}
# double pika
python3 data_to_hdf5.py --type multi_pika --useCameraPointCloud true --datasetDir {data_path}
# single pika teleop
python3 data_to_hdf5.py --type single_pika_teleop --useCameraPointCloud true --datasetDir {data_path}
# double pika teleop
python3 data_to_hdf5.py --type multi_pika_teleop --useCameraPointCloud true --datasetDir {data_path}
```
not use point cloud
```shell
source ~/{YOUR_WS}/devel/setup.sh
# aloha
python3 data_to_hdf5.py --type aloha --useCameraPointCloud "" --datasetDir {data_path}
# single pika
python3 data_to_hdf5.py --type single_pika --useCameraPointCloud "" --datasetDir {data_path}
# double pika
python3 data_to_hdf5.py --type multi_pika --useCameraPointCloud "" --datasetDir {data_path}
# single pika teleop
python3 data_to_hdf5.py --type single_pika_teleop --useCameraPointCloud "" --datasetDir {data_path}
# double pika teleop
python3 data_to_hdf5.py --type multi_pika_teleop --useCameraPointCloud "" --datasetDir {data_path}
```
By default, color, depth, and pointcloud in HDF5 use file indexing, so the original data files still need to be preserved. If you do not want to use indexes, you can use the following command:
```shell
source ~/{YOUR_WS}/devel/setup.sh
# aloha
python3 data_to_hdf5.py --type aloha --useCameraPointCloud "" --datasetDir {data_path} --useIndex "" --datasetTargetDir {hdf5_saving_path}
# single pika
python3 data_to_hdf5.py --type single_pika --useCameraPointCloud "" --datasetDir {data_path} --useIndex "" --datasetTargetDir {hdf5_saving_path}
# double pika
python3 data_to_hdf5.py --type multi_pika --useCameraPointCloud "" --datasetDir {data_path} --useIndex "" --datasetTargetDir {hdf5_saving_path}
# single pika teleop
python3 data_to_hdf5.py --type single_pika_teleop --useCameraPointCloud "" --datasetDir {data_path} --useIndex "" --datasetTargetDir {hdf5_saving_path}
# double pika teleop
python3 data_to_hdf5.py --type multi_pika_teleop --useCameraPointCloud "" --datasetDir {data_path} --useIndex "" --datasetTargetDir {hdf5_saving_path}
```
{hdf5_saving_path} is the path where saving your hdf5.
## How to publish data
use original data
```shell
source ~/{YOUR_WS}/devel/setup.sh
# aloha
roslaunch data_tools run_data_publish.launch type:=aloha datasetDir:={data_path} episodeIndex:=0
# single pika
roslaunch data_tools run_data_publish.launch type:=single_pika datasetDir:={data_path} episodeIndex:=0
# double pika
roslaunch data_tools run_data_publish.launch type:=multi_pika datasetDir:={data_path} episodeIndex:=0
# single pika teleop
roslaunch data_tools run_data_publish.launch type:=single_pika_teleop datasetDir:={data_path} episodeIndex:=0
# double pika teleop
roslaunch data_tools run_data_publish.launch type:=multi_pika_teleop datasetDir:={data_path} episodeIndex:=0
```
use hdf5 data
```shell
source ~/{YOUR_WS}/devel/setup.sh
# aloha
python3 data_publish.py --type aloha --datasetDir {data_path} --episodeName episode0
# single pika
python3 data_publish.py --type single_pika --datasetDir {data_path} --episodeName episode0
# double pika
python3 data_publish.py --type multi_pika --datasetDir {data_path} --episodeName episode0
# single pika teleop
python3 data_publish.py --type single_pika_teleop --datasetDir {data_path} --episodeName episode0
# double pika teleop
python3 data_publish.py --type multi_pika_teleop --datasetDir {data_path} --episodeName episode0
```
## How to load data from an HDF5 file for training

Here's an example of loading files. The paths in the program need to be manually specified as absolute paths. This code snippet does not have any actual functionality; it's just an example program. You can use this example to integrate into your own training code.

```shell
python3 load_data_example.py
```

## How to add sensors tailored to your needs
in data_tools/config
Modify the YAML configuration file to modify the topic and file name.

example:

  camera:

    color:
      names: ['left', 'front', 'right']  # file name 
      parentFrames: ['camera_l_link', 'camera_f_link', 'camera_r_link']  # camera center frame
      topics: ['/camera_l/color/image_raw', '/camera_f/color/image_raw', '/camera_r/color/image_raw']  # data topic 
      configTopics: ['/camera_l/color/camera_info', '/camera_f/color/camera_info', '/camera_r/color/camera_info']  # config topic
  arm:

    jointState:
      names: ['masterLeft', 'masterRight', 'puppetLeft', 'puppetRight']  # file name 
      topics: ['/master/joint_left', '/master/joint_right', '/puppet/joint_left', '/puppet/joint_right']  # joint state topic 
    endPose:
      names: []  # file name 
      topics: []  # `geometry_msgs::PoseStamped` topic 
      orients: []  # use quaternions, default true, set false when `geometry_msgs::PoseStamped` orient x y z is roll pitch yaw.

if your config file name is xxx_data_params.yaml, 
set:
```
python: --type xxx
roslaunch: type:=xxx
```

Ensure that all configured sensors are online and above 25Hz, otherwise data collection and synchronization will fail.

## Required sensors & data types
sensors type:

- camera/color: `sensor_msgs::Image`
- camera/depth: `sensor_msgs::Image`
- camera/pointCloud: `sensor_msgs::PointCloud2`
- arm/jointState: `sensor_msgs::JointState`
- arm/endPose: `geometry_msgs::PoseStamped`
- localization/pose: `geometry_msgs::PoseStamped`
- gripper/encoder: `data_msgs::Gripper`
- imu/9axis: `sensor_msgs::Imu`
- lidar/pointCloud: `sensor_msgs::PointCloud2`
- robotBase/vel: `nav_msgs::Odometry`

## Licence

The source code is released under [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0.html) license.
