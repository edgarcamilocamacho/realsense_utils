# Realsense utils

Tested on:

* ROS Foxy
* Ubuntu 20.04 / Linux Mint 20.2

## Components

* Camera Pose Estimation from AprilTags detection

## Requisites

### Realsense SDK

Instructions [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md).

Notes:

* If using Mint, replace `$(lsb_release -cs)` by `focal`.
* run `sudo apt-get update` just after `sudo add-apt-repository ...`.

### ROS Packages

``` bash
sudo apt-get install ros-foxy-realsense2-camera
sudo apt-get remove ros-doxy-apriltag
```

## Clone and build

``` bash
cd <your_workspace>/src
git clone -b foxy-devel https://github.com/edgarcamilocamacho/realsense_utils
cd ..
colcon build
```

## Run

### Only realsense and optical AprilTags

```
ros2 launch camera_pose_estimator optical_apriltag.launch.py launch_rs:=True use_gui:=True
```

### Optical AprilTag detection

Realsense:

``` bash
ros2 launch camera_pose_estimator camera_pose_estimator.launch.py launch_rs:=True use_gui:=True
```

