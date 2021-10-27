# Realsense utils

Tested on:

* ROS Noetic
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
sudo apt-get install ros-noetic-realsense2-camera
```

Ensure you don't have the apt apriltag package installed:

``` bash
sudo apt-get remove ros-noetic-apriltag*
```

## Clone and build

``` bash
cd <your_workspace>/src
git clone -b noetic-devel https://github.com/edgarcamilocamacho/realsense_utils
cd ..
catkin_make
```

## Run

Realsense:

``` bash
roslaunch realsense2_camera rs_camera.launch
```

``` bash
roslaunch camera_pose_estimator continuous_detection.launch
```

``` bash
rosrun camera_pose_estimator apriltags_tf.py
```

Then check in rviz.