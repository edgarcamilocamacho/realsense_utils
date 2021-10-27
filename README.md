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

## ROS Packages

``` bash
sudo apt-get install ros-noetic-apriltag-ros
sudo apt-get install ros-noetic-realsense2-camera
```
