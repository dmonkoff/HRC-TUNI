# Depth-based safety model
![zones](https://i.imgur.com/qtoepZ8.png)

## Description
This repository contains the code for depth-based model for HRC. The model is based on three zones, human, robot and danger, and their continuous update and safety monitoring. The module is intended to be used together with the projector- or holographic-based user interfaces.

## Launching the module
1. Start the robot: ```roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=<real robot 192.168.125.100 or simulated 127.0.0.1>```
2. Start the camera driver: ```roslaunch kinect2_bridge kinect2_bridge.launch max_depth:=<max depth in meters>```
3. Start the safety model: ```roslaunch hololens detect.launch safety_map_scale:=100 cluster_tolerance:=0.005 min_cluster_size:=200 viz:=false cloud_diff_threshold:=0.02```

Sensitivity of the model is defined by the following parameters:

- ```safety_map_scale``` Size of the depth map in pixels.
- ```cluster_tolerance``` The spatial cluster tolerance in the Euclidean clustering algorithm (i.e. lower value means less sensitive).
- ```min_cluster_size``` The minimum number of points that a cluster needs to contain in order to be considered valid (i.e. higher value will ignore smaller changes in the workspace).
- ```cloud_diff_threshold``` The minimum L2 distance between two depth pixels to be considered as a change.  




