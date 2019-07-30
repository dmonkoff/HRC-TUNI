# HRC-TUNI
## Maintainer
- [Antti Hietanen](https://research.tuni.fi/vision/) <<antti.hietanen@tuni.fi>>, [Tampere University](https://www.tuni.fi/en)


## Description
This repository contains libraries and tools for a depth-sensor based model for workspace monitoring and an interactive Augmented Reality (AR) User Interface (UI) for safe HRC. The AR UI is implemented on two different hardware: a projector-mirror setup and a wearable AR gear (HoloLens). 
The repository contains:
- [a projection-based user interface](kinect2_calibration)
- [a head-mounted AR (Hololens) user interface](kinect2_registration) Head-mounted AR display (Hololens) 
- [a depth-based safety system](kinect2_bridge) a depth-based model for ensuring safe human-robot collaboration

## Dependencies
- ROS Melodic
- OpenCV (2.4.x, using the one from the official Ubuntu repositories is recommended)
- PCL (1.7.x, using the one from the official Ubuntu repositories is recommended)
- [iai_kinect2](https://github.com/code-iai/iai_kinect2)

## Install

1. Install ROS. [Instructions for Ubuntu 18.04](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. Install [iai_kinect2](https://github.com/code-iai/iai_kinect2):
3. Clone this repository into your catkin workspace, install the dependencies and build it:
    ```bash
    cd ~/catkin_ws/src/
    git clone https://github.com/Herrandy/HRC-TUNI.git
    cd HRC-TUNI
    rosdep install -r --from-paths .
    cd ~/catkin_ws
    catkin_make -DCMAKE_BUILD_TYPE="Release"
    ```
## Citation
This is the reference implementation for the paper:

**_AR-based interaction for human-robot collaborative manufacturing_** _A. Hietanen, R. Pieters, M. Lanz, J. Latokartano, J.-K. Kämäräinen_ 

[PDF](https://arxiv.org/pdf/1803.09956.pdf) | [Video](http://vpg.cs.princeton.edu/)


If you find this code useful in your work, please consider citing:
```tex
@article{hietanen2019RCIM,
  title={AR-based interaction for human-robot collaborative manufacturing},
  author={Hietanen, Antti and Pieters, Roel and  Lanz, Minna and Latokartano, Jyrki and K{\"a}m{\"a}r{\"a}inen, J.-K},
  journal={RCIM},
  note = {to appear},
}
```
