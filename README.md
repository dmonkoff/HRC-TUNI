# HRC-TUNI
## Maintainer
- [Antti Hietanen](https://research.tuni.fi/vision/) <<antti.hietanen@tuni.fi>>, [Tampere University](https://www.tuni.fi/en)


## Description
This is a collection of tools and libraries for a ROS Interface to the Kinect One (Kinect v2).
It contains:
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
If you use our work, please cite it.
```tex
@article{hietanen2019RCIM,
  title={AR-based interaction for human-robot collaborative manufacturing},
  author={Hietanen, Antti and Pieters, Roel and  Lanz, Minna and Latokartano, Jyrki and K{\"a}m{\"a}r{\"a}inen, J.-K},
  journal={RCIM},
  note = {to appear},
}
```
