# Wearable AR (Hololens) User Interface
![ar_setup](https://i.imgur.com/VAcQsn5.png)

## Descriptions
This repository contains code for wearable AR (Hololens) user interface. It has the same UI components as the [projector-based](https://github.com/Herrandy/HRC-TUNI/tree/master/projector) but as a 3D holograms.

## Dependencies
To succesfully build the project the following software development kits must be installed (download the specific version):
- Microsoft Visual Studio
- Unity 3D editor 2017.4.1f1
- Vuforia Engine v7.5.20

## Configuration
An artifical marker is used to extrinsically calibrate Hololens and the robot. The distance of the marker center point to the robot base frame has to be manually measured and changed to the code.
_work in progress_

## Running a simple demo

1: Start the robot driver: ```roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=<real robot 192.168.125.100 or simulated 127.0.0.1>```

2: Start the HoloRobo application from the Hololens application list.

3: Establish the wireless communication channel between the computer and Hololens: ```rosrun ar tcp_server.py```


