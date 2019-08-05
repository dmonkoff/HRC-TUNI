# Projector-Mirror User Interface

<p float="left">
  <img width="420" height="260" src="https://i.imgur.com/jJ4nFwq.jpg">
  <img width="420" height="370" src="https://i.imgur.com/PpHlDED.png">
</p>

## Description
This repository contains libraries and tools for projection-based user interface. 
To replicate the research work a standard 3LCD projector must be installed above the working area pointing donwards to the shared workspace. A mirror can be used to expand the projection area. To be able to use the interactive buttons the Kinect V2 sensor must be installed next to the projector. 

Currently the module contains the following example UI components:
- a dynamic danger zone isolating the robot and the manipulated object
- _GO_ and _STOP_ button to start and stop the robot
- _CONFIRM_ button to verify and add changed regions to the current model
- _ENABLE_ button that needs to be pressed simultaneously with the _GO_ and _CONFIRM_ buttons to take effect
- a graphical display box (image and text) to show the robot status and instructions to the operator

## Configuration
The module contains example configurations for the UI components (see _unity_msgs_ folder). In addition to succesfully use the module in a real environment the homography matrix between image plane and the work table must be estimated. 

## Run the module
1: Start the robot driver: ```roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=<real robot 192.168.125.100 or simulated 127.0.0.1>```

2: Start the camera driver: ```roslaunch kinect2_bridge kinect2_bridge.launch max_depth:=<max depth in meters>```

3: Start the projector display: ```rosrun projector projector_interface.py```

4: Start the button monitor: ```rosrun projector handle_interaction_markers``` 


## Additional tools
Move and modify UI components: ```rosrun place_interface_components.py```
