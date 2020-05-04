## Calibration
In order to use the package succesfully, the robot and Kinect has to be calibrated.

Attach red dot marker on the robot end effector (see Fig. ) and calculate the offset from end effector to the 
marker center using for instance [this library](https://github.com/Jmeyer1292/tool_point_calibration).
Finally add the offset to *calibration_kinect2robot.launch* and launch:
```
# c to store calibration point, s to calculate transformation
$ roslaunch calibration calibration_kinect2robot.launch
```
Copy the calcualted transfromation to *tf_broacast.launch*
### Test robot-camera calibration
```
$ roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=<robot_ip>
$ roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true
$ roslaunch calibration test_calibration.launch
```
In the rviz, the virtual robot model should align well with the point cloud
![](https://i.imgur.com/gkVn3PT.png)
