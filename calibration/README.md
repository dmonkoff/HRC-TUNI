## Calibration
In order to use the package succesfully, the robot and Kinect has to be calibrated


### Test robot-camera calibration
```
$ roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=<robot_ip>
$ roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true
$ roslaunch calibration test_calibration.launch
```
In the rviz, the virtual robot model should align well with the point cloud
![](https://i.imgur.com/gkVn3PT.png)
