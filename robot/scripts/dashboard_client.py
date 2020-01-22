#!/usr/bin/env python
import numpy as np
import yaml
from ur5_kinematics import ur5

# ROS
import rospy
import tf2_ros
import rospkg
from unity_msgs.msg import MarkerDataArray
from unity_msgs.msg import NearestObject
from unity_msgs.msg import ManipulatedObject
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from control_msgs.msg import *
from std_msgs.msg import *

dashboard_cmd = [
    "play\n",
    "pause",
    "stop",
]

rospy.init_node('robot_node', anonymous=True)
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

class DashboardClientROS:
    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('unity_msgs')
        with open(self.package_path+"/configs/config.yaml", 'r') as ymlfile:
            self.cfg = yaml.safe_load(ymlfile)
        rospy.loginfo("Starting dashboard_client")
        rospy.on_shutdown(self.cleanup)
        self.interaction_sub_ = rospy.Subscriber("/unity/interaction_markers",
                                                 MarkerDataArray,
                                                self.cb_interaction_marker, queue_size=1)
        self.dashboard_cmd_pub_ = rospy.Publisher("/ur_driver/dashboard_command", String, queue_size=1)
        self.nearest_object_sub_ = rospy.Subscriber("/unity/nearest_object", NearestObject, self.cb_nearest_object)
        self.object_pose_pub_ = rospy.Publisher("/unity/manipulated_object", ManipulatedObject, queue_size=1)
        self.set_safety_system_mode_pub_ = rospy.Publisher("/unity/system_mode", String, queue_size=1)
        self.stop_ = True
        self._robot_kin = ur5()
        self._current_joint_values = None
        self._joint_sub = rospy.Subscriber('joint_states', JointState, self.cb_joint_state)
        rospy.sleep(0.5)
        rospy.loginfo("Initialization done!")

    def cleanup(self):
        rospy.loginfo("Shutting ROS dashboard client")

    def cb_interaction_marker(self, marker_data_array):
        stop_temp = False
        go_temp = False
        confirm_temp = False
        dead_man_temp = False
        for marker_data in marker_data_array.markers:
            # for debugging
            if marker_data.id == 'force_go':
                self.dashboard_cmd_pub_.publish(dashboard_cmd[0])
                rospy.loginfo("Force go!")
                self.stop_ = False
                return

            if marker_data.id == 'stop':
                if marker_data.data > self.cfg['interaction_button_thres']:
                    rospy.loginfo("Stop pressed!")
                    stop_temp = True

            if marker_data.id == 'go':
                if marker_data.data > self.cfg['interaction_button_thres']:
                    rospy.loginfo("Go pressed!")
                    go_temp = True

            if marker_data.id == 'confirm':
                if marker_data.data > self.cfg['interaction_button_thres']:
                    rospy.loginfo("Confirm pressed!")
                    confirm_temp = True

            if marker_data.id == 'dead_man':
                if marker_data.data > self.cfg['interaction_button_thres']:
                    rospy.loginfo("Dead_man pressed!")
                    dead_man_temp = True

        # if stop was pressed, we stop immediately
        if stop_temp:
            self.stop_ = True
            self.dashboard_cmd_pub_.publish(dashboard_cmd[1])
            return

        # if both buttons, confirm and go is pressed, return (only one button is allowed to be pressed)
        if go_temp and confirm_temp:
            self.stop_ = False
            rospy.loginfo("Go and confirm simultaneously pressed!")
            return

        # robot will only start if dead_man is also pressed
        if dead_man_temp and go_temp and self.stop_:
            print("Sending move command!")
            self.dashboard_cmd_pub_.publish(dashboard_cmd[0])
            self.stop_ = False

    def cb_joint_state(self, msg):
        self._current_joint_values = np.array(msg.position)

    def cb_nearest_object(self, data):
        if float(data.distance) < self.cfg['nearest_object_threshold']:
            rospy.loginfo("Stopping robot because nearest object! Distance: "+str(data.distance))
            self.stop_ = True
            self.confirm = True
            self.dashboard_cmd_pub_.publish(dashboard_cmd[1])

    def in_position(self, p):
        joint_values = self._current_joint_values.copy()
        robot_position = self._robot_kin.fwd_kin(joint_values)[:3, 3]
        dist = np.linalg.norm(np.array(p) - np.array(robot_position))
        if dist < 0.01:
            return True
        return False

    def sendObjectPoseMsg(self, id):
        pose_msg = ManipulatedObject()
        pose_msg.id = id
        pose_msg.pose = geometry_msgs.msg.Pose()
        self.object_pose_pub_.publish(pose_msg)

    def run(self):
        p1 = [0.15965, -0.6346, 0.251]  # frame pick location
        p6 = [0.159, -0.634, 0.336]  # frame confirm location
        p2 = [0.551, -0.128, 0.372]  # frame drop location
        p3 = [0.1134, -0.38028, 0.25512]   # rocket shaft pick location
        p7 = [0.113, -0.38027, 0.4536]   # rocker shaft confirm location
        p4 = [0.59873, -0.13710, 0.480]  # force mode activation location
        # p5 = [0.63731, -0.12930, 0.41194] # final position for the rocker shaft
        p5 = [0.63731, -0.12930, 0.19] # final position for the rocker shaft
        rocket_shaft_picked = False
        motor_frame_picked = False
        motor_frame_checked = False
        rocker_shaft_checked = False

        tool_vel = rospy.wait_for_message("/tool_velocity", TwistStamped)
        while sum(np.array([tool_vel.twist.linear.x, tool_vel.twist.linear.y, tool_vel.twist.linear.z])) < 0.01 and \
                not rospy.is_shutdown():
            tool_vel = rospy.wait_for_message("/tool_velocity", TwistStamped)
            rospy.sleep(0.005)
        start_time = rospy.Time.now().to_sec()
        rospy.loginfo("Timer started")

        object_picked = 0
        while not rospy.is_shutdown():
            safe_system_mode = "normal"
            if self.in_position(p1):
                rospy.loginfo("In frame picking location")
                motor_frame_picked = True
                object_picked = 1

            if self.in_position(p6) and motor_frame_picked:
                if not motor_frame_checked:
                    self.dashboard_cmd_pub_.publish(dashboard_cmd[1])
                    rospy.loginfo("Frame placement intention confirm STOP")
                    motor_frame_checked = True
                    self.stop_ = True
                safe_system_mode = "confirm_motor_frame"

            if self.in_position(p7) and rocket_shaft_picked:
                rospy.loginfo("Rocker shaft placement intention confirm")
                if not rocker_shaft_checked:
                    self.dashboard_cmd_pub_.publish(dashboard_cmd[1])
                    rospy.loginfo("Rocker shaft checked placement intention confirm STOP")
                    rocker_shaft_checked = True
                    self.stop_ = True
                safe_system_mode = "confirm_rocker_shaft"

            if self.in_position(p2):
                rospy.loginfo("In dropping location")
                rospy.sleep(0.3)
                object_picked = 0

            if self.in_position(p3):
                rospy.loginfo("In rocket shaft picking location")
                rocket_shaft_picked = True
                object_picked = 1

            if self.in_position(p4) and rocket_shaft_picked:
                # deactivate safety area
                rospy.loginfo("Force mode location")
                # safe_system_mode = "force_mode"
                break

            self.set_safety_system_mode_pub_.publish(safe_system_mode)
            self.sendObjectPoseMsg(object_picked)
            rospy.sleep(0.05)
        end_time = rospy.Time.now().to_sec()
        print('Time: ', end_time - start_time)
        while not self.in_position(p5) and not rospy.is_shutdown():
            self.set_safety_system_mode_pub_.publish("force_mode")
            rospy.sleep(0.1)
        self.dashboard_cmd_pub_.publish(dashboard_cmd[1])  # stop
        rospy.sleep(0.5)

def star_robot():
    dashboard = DashboardClientROS()
    try:
        dashboard.run()
    except ValueError:
        rospy.loginfo("Closing connection")

if __name__ == "__main__":
    try:
        star_robot()
    except rospy.ROSInterruptException:
        pass