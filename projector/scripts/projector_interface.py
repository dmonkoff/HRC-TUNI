#!/usr/bin/env python
# utils
import cv2
import sys
import time

import numpy as np
import rospy
import yaml
import rospkg
from sensor_msgs.msg import JointState
from unity_msgs.msg import MarkerDataArray
from unity_msgs.msg import ManipulatedObject
from std_msgs.msg import String

sys.path.append("/home/antti/work/utility_functions/opencv/")
sys.path.append('/home/antti/work/utility_functions/ur5_kinematics/')
from ur5_kinematics import ur5



class Pattern():
    def __init__(self, id, texture, location):
        self._texture = texture
        self._location = location
        self._id = id
        self._H, self._W = self._texture.shape[:2]

    def draw_pattern(self, img, active=True):
        if active:
            temp = self._texture.copy()
        else:
            temp = self._texture.copy()
            temp[temp > 100] = 100
        img[self._location[0]:self._location[0] + self._H, self._location[1]:self._location[1] + self._W] = temp
        return img

    def draw_pattern_with_text(self, img, text='Robot action:'):
        img[self._location[0]:self._location[0] + self._H, self._location[1]:self._location[1] + self._W] = self._texture.copy()
        cv2.putText(img, text, (self._location[1] + 80, (self._location[0] - 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), lineType=cv2.LINE_AA, thickness=2)
        return img

class Interface():
    def __init__(self, h, w):
        self._components = {}
        # self._screen = np.zeros([h, w, 3], dtype=np.uint8)

    def read_component(self, id, texture_path, location):
        texture = cv2.imread(texture_path)
        self._components[id] = Pattern(id, texture, location)

    def read_config(self, path):
        with open(path + 'projector_buttons.yaml', 'r') as f:
            data = yaml.safe_load(f)
        keys = data.keys()
        for i in range(len(keys)):
            self.read_component(keys[i], path + data[keys[i]]['img_path'], data[keys[i]]['loc'])

    def draw(self, screen, robot_state, button_state, system_state):
        if robot_state == 'moving':
            screen = self._components['GO'].draw_pattern(screen, False)
            screen = self._components['STOP'].draw_pattern(screen, True)
            screen = self._components['CONFIRM'].draw_pattern(screen, False)
            screen = self._components['DEAD_MAN'].draw_pattern(screen, False)
            screen = self._components['ROBOT_ACTIVE'].draw_pattern(screen, True)
        elif robot_state == 'stationary':
            screen = self._components['GO'].draw_pattern(screen, True)
            screen = self._components['STOP'].draw_pattern(screen, False)
            screen = self._components['CONFIRM'].draw_pattern(screen, False)
            screen = self._components['DEAD_MAN'].draw_pattern(screen, True)
            if button_state == 'dead_man_not_pressed':
                screen = self._components['DM_NOT_PRESSED'].draw_pattern(screen, True)
            else:
                screen = self._components['START_ROBOT'].draw_pattern(screen, True)
        else:
            print("Unkown robot state")

        if system_state == 'confirm_motor_frame':
            screen = self._components['MOTOR_FRAME'].draw_pattern_with_text(screen)

        return screen

class Projector():
    def __init__(self, interface_configs_path, common_configs, homogprahy_path):
        # self._interaction_sub = rospy.Subscriber("/unity/interaction_markers", MarkerDataArray, cb_interaction_marker)
        self._joint_sub = rospy.Subscriber('joint_states', JointState, self.cb_joint_state)
        self._marker_sub = rospy.Subscriber("/unity/interaction_markers", MarkerDataArray, self.cb_markers_state)
        self._system_state_sub = rospy.Subscriber("/unity/system_mode", String, self.cb_system_state)
        self._object_manipulation_sub = rospy.Subscriber("/unity/manipulated_object", ManipulatedObject,
                                                   self.cb_object_manipulation)

        self._current_joint_values = None
        self._robot_kin = ur5()
        self._interface = Interface(1080, 1920)
        self._robot_state = 'stationary'
        self._cfg = None
        self._button_info = None
        self._system_state = None
        self._robot_carrying_object = False
        self._H = np.loadtxt(homogprahy_path)
        # nearest_object_coords_sub = rospy.Subscriber("/unity/nearest_object", NearestObject, cb_nearest_object_coords)
        # system_mode_sub_ = rospy.Subscriber("/unity/system_mode", String, cb_safety_system_mode)
        self.init(interface_configs_path, common_configs)
        rospy.loginfo("Projector interface initialized!")

    def init(self, interface_configs_path, common_configs):
        with open(common_configs, 'r') as ymlfile:
            self._cfg = yaml.load(ymlfile, Loader=yaml.SafeLoader)
        self._interface.read_config(interface_configs_path)

    def cb_object_manipulation(self, msg):
        if msg.id == 0:
            self._robot_carrying_object = False
        else:
            self._robot_carrying_object = True

    def cb_system_state(self, msg):
        self._system_state = msg.data

    def cb_markers_state(self, msg):

        go_active = False
        confirm_active = False
        dead_man_active = False
        for marker in msg.markers:
            if marker.id == 'go':
                if marker.data > self._cfg['interaction_button_thres']:
                    go_active = True

            if marker.id == 'confirm':
                if marker.data > self._cfg['interaction_button_thres']:
                    confirm_active = True

            if marker.id == 'dead_man':
                if marker.data > self._cfg['interaction_button_thres']:
                    dead_man_active = True

        if dead_man_active == False and go_active:
            self._button_info = 'dead_man_not_pressed'
        else:
            self._button_info = None

    def cb_joint_state(self, msg):
        self._current_joint_values = np.array(msg.position)
        joint_speeds = np.array(msg.velocity)

        if np.sum(abs(joint_speeds)) > 1e-1:
            self._robot_state = 'moving'
        else:
            self._robot_state = 'stationary'

    def points_in_circum(self, r, ox, oy, n=50):
        return [(np.cos(2 * np.pi / n * x) * r + ox, np.sin(2 * np.pi / n * x) * r + oy) for x in xrange(0, n + 1)]

    def generate_hull(self, cs, H, circle_radius, offset):
        mask = np.zeros((1080, 1920), np.uint8)
        color_mask = np.zeros((1080, 1920, 3), np.uint8)
        for c in cs:
            points = self.points_in_circum(circle_radius, c[0], c[1], 50)
            cartesian = np.ones((len(points), 3))
            cartesian[:, :2] = points
            transformed = np.dot(H, cartesian.T).T
            for t in transformed:
                t = (t[:2] // t[2]).astype(int)

                # better idea, crop the points to the edge of the screen
                if t[0] < offset or t[1] < offset or t[0] > mask.shape[1] - offset or t[1] > mask.shape[0] - offset:
                    continue
                mask[t[1], t[0]] = 255
        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # we have points so merge all the contours which are around the points
        merged_contour = []
        for c1 in contours:
            for c2 in c1:
                merged_contour.append(c2)
        hull = cv2.convexHull(np.array(merged_contour), False)
        cv2.drawContours(color_mask, [hull], 0, (0, 0, 255), -1, 8)
        return color_mask

    def work_object_cpoints(self, ee_frame):
        obj = np.eye(4)
        obj[1, 3] = self._cfg['work_object_width'] / 2  # 0.12
        obj1_base_frame = np.dot(ee_frame, obj)[0:2, 3]  # take only x and y components
        obj[1, 3] = (-1.0) * self._cfg['work_object_width'] / 2
        obj2_base_frame = np.dot(ee_frame, obj)[0:2, 3]
        return np.array([obj1_base_frame, obj2_base_frame])


    def run(self):
        last_check = 0.0
        robot_state = None
        while not rospy.is_shutdown():
            interface_img = np.zeros((1080, 1920, 3), np.uint8)

            ### Generate interface components
            if time.time() - last_check > 0.3:
                robot_state = self._robot_state
                last_check = time.time()
            interface_img = self._interface.draw(interface_img, robot_state, self._button_info, self._system_state)

            ### Generate hull
            joint_values = self._current_joint_values.copy()
            c_points = self._robot_kin.get_link_xyz(joint_values)
            temp1 = self.generate_hull(c_points, self._H, self._cfg['dynamic_workspace_size'], 2)
            temp2 = self.generate_hull(c_points, self._H, self._cfg['dynamic_workspace_size'] + self._cfg['safety_area_offset'], 1)
            safety_line = temp2-temp1
            interface_img += safety_line

            if self._robot_carrying_object:
                ee_frame = self._robot_kin.fwd_kin(joint_values, gripper="onrobot_rg2")
                c_points = self.work_object_cpoints(ee_frame)
                temp1 = self.generate_hull(c_points, self._H, self._cfg['dynamic_workspace_size'], 2)
                temp2 = self.generate_hull(c_points, self._H, self._cfg['dynamic_workspace_size'] + self._cfg['safety_area_offset'], 1)
                safety_line = temp2 - temp1
                interface_img += safety_line

            cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty("window", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.imshow("window", interface_img)
            cv2.waitKey(1)

def main():
    rospy.init_node('robot_safety_area_node')
    rospack = rospkg.RosPack()
    config_prefix = rospack.get_path('unity_msgs') + '/configs/'
    interface_config_path = config_prefix + 'mobile_demo/'
    common_config_path = config_prefix + 'config.yaml'
    homography_file = config_prefix + 'robot_projector_homography.txt'
    proj = Projector(interface_config_path, common_config_path, homography_file)
    rospy.sleep(1.0)
    proj.run()

if __name__ == "__main__":
    main()

