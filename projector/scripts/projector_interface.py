#!/usr/bin/env python
import cv2
import sys
import time
import os
import numpy as np
import rospy
import yaml
import json
import rospkg
import pprofile
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../../robot/scripts/')
from ur5_kinematics import ur5

from sensor_msgs.msg import JointState
from unity_msgs.msg import MarkerDataArray
from unity_msgs.msg import ManipulatedObject
from unity_msgs.msg import HomographyMtx
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

# class for drawing simple images with text at given location 
class Pattern():
    def __init__(self, id, texture, location):
        self._texture = texture
        self._location = location
        self._id = id
        self._H, self._W = self._texture.shape[:2]
        self._count_time = None

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

    def draw_pattern_with_counter(self, img):
        if self._count_time is None:
            self._count_time = time.time() + 8.0
        time_left = self._count_time - time.time()
        if time_left < 0.0:
            time_left = 0.00
        img[self._location[0]:self._location[0] + self._H, self._location[1]:self._location[1] + self._W] = self._texture.copy()
        cv2.putText(img, "Grab the rocker shaft, going", (self._location[1] - 40, (self._location[0] - 55)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), lineType=cv2.LINE_AA, thickness=2)
        cv2.putText(img, "into force mode in " + "{:0.2f}".format(time_left) +
                    " sec.",  (self._location[1] - 40, (self._location[0] - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), lineType=cv2.LINE_AA, thickness=2)
        return img

#class for interface image
class Interface():
    def __init__(self):
        self._components = {}

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
        if system_state == 'force_mode':
            screen = self._components['GO'].draw_pattern(screen, True)
            screen = self._components['STOP'].draw_pattern(screen, False)
            screen = self._components['CONFIRM'].draw_pattern(screen, False)
            screen = self._components['DEAD_MAN'].draw_pattern(screen, True)
            screen = self._components['START_ROBOT'].draw_pattern(screen, False)
            screen = self._components['GRASP_ARM'].draw_pattern_with_counter(screen)
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

        if system_state == 'confirm_rocker_shaft':
            screen = self._components['ROCKER_SHAFT'].draw_pattern_with_text(screen)

        return screen

#class for visualizing with projector
class Projector():
    def __init__(self, interface_configs_path, common_configs, homogprahy_path):
        self.links = ['base_link','link_4','link_5','flange','tool0']
        linksubs = []
        for link in self.links:
            linksubs.append(message_filters.Subscriber('coords/'+link, TransformStamped))#topics for coords of different joints
        self.joint_ts = message_filters.ApproximateTimeSynchronizer(linksubs, 1, 2)
        self.joint_ts.registerCallback(self.cb_joints)
        self.projectors = []
        self.mainCamera = {}
        with open('/home/robolab/catkin_ws/projector_devices.json') as f: #data for projectors and the kinect
            prj_json = json.load(f)
            for prj_data in prj_json['projectors']:
                tmp = {}
                tmp['id'] = prj_data['id']
                tmp['directHom'] = np.load(prj_data['homDirect']) 
                tmp['vertHom'] = np.load(prj_data['homFromVert'])#homography for vertical view of RGB image on ground level
                tmp['vertHomTable'] = np.load(prj_data['homTableVert'])#homography for vertical view of RGB image on table level
                tmp['shiftX'] = prj_data['shiftX']#shifting opencv window to the projector
                self.projectors.append(tmp)
            for cam_data in prj_json['cameras']:
                if cam_data['mainCam'] == 1:
                    self.mainCamera['vertHomGround'] = np.load(cam_data['vertHomGround'])#homography from normal image to vertical view on ground level
                    self.mainCamera['camRobHom'] = np.load(cam_data['camRobHom'])#homography from robot coords to image
                    self.mainCamera['vertHomTable'] = np.load(cam_data['vertTable'])#homography from normal image to vertical view on table level
        self.is_init = False
        self.depthIm = None
        self.rgb_img = None
        self.depth2rgb = None
        self.aruco_prev_center_point = np.array([-1000.,-1000.])
        self.table_space_transform = None
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters_aruco = aruco.DetectorParameters_create()
        self.rgb_sub = message_filters.Subscriber("/k4a/rgb/image_rect_color", Image)
        self.depth_sub = message_filters.Subscriber("/k4a/depth/image_rect", Image)
        self.depth2RGB_sub = message_filters.Subscriber("/k4a/depth_to_rgb/image_raw", Image)
        self.UI_transform = None
        im_subs = [self.rgb_sub,self.depth_sub,self.depth2RGB_sub]
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.depth2RGB_sub], 1, 2)
        self.ts.registerCallback(self.img_cb_once,im_subs)
        self._table_homography_pub = rospy.Publisher("/unity/table_homography", HomographyMtx, queue_size=1)#homography for the table to do visualization
        #self._joint_sub = rospy.Subscriber('joint_states', JointState, self.cb_joint_state)
        self._marker_sub = rospy.Subscriber("/unity/interaction_markers", MarkerDataArray, self.cb_markers_state)
        self._system_state_sub = rospy.Subscriber("/unity/system_mode", String, self.cb_system_state)#getting state of the robot program
        self._object_manipulation_sub = rospy.Subscriber("/unity/manipulated_object", ManipulatedObject,
                                                   self.cb_object_manipulation)#not used at the moment
        self._screen_size = (1080, 1920)
        self._vert_view_size = (1280,720)
        self._current_joint_values = None
        #self._robot_kin = ur5()
        self._interface = Interface()
        self._robot_state = 'stationary'
        self.c_points = None
        self._cfg = None
        self._button_info = None
        self._system_state = None
        self._robot_carrying_object = False
        self._H = np.loadtxt(homogprahy_path)

        self.init(interface_configs_path, common_configs)
        rospy.loginfo("Projector interface initialized!")

    def cb_joints(self,*data):#get coordinates of the joints
	#print(len(data))
        num_of_joints = len(data)
        self.c_points = np.ones((num_of_joints,4))
        for i in range(num_of_joints):
            xyz_data = data[i]
            self.c_points[i] =[xyz_data.transform.translation.x,xyz_data.transform.translation.y,xyz_data.transform.translation.z,1]
        self.c_points = self.c_points.T
        #print(self.c_points)

    def img_cb_once(self, rgb_data, depth_data,depth2RGB_data,subscribers):
        self.depthIm = CvBridge().imgmsg_to_cv2(depth2RGB_data, "passthrough")
        self.rgb_img = CvBridge().imgmsg_to_cv2(rgb_data, "bgr8")
        self.depth2rgb = CvBridge().imgmsg_to_cv2(depth2RGB_data, "passthrough")
	#[sub.sub.unregister() for sub in subscribers]


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

    def cb_markers_state(self, msg):#data for interaction elements
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

        if dead_man_active == False and go_active or confirm_active:
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
        return [(np.cos(2 * np.pi / n * x) * r + ox, np.sin(2 * np.pi / n * x) * r + oy) for x in range(0, n + 1)]

    def generate_hull(self, circle_radius, offset,contour_thickness):
        if offset == 0:
            offset = [0,0]
        cs = self.c_points
        H = self.mainCamera['camRobHom']
        color_mask = np.zeros((self._vert_view_size[1],self._vert_view_size[0], 3), np.uint8)
        mask = np.zeros((self._vert_view_size[1],self._vert_view_size[0]), np.uint8) #kinect camera resolution, TODO read from parameters
        point_joints = np.dot(H,cs)
        point_joints = point_joints / point_joints[2, :]
        point_joints = np.dot(self.mainCamera['vertHomGround'],point_joints)
        point_joints = point_joints / point_joints[2, :]
        point_joints = point_joints[:2,:].astype(int)
        point_joints = point_joints.T
        for c in point_joints:#make circular areas around joints
            cv2.circle(mask, (c[0]+offset[0],c[1]+offset[1]), circle_radius, (255,), -1)
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)#generate contours
        merged_contour = []
        if len(contours) != 0:
            for c1 in contours:
                for c2 in c1:
                    merged_contour.append(c2)
        hull = cv2.convexHull(np.array(merged_contour), False)
        cv2.drawContours(color_mask, [hull], 0, (0, 0, 255), contour_thickness)
        return color_mask

    def work_object_cpoints(self, ee_frame):#not used
        obj = np.eye(4)
        obj[1, 3] = self._cfg['work_object_width'] / 2  # 0.12
        obj1_base_frame = np.dot(ee_frame, obj)[0:2, 3]  # take only x and y components
        obj[1, 3] = (-1.0) * self._cfg['work_object_width'] / 2
        obj2_base_frame = np.dot(ee_frame, obj)[0:2, 3]
        return np.array([obj1_base_frame, obj2_base_frame])
    def scaleRectangle(self, points,scaler, center = None):#not used
        points = np.array(points)
        if center is None:
            center = points[0]+(points[2]-points[0])/2
        tmp = points-center
        points_scaled = tmp*scaler+center
        return points_scaled.astype(np.int32)

    def find_ui_transform(self):#find homography for the table
        gray = cv2.cvtColor(self.rgb_img,cv2.COLOR_BGR2GRAY )
        M = self.mainCamera['vertHomTable']#transform to vertical view
        proj_space=cv2.warpPerspective(self.depthIm,M,(self._vert_view_size[0],self._vert_view_size[1]))
        imagefortest=cv2.warpPerspective(self.rgb_img,M,(self._vert_view_size[0],self._vert_view_size[1]))
        corners, ids, rejected_img_points = aruco.detectMarkers(cv2.cvtColor(imagefortest,cv2.COLOR_BGR2GRAY ), self.aruco_dict, parameters=self.parameters_aruco)
        if np.all(ids is not None):#check if marker was found TODO add check for a specific marker
            print('OK')
            aruco_middle_point = np.array([(corners[0][0][0][0]+corners[0][0][2][0])/2,(corners[0][0][0][1]+corners[0][0][2][1])/2])
            if np.linalg.norm(self.aruco_prev_center_point-aruco_middle_point)< 1:#if center point did not move much, do not recalculate the location
                return
            else:
                self.aruco_prev_center_point=aruco_middle_point#track center point

        else:
            rospy.loginfo('ARUCO NOT FOUND')
            return
        tl_point = np.array([corners[0][0][0][0],corners[0][0][0][1]])
        aruco_long_vec = np.array([corners[0][0][1][0]-tl_point[0],corners[0][0][1][1]-tl_point[1]])
        ##big table
        scale_long = 6.4#compute size of the marker relative to the size of the table to find table corners
        scale_short = 4.2
        ##small table
        #scale_long = 3.4
        #scale_short = 2.3
        tr_point = tl_point+aruco_long_vec*scale_long
        aruco_short_vec = np.array([corners[0][0][3][0]-tl_point[0],corners[0][0][3][1]-tl_point[1]])#marker is installed along the sides of table
        bl_point = tl_point+aruco_short_vec*scale_short
        img_contours = imagefortest #this used for tests, can be deleted later
        pts1 = np.float32([[0,0],[0,self._screen_size[0]],[self._screen_size[1],0]])#corners of the screen
        pts2 = np.float32([tl_point,bl_point,tr_point])#corners on the image in Kinect
        self.UI_transform= cv2.getAffineTransform(pts1,pts2)
        print(self.UI_transform)
        tmp_transform = np.zeros((3,3))
        tmp_transform[:2,...] = self.UI_transform
        tmp_transform[2,2] = 1
        self.UI_transform = tmp_transform
        
        # img_contours = cv2.circle(img_contours, (tl_point[0],tl_point[1]), radius=5, color=(0, 0, 255), thickness=-1)
        # img_contours = cv2.circle(img_contours, (bl_point[0],bl_point[1]), radius=5, color=(0, 255, 0), thickness=-1)
        # img_contours = cv2.circle(img_contours, (tr_point[0],tr_point[1]), radius=5, color=(255, 0, 0), thickness=-1)
        # img_contours = cv2.circle(img_contours, (int(aruco_middle_point[0]),int(aruco_middle_point[1])), radius=5, color=(255, 255, 255), thickness=-1)
        # cv2.imshow('image', img_contours)
        # cv2.waitKey(0)
        # cv2.destroyWindow('image')


        projected_table_shape = (288,320)
        pts1 = np.float32([tl_point,bl_point,tr_point])
        pts2 = np.float32([[0,0],[0,projected_table_shape[0]],[projected_table_shape[1],0]])
        tmp_transform = cv2.getAffineTransform(pts1,pts2)

        tmp_transform2 = np.zeros((3,3))
        tmp_transform2[:2,...] = tmp_transform
        tmp_transform2[2,2] = 1
        self.table_space_transform = np.matmul(tmp_transform2,M)#inverse transform to for detecting interactions with buttons
        # print(self.table_space_transform.shape)
        # print(M.shape)
        self._table_homography_pub.publish(self.table_space_transform.flatten())
        #ttt = cv2.warpPerspective(self.rgb_img,self.table_space_transform,(projected_table_shape[1],projected_table_shape[0]))
        #cv2.imshow('image', ttt)
        #cv2.waitKey(0)
        #cv2.destroyWindow('image')
        return

    def run(self):
        last_check = 0.0
        robot_state = None
        while not rospy.is_shutdown():
            interface_img = np.zeros((self._screen_size[0], self._screen_size[1], 3), np.uint8)
            tmp = np.zeros((self._screen_size[0], self._screen_size[1], 3), np.uint8)
            # Generate interface components
            if time.time() - last_check > 0.3:
                robot_state = self._robot_state #robot state does not change now
                last_check = time.time()
            interface_tmp = self._interface.draw(tmp, robot_state, self._button_info, self._system_state)#generate interface
            self.find_ui_transform()#find transformation to the table
            self._table_homography_pub.publish(self.table_space_transform.flatten())#pubshish inverse transform for detecting button interactions
            # Generate hull
            #joint_values = self._current_joint_values.copy()
            if self.c_points is not None:
                safety_line = self.generate_hull(90, [-15,-15],5)#generate safety line if topics exist
            else:
                safety_line = np.zeros((720,1280, 3), np.uint8)
            interface_img += safety_line #sum them up


            for proj in self.projectors:#iterate over projectors and display content for each
                windowname = "window"+str(proj['id'])
                cv2.namedWindow(windowname, cv2.WND_PROP_FULLSCREEN)
                cv2.moveWindow(windowname, proj['shiftX'], 0)#shift opencv window to projector
                cv2.setWindowProperty(windowname, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                im_tmp = cv2.warpPerspective(interface_tmp, np.matmul(proj['vertHomTable'], self.UI_transform),(1920,1080))#UI on table level
                safety_tmp = cv2.warpPerspective(safety_line,proj['vertHom'],(1920,1080))#border on ground level
                im_tmp = cv2.add(im_tmp,safety_tmp)
                cv2.imshow(windowname, im_tmp)
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
	#print(os.environ)
	#prof = pprofile.StatisticalProfile()
	#with prof():
	#	main()
	#with open('/home/robolab/catkin_ws/profiling_results.bin', 'w') as f:
	#	prof.callgrind(f)

