#!/usr/bin/env python
import SocketServer
import struct
import sys
import threading
import time
import yaml
from safety_zone import *
sys.path.insert(0, '/home/antti/work/utility_functions/ur5_kinematics/')
from ur5_kinematics import ur5

# ROS related
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from unity_msgs.msg import ManipulatedObject
from unity_msgs.msg import MarkerDataArray
import rospkg

msg_lock = threading.Lock()
hololens_world_coords = []
msg_write = "empty\n"

# https://stackoverflow.com/questions/16022556/has-python-3-to-bytes-been-back-ported-to-python-2-7/20793663
def to_bytes(n, length, endianess='big'):
    h = '%x' % n
    s = ('0'*(len(h) % 2) + h).zfill(length*2).decode('hex')
    return s if endianess == 'big' else s[::-1]

# https://stackoverflow.com/questions/17667903/python-socket-receive-large-amount-of-data
def recv_msg(sock):
    # Read message length and unpack it into an integer
    raw_msglen = recvall(sock, 4)
    if not raw_msglen:
        return None
    msglen = struct.unpack('>I', raw_msglen)[0]
    # print(msglen)
    # Read the message data
    return recvall(sock, msglen)


def recvall(sock, n):
    # Helper function to recv n bytes or return None if EOF is hit
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data

class ThreadedTCPRequestHandler(SocketServer.BaseRequestHandler):
    # https://stackoverflow.com/questions/5793252/tcp-server-is-closing-connections
    # the handle() method is called once a new TCP connection is set up, not once every time there is data available.
    def handle(self):
        close = False

        print("Connection request from: "+self.client_address[0])
        request = None

        prev_time = time.time()
        fps = 0
        while not close and request != "":
            start_time = time.time()

            # Receive request from client
            request = recv_msg(self.request)
            # print("Received: " + request)

            global msg_write
            msg_lock.acquire()
            msg_write_temp = msg_write
            # msg_write = "empty"  # to make sure we don't send the same message more than a once
            msg_lock.release()

            msg_prefix = to_bytes(len(msg_write_temp), 4)
            msg_write_temp = msg_prefix + msg_write_temp
            self.request.sendall(msg_write_temp)
            time.sleep(0.01) # Very important !

            if time.time() - prev_time > 1:
                prev_time = time.time()
                print("FPS %f" % fps)
                print(msg_write_temp[4:8])
            divider = (time.time() - start_time)
            if divider != 0:
                fps = 1 / divider
            else:
                fps = 0

    def finish(self):
        print 'Client ' + str(self.client_address[0]) + ' Disconnected'

class ThreadedTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    pass


class HoloServer:
    def __init__(self):
        self.ur5_kin = ur5()
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('unity_msgs')
        with open(package_path + "/configs/config.yaml", 'r') as ymlfile:
            self._cfg = yaml.load(ymlfile)

        self._sub_tf = rospy.Subscriber("/joint_states", JointState, self.cb_js, queue_size=1)
        self._sub_om = rospy.Subscriber("/unity/manipulated_object", ManipulatedObject,
                                        self.cb_object_manipulation, queue_size=1)
        self._marker_sub = rospy.Subscriber("/unity/interaction_markers", MarkerDataArray, self.cb_markers_state)
        self.system_mode_sub_ = rospy.Subscriber("/unity/system_mode", String, self.cb_safety_system_mode, queue_size=1)
        self._button_info = None
        self._robot_carrying_object = False
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        self._last_check = rospy.Time.now()
        self._prev_joint_position = None
        self._robot_status = "stat"  # stat move conf
        self._safety_system_mode = "normal"

    def cb_safety_system_mode(self, msg):
        self._safety_system_mode = msg.data

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

        if dead_man_active is False and go_active:
            self._button_info = 'dead_man_not_pressed'
        else:
            self._button_info = None


    def cb_object_manipulation(self, data):
        if data.id == 0:
            self._robot_carrying_object = False
        else:
            self._robot_carrying_object = True

    def current_state_of_robot(self, joint_values):
        if self._prev_joint_position is None:
            self._prev_joint_position = joint_values
            return "stat"

        joint_difference = np.abs([self._prev_joint_position - joint_values])
        self._prev_joint_position = joint_values

        sum_diff = np.sum(joint_difference)
        if sum_diff > 0.002:
            return "move"
        return "stat"

    def cb_js(self, data):
        j_values = np.array(data.position)
        if len(data.position) > 6:
            j_values = self.ur5_kin.sort_joints(j_values, data.name)
        coords = self.ur5_kin.get_link_xyz(j_values)

        if self._robot_carrying_object:
            ee_frame = self.ur5_kin.fwd_kin(j_values, gripper="onrobot_rg2")
            obj = np.eye(4)
            obj[1, 3] = self._cfg['work_object_width'] / 2  # 0.12
            obj1_base_frame = np.append(np.dot(ee_frame, obj)[0:2, 3], 0) # take only x and y components
            obj[1, 3] = (-1.0) * self._cfg['work_object_width'] / 2
            obj2_base_frame = np.append(np.dot(ee_frame, obj)[0:2, 3], 0)
            obj_coords = np.array([obj1_base_frame, obj2_base_frame])
            coords = np.append(coords, obj_coords, axis=0)
	
	# calculate safety hull as 2D points which are then sent to Hololens
        zone_boundary, zone_boundary_str = calculate_safety_zone(coords,
                                                                 self._cfg['dynamic_workspace_size'] +
                                                                 self._cfg['safety_area_offset'], False)

        current_time = rospy.Time.now()
        if current_time.to_sec() - self._last_check.to_sec() > 0.3:
            self._robot_status = self.current_state_of_robot(j_values)
            self._last_check = current_time
        if self._safety_system_mode == 'confirm_motor_frame':
            self._robot_status = 'con1'
        if self._safety_system_mode == 'confirm_rocker_shaft':
            self._robot_status = 'con2'
        if self._safety_system_mode == 'force_mode':
            self._robot_status = 'forc'
        if self._button_info == 'dead_man_not_pressed':
            self._robot_status = 'dmnp'

        msg = self._robot_status + zone_boundary_str
        global msg_write
        msg_lock.acquire()
        msg_write = msg
        msg_lock.release()


if __name__ == "__main__":

    rospy.init_node('TCP_server', anonymous=True)
    HOST, PORT = '', 9001
    server = ThreadedTCPServer((HOST, PORT), ThreadedTCPRequestHandler)
    ip, port = server.server_address
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    hs = HoloServer()
    server.middleware = hs
    server_thread.start()
    print "Server loop running in thread:", server_thread.name

    rospy.spin()
    server.shutdown()
    server.server_close()
