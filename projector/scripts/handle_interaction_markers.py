#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np
from unity_msgs.msg import MarkerDataArray
from unity_msgs.msg import MarkerData

def depth2pc_vectorized(z, camera_matrix, scale=1000.0):
    C, R = np.indices(z.shape)
    R = np.subtract(R, camera_matrix[0, 2])
    R = np.multiply(R, z)
    R = np.divide(R, camera_matrix[0, 0] * scale)
    C = np.subtract(C, camera_matrix[1, 2])
    C = np.multiply(C, z)
    C = np.divide(C, camera_matrix[1, 1] * scale)
    return np.column_stack((R.ravel(), C.ravel(), (z.ravel() / scale)))

def cameraInfo2intrinsic(msg):
    matrix = np.array(msg.K).reshape(3, 3)
    return matrix

class Pattern():
    def __init__(self, id, location, size):
        self._location = location
        self._id = id
        self._size = size
        # self._H, self._W = self._texture.shape[:2]

class MarkerHandler():
    def __init__(self):
        self._bridge = CvBridge()
        camera_msg = rospy.wait_for_message("/kinect2/sd/camera_info", CameraInfo)
        self._camera_matrix = cameraInfo2intrinsic(camera_msg)
        self._rgb_sub = message_filters.Subscriber("/kinect2/sd/image_color_rect", Image)
        self._depth_sub = message_filters.Subscriber("/kinect2/sd/image_depth_rect", Image)
        self._marker_publisher = rospy.Publisher("/unity/interaction_markers", MarkerDataArray, queue_size=1)
        self._ts = message_filters.ApproximateTimeSynchronizer([self._rgb_sub, self._depth_sub], 1, 2)
        self._ts.registerCallback(self.img_cb)
        self._patterns = []
        self._cut_off = 1.3
        self._initialized = False
        self._init_view = None
        self.init()

    def init(self):
        # The marker locations in the registered color_image (or depth image) should be defined
        self._patterns.append(Pattern('go', np.array([379, 108]), 10))
        self._patterns.append(Pattern('stop', np.array([411, 108]), 10))
        self._patterns.append(Pattern('confirm', np.array([344, 108]), 10))
        self._patterns.append(Pattern('dead_man', np.array([124, 115]), 10))

    def show_cropped_area(self, registered_rgb):
        for p in self._patterns:
            registered_rgb = cv2.circle(registered_rgb, (p._location[0], p._location[1]), p._size, 255, -1)
        cv2.imshow("Buttons", registered_rgb)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def check_button(self, img_shape, pattern_location, pattern_size, current_points, rgb_img, init_points):
        mask = np.zeros(img_shape, np.uint8)
        mask = cv2.circle(mask, (pattern_location[0], pattern_location[1]), pattern_size, 255, -1).reshape(-1, 1)
        idx = mask > 0
        current_target_points = current_points[idx.reshape(-1, )]
        init_target_points = init_points[idx.reshape(-1, )]

        diff = np.sqrt(np.sum((current_target_points - init_target_points)**2))
        if diff > self._cut_off:
            return 0.0
        else:
            return diff

    def img_cb(self, rgb_data, depth_data):
        rgb_img = None
        depth_raw = None
        try:
            rgb_img = self._bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            depth_raw = self._bridge.imgmsg_to_cv2(depth_data, "passthrough")
        except CvBridgeError as e:
            print(e)

        points = depth2pc_vectorized(depth_raw, self._camera_matrix)
        # TODO: alternative transform points using the kinect to robot transformation

        if not self._initialized:
            self._init_view = points
            self._initialized = True
            self.show_cropped_area(rgb_img)
            return

        marker_array = MarkerDataArray()
        for p in self._patterns:
            marker_data = MarkerData()
            marker_data.id = p._id
            marker_data.data = self.check_button(depth_raw.shape, p._location, p._size, points, rgb_img, self._init_view)
            marker_array.markers.append(marker_data)
        self._marker_publisher.publish(marker_array)
        rospy.sleep(0.01)

def main():
    rospy.init_node('handle_interaction_markers')
    marker = MarkerHandler()
    rospy.spin()

if __name__ == "__main__":
    main()