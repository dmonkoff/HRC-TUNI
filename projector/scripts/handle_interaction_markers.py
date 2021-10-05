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
from unity_msgs.msg import HomographyMtx
import time

def depth2pc_vectorized(z, camera_matrix, scale=1000.0):
    #rospy.loginfo(camera_matrix)
    C, R = np.indices(z.shape)
    R = np.subtract(R, camera_matrix[0, 2])
    R = np.multiply(R, z)
    R = np.divide(R, camera_matrix[0, 0] * scale)
    C = np.subtract(C, camera_matrix[1, 2])
    C = np.multiply(C, z)
    C = np.divide(C, camera_matrix[1, 1] * scale)
    return np.column_stack((R.ravel(), C.ravel(), (z.ravel() / scale)))


#def projective_depth_rgb_transform(im,points, size=(320,288)):
#    points = np.array(points,dtype=np.float32)
#    outputPoints = np.array([[0,0],[size[0],0],[size[0],size[1]],[0,size[1]]],dtype=np.float32)
#    M = cv2.getPerspectiveTransform(points,outputPoints)
#    im = np.float32(im)
#    res = cv2.warpPerspective(im,M,size)
#    res =res.flatten()
#    return res 





class Pattern():
    def __init__(self, id, location, size):
        self._location = location
        self._id = id
        self._size = size


class MarkerHandler():
    def __init__(self):
        self._bridge = CvBridge()
	self.mapTS = time.time()
	self.curTS = time.time()
	self.prevTS = time.time()
	self.enumMaps = 0
	self.saveRGBD_flag = True
	#self.table_homography = rospy.wait_for_message("/unity/table_homography", HomographyMtx)
	#self.table_homography = getHomographyMatrix(self.table_homography)
	self.table_homography_sub = rospy.Subscriber("/unity/table_homography", HomographyMtx,self.getHomographyMatrix)
	self.table_homography = None
        self._rgb_sub = message_filters.Subscriber("/k4a/rgb/image_rect_color", Image)
        self._depth_sub = message_filters.Subscriber("/k4a/depth/image_rect", Image)
	self.depth2RGB_sub = message_filters.Subscriber("/k4a/depth_to_rgb/image_raw", Image)
        self._marker_publisher = rospy.Publisher("/unity/interaction_markers", MarkerDataArray, queue_size=1)
        self._ts = message_filters.ApproximateTimeSynchronizer([self._rgb_sub, self._depth_sub,self.depth2RGB_sub], 1, 2)
        self._ts.registerCallback(self.img_cb)
        self._patterns = []
        self._cut_off = 1.3
        self._initialized = False
        self._init_view = None
        self.init()

    def init(self):
        # The marker locations in the registered color_image (or depth image) should be defined
        self._patterns.append(Pattern('stop', np.array([31, 261]), 10))
        self._patterns.append(Pattern('go', np.array([63, 257]), 10))
        self._patterns.append(Pattern('confirm', np.array([96, 261]), 10))
        self._patterns.append(Pattern('dead_man', np.array([278, 251]), 10))

    def getHomographyMatrix(self,msg):
        matrix = np.array(msg.matrix).reshape(3, 3)
        #print(matrix)
        self.table_homography = matrix
        return matrix
	
    def show_cropped_area(self, registered_rgb):
        for p in self._patterns:
            registered_rgb = cv2.circle(registered_rgb, (p._location[0], p._location[1]), p._size, 255, -1)
        cv2.imshow("Buttons", registered_rgb)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def check_button(self, img_shape, pattern_location, pattern_size, current_points, rgb_img, init_points, btn_id='untitled'):
        mask = np.zeros(img_shape[::-1], np.uint8)
        mask = cv2.circle(mask, (pattern_location[0], pattern_location[1]), pattern_size, 255, -1).flatten()
        idx = mask > 0
        current_target_points = current_points[idx]
        init_target_points = init_points[idx]

        #diff = np.sqrt(np.sum((current_target_points - init_target_points)**2)) # I don't like using equcledian distance here, it is dependent on area sizes, mean is better?
	diff = np.mean(np.abs(current_target_points - init_target_points))
	#if btn_id == 'go':
	print(btn_id+' diff: '+str(diff))
	#if np.all(pattern_location == [367, 108]):
		#rospy.loginfo("orig av: " + str(np.mean(init_target_points))+" cur av: "+str(np.mean(current_target_points)))
		#rospy.loginfo(str(np.mean(idx)))
	

	return diff


       # if diff > self._cut_off:
            #rospy.loginfo("Diff if: " + str(diff))

            #return 0.0
        #else:
            #rospy.loginfo("Diff else: " + str(diff))
            #return diff

    def img_cb(self, rgb_data, depth_data,depth2rgb_data):
        
        rgb_img = None
        depth_raw = None
	if self.table_homography is None:
		return
        try:
            rgb_img = self._bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #try:
        #    depth_raw = self._bridge.imgmsg_to_cv2(depth_data, "passthrough")
        #except CvBridgeError as e:
        #    print(e)
        try:
            depth_raw = self._bridge.imgmsg_to_cv2(depth2rgb_data, "passthrough")
        except CvBridgeError as e:
            print(e)
	#rospy.loginfo(str(np.mean(depth_raw)))
	#if self.saveRGBD_flag:
            #cv2.imwrite('depthIm.png',depth_raw)
            #cv2.imwrite('rgbIm.png',rgb_img)
	#rospy.loginfo(str(self._camera_matrix))
	self.curTS = time.time()
	#checkPoints = [[212,256],[112,256],[382,256],[212,50],[212,400]]
	#if self.curTS - self.prevTS > 10:
		#output_str = ''
		#for point in checkPoints:
		#	output_str += str(depth_raw[point[0],point[1]])+' '
		#output_str = output_str[:-1]
		#output_str += '\n'
		#with open('depthDrift.txt','a') as df:
		#	df.write(output_str)
		#self.prevTS = self.curTS
	#if self.curTS - self.mapTS > 60*30:
		#np.savetxt('map'+str(self.enumMaps),depth_raw)
		#self.enumMaps += 1
		#self.mapTS = self.curTS
	#depth_raw = cv2.rotate(depth_raw,cv2.ROTATE_180)
	#points = projective_depth_rgb_transform(depth_raw,[[164, 215],[320, 217],[316, 328],[158, 322]])
	projected_table_shape = (320,288)
	points = cv2.warpPerspective(depth_raw,self.table_homography,projected_table_shape).flatten()
        # TODO: alternative transform points using the kinect to robot transformation
	
        if not self._initialized:
	    #np.savetxt('map'+str(self.enumMaps),depth_raw)
	    #self.enumMaps += 1
	    #self.mapTS = self.curTS
            self._init_view = points
            self._initialized = True
            #self.show_cropped_area(rgb_img)
            return
        points2 = np.reshape(points,projected_table_shape[::-1])
        points2 = np.uint8(points2/np.max(points2)*255)
        cv2.circle(points2, (self._patterns[0]._location[0], self._patterns[0]._location[1]), self._patterns[0]._size, 255, -1)
	cv2.circle(points2, (self._patterns[1]._location[0], self._patterns[1]._location[1]), self._patterns[1]._size, 255, -1)
        cv2.imshow("Buttons", points2)
        cv2.waitKey(1)
        marker_array = MarkerDataArray()
        for p in self._patterns:
            marker_data = MarkerData()
            marker_data.id = p._id
            marker_data.data = self.check_button(projected_table_shape, p._location, p._size, points, rgb_img, self._init_view,p._id)
            marker_array.markers.append(marker_data)
        self._marker_publisher.publish(marker_array)
        rospy.sleep(0.01)

def main():
    
    rospy.init_node('handle_interaction_markers')
    rospy.loginfo("Trying to start . . ")
    rospy.loginfo("test 0 ")
    marker = MarkerHandler()
    rospy.loginfo("test 1 ")
    rospy.spin()
    rospy.loginfo("test 2 ")
    rospy.loginfo("End of main()")

if __name__ == "__main__":
    main()
