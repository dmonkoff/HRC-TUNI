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
import time
import cv2.aruco as aruco
import pickle
class TestAzure():
	def draw_circle(self, event,x,y,flags,param):
    		if event == cv2.EVENT_LBUTTONDBLCLK:
			cv2.circle(self.depth_raw,(x,y),100,255,-1)
			rospy.loginfo(str([x,y]))
			self.m_pos =[x,y]
	def __init__(self):

		#%%
		self.testim = np.zeros((1080, 1920,3), dtype="uint8")#+255
		self.imgpoints = []

		self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
		#pts1 = np.float32([[510.5, 292.0], [620.5, 314.0], [725.5, 386.0],[728.5, 277.0]])
		#pts2 = np.float32([[455.0, 455.0], [960.0, 540.0], [1455.0, 855.0],[1455.0, 355.0]])
		aruco_pixel_size=311
		#pts1 =np.float32([[436., 208.],[759., 219.],[754., 378.],[409., 370.]])
		#pts2 = np.float32([[100,10],[1920-aruco_pixel_size-10,10],[1920-aruco_pixel_size-10,1080-10-aruco_pixel_size-10],[10,1080-aruco_pixel_size-10]])

		pts2 = np.float32([[ 332.,   10.], [1799.,   10.],[1799.,  949.],[ 232.,  959.]])
		pts1 = np.float32([[300.,529. ],[ 319.,236.],[ 504.,237.],[488.,572. ]])

		self.M = cv2.getPerspectiveTransform(pts1,pts2)
		self.H = np.eye(3,dtype=np.float32)
		print(self.H)
		self.flagChess = True
		block_width = 192
		block_height = 108
		self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
		self.parameters_aruco = aruco.DetectorParameters_create()
		#for i in range(0,10,2):
		#    for j in range(2,10,2):
		#	#print((i,j))
		#	self.testim[i*block_height:(i+1)*(block_height),j*block_width:(j+1)*(block_width)] = 255
		#for i in range(1,10,2):
		#    for j in range(3,10,2):
		#	#print((i,j))
		#	self.testim[i*block_height:(i+1)*(block_height),j*block_width:(j+1)*(block_width)] = 255
		#self.testim = 255-self.testim
		#ret, corners = cv2.findChessboardCorners(self.testim, (7,9), None)
		
		#if ret == True:
		#	corners2 = cv2.cornerSubPix(self.testim,corners, (11,11), (-1,-1), self.criteria)
		#	self.pointstest = np.copy(self.testim)
		#	cv2.drawChessboardCorners(self.pointstest, (7,9), corners2, ret)
		#	corners2 = np.squeeze(corners2)
		#	print(corners2.shape)
		#	self.imgpoints =np.float32(corners2)
		#else:
		#	rospy.loginfo('DETECT IMAGE CORNERS ERROR')
		
		#with open('calib.pickle', 'rb') as handle:
			#(ret, mtx, dist, rvecs, tvecs) =pickle.load(handle)
		(w,h) = (1920,1080)
		#newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
		#self.pointstest = cv2.undistort(self.pointstest, mtx, dist, None, newcameramtx)
		## crop the image
		#x, y, w, h = roi
		#self.pointstest = self.pointstest[y:y+h, x:x+w]
		#cv2.imwrite('1.png',self.pointstest)


		#cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
		#cv2.moveWindow("window", 1920, 0)
		#cv2.setWindowProperty("window", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
		#cv2.imshow("window", self.pointstest)
		#cv2.waitKey(0)
		self.depth_raw = None
		rospy.loginfo('azaza1')
		self.bridge = CvBridge()
		rospy.loginfo('azaza2')
		self.rgb_sub = message_filters.Subscriber("/k4a/rgb/image_rect_color", Image)
		self.depth_sub = message_filters.Subscriber("/k4a/depth/image_rect", Image)
		self.depth2RGB_sub = message_filters.Subscriber("/k4a/depth_to_rgb/image_raw", Image)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.depth2RGB_sub], 1, 2)
		self.ts.registerCallback(self.img_cb)
		rospy.loginfo('azaza3')
	def find_outer_corners(self,corners):
		distances = []
		for c1 in corners:
			tmp = 0
			for c2 in corners:
				tmp += np.linalg.norm(c1-c2)
			distances.append(tmp)
		tmp = np.argsort(-np.array(distances))[:4]
		outer_corners = np.array([corners[i] for i in tmp] )
		return outer_corners
		print(outer_corners)		
	def img_cb(self, rgb_data, depth_data,depth2RGB_data):
		self.rgb_img = None
		self.depth_raw = None
		self.depth2RGB_raw = None
		try:
		    self.rgb_img = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
		except CvBridgeError as e:
		    print(e)

		try:
		    self.depth2RGB_raw = self.bridge.imgmsg_to_cv2(depth2RGB_data, "passthrough")
		except CvBridgeError as e:
		    print(e)

		try:
		    self.depth_raw = self.bridge.imgmsg_to_cv2(depth_data, "passthrough")
		except CvBridgeError as e:
		    print(e)


		self.wait = True
		tags = []
		size = 80
		for i in range(16):
		    tag = np.zeros((size, size, 1), dtype="uint8")
		    cv2.aruco.drawMarker(self.aruco_dict, i, size, tag, 1)
		    tags.append(tag)
		
		cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
		cv2.moveWindow("window", 1920, 0)
		cv2.setWindowProperty("window", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
		cv2.imshow("window", self.testim)
		cv2.waitKey(1)
		imagefortest=cv2.warpPerspective(self.rgb_img,self.M,(1920,1080))
		im = cv2.cvtColor(imagefortest, cv2.COLOR_BGR2GRAY)	
		#cv2.imshow("chessboard image plane",im)
		#cv2.waitKey(0) 
		#cv2.destroyWindow("chessboard image plane") 
		if self.flagChess:
			ret, corners = cv2.findChessboardCorners(imagefortest, (4,4),cv2.CALIB_CB_ADAPTIVE_THRESH)
			print('aaaaa')
			if ret == True:
				corners2 = cv2.cornerSubPix(im,corners, (11,11), (-1,-1), self.criteria)
				corners2 = np.squeeze(corners2)
				self.corners_chessboard  = corners2
				print(corners2.shape)
				tmp = np.copy(self.testim)
				tmp = 255-tmp
				for i,corner in enumerate(np.int32(corners2)):
					print(self.testim.shape)
					self.testim = cv2.circle(self.testim, (corner[0],corner[1]), radius=1, color=(255, 255, 255), thickness=2)
					print(self.testim[corner[1]:corner[1]+size,corner[0]:corner[0]+size].shape)
		    			#self.testim[corner[1]:corner[1]+size,corner[0]:corner[0]+size] = tags[i]
					


					#print(self.H)
					#self.testim = cv2.warpPerspective(self.testim[...,np.newaxis],self.H,(1920,1080))
					#self.testim = self.testim[...,np.newaxis]
					#print(self.testim.shape)
					#tmp[corner[1]:corner[1]+size,corner[0]:corner[0]+size] = tags[i]
					#tmp = cv2.circle(tmp, (corner[0],corner[1]), radius=10, color=(255, 255, 255), thickness=2)
				#cv2.imshow("chessboard image plane",self.testim)
				#cv2.waitKey(0) 
				#cv2.destroyWindow("chessboard image plane") 
				tmp = np.copy(self.testim)

				#self.testim = 255 - self.testim
				cv2.drawChessboardCorners(im, (4,4), corners2, ret)
				#print(self.testim.shape)
				#test = np.zeros((1080,1920,3),dtype=np.uint8)
				#test[...,0] = im
				#test[...,1] = im
				#test[...,2] = np.uint8(0.93*im+0.07*self.pointstest)
				#corners2 = np.squeeze(corners2)
				#print(corners2.shape)
				#tmp = np.zeros((corners2.shape[0],3))
				#print(tmp.shape)
				#print(corners2.shape)
				#tmp[...,:-1] = corners2
				#corners2 = tmp
				#print(corners2.shape)
				#ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(np.array([[corners2]],dtype=np.float32), np.array([self.imgpoints],dtype=np.float32), im.shape, None, None)
				#with open('calib.pickle', 'wb') as handle:
	    			#	pickle.dump((ret, mtx, dist, rvecs, tvecs), handle)
				#cv2.imshow("chessboard image plane",cv2.cvtColor(imagefortest, cv2.COLOR_BGR2GRAY))
				#cv2.imshow("chessboard image plane",tmp)
				#cv2.waitKey(0) 
				#cv2.destroyWindow("chessboard image plane") 
			
				#cv2.imwrite('2.png',im)
				self.flagChess = False
		else:
			#cv2.imwrite('circles.png',imagefortest)
			gray = cv2.cvtColor(imagefortest, cv2.COLOR_BGR2GRAY)
			ttt = np.copy(imagefortest)
			#circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 100,maxRadius=20)
			#cv2.imshow("chessboard image plane",imagefortest)
			(corners, ids, rejected) = cv2.aruco.detectMarkers(gray, self.aruco_dict,parameters=self.parameters_aruco )
			if len(corners) == 16:
				ids = ids.flatten()
				cv2.aruco.drawDetectedMarkers(ttt, corners)
				cv2.imshow("chessboard image plane",ttt)
				cv2.waitKey(0) 
				cv2.destroyWindow("chessboard image plane")
				corners_aruco = np.array([corner[0][0] for corner in corners])
				outer_corners_aruco = self.find_outer_corners(corners_aruco)
				outer_corners_chessboard = self.find_outer_corners(self.corners_chessboard)
				closest_idxs = []
				for outer_aruco in outer_corners_aruco:
					distances = []
					for outer_chess in outer_corners_chessboard:
						distances.append(np.linalg.norm(outer_aruco-outer_chess))

					closest_idxs.append(np.argmin(distances))
				chessboard_corners_ordered = np.array([outer_corners_chessboard[i] for i in closest_idxs])
				self.H_tmp, mask = cv2.findHomography(outer_corners_aruco, chessboard_corners_ordered)
				self.H = np.matmul(self.H_tmp,self.H)
				print(self.H)
				#self.flagChess = True
				self.testim[...] = 255
			#cv2.waitKey(0) 
			#cv2.destroyWindow("chessboard image plane")

			#if circles != None:
				#for (x, y, r) in circles:
					# draw the circle in the output image, then draw a rectangle
					# corresponding to the center of the circle
					#cv2.circle(gray, (x, y), r, (0, 255, 0), 4)
				#cv2.imshow("chessboard image plane",imagefortest)

				#cv2.waitKey(0) 
				#cv2.destroyWindow("chessboard image plane")

def main():

	rospy.init_node('azureTest')
	rospy.loginfo("Trying to start . . ")
	rospy.loginfo("test 0 ")
	rospy.loginfo("test 1 ")
	nodeMain = TestAzure()
	rospy.spin()
	rospy.loginfo("test 2 ")
	rospy.loginfo("End of main()")

if __name__ == "__main__":
    main()
