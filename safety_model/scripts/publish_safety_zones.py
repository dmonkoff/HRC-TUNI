#!/usr/bin/env python
# utils
import cv2
import sys

import numpy as np
import rospy
import cv2
import struct
from sensor_msgs import point_cloud2

from std_msgs.msg import Header
sys.path.append('/home/antti/work/utility_functions/ur5_kinematics/')
sys.path.append('/home/antti/work/ur_interface/realtime_client/')
from ur5_kinematics import ur5
import realtime_client as rc
from sensor_msgs.msg import PointCloud2, PointField

fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          # PointField('rgb', 12, PointField.UINT32, 1),
          PointField('rgba', 12, PointField.UINT32, 1),
          ]

class ZonePublisher():
    def __init__(self, host):
        self._rviz_pub = rospy.Publisher('/safety_zones', PointCloud2, queue_size=2)
        self._publish_state = 3
        self._mask_shape = (1680, 1920)
        self._M = np.eye(3)
        self._M[:2, :2] *= 1000 # mm -> m
        self._M[0, 2] = self._mask_shape[0] // 2
        self._M[1, 2] = self._mask_shape[1] // 2
        self._robot_client = rc.RealTimeClientInterface(host)
        self._ur_kinematics = ur5()


    def points_in_circum(self, r, ox, oy, n=50):
        return [(np.cos(2 * np.pi / n * x) * r + ox, np.sin(2 * np.pi / n * x) * r + oy) for x in xrange(0, n + 1)]

    def generate_hull(self, cp, circle_radius):
        mask = np.zeros(self._mask_shape, np.uint8)
        for c in cp:
            points = self.points_in_circum(circle_radius, c[0], c[1], 50)
            cartesian = np.ones((len(points), 3))
            cartesian[:, :2] = points
            transformed = np.dot(self._M, cartesian.T).T
            for t in transformed:

                if self._mask_shape[0] < t[1] or t[0] < 0.0 or self._mask_shape[1] < t[0] or t[1] < 0.0:
                    continue
                mask[int(t[1]), int(t[0])] = 255
        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # we have points so merge all the contours which are around the points
        merged_contour = []
        for c1 in contours:
            for c2 in c1:
                merged_contour.append(c2)
        hull = cv2.convexHull(np.array(merged_contour), False)
        cv2.drawContours(mask, [hull], 0, 255, -1, 8)
        return mask

    def mask_to_points(self, mask, coeff):
        points = cv2.findNonZero(mask)[::coeff]
        points = points.reshape(-1, 2)
        cartesian = np.ones((len(points), 3))
        cartesian[:, :2] = points
        cartesian = np.dot(np.linalg.inv(self._M), cartesian.T).T
        return cartesian

    def create_pointcloud2(self, points, pointcloud_pts, color):
        for point in points:
            r, g, b = color
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0] # last is alpha
            pointcloud_pts.append([point[0], point[1], 0.05, rgb])

    def publish_point_clouds(self, robot_zone, critical_zone, danger_zone):

        pointcloud_pts = []
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base'

        robot_zone_pts = self.mask_to_points(robot_zone,20)
        danger_zone_pts = self.mask_to_points(danger_zone,15)
        critical_zone_pts = self.mask_to_points(critical_zone,15)
        self.create_pointcloud2(robot_zone_pts, pointcloud_pts, (0, 0, 255))
        self.create_pointcloud2(danger_zone_pts, pointcloud_pts, (255, 255, 0))
        self.create_pointcloud2(critical_zone_pts, pointcloud_pts, (255, 0, 0))

        pc2 = point_cloud2.create_cloud(header, fields, pointcloud_pts)
        print("Number of points: ", len(pointcloud_pts))
        self._rviz_pub.publish(pc2)

    def run(self, viz=True):
        while not rospy.is_shutdown():
            q = self._robot_client.get_feedback('joint_values')
            transformations = self._ur_kinematics.get_transformations(q)
            cp = [c[:3, 3] for c in transformations]

            ## generate all the zones
            robot_zone = self.generate_hull(cp, 0.20)
            temp_critical_zone = self.generate_hull(cp, 0.21)
            temp_danger_zone = self.generate_hull(cp, 0.35)
            critical_zone = temp_critical_zone - robot_zone
            danger_zone = temp_danger_zone - temp_critical_zone

            # critical_zone_cartesian_pts = self.mask_to_points(critical_zone)
            self.publish_point_clouds(robot_zone, critical_zone, danger_zone)

            if viz:
                cv2.imshow("img", critical_zone)
                cv2.waitKey(5)



def main():
    rospy.init_node('publish_safety_area')
    zp = ZonePublisher('192.168.125.100')
    zp.run(viz=False)

if __name__ == "__main__":
    main()