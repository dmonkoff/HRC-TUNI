#!/usr/bin/env python
import rospy, geometry_msgs.msg
import tf
from tf import transformations as ts

from geometry_msgs.msg import Transform, Vector3, Quaternion
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

if __name__ == '__main__':
	links = ['base_link','link_1','link_2','link_3','link_4','link_5','link_6','flange','tool0']
	rospy.init_node('global_coords_broadcaster')
	publishers = {}
	for link in links:
		publishers[link] = rospy.Publisher('coords/'+link, TransformStamped, queue_size=10)
	#publishers['all'] = rospy.Publisher('coords/all', TransformStamped, queue_size=10)
	rospy.loginfo("starting node")
	br = tf.TransformBroadcaster()
	listener = tf.TransformListener()
	listener.waitForTransform('base','flange',rospy.Time(),rospy.Duration(4))
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'base'   # the parent link
		for link in links:
			header = Header()
			header.stamp = rospy.Time.now()
			header.frame_id = 'base'   # the parent link
			try:
				(trans, rot) = listener.lookupTransform('base', link, rospy.Time(0))
				# because it is not the tf::Transform, there is no reverse member function call, do it manually
				transform = ts.concatenate_matrices(ts.translation_matrix(trans), ts.quaternion_matrix(rot))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				print('something went wrong')
				continue

			# now publish the inverse frame back, it should stay at the same location of '/world'
			trans = Transform(translation=Vector3(*ts.translation_from_matrix(transform)),
					  rotation=Quaternion(*ts.quaternion_from_matrix(transform))
					)
			#trans = Transform(translation=Vector3(trans), rotation=Quaternion(rot))
			trans_stamp = TransformStamped(header, link, trans)

			#br.sendTransformMessage(trans_stamp)
			publishers[link].publish(trans_stamp)
		rate.sleep()
