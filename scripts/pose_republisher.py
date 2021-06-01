#!/usr/bin/env python
"""
@author: Lars Schilling

"""
#additional imports
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def callback(data):
	odom = data
	transformed_pose = transform_pose(odom, "odom", "map")
	print(transformed_pose)
	pose_republisher.publish(transformed_pose)

def transform_pose(input_pose, from_frame, to_frame):

	pose_stamped = PoseStamped()
	pose_stamped.pose.position = input_pose.pose.pose.position
	pose_stamped.pose.orientation = input_pose.pose.pose.orientation
	pose_stamped.header.frame_id = from_frame
	pose_stamped.header.stamp = input_pose.header.stamp
	return pose_stamped
	

if __name__ == '__main__':
	try:
		rospy.init_node('pose_transform')
		pose_subscriber = rospy.Subscriber('/odometry/gps', Odometry, callback)
		pose_republisher = rospy.Publisher('/pose', PoseStamped, queue_size=1)
		rospy.spin()
	except rospy.ROSInterruptException: pass
