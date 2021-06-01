#!/usr/bin/env python
"""
@author: Lars Schilling

"""
#imports
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt


def get_data():
	#get map and metadata to transfrom into a matrix

	msg=rospy.wait_for_message('/map_image/tile', Image)
	odom=rospy.wait_for_message('/odometry/gps', Odometry)
	#odom=rospy.wait_for_message('/pose_gt', Odometry)
	metadata=rospy.wait_for_message('/map_metadata', MapMetaData)
	resolution=metadata.resolution

	bridge = CvBridge()
	data = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
	data=np.array(data)
	data[data==0] = 1
	data[data==127] = 2
	data[data==255] = 0

	return data, odom, resolution

def frontier_exploration():

	data, odom, resolution = get_data()

	#visualize data
	plt.imshow(data)
	plt.show()

	#find all frontier points, can be defined as edge detection problem, cv2.Canny can be used


	#calculate information gain, tip: set everything to zero expect unexplored area
	#you can use cv2.filter2D with your own kernel


	#find the frontier point with the biggest information gain, this will be our goal point


	#define a PoseStamped message here and publish it on the move_base_publisher
	goal=PoseStamped()
	goal.header.stamp=rospy.Time.now()
	goal.header.frame_id="odom"
	goal.pose.orientation.w=1
	#define x and y position of the pose here
	#use the odom position and the goal point
	#reminder: the "odom position of the boat" is the center of the image!
	#the goal point must still be converted, you will need the resolution of the map for this which is given with the parameter "resolution"

if __name__ == '__main__':
	try:

		rospy.init_node('frontier_exploration')
		move_base_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
		while not rospy.is_shutdown():
			frontier_exploration()
	except rospy.ROSInterruptException: pass
