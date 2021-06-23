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
	rate = rospy.Rate(10)
	
	#visualize data
	plt.imshow(data)
	plt.title('Original Image')
	plt.show()

	#find all frontier points, can be defined as edge detection problem, cv2.Canny can be used
	edges = cv2.Canny(data, 11, 11, L2gradient = False)	
	plt.imshow(edges)
	plt.title('Edged Image')
	plt.show()
	#calculate information gain, tip: set everything to zero expect unexplored area
	#you can use cv2.filter2D with your own kernel
	info_gain = np.zeros((64,64), dtype=np.float64)
	indices = np.where(data == 2)
	info_gain[indices] = 2
	
	# define the kernel K
	K = np.ones((9,9),np.float64)/25
	#you can use cv2.filter2D with your own kernel
	conv_info_gain = cv2.filter2D(info_gain,-1,K)
	
	plt.imshow(conv_info_gain)
	plt.title('Convoluted on unexplored area')
	plt.show()

	#find the frontier point with the biggest information gain, this will be our goal point
	max_info_indices = np.where(conv_info_gain == np.max(conv_info_gain))	
	listOfCoordinates = list(zip(max_info_indices[1],max_info_indices[0]))
	
	# We will use the last found point as goal point
	max_info_gain = cv2.circle(edges, listOfCoordinates[len(listOfCoordinates)-1], radius = 0, color=(139,0,0), thickness=-1)

	plt.imshow(max_info_gain)
	plt.title('Goal point coordinates: ' + str(listOfCoordinates[len(listOfCoordinates) - 1]))	
	plt.show()
	

	#define a PoseStamped message here and publish it on the move_base_publisher
	goal=PoseStamped()
	goal.header.stamp=rospy.Time.now()
	goal.header.frame_id="odom"
	goal.pose.orientation.w=1
	#define x and y position of the pose here
	goal_y = listOfCoordinates[len(listOfCoordinates)-1][0]
	goal_x = listOfCoordinates[len(listOfCoordinates)-1][1]
	#use the odom position and the goal point
	goal_y = goal_y - odom.pose.pose.position.x
	goal_x = goal_x - odom.pose.pose.position.y
	#reminder: the "odom position of the boat" is the center of the image!
	#the goal point must still be converted, you will need the resolution of the map for this which is given with the parameter "resolution"
	goal_y = goal_y * resolution
	goal_x = goal_x * resolution

	goal.pose.position.x = odom.pose.pose.position.x + goal_y
	goal.pose.position.y = -(odom.pose.pose.position.y + goal_x)

	move_base_publisher.publish(goal)
	rate.sleep()

if __name__ == '__main__':
	try:

		rospy.init_node('frontier_exploration')
		move_base_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
		while not rospy.is_shutdown():
			frontier_exploration()
	except rospy.ROSInterruptException: pass
