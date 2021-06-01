#!/usr/bin/env python
"""
@author: Lars Schilling

"""
import rospkg
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image


def transform_data(data):

	data=np.array(data)
	data[data==0] = 1
	data[data==127] = 2
	data[data==255] = 0

	return data



if __name__ == '__main__':
	rospack = rospkg.RosPack()
	img = Image.open(rospack.get_path('heron_exploration')+'/sample_maps/sample_map_1.png')
	img_work = transform_data(img)
	plt.imshow(img_work)
	plt.show()

	###try frontier point detection with img_work
	#find all frontier points, can be defined as edge detection problem, cv2.Canny can be used


	#calculate information gain, tip: set everything to zero expect unexplored area
	#you can use cv2.filter2D with your own kernel


	#find the frontier point with the biggest information gain, this will be our goal point
