#!/usr/bin/env python  

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math
import tf
import geometry_msgs.msg
import numpy as np
from std_msgs.msg import String,Float32,Float32MultiArray,MultiArrayLayout,MultiArrayDimension
import yaml
import json
from matplotlib import pyplot as plt

bridge = CvBridge()

def show_image(title,img):
	cv2.imshow(title, img)
	cv2.waitKey(3)

def show_thresh(img):
	cv2.imshow("shape threshold", img)
	cv2.waitKey(3)

def show_all(images):
	for i in xrange(4):
		plt.subplot(2,2,i+1),plt.imshow(images[i],'gray')
		plt.title(['Original','Processed','Threshold','Detections'][i])
		plt.xticks([]),plt.yticks([])
	plt.show()

def detect_shape(img):
	original = img
	
	#img = cv2.blur(img,(4,4))

	img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	lower_blue = (110,50,30)
	upper_blue = (140,255,140)

	lower_green = (35,50,10)
	upper_green = (80,255,140)


	blue_mask = cv2.inRange(img_hsv, lower_blue, upper_blue)
	blue_mask = cv2.erode(blue_mask, None, iterations=2)
	blue_mask = cv2.dilate(blue_mask, None, iterations =2)

	green_mask = cv2.inRange(img_hsv, lower_green, upper_green)
	green_mask = cv2.erode(green_mask, None, iterations=2)
	green_mask = cv2.dilate(green_mask, None, iterations=2)
	mask = blue_mask + green_mask

	#res = cv2.bitwise_and(img,img,mask=mask)

	show_image("mask",mask)

	#imgGry = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
	
	#imgGry = cv2.blur(imgGry,(5,5))
	#imgGry = cv2.equalizeHist(imgGry)
	_, thrash = cv2.threshold(mask, 240, 255, cv2.THRESH_BINARY)
	#thrash = cv2.adaptiveThreshold(imgGry, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
	_, contours, hierarchy = cv2.findContours(thrash, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
	#show_image("Processed",imgGry)
	#show_image("Threshold",thrash)
	for contour in contours:
		#rospy.loginfo("Arclength: "+str(cv2.arcLength(contour,True)))
		if cv2.arcLength(contour,True) < 100:
			continue
		approx = cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour, True),True)
		cv2.drawContours(img, [approx], 0, (0,0,0),5)
		x = approx.ravel()[0]
		y = approx.ravel()[1]

		if len(approx) == 3:
			cv2.putText(img, "Triangle", (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
		elif len(approx) == 4:
			cv2.putText(img, "rectangle", (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))
			#x,y,w,h = cv2.boundingRect(approx)
			#aspectRatio = float(w)/h
			#if aspectRatio >= 0.95 and aspectRatio < 1.05:
			#	cv2.putText(img, "square", (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))
			#else:
			#	cv2.putText(img, "rectangle", (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))

		elif len(approx) == 5:
			cv2.putText(img, "pentagon", (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))
		#elif len(approx) == 10:
		#	cv2.putText(img, "star", (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))	
		#else:
		#	cv2.putText(img, "circle", (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))
	show_image("shape detections",img)
	#show_all([original,imgGry,thrash,img])

def callback(img_msg):
	#rospy.loginfo(img_msg.header)

	try:
		cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
	except CvBridgeError, e:
		rospy.logerr("CvBridge error: {0}".format(e))

	detect_shape(cv_image)

if __name__ == '__main__':
    rospy.init_node('listener')

    listener = tf.TransformListener()
    pub = rospy.Publisher('shapesList', String , queue_size=10)
    sub_image = rospy.Subscriber("/raspicam_node/image", Image, callback)
    rate = rospy.Rate(10.0)

    
    while not rospy.is_shutdown():
        rate.sleep()
           
