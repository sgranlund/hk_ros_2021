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

    hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    blue_lower = numpy.array([99,70,68])
    blue_higher = numpy.array([109,255,255])
    blue_mask = cv2.inRange(hsv_img,blue_lower,blue_higher)

    green_lower = numpy.array([52,106,0])
    green_higher = numpy.array([179, 255,255])
    green_mask = cv2.inRange(hsv_img,green_lower,green_higher)

    mask = green_mask + blue_mask
    img_mask = cv2.bitwise_and(img,img, mask = mask)

    imgBlur = cv2.GaussianBlur(img_mask, (7,7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)

    threshold1 = 0 #cv2.getTrackbarPos("Threshold1", "Parameters")    #0
    threshold2 = 43 #cv2.getTrackbarPos("Threshold2", "Parameters")    #43
    
    imgCanny = cv2.Canny(imgGray, threshold1, threshold2)

    kernel = numpy.ones((5,5))
    imgDil = cv2.dilate(imgCanny,kernel,iterations=0) #first argument changed from imCanny and second from kernel to None

    
    #_, thrash = cv2.threshold(imgDil,240,255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(imgDil, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

     for contour in contours:
        if cv2.arcLength(contour,True) < 50 :
           continue
        approx = cv2.approxPolyDP(contour, 0.06* cv2.arcLength(contour, True), True)
        area = cv2.contourArea(approx)
        if area < 300 or area > 1100:
            continue
        cv2.drawContours(img, [approx], 0, (255, 0, 255), 4)
        x = approx.ravel()[0]
        y = approx.ravel()[1]-5
        if len(approx) == 3:
            cv2.putText( img, "Triangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 255) )
            print("Triangle: " + str(area))
        elif len(approx) == 4 :
            cv2.putText(img, "rectangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 255))
            print("Rectangle: " + str(area))
        elif len(approx) == 5 :
            cv2.putText(img, "pentagon", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 255))
            print("Pentagon:" + str(area))

    cv2.imshow("Result",img)

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
           


