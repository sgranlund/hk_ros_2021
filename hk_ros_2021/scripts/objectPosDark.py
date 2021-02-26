#!/usr/bin/env python  

import rospy
import math
import tf
import geometry_msgs.msg
import numpy
from std_msgs.msg import String,Float32,Float32MultiArray,MultiArrayLayout,MultiArrayDimension
import yaml
import json
import cv2
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel, cameramodels

class animalDetect(object):
    def __init__(self):
        self.fx=0
        self.fy=0
        self.cx=0
        self.cy=0
        self.bounding_boxes = BoundingBoxes()
        rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.update_bounding_boxes)
        rospy.Subscriber('raspicam_node/camera_info', CameraInfo, self.call)
    def update_bounding_boxes(self, data):
        self.bounding_boxes = data
        bounding_boxes=self.bounding_boxes
        modelObject = numpy.array(([0, 0, 0],                 #xmin ymin
                                     [0, 100, 0],                  #xmin ymax
                                     [100, 100, 0],                  #xmax ymax 
                                     [100, 0, 0]), dtype=numpy.double) #xmax ymin

		
        modelImage = numpy.array(([0, 0],                 #xmin ymin
                                     [0, 10],                  #xmin ymax
                                     [10, 10],                  #xmax ymax 
                                     [10, 0]), dtype=numpy.double) #xmax ymin

        cameraMatrix= numpy.array([self.fx,  0, self.cx,
                           0,  self.fy, self.cy,
                           0,   0,  1])
        dist = numpy.float32([0.0, 0.0, 0.0])
        self.solve_pnp(modelObject,modelImage,cameraMatrix,dist)

        if bounding_boxes.bounding_boxes[0].Class=="cat" or bounding_boxes.bounding_boxes[0].Class=="dog" or bounding_boxes.bounding_boxes[0].Class=="cow":
            rospy.loginfo(rospy.get_caller_id() + "Found %s",bounding_boxes.bounding_boxes[0].Class)
   
    def call(self,data):
        camera = PinholeCameraModel()
        camera.fromCameraInfo(data)
        self.fx= camera.fx()
        self.fy= camera.fy()
        self.cx= camera.cx()
        self.cy= camera.cy()
        rospy.loginfo(rospy.get_caller_id() + "I heard %s",self.fx)
    

    def solve_pnp(self,modelPoints, imagePoints,cameraMat, distCoeff):
        _, rotVec, transVec= cv2.solvePnP(modelPoints, imagePoints,cameraMat, distCoeff)
        rospy.loginfo(rospy.get_caller_id() + "I heard %s",transVec)
           
if __name__ == '__main__':
    rospy.init_node('listeningDark')
    animalDetect = animalDetect()

    rospy.spin()