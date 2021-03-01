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
        rospy.Subscriber('raspicam_node/camera_info', CameraInfo, self.call)
        rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.update_bounding_boxes)
        
    def update_bounding_boxes(self, data):
        self.bounding_boxes = data
        bounding_boxes=self.bounding_boxes
        # modelObject = numpy.array(([0, 0, 0],                 #xmin ymin
        #                              [0, 100, 0],                  #xmin ymax
        #                              [100, 100, 0],                  #xmax ymax 
        #                              [100, 0, 0]), dtype=numpy.double) #xmax ymin
        tag_size = 0.048
        tag_radius = tag_size/2
        c1 = numpy.array([454.5, 331.5])
        c2 = numpy.array([485.5, 333.5])
        c3 = numpy.array([487.5, 305.5])
        c4 = numpy.array([454.5, 303.5])
        modelImage = numpy.array([c4, c3, c2, c1])
        M1 = numpy.array([-tag_radius, -tag_radius, 0.0])
        M2 = numpy.array([ tag_radius, -tag_radius, 0.0])
        M3 = numpy.array([ tag_radius,  tag_radius, 0.0])
        M4 = numpy.array([-tag_radius,  tag_radius, 0.0])

        modelObject = numpy.array([M1, M2, M3, M4])

        # modelImage = numpy.array(([0, 0],                 #xmin ymin
        #                              [0, 10],                  #xmin ymax
        #                              [10, 10],                  #xmax ymax 
        #                              [10, 0]), dtype=numpy.double) #xmax ymin
        fx = 529.2945040622658
        fy = 531.2834529497384
        cx = 466.96044871160075
        cy = 273.2593671723483
        cameraMatrix= numpy.array([[fx,  0, cx,
                           0,  fy, cy,
                           0.0,   0.0,  1.0]])
        dist = numpy.array([[0.0676550948466241, -0.058556753440857666, 0.007350271107666055, -0.00817256648923586]])
        #self.solve_pnp(modelObject,modelImage,cameraMatrix,dist)
        print  ' *********************************************** '
        ret, rvec, tvec= cv2.solvePnP(modelObject,modelImage,cameraMatrix,dist)
        rospy.loginfo(rospy.get_caller_id() + "I heard %s",tvec)
        print  ' *********************************************** '
        if bounding_boxes.bounding_boxes[0].Class=="cat" or bounding_boxes.bounding_boxes[0].Class=="dog" or bounding_boxes.bounding_boxes[0].Class=="cow":
            rospy.loginfo(rospy.get_caller_id() + "Found %s",bounding_boxes.bounding_boxes[0].Class)
   
    def call(self,data):
        camera = PinholeCameraModel()
        camera.fromCameraInfo(data)
        self.fx= camera.fx()
        self.fy= camera.fy()
        self.cx= camera.cx()
        self.cy= camera.cy()
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s",self.fx)
    

    def solve_pnp(self,modelPoints, imagePoints,cameraMat, distCoeff):
        print  ' *********************************************** '
        retval, rotVec, transVec= cv2.solvePnP(modelPoints, imagePoints,cameraMat, distCoeff, numpy.array([]), numpy.array([]), False, 0)
        rospy.loginfo(rospy.get_caller_id() + "I heard %s",transVec)
        print  ' *********************************************** '
           
if __name__ == '__main__':
    rospy.init_node('listeningDark')
    animalDetect = animalDetect()

    rospy.spin()