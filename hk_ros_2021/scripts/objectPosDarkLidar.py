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
from sensor_msgs.msg import LaserScan
import turtlesim.msg 

import geometry_msgs.msg
class animalDetect(object):
    def __init__(self):
        self.range=[]
        self.objAng=None
        self.x =0
        self.y=0
        self.positioned=[]
        
        self.bounding_boxes = BoundingBoxes()
        self.objectCount = ObjectCount()
        rospy.Subscriber('/scan', LaserScan, self.scan)
        rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.update_bounding_boxes)
  
    def update_bounding_boxes(self, data):
        self.bounding_boxes = data
        bounding_boxes=self.bounding_boxes
       
        if bounding_boxes.bounding_boxes[0].Class=="cat" or bounding_boxes.bounding_boxes[0].Class=="dog" or bounding_boxes.bounding_boxes[0].Class=="cow":
            xPos = (bounding_boxes.bounding_boxes[0].xmax + bounding_boxes.bounding_boxes[0].xmin )/2 
            pixToAng = 640/62.2 #resolution is 640x480, camera angle is 53.5
            print('**************')
            print(xPos)
            print('**************')
            xAng= int(xPos/pixToAng-31.1)
            if xAng<0:
                self.objAng= -xAng
            else:
                self.objAng=359-xAng
            self.publish()

    def scan(self,scanData):
        self.range= scanData.ranges

    def publish(self):
        
        x = math.cos(math.radians(self.objAng))*self.range[self.objAng]
        y= math.sin(math.radians(self.objAng))*self.range[self.objAng]
        try:
            msg= turtlesim.msg.Pose()
            msg.x= x
            msg.y= y
            pub.publish(msg)
              
        except rospy.ROSInterruptException:
            pass


           
if __name__ == '__main__':
   
    rospy.init_node('listeningDark')
    pub = rospy.Publisher('animalCoords',turtlesim.msg.Pose , queue_size=1)

    animalDetect = animalDetect()

    rospy.spin()
