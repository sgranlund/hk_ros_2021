#!/usr/bin/env python  

import rospy
import math
import tf
import geometry_msgs.msg
import numpy
from std_msgs.msg import String,Float32,Float32MultiArray,MultiArrayLayout,MultiArrayDimension
import yaml
import json

def detectTag(tagId):
    try:
        (trans,rot) = listener.lookupTransform('/map', tagId, rospy.Time(0))
        (odomTrans,odomRot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None
    (x, y, z) = trans
    (x2,y2,z2)=odomTrans
    (wq,xr,yr,zr)=rot
  
    #x= x1-x2
    #y=y1-y2
    #print(x)
    #print(y)
    return (-x, -y)


if __name__ == '__main__':
    rospy.init_node('listener')

    listener = tf.TransformListener()
    pub = rospy.Publisher('apriltagList', String , queue_size=10)
    rate = rospy.Rate(10.0)

    
    tagNames = ["tag_9","tag_8","tag_7","tag_6","tag_5","tag_4","tag_3","tag_2", "tag_1"] 
    global tagTable
    tagTable={}
    
    while not rospy.is_shutdown():
        for tagName in tagNames:
            coordinates = detectTag(tagName)
            if coordinates is not None:
                tagTable[tagName] = coordinates
                pub.publish(json.dumps({'coords' : tagTable[tagName],'name': tagName}))

        #print tagTable
        rate.sleep()
           