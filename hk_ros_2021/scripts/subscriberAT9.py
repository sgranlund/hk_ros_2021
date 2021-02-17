#!/usr/bin/env python  

import rospy
import math
import tf
import geometry_msgs.msg
import numpy


def detectTag(tagId):
    try:
        (trans,rot) = listener.lookupTransform('/map', tagId, rospy.Time(0))
        (camTrans,camRot) = listener.lookupTransform('/map', '/camera_rgb_optical_frame', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None
    (x, y, z) = trans
    (rx, ry, rz) = camTrans
    
    return (-x, -y)


if __name__ == '__main__':
    rospy.init_node('listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    
    tagNames = ["tag_9","tag_8"] 
    global tagTable
    tagTable={}
    
    while not rospy.is_shutdown():
        for tagName in tagNames:
            coordinates = detectTag(tagName)
            if coordinates is not None:
                tagTable[tagName] = coordinates

        
        print tagTable
        rate.sleep()
           