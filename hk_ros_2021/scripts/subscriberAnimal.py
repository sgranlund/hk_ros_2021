#!/usr/bin/env python  


import rospy
import math
import tf
import geometry_msgs.msg
import numpy
from std_msgs.msg import String,Float32,Float32MultiArray,MultiArrayLayout,MultiArrayDimension
import json

def detectAnimal():
    try:
        (trans,rot) = listener.lookupTransform('/plotFrame', '/animal', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None
    (x, y, z) = trans
    return (x, y)


if __name__ == '__main__':
    rospy.init_node('listener')

    listener = tf.TransformListener()
    pub = rospy.Publisher('animalDetections', String , queue_size=10)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        coord = detectAnimal()
        pub.publish(json.dumps({'coords' :coord}))
        rate.sleep()
           