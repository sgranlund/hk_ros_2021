#!/usr/bin/env python  
import rospy
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import String


def tags():
    rospy.init_node('listener')

    listener = tf.TransformListener()
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/tag_8', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #posX= str(trans[0])
        # #posY = str(trans[1])
        # #rospy.loginfo("8:" +"x: "+posX+" "+"y: "+posY)
        rate.sleep()
    

if __name__ == '__main__':
    tags()
