#!/usr/bin/env python2

# Example how to generate the output file

import yaml
import rospkg
import subscriberAT9
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
           
# 1 create an empty list to store the detections

detections = []

# 2 append detections during the run
# remember to add logic to avoid duplicates

# first dummy detection (apriltag)
detections.append({"obj_type": "A", "XY_pos": [0.756,3.332]})

# second dummy detection (geometric shape)
detections.append({"obj_type": "B", "XY_pos": [3.396,0.123]})

# third dummy detection (animal)
detections.append({"obj_type": "C", "XY_pos": [6.001,2.987]})   
    
# 3 save the file
filename = "latest_output_file.yaml"
filepath = rospkg.RosPack().get_path('hk_ros_2021') + '/exported_detection_logs/' 

with open(filepath + filename, 'w') as outfile:
    yaml.dump_all(detections, outfile,explicit_start=True)
