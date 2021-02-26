#!/usr/bin/env python2

# Example how to generate the output file

import yaml
import rospkg
import subscriberAT

# 1 create an empty list to store the detections
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json
detections = []

def callback(data):
    
    x= json.loads(data.data)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", x)
    i = 0
    for d in detections:
        if d['name'] == x['name']:
            d['XY_pos'] = x['coords']
            i = 1
    
    if i == 0:
        detections.append({"obj_type": "A", "XY_pos" : x['coords'], "name" : x['name']}) 
    
def outputListener():

    rospy.init_node('outputListener', anonymous=True)
    rospy.Subscriber("apriltagList", String, callback)
    rospy.spin()

if __name__ == '__main__':
    outputListener()
    
    
# 2 append detections during the run
# remember to add logic to avoid duplicates

    
# 3 save the file
filename = "latest_output_file.yaml"
filepath = rospkg.RosPack().get_path('hk_ros_2021') + '/exported_detection_logs/' 

for d in detections:
    d.pop('name', None)

with open(filepath + filename, 'w') as outfile:
    yaml.dump_all(detections, outfile,explicit_start=True)
