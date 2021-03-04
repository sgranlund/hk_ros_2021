#!/usr/bin/env python2

# Example how to generate the output file

import yaml
import rospkg
import subscriberAT
import kmeans as k

# 1 create an empty list to store the detections
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json
detections1 = [] #apriltags
detections2 =[] #shapes
detections3 = [] #animals
detections=[] 
checkx =0
checky=0
checkshapex = 0
checkshapey = 0

def aprilTagCallback(data):
   
    x= json.loads(data.data)
    x= json.loads(data.data)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", x)
    i = 0
    for d in detections1:
        if d['name'] == x['name']:
            d['XY_pos'] = x['coords']
            i = 1
    
    if i == 0:
        detections1.append({"obj_type": "A", "XY_pos" : x['coords'], "name" : x['name']}) 

def shapesCallback(data):
    f = json.loads(data.data)
    if f['coords'] is not None:
        if f['coords'][1]>= (checkx+0.2) or f['coords'][1]<= (checkx-0.2):
            if f['coords'][0]>= (checky+0.2) or f['coords'][0]<= (checky-0.2):
                print('**********')
                print('True')
                print(checkx)
                print(f['coords'])
                print('**********')
                detections2.append(f['coords']) 
                listOfGloabals = globals()
                listOfGloabals['checkx'] = f['coords'][1]
                listOfGloabals['checky'] = f['coords'][0]

def animalsCallback(data):   
    f= json.loads(data.data)

    if f['coords'] is not None:
        if f['coords'][1]>= (checkx+0.2) or f['coords'][1]<= (checkx-0.2):
            if f['coords'][0]>= (checky+0.2) or f['coords'][0]<= (checky-0.2):
                detections3.append(f['coords']) 
                listOfGloabals = globals()
                listOfGloabals['checkx'] = f['coords'][1]
                listOfGloabals['checky'] = f['coords'][0]



    
def outputListener():

    rospy.init_node('outputListener', anonymous=True)
    rospy.Subscriber("apriltagList", String, aprilTagCallback)
    rospy.Subscriber("animalDetections", String, animalsCallback)
    rospy.Subscriber("shapeDetection", String, shapesCallback)
    rospy.spin()

if __name__ == '__main__':
    outputListener()
    
    
# 2 append detections during the run
# remember to add logic to avoid duplicates


k_tests = [2,3,4,5,6,7,8,9,10,11,12,13]

#kmeans for apriltags
#detections1 = k.concentrate_points(detections1,0.5,0.5,3)
#if len(detections1)>0:
#    k_likely_april = k.most_likely_k(k_tests,detections1)
#    centroids_april,_ = k.k_means(k_likely_april, detections1)
#    for c in centroids_april:
#        detections.append({"obj_type": "A", "XY_pos" : c})


#kmeans for shapes
#detections2 = k.concentrate_points(detections2,0.5,0.5,3)
if len(detections2)>0:
    k_likely_shapes = k.most_likely_k(k_tests,detections2)
    centroids_shapes,_ = k.k_means(k_likely_shapes, detections2)
    for c in centroids_shapes:
        detections.append({"obj_type": "B", "XY_pos" : c})
 
#kmeans for animals
#detections3 = k.concentrate_points(detections3,0.5,0.5,3)
if len(detections3)>0:
    k_likely_animals = k.most_likely_k(k_tests,detections3)
    centroids_animals,_ = k.k_means(k_likely_animals, detections3)
    for c in centroids_animals:
        detections.append({"obj_type": "C", "XY_pos" : c}) 


detections = detections + detections1
# 3 save the file
filename = "latest_output_file.yaml"
filepath = rospkg.RosPack().get_path('hk_ros_2021') + '/exported_detection_logs/' 


with open(filepath + filename, 'w') as outfile:
    yaml.dump_all(detections, outfile,explicit_start=True)
