#!/usr/bin/env python2

# Description: plot script for autonomous inspection task of HK2021

import yaml
import matplotlib.pylab as plt
plt.close()

# load GT
stream = open("ground_truth.yaml", "r")
yamldocs_gt = yaml.load_all(stream,Loader=yaml.SafeLoader)
dets_gt = []
for entry in yamldocs_gt: 
    dets_gt.append(entry)

# load output file to evaluate
filename_eval = "example_output_file.yaml"
stream = open(filename_eval, "r")
yamldocs_eval = yaml.load_all(stream,Loader=yaml.SafeLoader)
dets_eval = []
for entry in yamldocs_eval: 
    if (len(dets_eval) < len(dets_gt)):   # Only the first len(dets_gt) detections will be counted (to prevent cheating by just randomly placing objects)
        dets_eval.append(entry)

# plot initial robot pose
fig, ax = plt.subplots()
ax.plot(0,0, 'b>', markersize=12)

# plot gt detections
for det in dets_gt:
    if(det["obj_type"] == "A"):
        markerstyle = "bo"
        mA, = ax.plot(det["XY_pos"][0],det["XY_pos"][1], markerstyle)
    elif(det["obj_type"] == "B"):
        markerstyle = "ro"
        mB, = ax.plot(det["XY_pos"][0],det["XY_pos"][1], markerstyle)
    elif(det["obj_type"] == "C"):
        markerstyle = "go"
        mC, = ax.plot(det["XY_pos"][0],det["XY_pos"][1], markerstyle)  

# plot eval detections
for det in dets_eval:
    if(det["obj_type"] == "A"):
        markerstyle = "bx"
        ax.plot(det["XY_pos"][0],det["XY_pos"][1], markerstyle)
    elif(det["obj_type"] == "B"):
        markerstyle = "rx"
        ax.plot(det["XY_pos"][0],det["XY_pos"][1], markerstyle)
    elif(det["obj_type"] == "C"):
        markerstyle = "gx"
        ax.plot(det["XY_pos"][0],det["XY_pos"][1], markerstyle)
    else:
        print "WARNING! Faulty obj_type in eval file"
        
ax.legend((mA, mB, mC), ('A', 'B', 'C'))
ax.set_aspect('equal', 'box') 
plt.show()
