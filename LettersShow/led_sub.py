import rospy
from clover.srv import SetLEDEffect
import math
from clover import srv
from std_srvs.srv import Trigger
import json
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import RCIn
import os
import sys

# ...

rospy.init_node('led')

def open_json_rgb(filename):
    with open(filename) as json_file:
        filedrone = json.load(json_file)

    r = []
    g = []
    b = []

    numberdots = len(filedrone["trajectory"])

    l=len(filedrone["timeline_sec"])

    maxtime=filedrone["timeline_sec"][l-1]

    timestamp = filedrone["timeline_sec"][1] - filedrone["timeline_sec"][0]
    j = 0
    for j in range(l):
        rr = filedrone["rgb"][j][0]
        r.append(rr)
        gg = filedrone["rgb"][j][1]
        g.append(gg)
        bb = filedrone["rgb"][j][2]
        b.append(bb)

    return r, g, b, numberdots, timestamp, maxtime

#begin_program

red,green,blue,numberdots,timestamp, maxtime=open_json(filename)

set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service



for i in range(numberdots):
    set_effect(r=red[i], g=green[i], b=blue[i])  # fill strip with red color
    rospy.sleep(timestamp)


