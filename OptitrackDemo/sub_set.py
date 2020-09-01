from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import RCIn

from std_msgs.msg import String
import rospy
import math
from clover import srv
from std_srvs.srv import Trigger
import json

import os
import sys
from clover.srv import SetLEDEffect

rospy.init_node('subscriber')

set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

dx=1.5
dy=2
i=0

xx=2
yy=2
zz=1.5
yaww=0


def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

def takeoff_wait(alt, speed=0.5, tolerance=0.2):
    start = get_telemetry()
    print navigate(z=alt, speed=speed, frame_id='body', auto_arm=True)

    while not rospy.is_shutdown():
        if start.z + alt - get_telemetry().z < tolerance:
            break

        rospy.sleep(0.2)

def navigate_wait(x, y, z, speed, frame_id, tolerance=0.2):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id=frame_id)
        if get_distance(x, y, z, telem.x, telem.y, telem.z) < tolerance:
            break
        rospy.sleep(0.2)

def pose_update(os):
    print("New message")
    print(os)

def callback(msg):
   
    global xx
    xx=msg.pose.position.x+dx
    global yy
    yy=msg.pose.position.y
    global zz
    zz=msg.pose.position.z
    #global yaww
    #yaww=round(msg.pose.orientation.z,2)
    
 
def open_json(filename):
    with open(filename) as json_file:
        filedrone = json.load(json_file)

    xsim = []
    ysim = []
    zsim = []

    numberdots = len(filedrone["trajectory"])

    l=len(filedrone["timeline_sec"])

    maxtime=int(filedrone["timeline_sec"][l-1])

    timestamp = filedrone["timeline_sec"][1] - filedrone["timeline_sec"][0]

    j = 0
    for j in range(3):
        k = 0
        if j == 0:
            for k in range(numberdots):
                xr = filedrone["trajectory"][k][j]
                xsim.append(xr)

        if j == 1:
            for k in range(numberdots):
                yr = filedrone["trajectory"][k][j]
                ysim.append(yr)

        if j == 2:
            for k in range(numberdots):
                zr = filedrone["trajectory"][k][j]
                zsim.append(zr)
    return xsim, ysim, zsim, numberdots, timestamp, maxtime

set_effect(r=34, g=139, b=34)

takeoff_wait(1.5,speed=0.5,tolerance=0.2)

start_stamp = rospy.get_rostime()

time=float(sys.argv[1])

print(time)

vremya=int(sys.argv[2])

pi=3.14159

rospy.Subscriber('/optitrack_data', PoseStamped, callback)
vr=0
j=0
while vr<vremya:
    vr = (rospy.get_rostime() - start_stamp).to_sec()
    print("Dot:",j)
    print(xx)
    print(" ")
    print(yy)
    print(" ")
    print(yaww)
    print(" ")
    set_position(x=xx,y=yy,z=zz,yaw=yaww,frame_id='aruco_map')
    j=j+1
    rospy.sleep(time)
