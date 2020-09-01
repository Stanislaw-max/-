import rospy
import math
from clover import srv
from std_srvs.srv import Trigger
import json
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import RCIn
import os
import sys

rospy.init_node('circle')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


RADIUS = 1  # m
SPEED = 0.6  # rad / s
start = get_telemetry(frame_id='body')
xx=2.5
yy=2.5
zz=2


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

def callback(msg):
    global xx
    xx = msg.pose.position.x 
    global yy
    yy = msg.pose.position.y
    global zz
    zz = msg.pose.position.z
   # navigate_wait(xx,yy,zz,1,'aruco_map',tolerance=0.2)
   # rospy.sleep(2)

takeoff_wait(1.8,speed=0.5, tolerance=0.2)


start_stamp = rospy.get_rostime()

r = rospy.Rate(30)

rospy.Subscriber('/optitrack_data', PoseStamped, callback)

while not rospy.is_shutdown():
    angle = (rospy.get_rostime() - start_stamp).to_sec() * SPEED
    x1 = xx + math.sin(angle) * RADIUS
    y1 = yy + math.cos(angle) * RADIUS
    z1 = zz+0.7
    set_position(x=x1, y=y1, z=z1, frame_id='aruco_map',yaw=-angle)
    tel = get_telemetry(frame_id='aruco_map')
    print(xx,yy)
    print(x1,y1)
    print(tel.x,tel.y)
    print(math.sin(angle))
    print(" ")
    r.sleep()
