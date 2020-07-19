from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import RCIn

from std_msgs.msg import String
import rospy
import math
from clover import srv
from std_srvs.srv import Trigger
import json

rospy.init_node('test')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

x=0
y=0
z=0

def pose_update(os):
    print("New message")
    print(os)

def callback(msg):
    print("Haha")
    global x
    x=msg.pose.position.x
    global y
    y=msg.pose.position.y
    global z
    z=msg.pose.position.z
    print(msg)

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

dx=1
dy=1

data2=open("data_test.txt",'w')

xsim,ysim,zsim,numberdots,timestamp, maxtime=open_json("10secdrone_1.json")

start_stamp = rospy.get_rostime()

#rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_update)

rospy.Subscriber('/mavros/setpoint_position/local', PoseStamped, callback)

w=20

for i in range(maxtime*w):
    vr = (rospy.get_rostime() - start_stamp).to_sec()
    tel = get_telemetry(frame_id='aruco_map')
    print(tel.x)
    print(" ")
    g = str(tel.x - dx) + ' ' + str(tel.y - dy) + ' ' + str(tel.z) + ' ' + str(x) + ' '+ str(y) +' '+str(z) +' '+ str(vr) + '\n'
    data2.write(g)
    rospy.sleep(1/w)
