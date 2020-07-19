import rospy
import math
from clover import srv
from std_srvs.srv import Trigger
import json

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


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

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

def open_json(filename):
    with open(filename) as json_file:
        filedrone = json.load(json_file)

    xsim = []
    ysim = []
    zsim = []

    numberdots = len(filedrone["trajectory"])

    l=len(filedrone["timeline_sec"])

    maxtime=filedrone["timeline_sec"][l-1]

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



#start program

#start time
start_stamp = rospy.get_rostime()

#open file with coordinates
xsim,ysim,zsim,numberdots,timestamp, maxtime=open_json("10secdrone_1.json")

tol=0.1

sped=0.5

dx=1
dy=1

visota=2.5

takeoff_wait(visota,speed=sped, tolerance=0.2)
t1=visota/sped
#tel1=get_telemetry(frame_id='aruco_map')
#x1=tel1.x-dx
#y1=tel1.y-dy
#z1=tel1.z

medDeltaX=0
medDeltaY=0
medDeltaZ=0

#data=open("data.txt",'w')

for i in range(numberdots):
    if (i>0):
        sped=get_distance(xsim[i-1],ysim[i-1],zsim[i-1],xsim[i],ysim[i],zsim[i])/timestamp
        print("Speed:")
        print(sped)
    navigate_wait(xsim[i] + dx, ysim[i] + dy, zsim[i], speed=sped, frame_id='aruco_map', tolerance=tol)
    print("Vremya:")
    print((rospy.get_rostime()-start_stamp).to_sec())
    if i==0:
       nachvremya=(rospy.get_rostime()-start_stamp).to_sec()
    if i==numberdots-1:
        konvremya=(rospy.get_rostime()-start_stamp).to_sec()-nachvremya
        print(" ")
        print("Time:")
        print(konvremya)
    print(i+1," dot:")
    tel = get_telemetry(frame_id='aruco_map')
    print("Telemetry Data")
    print('x=', tel.x-dx, 'y=', tel.y-dy, 'z=', tel.z)
    #if i==0:
        #dist1=get_distance(x1,y1,z1,tel.x-dx,tel.y-dy,tel.z)
        #t2=t1+dist1/sped
        #g = str(tel.x - dx) + ' ' + str(tel.y - dy) + ' ' + str(tel.z) + ' ' + str(t2) + '\n'
        #data.write(g)
    #if i>0:
        #t3=t2+timestamp*i
        #g=str(tel.x-dx)+' '+str(tel.y-dy)+' '+str(tel.z)+' '+str(t3)+'\n'
        #data.write(g)
    print("Real Data")
    print('x=', xsim[i], 'y=', ysim[i], 'z=', zsim[i])
    DeltaX=abs(xsim[i]-(tel.x-dx))
    DeltaY=abs(ysim[i]-(tel.y-dy))
    DeltaZ=abs(zsim[i]-tel.z)
    medDeltaX=medDeltaX+DeltaX
    medDeltaY = medDeltaY + DeltaY
    medDeltaZ = medDeltaZ + DeltaZ
    print("Delta")
    print('dx=', DeltaX, 'dy=', DeltaY, 'dz=', DeltaZ)
    print("  ")

    if i==numberdots-1:
        print("Medium Delta X:")
        print(medDeltaX/numberdots)
        print("Medium Delta Y:")
        print(medDeltaY / numberdots)
        print("Medium Delta Z:")
        print(medDeltaZ / numberdots)
        land_wait()


