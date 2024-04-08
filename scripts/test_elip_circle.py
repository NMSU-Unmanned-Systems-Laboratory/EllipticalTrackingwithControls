#!/usr/bin/env python

import rospy
import time
import tf
import math
import csv

from threading import Thread

from nav_msgs.msg import Odometry

from ellipse_detect.msg import Location

import numpy as np
import socket
import threading

import keyboard

from drone import Drone
from drone import Controller

end = False
lastT = time.time()

debug = False

elVars = []
mPos = [0, 0, 0, 0]
mocapPos = []
timeStamps = []

switchTimes = []

goal = [
    [0, 0, 1, 0]
]

goals = [
    [-0.75, 0.55, 1, 0],
    [0, 0, 1, 0],
    [0.75, 0.55, 1, 0]
]


rospy.init_node('myNode', anonymous=True)


def opti_callback(msg, drone, procedureRun):
    global mPos
    
    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    yaw_rad = euler[2] - (math.pi/2)
    if (yaw_rad < -(math.pi)):
        yaw_rad += (2 * math.pi)
    
    mPos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw_rad]
    
    #if Drone.procedureRun:
    #    drone.update_vel(mPos, rotPosVels=True)

    #currT = time.time()

    #print(pos)
    #print(f"time since last: {currT - lastT}")

    #lastT = currT


def opti_callback_tune(msg, drone, procedureRun):
    
    
    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    yaw_rad = euler[2] - (math.pi/2)
    if (yaw_rad < -(math.pi)):
        yaw_rad += (2 * math.pi)
    
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw_rad]
    
    if procedureRun:
        drone.update_vel(pos, rotPosVels=True)

    




contParams = [
    	[0.22, 0, 0.4],
    	[0.22, 0, 0.4],
        [0.6, 0, 0.2],
    	[1, 0, 0]
    ]


myDrone = Drone('bebop_03', 'bebop1', Controller('PID', 3, 1, contParams), opti_callback)

      


def ell_callback(msg):
    global myDrone, elVars, timeStamps


    pos = [msg.x, msg.y, msg.z, mPos[3]]

    if (debug):
        print(pos)


    if (Drone.procedureRun):
        timeStamps.append(time.time())
    
        elVars.append([msg.x, msg.y, msg.z, msg.v])
    
        mocapPos.append(mPos)

        myDrone.update_vel(pos, rotPosVels=True, verbose=False)






def genCircle(rads, r, yOff):
    x = r * math.cos(rads)
    y = r * math.sin(rads) + yOff
    
    return x, y


def emergency_stop():
    input('hit enter to stop')

    Drone.procedureRun = False


def main():
    global myDrone, debug

    debug = False

    rospy.Subscriber(f'/Coordinates', Location, ell_callback)


    #input()

    if (False):

        Drone.procedureRun = True

        input('hit enter to stop error analysis')

        errsY = []
        errsX = []
        errsZ = []

        # write to csv file then break
        #csvfile = open('eliptical_verification_data.csv','w',encoding="utf-8")
        #dronewriter = csv.writer(csvfile, quotechar = '\'', delimiter=',',quoting = csv.QUOTE_NONE,escapechar = ' ')
        # write the header
        #dronewriter.writerow(["Timestamp,el_X,el_Y,el_Z,el_Vel,mocap_X,mocap_Y,mocap_Z,mocap_Yaw"])
        # write the data
        for i in range(len(timeStamps)):
            errsX.append(mocapPos[i][0] - elVars[i][0])
            errsY.append(mocapPos[i][1] - elVars[i][1])
            errsZ.append(mocapPos[i][2] - elVars[i][2])

            
            #dronewriter.writerow([timeStamps[i], elVars[i][0], elVars[i][1], elVars[i][2], elVars[i][3], mocapPos[i][0], mocapPos[i][1], mocapPos[i][2], mocapPos[i][3]])


        print(f"Average X Err: {sum(errsX) / len(errsX)}")
        print(f"Average Y Err: {sum(errsY) / len(errsY)}")
        print(f"Average Z Err: {sum(errsZ) / len(errsZ)}")

        exit(1)

    #wait after creating drones to make sure all ros components are ready
    time.sleep(2)
    

    

    myIn = input("'t' to takeoff: ")

    if (myIn != 't'):
        print(f"invalid input '{myIn}'")
        print('exiting...')
        exit(1)


    myDrone.takeoff()
    print('takeoff')

    time.sleep(1)
    
    myIn = input("'s' to start procedure after 1 seconds: ")


    time.sleep(1)


    if (myIn != 's'):
        myDrone.stop_movement()
        myDrone.land()
        print(f"invalid input '{myIn}'")
        print('landing and exiting...')
        exit(1)
    

    print('procedure start')



    radius = 0.75
    yOffset = 0
    sPerRot = 10
    myDrone.set_controller_points([radius, yOffset, 1, 0], reset=True)
    Drone.procedureRun = True
    
    


    input('hit enter to start rotation')
    print('henter hjit')
    lastTime = time.time()
    rads = 0
    
    
    thread = Thread(target=emergency_stop)
    thread.start


    while (rads < math.pi * 4):
        currTime = time.time()

        diff = currTime - lastTime

        rads += diff * (math.pi*2)/sPerRot

        x, y = genCircle(rads, radius, yOffset)
        #print(x, y)

        myDrone.set_controller_points([x, y, 1, 0], reset=False)

        lastTime = currTime
        
    time.sleep(2)


    Drone.procedureRun = False

    #input("hit enter to stop")


    print('procedure finished')
    myDrone.stop_movement()
    time.sleep(0.05)
    myDrone.land()
    #time.sleep(2)

    print('landing')

    #myDrone.stop()
    #myDrone.land()

    print(len(mocapPos))
    print(len(elVars))
    print(len(timeStamps))

    # write to csv file then break
    csvfile = open('elliptical_test_data.csv','w',encoding="utf-8")
    dronewriter = csv.writer(csvfile, quotechar = '\'', delimiter=',',quoting = csv.QUOTE_NONE,escapechar = ' ')
    # write the header
    dronewriter.writerow(["Timestamp","el_X","el_Y","el_Z","el_Vel","mocap_X","mocap_Y","mocap_Z","mocap_Yaw"])
    # write the data
    for i in range(min(len(timeStamps), len(mocapPos), len(elVars))):
        dronewriter.writerow([timeStamps[i],elVars[i][0],elVars[i][1],elVars[i][2],elVars[i][3],mocapPos[i][0],mocapPos[i][1],mocapPos[i][2],mocapPos[i][3]])
   
    csvfile.close()


    csvfile2 = open('time_switch.csv','w',encoding="utf-8")
    
    dronewriter2 = csv.writer(csvfile2, quotechar = '\'', delimiter=',',quoting = csv.QUOTE_NONE,escapechar = ' ')

    dronewriter2.writerow(["Timestamp"])
    
    for i in range(len(switchTimes)):
        dronewriter2.writerow([switchTimes[i]])
   
    csvfile2.close()


if __name__ == '__main__':
    print("Starting main()")
    main()
    print("Done!!")

