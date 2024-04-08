import sys
import math

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

from simple_pid import PID
import numpy as np

class Controller:
    'class to define different controllers'

    def  __init__(self, type:str, numPosAxis:int, numRotAxis:int, contParams:list, setPoints = None):
        
        self.conts = []
        self.numPosAxis = numPosAxis
        self.numRotAxis = numRotAxis
        
        if type.upper() == 'PID':
            self.type = 'PID'

            if numPosAxis < 1:
                print("Parameter 'numAxis' for class 'Controller' must be either 1, 2, or 3", file=sys.stderr)
                sys.exit(1)

            if numRotAxis < 0:
                print("Parameter 'numAxis' for class 'Controller' must be 0 or greater", file=sys.stderr)
                sys.exit(1)

            if len(contParams) != numPosAxis + numRotAxis:
                print("Parameter 'contParams' for Controller type 'PID' requires parameters for each axis", file=sys.stderr)
                sys.exit(1)

            if len(contParams[0]) != 3:
                print("Parameter 'contParams' for Controller type 'PID' requires 3 values for each axis (Proportunal, Integral, Derivative)", file=sys.stderr)
                sys.exit(1)

            for params in contParams:
                self.conts.append(PID(params[0], params[1], params[2]))
            
            if setPoints != None:
                self.set_points(setPoints)


    #class function to transition any amount of degree input to the
    # degree or RAD range of -180 to 180
    def rotation_goal_standardize(self, goal, out = 'RAD'):
        #remove extrenuous rotations in goal
        goal %= 360
        
        #translate to range -180 < h < 180
        if (goal < -180):
            goal += 360
        elif (goal > 180):
            goal -= 360
            
        #return radians or degrees
        if (out == 'RAD'):
            return goal * math.pi / 180
        else:
            #return in Degrees
            return goal


    def set_points(self, setPoints:list, reset = True):
        if self.type == 'PID':
            if len(setPoints) != len(self.conts):
                print("Number of points in parameter 'setPoints' do not match number of axis", file=sys.stderr)
                sys.exit(1)

            for i in range(self.numPosAxis, self.numPosAxis + self.numRotAxis):
                setPoints[i] = self.rotation_goal_standardize(setPoints[i])
            

            for i in range(len(setPoints)):
                if reset:
                    self.conts[i].reset()
                self.conts[i].setpoint = setPoints[i]

    def get_optimal_rot_pos(self, rot_curr, goal):
        #these should all be in radians
        diff = goal - rot_curr
        
        #if difference is less than PI or 180 deg, then the optimal path is the current values
        if (abs(diff) <= math.pi):
            return rot_curr
        #else, the other path is more optimal and we need to feed the PID a value that is shifted by 360 degrees accordingly
        # (ie the degree > |180| that is equivalent and 'closer')
        elif (diff <= 0):
            return rot_curr - (2*math.pi)
        else:
            return rot_curr + (2*math.pi)

    '''
       Gives outs of the controller for each axis given current position
    '''
    def get_out(self, pos:list):
        if self.type == 'PID':
            if len(pos) != len(self.conts):
                    print("Number of points in parameter vector 'pos' do not match number of axis", file=sys.stderr)
                    sys.exit(1)
            
            outs = [0] * len(pos)
            for i in range(self.numPosAxis):
                outs[i] = self.conts[i](pos[i])
            
            for i in range(self.numPosAxis, self.numPosAxis + self.numRotAxis):
                outs[i] = self.conts[i](self.get_optimal_rot_pos(pos[i], self.conts[i].setpoint))

            return outs
    

    def get_points(self):
        return [cont.setpoint for cont in self.conts]
                
        
        




class Drone:
    'basic class that defines a drones controller and callback function for ROS'

    procedureRun = False


    def  __init__(self, name:str, type:str, controller:Controller, callback, bounds = None):
        self.name = name
        self.type = type
        self.controller = controller
        self.callbackFunc = callback
        self.rosTwist = Twist()
        self.bounds = bounds


        if bounds != None and len(bounds) != self.controller.numPosAxis:
            print(f"Parameter 'bounds' must contain a list of tuples where each tuple is the bounds of the axis!", file=sys.stderr)
            sys.exit(1)

        #subscriber for the optitrack system
        if self.callback != None:
            rospy.Subscriber(f'/mocap_node/{self.name}/Odom', Odometry, self.callback)

        #create publishers for drone type
        if (type == 'bebop1' or 'bebop2'):
            #vel publisher
            self.velPub = rospy.Publisher(f'/{self.name}/cmd_vel', Twist, queue_size=1)
            #landing publisher
            self.landPub = rospy.Publisher(f'/{self.name}/land', Empty, queue_size=5)
            #takeoff Publisher
            self.takeoffPub = rospy.Publisher(f'/{self.name}/takeoff', Empty, queue_size=5)
        elif (type == 'wand'):
            pass
        else:
            print(f"Unknown type '{type}' for class Drone", file=sys.stderr)
            sys.exit(1)


    #set the points of the controller
    def set_controller_points(self, setPoints:list, reset = True):

        if self.bounds != None:
            for i in range(self.controller.numPosAxis):
                setPoints[i] = max(self.bounds[i][0], min(self.bounds[i][1], setPoints[i]))

        self.controller.set_points(setPoints, reset = reset)






    #function to take positional velocities and rotate by a certain amount
    def rotate_vels(self, v, rotation):
        rot_mat = [
            [math.cos(rotation), -math.sin(rotation), 0], 
            [math.sin(rotation), math.cos(rotation), 0],
            [0, 0, 1]
            ]
        
        return np.dot(rot_mat, v)


    #function to update drone velocities in accordance with a given position and its controller
    # account for necessary rotations if desired
    def update_vel(self, pos:list, rotPosVels:bool = True, euler:bool = True, rotAxis:str = 'yaw', verbose = False, raw_vels = None):
        #vels order is [x, y, z, yaw, pitch, roll]
        if raw_vels != None:
            if (len(raw_vels) != (self.controller.numPosAxis + self.controller.numRotAxis)):
                print("'values' parameter formatted incorrectly", file=sys.stderr)
                sys.exit(1)

            vels = raw_vels

        else:
            vels = self.controller.get_out(pos)

        if rotPosVels:
            if (self.controller.numRotAxis == 0):
                print("Cannot rotate velocities if there is no rotational axis defined in Controller", file=sys.stderr)
                sys.exit(1)

            if euler:
                if rotAxis == 'yaw':

                    if self.controller.numPosAxis == 3:
                        v_rot = self.rotate_vels([[vels[0]], [vels[1]], [vels[2]]], -pos[3])
                    elif self.controller.numPosAxis == 2:
                        v_rot = self.rotate_vels([[vels[0]], [vels[1]], [0]], -pos[2])
                    else:
                        print("Can only rotate velocity vecotrs of 3 and 2", file=sys.stderr)
                        sys.exit(1)
                else:
                    print(f"No support for rotations along axis {rotAxis} values", file=sys.stderr)
                    sys.exit(1)

            else:
                print("Currently do not support non-euler values", file=sys.stderr)
                sys.exit(1)

            
            self.rosTwist.linear.x = v_rot[0]
            self.rosTwist.linear.y = v_rot[1]
            self.rosTwist.linear.z = v_rot[2]

            


        else:
            if self.controller.numPosAxis == 3:
                self.rosTwist.linear.x = vels[0]
                self.rosTwist.linear.y = vels[1]
                self.rosTwist.linear.z = vels[2]
            elif self.controller.numPosAxis == 2:
                self.rosTwist.linear.x = vels[0]
                self.rosTwist.linear.y = vels[1]
            else:
                self.rosTwist.linear.x = vels[0]



        if self.controller.numRotAxis > 0:
            self.rosTwist.angular.z = vels[self.controller.numPosAxis]


        self.velPub.publish(self.rosTwist)


        if (verbose):
            print("---------------------")
            print(f"x vel:::::::{self.rosTwist.linear.x}")
            print(f"y vel:::::::{self.rosTwist.linear.y}")
            print(f"z vel:::::::{self.rosTwist.linear.z}")
            
            if (self.controller.numRotAxis > 0):
                print(f"yaw vel:::::{self.rosTwist.angular.z}")
            
        
    def stop_movement(self):
        stopTwist = Twist()

        stopTwist.linear.x = 0
        stopTwist.linear.y = 0
        stopTwist.linear.z = 0
        stopTwist.angular.z = 0

        self.velPub.publish(self.rosTwist)


    #function to call given callback function which requires two parameters:
    # msg: which is the optitrack position msg
    # drone: which is the drone object that the msg was called on
    def callback(self, msg):
        self.callbackFunc(msg, drone=self, procedureRun= Drone.procedureRun)


    #function to publish to the land topic for the drone
    def land(self):
        self.landPub.publish(Empty())


    #function to publish to the takeoff topic for the drone
    def takeoff(self):
        self.takeoffPub.publish(Empty())


    #function to return if the drone is at the desired position.
    def at_setpoint(self, pos, admittedErrs):
        setPoints = self.controller.get_points()
        
        if len(pos) != len(setPoints):
            print("Number of points in parameter vector 'pos' do not match number of axis", file=sys.stderr)
            sys.exit(1)

        for i, point in enumerate(setPoints):
            if abs(point - pos[i]) > admittedErrs[i]:
                return False
        
        return True




    
