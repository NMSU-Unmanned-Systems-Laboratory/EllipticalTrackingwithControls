#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
#Created on Tue Oct  5 13:38:42 2021#
#
#@author: hunterstuckey
#
#
#

import csv
import math
import time
from cmath import pi
import cv2
import imutils
import numpy as np
import rospy


from std_msgs.msg import Float64
from ellipse_detect.msg import Location
from filterpy.kalman import KalmanFilter
from KalmanFilter3d import KalmanBox3dTracker

WIDTH  = 1280
HEIGHT = 720


##############################################################################
#
#    Ellipse Detection Function
#
##############################################################################

def detect_ellipse(image, contours, edges):
    # size of image
    Width_dimension  = WIDTH
    Height_dimension = HEIGHT
    # set the parameters for ellipse detection
    min_Contour_Size      = 40  # if this is too large, i can't detect it far away, too small noise gets through
    min_Axis_Size         = 5  # if this is too large, i can't detect it far away. if it's too small noisy ellipses get through
    max_Axis_Size         = 1000
    max_Axis_Ratio        = 50
    min_Axis_Ratio        = .05
    small_contour_overlap = .35 # previous .25
    small_ellipse_overlap = .10 # previous .0
    min_angle             = 60
    max_angle             = 120

    # create empty list for ellipses
    ellipses_detected = []
    ellipses_contours = []
    
    # iterate through each contour
    for (i, c) in enumerate(contours):
        
        
        # get the contour pixel count
        c_pixels = np.count_nonzero(c)
        
        # skip this contour if it is too small
        if c_pixels < min_Contour_Size:
            continue

        # reject contours that are near the edge of the image
        x,y,w,h = cv2.boundingRect(c)
        if x <= 2 or y <= 2 or w >= Width_dimension - 2 or h >= Height_dimension - 2:
            continue

        # fill the ellipse
        ellipse = cv2.fitEllipse(c)
        (xc,yc),(smallAxis,largeAxis),angle = ellipse
        
        # reject unreasonable dimensions
        if largeAxis > max_Axis_Size:
            continue
        if smallAxis < min_Axis_Size:
            continue
        
        # reject unreasonable ratio
        axisRatio =  largeAxis / smallAxis
        if axisRatio > max_Axis_Ratio:
            continue
        if axisRatio < min_Axis_Ratio:
            continue

        #print(angle)

        # reject if the angle is too large/vertical, the drone shouldn't be stable in those states
        # it looks like it calculates angle's origin from the vertical axis, not horizontal
        if angle < min_angle or angle > max_angle:
            continue

        # make masks for contour/ellipse
        mask_contour   = cv2.drawContours(np.zeros(shape=edges.shape,dtype=np.uint8), [contours[i]], -1, 255, 1)
        mask_ellipse   = cv2.ellipse(np.zeros(image.shape,dtype=np.uint8),ellipse,(0,255,0), 3)
        mask_ellipse   = cv2.cvtColor(mask_ellipse, cv2.COLOR_BGR2GRAY)
        ellipse_pixels = np.count_nonzero(mask_ellipse)
        
        # find the contour overlap with the filled ellipse
        contour_overlap = np.count_nonzero(np.logical_and(mask_contour,mask_ellipse))/c_pixels
    
        # if the overlap is too small reject
        if contour_overlap < small_contour_overlap:
            continue
        
        # find the ellipse overlap with the edge map
        ellipse_overlap = np.count_nonzero(np.logical_and(mask_ellipse,edges))/ellipse_pixels
        
        # if the overlap is too small reject
        if ellipse_overlap < small_ellipse_overlap:
            continue 
        
        # add ellipse that passed all conditions
        ellipses_detected.append(ellipse)
        # add associated contour
        ellipses_contours.append(c)

        
        
    return ellipses_detected,ellipses_contours



##############################################################################
#
#   Spherical To Cartesian Function
#
##############################################################################

def sph2cart(phi,theta,rho):
    # flipped y and z
    # add the shifts as well
    #z = rho * np.sin( theta ) + 1.35
    #x = -(rho * np.sin( phi   ))  - .28
    #y = -rho * np.cos( theta ) + 3.35

    z = rho * np.sin( theta ) + .30
    x = -(rho * np.sin( phi   )) + .05
    y = -rho * np.cos( theta ) + 2.7

    return np.array([x,y,z])


    
##############################################################################
#
#   Optical Localization Function
#
##############################################################################


def find_location(ellipse_contour):
    # find the distance of the detected ellipse in this function

    # parameters for the camera, 1920x1080p which is regular hd
    Width_dimension  = 1280
    Height_dimension = 720
    VFOV             = (56.00 * np.pi) / 180
    HFOV             = (85.80 * np.pi) / 180
    VFOV_constant    = np.tan(VFOV/2)/Height_dimension
    HFOV_constant    = np.tan(HFOV/2)/Width_dimension
    imcenter_y       = Height_dimension/2
    imcenter_x       = Width_dimension/2
    ellipse_diameter = .2742 # in meters

    # find xl,xr,xt,xb and calculate the center
    xl     = (ellipse_contour[ellipse_contour[:, :, 0].argmin()][0])
    xr     = (ellipse_contour[ellipse_contour[:, :, 0].argmax()][0])
    center = (xl+xr) / 2
    xt     = (ellipse_contour[ellipse_contour[:, :, 1].argmin()][0])
    xb     = (ellipse_contour[ellipse_contour[:, :, 1].argmax()][0])

    # center each of the points
    Xo = (center[0] - imcenter_x)
    Yo = -1.0 * (center[1] - imcenter_y)

    # calculations for polar coordinates
    theta = math.degrees(np.arctan(math.radians(Yo*2*VFOV_constant)))
    phi   = math.degrees(np.arctan(math.radians(Xo*2*HFOV_constant)))
    rho   = (ellipse_diameter/2) * (abs(Xo/(center[0]-xr[0])))*np.sqrt((1+(1/pow(np.tan(phi),2))))

    # calculations for roll and pitch
    roll  = np.arctan((xr[1]-xl[1])/(xr[0]-xl[0]))

    if abs(roll) <= .05:
        pitch = np.arctan(((xt[1]-[xb[1]])/(xr[0]-xl[0])) - (2*Yo*(np.tan(phi/2)/Height_dimension)) )
    else:
        smallAngleApprox =  np.cos(roll)*(xt[1]-[xb[1]]) 
        pitch = np.arctan((smallAngleApprox/(xr[0]-xl[0])) - (2*Yo*(np.tan(phi/2)/Height_dimension)) )

    # convert to cartesian coordinates
    xyz   = sph2cart(phi,theta,rho)

    return xyz,center,roll,pitch



##############################################################################
#
#  Line Detection, nose-finding Function
#
##############################################################################

def find_nose(small_img):

    #go-pro lab
    lower_green = (80, 55, 97)
    upper_green = (103,255,255)

    # make sure there is actually an image created by the bounding box
    if type(small_img) is np.ndarray:
        if small_img.size > 0:
             # color segmentation
            hsv_img    = cv2.cvtColor(small_img, cv2.COLOR_BGR2HSV)
            # bitwise'and' over the original image with mask
            mask            = cv2.inRange(hsv_img, lower_green, upper_green)
            # edge detection
            img_edges       = cv2.Canny(mask, 25, 200)

            # morphological closing
            se = np.ones((11, 11), dtype = 'uint8') 
            img_edges = cv2.morphologyEx(img_edges, cv2.MORPH_CLOSE, se)
            # eroding into the shape of one line.
            se = np.ones((3, 3), dtype = 'uint8') 
            img_edges = cv2.erode(img_edges, se)

            # for testing purposes
            #cv2.imshow('small_segment',img_edges)

            lines = cv2.HoughLinesP(
            img_edges, # Input edge image
            1, # Distance resolution in pixels
            np.pi/180, # Angle resolution in radians
            threshold=15, # Min number of votes for valid line
            minLineLength=3, # Min allowed length of line
            maxLineGap=10 # Max allowed gap between line for joining them
            )

            # Iterate over points in all the detected lines
            if lines is not None:
                for points in lines:
                    # get the pixel locations of the line
                    x1,y1,x2,y2 = points[0]

                    # return all relevant values
                    return x1,y1,x2,y2

            else:
                return

    


##############################################################################
#
#   ROS Talker Class
#
##############################################################################


class RosTalker(object):
    """
    The RosTalker object used to handle the ROS I/O.
    
    It subscribes to the camera topic, it will output a 
    """
    def __init__(self):
        """
        Create RosTalker object.
        """

        # initialize the node
        rospy.init_node('camera', anonymous=True)

        # pubs
        self._xyz_pub   = rospy.Publisher('Coordinates', Location, queue_size=1)

        # subs, shouldn't be any here, but will keep for later
        #self._camera_sub = rospy.Subscriber('camera', Camera, self._camera_cb, queue_size=1) 

    def publish_message(self, coordinates,angles):

        msg = Location()
        msg.x = coordinates[0]
        msg.y = coordinates[1]
        msg.z = coordinates[2]
        msg.v = coordinates[3] 
        msg.roll  = angles[0]
        msg.pitch = angles[1]
        msg.yaw   = angles[2]

        self._xyz_pub.publish(msg)

        return



##############################################################################
#
#   Main/Video Streaming
#
##############################################################################

# videoWriter for stream
#out = cv2.VideoWriter('testVideo2.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (1920,1080))

# start camera stream
cap = cv2.VideoCapture(2)

# settings for font
font      = cv2.FONT_HERSHEY_SIMPLEX
fontScale = .48
color     = (255, 0, 0)
thickness = 1

# lists for writing to csv at the end
time_stamps = []
location_data = []

#go-pro lab
lower_pink = (137, 90, 150)
upper_pink = (152,255,255)

#outdoors
#lower_pink = (160, 40, 100)
#upper_pink = (175,255,255)

# boolean for initializing kalman filter
FilterCreated = False

# initialize the ros node
rosMessager = RosTalker()

# initial angles, gets set as the nose is detected.
angles = None

while True:
    
    #take one frame of the video
    _, frame = cap.read()

    frame = cv2.resize(frame, (1280,720))

    # color segmentation
    hsv_frame    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
    # bitwiseand over the original image with mask
    mask            = cv2.inRange(hsv_frame, lower_pink, upper_pink)
    frame_segmented = cv2.bitwise_and(frame, frame, mask=mask)

    # undo this in future if it doesn't work well
    accumEdged      = cv2.Canny(mask, 25, 200)

    # add median filter for getting rid of small circles
    #accumEdged = cv2.medianBlur(accumEdged, 3)

    # close the contours up, additional cleaning added
    #se = np.ones((11, 11), dtype = 'uint8')
    # outdoors
    se = np.ones((21, 21), dtype = 'uint8') 
    accumEdged = cv2.morphologyEx(accumEdged, cv2.MORPH_CLOSE, se)
        
    # find contours in the accumulated image, keeping only the largest
    cnts = cv2.findContours(accumEdged.copy(), cv2.RETR_EXTERNAL,
	    cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    
    # run ellipse detection
    ellipses,contours = detect_ellipse(frame, cnts, accumEdged)
    
    # if there were ellipses detected
    if len(ellipses) > 0:
        # draw all the ellipses detected
        for i in range(len(ellipses)):
            
            # find location and draw location data next to it
            location,center,roll,pitch = find_location(contours[i])

            # create a small image from the bounding box of the ellipse
            x,y,w,h   = cv2.boundingRect(contours[i])
            small_img = frame[y:y+h,x:x+w]

            # find the yaw, pitch and roll coordinates
            line_points = find_nose(small_img)

            # draw the ellipse onto the image
            frame = cv2.ellipse(frame, ellipses[i], (255, 0, 0), 10)

            # draw the line onto the bigger image
            if line_points is not None:
                cv2.line(frame,(x+line_points[0],y+line_points[1]),(x+line_points[2],y+line_points[3]),(255,0,200),20)
                # do the math for yaw on the image, then the corrections for each pitch and roll
                # select the set of line points furthest from the center of the ring
                if abs(x+line_points[0] - center[0]) + abs(y+line_points[1] - center[1]) > abs(x+line_points[2] - center[0]) + abs(y+line_points[3] - center[1]):
                    yaw = -(np.arctan2((y+line_points[1] - center[1]), (x+line_points[0] - center[0]))) + math.pi
                else:
                    yaw = -(np.arctan2((y+line_points[3] - center[1]), (x+line_points[2] - center[0]))) + math.pi

                # adjust the roll and pitch with respect to the new yaw, convert to degrees as well
                corrected_pitch = math.degrees(np.cos(yaw)*roll + np.sin(yaw)*pitch)
                corrected_roll  = math.degrees(np.sin(yaw)*roll + np.cos(yaw)*pitch)
                yaw             = math.degrees(yaw)
                angles          = [corrected_roll,corrected_pitch,yaw]

            # add timestamp
            time_stamps.append(time.time())

            # skips the rest of the loop if there is a nan in the data
            if (math.isnan(location[0])):
                continue

            # if kalman filter doesn't exist yet, create new one with initial state
            if FilterCreated == False:
                FilterCreated = True
                XYZFilter     = KalmanBox3dTracker(location)
            else:
                XYZFilter.update(location)
                XYZFilter.predict()

            # get the filtered location
            filtered_location = XYZFilter.get_state()

            # add to the array to write to the csv
            location_data.append(filtered_location)

            # publish the filtered_location information
            if angles is not None:
                rosMessager.publish_message(filtered_location,angles)
            
            # will replace this with the measured mocap data
            # Write the filtered location data on the top left
            frame = cv2.putText(frame,
            'x:' + str(round(float(filtered_location[0]),2)) + '\ny:' + str(round(float(filtered_location[1]),2)) + 
            '\nz:' + str(round(float(filtered_location[2]),2)),
            org=(100,100), fontFace=font, fontScale=2, color=(0,20,255), thickness=5)

            # Write the angle data on the top left
            if line_points is not None:
                frame = cv2.putText(frame,
                'roll:' + str(round(corrected_roll,2)) + '\npitch:' + str(round(float(corrected_pitch),2)) + '\nyaw:' + str(round(float(yaw),2)),
                org=(100,200), fontFace=font, fontScale=2, color=(0,100,255), thickness=5)
    
    
    #frame = cv2.resize(frame,(1280,720))

    # show the new image, and write to file
    cv2.imshow("Detected Ellipses",frame)

    # show the accumulated edge map
    #cv2.imshow("Edge Map", accumEdged)

    # stop program if q key is pressed
    if cv2.waitKey(1) == ord('q'):
        # write to csv file then break
        csvfile = open('3D_DroneDataCtrl2.csv','w',encoding="utf-8")
        dronewriter = csv.writer(csvfile, quotechar = '\'', delimiter=',',quoting = csv.QUOTE_NONE,escapechar = ' ')
        # write the header
        dronewriter.writerow(["Timestamp,X,Y,Z"])
        # write the data
        for i in range(len(location_data)):
            dronewriter.writerow([str(time_stamps[i])+","+str(location_data[i][0])+","
            +str(location_data[i][1])+","+str(location_data[i][2])])

        cap.release()
        #out.release()
                
        #out.close()
        # close the file
        csvfile.flush()
        csvfile.close()
        # exit the loop
        break
    

