
# EllipticalTrackingwithControls

This module contains python scripts as well as a ROS workspace to implement a monocular vision based global localization system. 

The system broadcasts localization estimations over a  ROS topic via custom messages that other algorithms can utilize.

The requirements for implementation are as follows:

- A Linux system installed with a version of ROS 1
- An RGB web-camera with known resolution and FOV
- distinctly colored ring object with known diameter. (The center of this ring is what will be tracked by the system)
- Installation of Python as well as open CV

## ROS Workspace Setup
The 'ellipse_detect' directory in this repository is a ROS workspace containing a custom message used by the python scripts provided. 

To build the ROS workspace on the Linux computer execute the following steps:

1. Run the following command in the terminal. This will create a workspace framework in the terminal's current working directory.
	```
	sudo mkdir -p WS_Root/src/
	```
2. Place the 'ellipse directory' in the src folder of the ROS workspace.
	```
	mv -r /location/of/ellipse_detect WS_Root/src
	```
4. Execute the following commands to build the workspace root (in 'WS_Root' for example)
	```
	cd WS_Root
	catkin_make
	```
5. The workspace is now built, to use it and the custom message provided, source the workspace. From the workspace root:
	```
	source devel/setup.bash
	```
6. Now you are able to execute the python scripts in 'ellipse_detect/src'. These scripts start the localization system. Refer to the next sections 'Parameter Setup' and 'How to Run'

## Parameter Setup
To configure the localization system to your unique setup, parameters will need to be changed in the 'Ellipse_Detect_ROS.py' script in the 'ellipse_detect/src' directory of this repository.

### 1. Setting Camera parameters
Set the following values on the respective lines in the Ellipse_Detect_ROS.py script:
	
**A.  Camera resolution**  
**Line 34:** Change Constant 'FRAME_WIDTH ' to webcam's width resolution  
**Line 35:** Change Constant 'FRAME_HEIGHT ' to webcam's height resolution  
	
**B.  Camera Field of View**  
**Line 160:** Change Constant 'VFOV ' to webcam's Vertical Field of view in degrees (current value: 45 deg)  
**Line 161:** Change Constant 'HFOV ' to webcam's Horizontal Field of view in degrees (current value: 73 deg)  

**C.  Camera Source Index**  
**Line 293:** Change the integer in the VideoCapture(#) function to the source index of your webcam (currently '0').  
-To find the current available webcams and indies execute the following:  
	```
	ls -al /dev/video*
	```

**D.  Camera Location**  
This parameter will change the origin of the estimated localization of the system. These values are offset from the center of the camera lens  
**Lines 138 - 140:** Change the constant offset values (currently 1.56, -0.29, and 3.97) to fit where you want the origin of your global localization system. These will be in the same units used for your ring diameter in the next section. If they are all set to 0.0, then the center of the camera lens will be the origin.   
**NOTE:** The camera lens is to be parallel to your desired global localization frame. If the camera lens is tilted, the global frame will be by the same amount of degrees.  


### 2. Setting Circular Ring Target Parameters 
Set the following values on the respective lines in the Ellipse_Detect_ROS.py script:
	
**A.  Ring Diameter**    
**Line 166:** Change variable 'ellipse_diameter' to the rings diameter. The units of this measurement will decide the positional units of the localization system  

**A.  Ring Color**   
**Line 318:** Change variable 'lower_pink' to the lower HSV threshold values for the rings color.  
**Line 319:** Change variable 'upper_pink' to the upper HSV threshold values for the rings color.  
**NOTE:** These can be found using the script '___' in the 'Examples' directory. Run the script using an image of the tracked ring and move the sliders until everything but the ring is being filtered.


## How to Run
To run the system after building the workspace and setting the correct parameters, all that needs to be done is to source the workspace and run 'Ellipse_Detect_ROS.py' script.

From the workspace root, run the following commands:
	```
	source devel/setup.bash
	roscore
	python3 /src/ellipse_detect/src/Ellipse_Detect_ROS.py
	```

This will begin the localization system.

If everything works, a video screen of the webcam will display and draw detected ellipses over their found locations. In addition, the estimated position will display in the corner.

Make sure that ellipses are only being detected on your ring object and that the estimated position is what you assume it to be.

All of the localization estimations are broadcasted on the ROS topic '\Coordinates' using the custom message defined in the 'ellipse_detect/msg' directory.

Example scripts are provide in the 'Examples' directory that are used for real-time experimentation with DJI Tello drones. These can be used for experimentation or you can develop your own interface using the localization.
