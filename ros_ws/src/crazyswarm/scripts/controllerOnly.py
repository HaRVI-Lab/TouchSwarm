#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import socket
import threading
import numpy as np
from pycrazyswarm import Crazyswarm
import rospy
from crazyswarm.msg import GenericLogData
from geometry_msgs.msg import TransformStamped
import datetime
import os
import tf
import math
import time
Z = 0.50
totalTime = 60.0
DRONE_DISTANCE = 0.15  # 15 cm
TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
#Drone Config
z = 0.75
goal_position = np.array([0, 0, z])
radius = 0.15
now = datetime.datetime.now()
goal = np.array([0,0,z])
takeoff=False
land = False
takeoff_height = 0.8
hand_Pos = [0,0,0]
hand_Ori = [0,0,0]
numClicked = 0
cmd_ori = [0,0,1.0]

#Logging
statefile_name = now.strftime("%Y_%m_%d-%H_%M_%S_state.log")
posfile_name = now.strftime("%Y_%m_%d-%H_%M_%S_pos.log")
file_path = os.path.join("/home/harvilab/logs/assemble", statefile_name)  # Set the directory path where you want to save the file
file_path2 = os.path.join("/home/harvilab/logs/assemble", posfile_name)  # Set the directory path where you want to save the file


def is_within_radius(point1, point2, radius):
    x1, y1, z1 = point1[0], point1[1], point1[2]
    x2, y2, z2 = point2[0], point2[1], point2[2]
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
    #print("D = ", distance)
    return distance <= radius


def update_position(cf,timeHelper):
    global numClicked, goal_position, hand_Pos, hand_Ori,takeoff, land
    lastPos = [0,0,0]
    lastClick = 0
    held = False
    while True:
        #Boundary check
        
        # print(hand_Pos)
        
        #Takeoff Detection by z first arriving over 0.8
        if not takeoff and hand_Pos[2] > takeoff_height and is_within_radius(hand_Pos, cf.position(), 0.9):
            cf.takeoff(targetHeight=z, duration=TAKEOFF_DURATION)
            rospy.sleep(TAKEOFF_DURATION)
            takeoff = True
            print("taking off")
        
        if takeoff and not land:
        #Main operation Loop, based on hands pitch angle
            cf.cmdPosition(lastPos)
            if(is_within_radius(hand_Pos, cf.position(), 0.1)) :
                if timeHelper.time() - lastClick > 2:
                    print("turning off motor")
                    cf.setParam("powerDist/thrustOff",1)
                if not held:
                    print("starting to held")
                    lastClick = timeHelper.time()
                    held = True
            else:
                print("too far")
                held = False
            # if abs(hand_Ori[1])<0.5:
            #     lastPos = hand_Pos
            #     #print("following hands")
            #     if lastPos[2] > 1.7:
            #         print(lastPos)
            #         land = True
            #         print("Too high, landing")
            #         cf.land(targetHeight=0.04, duration=3.5)
            #         rospy.sleep(3.5)
            #     if(lastPos[2] < 0.5):
            #         print("Too low, adding z")
            #         lastPos[2] = z
                    
            #     #following the hand
            #     lastPos=lastPos+[0,0,-0.2]
            #     cf.cmdPosition(lastPos)
            #     #print("CMDgoal",lastPos)
            # if abs(hand_Ori[1]) > 0.7 and timeHelper.time() - lastClick > 1.0 :
            #     #print("clicking, maintaining pos")
            #     if (cf.position()[2] - lastPos[2]) > 0.1:
            #         numClicked += 1
            #         lastClick = timeHelper.time()
            #         print("clicked down @",lastPos)
            #     cf.cmdPosition(lastPos)
        
        rospy.sleep(0.033)
        
def calculate_target_position(msg):
    """
    Calculate the target position of the drone based on hand's position and orientation.
    The drone should be 15 cm away from the hand in the direction the hand is pointing.
    """
    # Convert Quaternion to Euler angles
    global hand_Pos, hand_Ori
    hand_Ori = tf.transformations.euler_from_quaternion([
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z,
        msg.transform.rotation.w
    ])
    
    hand_Pos = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
    
if __name__ == "__main__":
    #init
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf = swarm.allcfs.crazyflies[0]
    rospy.Subscriber('/vicon/hand/hand', TransformStamped, calculate_target_position)
    # update_position(cf, timeHelper)
    print("param:"+str(cf.getParam("powerDist/thrustOff")))
    
    # position_thread = threading.Thread(target=update_position, args=(cf,timeHelper))
    # position_thread.daemon = True
    # position_thread.start()
    
    

    #timeHelper.sleep(35)

    
    
    #cf.setParam("powerDist/thrustOff",1)


    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)
