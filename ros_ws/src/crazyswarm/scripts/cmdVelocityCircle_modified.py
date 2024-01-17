
#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import logging
import time
#!/usr/bin/env python
import rospy
from crazyswarm.msg import GenericLogData
import datetime
import os

# Function to handle incoming log data
def log_callback(data):
    # Open the file in append mode
    with open(file_path, 'a') as file:
        # Write data to file; you might need to adjust this depending on the actual structure of GenericLogData
        file.write(str(data) + '\n')

# Initialize the ROS node
#rospy.init_node('log_listener')



# Initialize logging to a file

Z = 0.750
sleepRate = 30


# Main control function
def goCircle(timeHelper, cf, totalTime, radius, kPosition):
    # Log configuration for the Crazyflie

    startTime = timeHelper.time()
    while timeHelper.time() - startTime < totalTime:
        # Get the current time and position
        current_time = timeHelper.time() - startTime
        current_position = cf.position()
        
        
        # Compute the desired position
        desiredPos = np.array([0, 0, Z])

        # Compute the error vector
        error = desiredPos - current_position
        distance = np.linalg.norm(error)

        # Apply haptic feedback within 20 cm of the center
        if distance > 0.2:
            kPosition = 0  # Stop motion if outside 20 cm radius
        else:
            kPosition = 1  # Allow motion within 20 cm radius

        # Command the drone with velocity control
        cf.cmdVelocityWorld(np.array([0, 0, 0]) + kPosition * error, yawRate=0)

        # Sleep to maintain the loop rate
        timeHelper.sleepForRate(sleepRate)

    # Stop the log configuration
   

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    # Get the current time for the file name
    now = datetime.datetime.now()
    file_name = now.strftime("%Y_%m_%d-%H_%M_%S_state.log")
    file_path = os.path.join("/home/harvilab/logs/cmdVel", file_name)  # Set the directory path where you want to save the file

    # Subscribe to the /cf1/log1 topic
    rospy.Subscriber("/cf1/log1", GenericLogData, log_callback)

    # Spin to keep the script for exiting
    #rospy.spin()
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(2 + Z)
    goCircle(timeHelper, allcfs.crazyflies[0], totalTime=40, radius=1, kPosition=1)
    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)