
#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import logging
import time

# Initialize logging to a file
logging.basicConfig(filename='/home/harvilab/logs/SidewayTest/crazyflie_log.txt', level=logging.INFO,
                    format='%(asctime)s:%(levelname)s:%(message)s')

Z = 0.80
sleepRate = 30

# Callback function for logging state estimate and control target
def log_callback(timestamp, data, logconf):
    # Log the data from the subscribed log variables
    print("Data: Timestamp: {timestamp}, Data: {data}")

# Main control function
def goCircle(timeHelper, cf, totalTime, radius, kPosition,cf2):
    startTime = timeHelper.time()
    pos = cf.position()
    startPos = cf.initialPosition + np.array([0, 0, Z])
    center_circle = startPos - np.array([radius, 0, 0])
    while True:
        time = timeHelper.time() - startTime
        omega = 2 * np.pi / totalTime
        vx = -radius * omega * np.sin(omega * time)  
        vy = radius * omega * np.cos(omega * time)
        desiredPos = center_circle + radius * np.array(
            [np.cos(omega * time), np.sin(omega * time), 0])
        errorX = desiredPos - cf.position() 
        cf.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * errorX), yawRate=0)
        cf2.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * errorX), yawRate=0)
        timeHelper.sleepForRate(sleepRate)
        
        
def Move(timeHelper, cf, totalTime, radius, kPosition,cf2):
    startTime = timeHelper.time()
    pos = cf.position()
    startPos = cf.initialPosition + np.array([0, 0, Z])
    center_circle = startPos - np.array([radius, 0, 0])
    while True:
        time = timeHelper.time() - startTime
        omega = 2 * np.pi / totalTime
        vx = -radius * omega * np.sin(omega * time)  
        vy = radius * omega * np.cos(omega * time)
        desiredPos = center_circle + radius * np.array(
            [np.cos(omega * time), np.sin(omega * time), 0])
        errorX = desiredPos - cf.position() 
        cf.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * errorX), yawRate=0)
        cf2.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * errorX), yawRate=0)
        timeHelper.sleepForRate(sleepRate)
    # Stop the log configuration
    #logconf.stop()

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(2 + Z)
    goCircle(timeHelper, allcfs.crazyflies[0], totalTime=30, radius=0.6, kPosition=1, cf2 = allcfs.crazyflies[1])
    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)