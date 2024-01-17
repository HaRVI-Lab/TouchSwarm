
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
def goCircle(timeHelper, cf, totalTime, radius, kPosition):
    # Log configuration for the Crazyflie
    # logconf = LogConfig(name='Position', period_in_ms=100)
    # logconf.add_variable('stateEstimate.x', 'float')
    # logconf.add_variable('ctrltarget.x', 'float')

    # # Start logging
    # cf.log.add_config(logconf)
    # logconf.data_received_cb.add_callback(log_callback)
    # logconf.start()
    desiredPos = np.array([0, 0, Z])
    startTime = timeHelper.time()
    while timeHelper.time() - startTime < totalTime:
        # Get the current time and position
        current_time = timeHelper.time() - startTime
        current_position = cf.position()
        
        # Log the current position from cf.position()
        # logging.info(f"Current Position: {current_position}")

        # Compute the desired position
        
        
        # Compute the error vector
        error = desiredPos - current_position
        distance = np.linalg.norm(error)

        # Apply haptic feedback within 20 cm of the center
        if distance > 0.2:
            kPosition = 0  # Stop motion if outside 20 cm radius
        else:
            kPosition = 2.5  # Allow motion within 20 cm radius
        print("POS:" + str(cf.position()[0])+","+str(cf.position()[1])+","+str(cf.position()[2])+","+str(kPosition))
        # Command the drone with velocity control
        cf.cmdVelocityWorld(np.array([0, 0, 0]) + kPosition * error, yawRate=0)

        # Sleep to maintain the loop rate
        timeHelper.sleepForRate(sleepRate)

    # Stop the log configuration
    #logconf.stop()

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(2 + Z)
    goCircle(timeHelper, allcfs.crazyflies[0], totalTime=30, radius=1, kPosition=1)
    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)