"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
import numpy as np

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
totalTime = 10.0
import rospy
from crazyswarm.msg import GenericLogData
import datetime
import os
z = 0.75
radius = 0.15
now = datetime.datetime.now()
    
goal = np.array([0,0,z])
statefile_name = now.strftime("%Y_%m_%d-%H_%M_%S_state.log")
posfile_name = now.strftime("%Y_%m_%d-%H_%M_%S_pos.log")
file_path = os.path.join("/home/harvilab/logs/assemble", statefile_name)  # Set the directory path where you want to save the file
file_path2 = os.path.join("/home/harvilab/logs/assemble", posfile_name)  # Set the directory path where you want to save the file

# Function to handle incoming log data
def log_callback(data):
    # Open the file in append mode
    with open(file_path, 'a') as file:
        # Write data to file; you might need to adjust this depending on the actual structure of GenericLogData
        file.write("***************"+str(data) +'\n')
        #print(str(data) +'\n')
    

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cfr = swarm.allcfs.crazyflies[0]
    cfl = swarm.allcfs.crazyflies[1]
    # Subscribe to the /cf1/log1 topic
    rospy.Subscriber("/cf1/log1", GenericLogData, log_callback)

    
    cfr.takeoff(targetHeight=z, duration=TAKEOFF_DURATION)
    cfl.takeoff(targetHeight=z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)
    print("take off")
    
    
    #go to a close distance
    posl = np.array([0,-0.5, z])
    posr = np.array([0,-0.7, z])
    cfr.goTo(posr, 0,9.0)
    cfl.goTo(posl, 0,9.0)
    print("goto 0 05 075")
    timeHelper.sleep(15.0)
    
    #assemble
    posl = np.array([0,-0.55, z])
    posr = np.array([0,-0.65, z])
    
    cfr.goTo(posr, 0,5.0)
    cfl.goTo(posl, 0,5.0)
    print("goto 0 0 075")
    
    timeHelper.sleep(5.0)

    # pos = np.array([0, 0, z])
    # cf.cmdPosition(pos, 0)

    #timeHelper.sleep(totalTime)
    #print("go to 000")  
    cfl.land(targetHeight=0.04, duration=3.5)
    cfr.land(targetHeight=0.04, duration=3.5)
    timeHelper.sleep(TAKEOFF_DURATION)
    print("land")  

if __name__ == "__main__":
    main()
