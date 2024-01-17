"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
import numpy as np

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
totalTime = 25.0
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
file_path = os.path.join("/home/harvilab/logs/cmdVel", statefile_name)  # Set the directory path where you want to save the file
file_path2 = os.path.join("/home/harvilab/logs/cmdVel", posfile_name)  # Set the directory path where you want to save the file

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
    cf = swarm.allcfs.crazyflies[0]
    
    # Subscribe to the /cf1/log1 topic
    rospy.Subscriber("/cf1/log1", GenericLogData, log_callback)

    
    cf.takeoff(targetHeight=z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)
    startTime = timeHelper.time()
    print("starting")
    with open(file_path2, 'a') as file2:
    # Write data to file; you might need to adjust this depending on the actual structure of GenericLogData
        
        while timeHelper.time() - startTime < totalTime:
            desiredPos = np.array([0, 0, z])
            error = desiredPos - cf.position()
            distance = np.linalg.norm(error)
            goal = np.array([0,0,z])
            
            file2.write(str(timeHelper.time())+","+str(cf.position()[0])+ ","+str(cf.position()[1])+","+ str(cf.position()[2])+","+ str(distance)+'\n')
            print(str(cf.position()[0])+ ","+str(cf.position()[1])+","+ str(cf.position()[2])+","+ str(distance)+' ****************\n')
            # Apply haptic feedback within 20 cm of the center
            # if distance < radius:
                
            #     # print(str(goal[0])+ ","+str(goal[1])+","+ str(goal[2])+str(distance)+'\n')
            #     goal = np.array([0,0,z]) # Stop motion if outside 20 cm radius
            # else:
            #     #goal = np.array([-1,-1,z])
            #     goal = cf.position() # Sow motion 
            #print("***********************"+str(goal[0])+ ","+str(goal[1])+","+ str(goal[2])+str(distance)+'\n')

            #     #print("in staing other pos")
            #     goal[2] = z

            # file.write(str(goal[0])+ ","+str(goal[1])+","+ str(goal[2])+'\n')
            
            #cf.cmdPosition(goal, yaw =  0)


            timeHelper.sleepForRate(80)




    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == "__main__":
    main()
