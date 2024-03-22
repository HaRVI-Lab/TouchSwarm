"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
import numpy as np

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 15.0
totalTime = 10.0
import rospy
from crazyswarm.msg import GenericLogData
import datetime
import os
z = 0.5
radius = 0.5
now = datetime.datetime.now()
    
goal = np.array([0,0,z])
statefile_name = now.strftime("%Y_%m_%d-%H_%M_%S_state.log")
posfile_name = now.strftime("%Y_%m_%d-%H_%M_%S_pos.log")
file_path = os.path.join("/home/harvilab/logs/pid_test", statefile_name)  # Set the directory path where you want to save the file
file_path2 = os.path.join("/home/harvilab/logs/pid_test", posfile_name)  # Set the directory path where you want to save the file

# Function to handle incoming log data
def log_callback(data):
    # Open the file in append mode
    with open(file_path, 'a') as file:
        # Write data to file; you might need to adjust this depending on the actual structure of GenericLogData
        file.write(str(data) +'\n')
        #print(str(data) +'\n')
    

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf = swarm.allcfs.crazyflies[0]
    cf2 = swarm.allcfs.crazyflies[1]
    
    # Subscribe to the /cf1/log1 topic
    rospy.Subscriber("/cf1/log1", GenericLogData, log_callback)
    rospy.Subscriber("/cf2/log1", GenericLogData, log_callback)
    
    
    print(cf.getParam("pid_attitude/roll_kp"))
    print(cf.getParam("pid_attitude/roll_ki"))
    print(cf.getParam("pid_attitude/roll_kd"))
    print(cf.getParam("pid_attitude/pitch_kp"))
    print(cf.getParam("pid_attitude/pitch_ki"))
    print(cf.getParam("pid_attitude/pitch_kd"))
    print(cf2.getParam("pid_attitude/roll_kp"))
    print(cf2.getParam("pid_attitude/roll_ki"))
    print(cf2.getParam("pid_attitude/roll_kd"))
    print(cf2.getParam("pid_attitude/pitch_kp"))
    print(cf2.getParam("pid_attitude/pitch_ki"))
    print(cf2.getParam("pid_attitude/pitch_kd"))
    
    
    allcfs.takeoff(targetHeight=z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)
    
    #initialize position based on drone's x and y
    startTime = timeHelper.time()
    print("starting")
    goal = cf.position()
    goal2 = cf2.position()
    goal[2] = z
    goal2[2] = z
    lastPos = cf.position()
    lastUpdate = timeHelper.time()
    stabilized = False
    
    
    with open(file_path2, 'a') as file2:
    # Write data to file; you might need to adjust this depending on the actual structure of GenericLogData
        
        while timeHelper.time() - startTime < totalTime:
            
            error = goal - cf.position()
            distance = np.linalg.norm(error)
            if timeHelper.time() - lastUpdate > 0.50:
                lastUpdate = timeHelper.time()
                if np.linalg.norm(cf.position() - lastPos) < 0.1:
                    stabilized = True
                lastPos = cf.position()
            
            file2.write(str(timeHelper.time())+","+str(cf.position()[0])+ ","+str(cf.position()[1])+","+ str(cf.position()[2])+","+ str(distance)+'\n')
            print("cf1: "+str(timeHelper.time())+","+str(cf.position()[0])+ ","+str(cf.position()[1])+","+ str(cf.position()[2])+","+ str(distance)+'\n')
            print("cf2: "+str(timeHelper.time())+","+str(cf2.position()[0])+ ","+str(cf2.position()[1])+","+ str(cf2.position()[2])+","+ str(distance)+'\n')
            
            
            if distance > radius and stabilized:
                
                goal = cf.position() 
                goal2 = cf2.position()
                goal[2] = z
                goal2[2] = z 
                
          

            #file.write(str(goal[0])+ ","+str(goal[1])+","+ str(goal[2])+'\n')
            
            # allcfs.goTo(goal, yaw =  0)
            # cf.cmdPosition(goal, yaw =  0)
            # cf2.cmdPosition(goal2, yaw =  0)
            #cf.cmdVelocityWorld(np.array([0, 0, 0]) , yawRate=0)#+ kPosition * error,

            timeHelper.sleepForRate(80)




    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == "__main__":
    main()
