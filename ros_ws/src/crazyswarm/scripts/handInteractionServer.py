"""Takeoff-hover-land for one CF. Useful to validate hardware config."""
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

#Sockets
IP = "127.0.0.1"
PORT = 8008
BUFFER_SIZE = 1024  # Adjust as needed
DRONE_DISTANCE = 0.15  # 15 cm
TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
totalTime = 60.0

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

# Function to handle incoming log data
def log_callback(data):
    # Open the file in append mode
    with open(file_path, 'a') as file:
        # Write data to file; you might need to adjust this depending on the actual structure of GenericLogData
        file.write("***************"+str(data) +'\n')
        #print(str(data) +'\n')


def udp_listener():
    global goal_position
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IP, PORT))

    while True:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        try:
            goal_position = np.fromstring(data.decode(), sep=',')
            #print("goal:",goal_position)
        except ValueError:
            print("error")
            pass 

def update_position(cf,timeHelper):
    global numClicked, goal_position, hand_Pos, hand_Ori,takeoff, land
    lastPos = [0,0,0]
    lastClick = 0
    
    while True:
        #Boundary check
        
        #print(hand_Pos)
        
        #Takeoff Detection by z first arriving over 0.8
        if not takeoff and hand_Pos[2] > takeoff_height and is_within_radius(hand_Pos, cf.position(), 0.9):
            cf.takeoff(targetHeight=z, duration=TAKEOFF_DURATION)
            rospy.sleep(TAKEOFF_DURATION)
            takeoff = True
        
        if takeoff and not land:
        #Main operation Loop, based on hands pitch angle
            if abs(hand_Ori[1])<0.5:
                lastPos = hand_Pos
                #print("following hands")
                if lastPos[2] > 1.7:
                    print(lastPos)
                    land = True
                    print("Too high, landing")
                    cf.land(targetHeight=0.04, duration=3.5)
                    rospy.sleep(3.5)
                if(lastPos[2] < 0.5):
                    print("Too low, adding z")
                    lastPos[2] = z
                    
                #following the hand
                lastPos=lastPos+[0,0,-0.2]
                cf.cmdPosition(lastPos)
                #print("CMDgoal",lastPos)
            if abs(hand_Ori[1]) > 0.7 and timeHelper.time() - lastClick > 1.0 :
                #print("clicking, maintaining pos")
                if (cf.position()[2] - lastPos[2]) > 0.1:
                    numClicked += 1
                    lastClick = timeHelper.time()
                    print("clicked down @",lastPos)
                cf.cmdPosition(lastPos)
        
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
    
    # Calculate forward direction (assuming pointing direction is along x-axis)
    # frontOfX = np.array([
    #     np.cos(euler[1]) * np.cos(euler[0]),
    #     np.sin(euler[1]) * np.cos(euler[0]),
    #     np.sin(euler[0])
    # ])

    #now determine where the drone needs to move based on hands gesture
    # print(forward_dir)
    # #if(forward_dir[1] )
    
    # # Calculate the target position
    # target_pos = position +  [0,0,-0.1]# DRONE_DISTANCE * forward_dir
    # #print("target Pos: ", target_pos,", HandPos: ", position)
    # return target_pos



def main():
    
    #starting UDP listener
    udp_thread = threading.Thread(target=udp_listener)
    udp_thread.daemon = True
    udp_thread.start()
    print("listening")
    
    
    rospy.Subscriber('/vicon/Hand/Hand', TransformStamped, calculate_target_position)
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    cfs = swarm.allcfs
    # Subscribe to the /cf1/log1 topic
    rospy.Subscriber("/cf1/log1", GenericLogData, log_callback)
    
    #Takeoff
    # cfs.takeoff(targetHeight=z, duration=TAKEOFF_DURATION)
    # timeHelper.sleep(TAKEOFF_DURATION)
    # print("finished tookoff and starting UDP")
    
    
    #start streaming Pos thru UDP
    position_thread = threading.Thread(target=update_position, args=(cf,timeHelper))
    position_thread.daemon = True
    position_thread.start()
    print("UDP started")
    while( not takeoff or not land):
        rospy.sleep(0.1)
    
    print("click so far: " + str(numClicked))
    #25 seconds max, then land
    cfs.land(targetHeight=0.04, duration=3.5)
    timeHelper.sleep(TAKEOFF_DURATION)
    print("land")  

if __name__ == "__main__":
    main()
