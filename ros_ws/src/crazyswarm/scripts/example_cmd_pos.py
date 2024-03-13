from pycrazyswarm import *
import numpy as np


def interpolate(start, end, steps):
    """Generate a linear interpolation between start and end points."""
    return [np.array([end[0], #start[0] + (end[0] - start[0]) * t / steps, ensuring it stays on the same tunnel
                      start[1] + (end[1] - start[1]) * t / steps, 
                      end[2]]) for t in range(steps + 1)]


swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs
cfl= allcfs.crazyflies[1]
cfr= allcfs.crazyflies[0]
Z = 0
print(cfl.position())
print(cfr.position())

takeOffTime = 5.0
totalTime=5.0
startTime = timeHelper.time()

#while timeHelper.time() - startTime < totalTime:

# takeoff
while cfl.position()[2] < 0.5 and (timeHelper.time() - startTime < takeOffTime):
    print(cfl.position())
    print("Z"+str(Z))
    for cf in allcfs.crazyflies:
        temp = np.array(cf.initialPosition)
        temp[2] = 0.0
        pos = temp + np.array([0, 0, Z])
        cf.cmdPosition(pos)
    timeHelper.sleepForRate(5)
    if Z < 0.80:
        Z += 0.03
    
#go to a mid point

# Assuming current_setpoint_l and current_setpoint_r are the current setpoints for left and right drones
current_setpoint_l =    cfl.position()  # Replace with the actual current setpoint
current_setpoint_r =    cfr.position()  # Replace with the actual current setpoint
print(cfl.position())
print(cfr.position())
posl = np.array([0, 0, Z])
posr = np.array([0.0, -0.05, Z])

# Number of steps for interpolation
steps = 200  # Adjust this as needed
print("Going to setpoints right now")
# Interpolate setpoints
setpoints_l = interpolate(current_setpoint_l, posl, steps)
setpoints_r = interpolate(current_setpoint_r, posr, steps)

for setpoint_l, setpoint_r in zip(setpoints_l, setpoints_r):
    print(setpoint_l)
    print(setpoint_r)
    print(cfl.position())
    print(cfr.position())
    print("\n")
    cfl.cmdPosition(setpoint_l)
    cfr.cmdPosition(setpoint_r)
    timeHelper.sleepForRate(20) 

startTime = timeHelper.time()
print("hovering")
print(cfl.position())
print(cfr.position())
while timeHelper.time() - startTime < totalTime:
    cfr.cmdPosition(posr)
    cfl.cmdPosition(posl)
    timeHelper.sleepForRate(80) 
   
print("landing")
# land
while Z > 0.0:
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.cmdPosition(pos)
    timeHelper.sleep(0.1)
    Z -= 0.05

# turn-off motors
for cf in allcfs.crazyflies:
    cf.cmdStop()
