"""
V-rep remote control API, python
"""
import numpy as np
import math
import vrep
import time

# Setting
RAD2DEG = 180 / math.pi  # degree to radius
time_step = 0.005  # simulation timestep
# joint information
joint_num = 7
base_name = 'LBR_iiwa_7_R800'
joint_name = 'LBR_iiwa_7_R800_joint'
obstacle_name = 'diningTable'

# Initialization
# close previous connection
vrep.simxFinish(-1)
# check every 0.2s connection with v-rep
while True:
    clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if clientID > -1:
        break
    else:
        time.sleep(0.2)
        print("Failed connecting to remote API server!")
print("Connection success!")

# Configuration
# set simulation timestep for v-rep and api the same
vrep.simxSetFloatingParameter(
    clientID, vrep.sim_floatparam_simulation_time_step, time_step, vrep.simx_opmode_oneshot)
# using synchronous mode
vrep.simxSynchronous(clientID, True)
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# obtain base and joints handles
joint_handle = np.zeros((joint_num,), dtype=np.int)
for i in range(joint_num):
    _, return_handle = vrep.simxGetObjectHandle(clientID, joint_name + str(i+1), vrep.simx_opmode_blocking)
    joint_handle[i] = return_handle

_, base_handle = vrep.simxGetObjectHandle(clientID, base_name, vrep.simx_opmode_blocking)
print('Handles available!')

# obtain joint initial values using streaming mode
joint_config = np.zeros((joint_num,))
for i in range(joint_num):
    _, joint_position = vrep.simxGetJointPosition(clientID, joint_handle[i], vrep.simx_opmode_streaming)
    joint_config[i] = joint_position

# obtain table handles
_, table_handle = vrep.simxGetObjectHandle(clientID, obstacle_name, vrep.simx_opmode_blocking)
# get table position relative to the robot
_, table_position = vrep.simxGetObjectPosition(clientID, table_handle, base_handle, vrep.simx_opmode_streaming)
# set table position relative to the robot
table_position[1] = table_position[1] + 1
table_position[2] = table_position[2] + 0.3
vrep.simxSetObjectPosition(clientID, table_handle, base_handle, table_position, vrep.simx_opmode_oneshot)

# open files for joint trajectories
file_object = [open("./test_data/joint_angles/q" + str(i+1) + ".txt", "r") for i in range(joint_num)]

# record current time
last_cmd_time = vrep.simxGetLastCmdTime(clientID)
# simulation moves one step forward
vrep.simxSynchronousTrigger(clientID)
# start simuation, stop simulation at the end of file
not_end = True
while vrep.simxGetConnectionId(clientID) != -1 and not_end:
    # record current time
    current_cmd_time = vrep.simxGetLastCmdTime(clientID) 
    # record current time step
    dt = current_cmd_time - last_cmd_time
    # get current state value
    for i in range(joint_num):
        _, joint_position = vrep.simxGetJointPosition(clientID, joint_handle[i], vrep.simx_opmode_buffer)
        print(round(joint_position, 2))
        joint_config[i] = joint_position

    # control command need to send simultaneously, so pause communicatiom
    vrep.simxPauseCommunication(clientID, True)
    for i in range(joint_num):
        q = file_object[i].readline()
        if not q: 
            not_end = False
            break
        vrep.simxSetJointTargetPosition(clientID, joint_handle[i], float(q), vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)
    # record current time
    last_cmd_time = current_cmd_time
    # simulation moves one step forward
    vrep.simxSynchronousTrigger(clientID)
    # finish simulation
    vrep.simxGetPingTime(clientID)
    time.sleep(0.1)
