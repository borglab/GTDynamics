"""Run kinematic motion planning using GTDynamics outputs."""
# TODO(Varun): This whole file needs some cleaning up

import time

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pybullet as p
import pybullet_data

import gtdynamics as gtd

_ = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # To load plane SDF.
print(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(1)
planeId = p.loadURDF(gtd.URDF_PATH + "plane.urdf", useFixedBase=True)
p.changeDynamics(planeId, -1, lateralFriction=0.4)
robot = p.loadURDF(gtd.URDF_PATH + "/a1/a1.urdf",[0,0,0.45],[0,0,0,1.0])

robot_id = robot

joint_to_jid_map = {}
for i in range(p.getNumJoints(robot_id)):
    jinfo = p.getJointInfo(robot_id, i)
    jointType = jinfo[2]
    if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
        joint_to_jid_map[jinfo[1].decode("utf-8")] = jinfo[0]

#Read walk forward trajectory file from build (after running the example)
df = pd.read_csv('a1_traj_DG_mass.csv')

pos, orn = p.getBasePositionAndOrientation(robot_id)

print("Init Base\n\tPos: {}\n\tOrn: {}".format(pos,
                                               p.getEulerFromQuaternion(orn)))

# for i in range(0,20,1):
    # print(i, p.getJointInfo(robot_id, i))
    #import pdb;pdb.set_trace()
    #print(i, p.getLinkInfo(robot_id, i)[1])
#print(p.getDynamicsInfo(planeId,-1))
debug_iters = 1

max_traj_replays = 1
num_traj_replays = 0

t = 0
t_f = None
ts = []
all_pos_sim = []
all_torques_hip = []
all_torques_upper = []
all_torques_lower = []

link_dict = {}
link_to_num = {10:1, 5: 4, 20: 10, 15: 7}
constrained = []

i = 1
k = 0 
# for i in range(0,10,1):
    # print(i)
    # time.sleep(0.5)

while True:
    if num_traj_replays == max_traj_replays:
        break

    if i == len(df) - 1:
        i = 1
        if num_traj_replays == 0:
            t_f = t
        num_traj_replays += 1

    jangles = df.loc[i][np.arange(12)]
    jvels = df.loc[i][np.arange(12, 24)]
    jaccels = df.loc[i][np.arange(24, 36)]
    jtorques = df.loc[i][np.arange(36, 48)]
    
    for joint_id in joint_to_jid_map.values():
        p.setJointMotorControl2(robot_id,joint_id,
                                    controlMode=p.VELOCITY_CONTROL,
                                    force=50000)


    for joint, angle in jangles.items():
        target_velocity = jvels.get(joint + '.1', 0.0)
        p.setJointMotorControl2(bodyUniqueId=robot_id,
                                        jointIndex=joint_to_jid_map[joint],
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=angle,
                                        targetVelocity=0.0)

    # Update body CoM coordinate frame.
    new_pos, new_orn = p.getBasePositionAndOrientation(robot_id)
    new_pos = np.array(new_pos)
    new_R = np.array(p.getMatrixFromQuaternion(new_orn)).reshape(3, 3)
    # print(i)
    t_hip = p.getJointState(bodyUniqueId=robot_id, jointIndex = 6)[3]
    t_upper= p.getJointState(bodyUniqueId=robot_id, jointIndex = 8)[3]
    t_lower = p.getJointState(bodyUniqueId=robot_id, jointIndex = 9)[3]

    # Detect collision points and constrain them.
    '''cp = np.asarray(p.getContactPoints(bodyA=planeId, bodyB=robot_id))
    #import pdb;pdb.set_trace()
    if cp.shape[0] > 1 and i > 1:
        new_cps = set(cp[:, 4])

        change = list(df.loc[i][np.arange(0, 12)] - df.loc[i-1][np.arange(0, 12)])
        #import pdb;pdb.set_trace()
        # Initial collision
        just_collided = [
            x for x in new_cps if x not in constrained and x in link_to_num.keys()]
        # print(new_cps)
        for x in just_collided:
            print(change[link_to_num[x]] )
           # if (link_to_num[x] < 4 and change[link_to_num[x]] >= 0) or (link_to_num[x] >= 4 and change[link_to_num[x]] <= 0):
            if (change[link_to_num[x]] >= 0.01) or (change[link_to_num[x]] <= -0.01):
                link_dict[x] = p.createConstraint(robot_id, x, planeId, -1, p.JOINT_POINT2POINT, [
                                                  0, 0, 0], [0, 0, 0], p.getLinkState(robot_id, x)[0])
                constrained.append(x)
        #print(link_dict)
        # Wants to lift
        for x in constrained:
            #if (link_to_num[x] < 4 and change[link_to_num[x]] <= 0) or (link_to_num[x] >= 4 and change[link_to_num[x]] >= 0) and link_dict.get(x) != None:
            if (change[link_to_num[x]] >= 0.01) or (change[link_to_num[x]] <= -0.01) and link_dict.get(x) != None:
                numConstraints_before = p.getNumConstraints()
                p.removeConstraint(link_dict[x])
                if numConstraints_before != p.getNumConstraints():
                    constrained.remove(x) '''

    if (i % debug_iters) == 0:
        # print("\tIter {} Base\n\t\tPos: {}\n\t\tOrn: {}".format(
        # i, new_pos, p.getEulerFromQuaternion(new_orn)))

        if (num_traj_replays == 0):
            p.addUserDebugLine(pos, new_pos, lineColorRGB=[
                               1, 0, 1], lineWidth=2.5)
        else:
            p.addUserDebugLine(pos, new_pos, lineColorRGB=[
                               0, 1, 1], lineWidth=2.5) 
        pos, orn = new_pos, new_orn

        '''bod_debug_line_x = p.addUserDebugLine(
            np.array([0, 0, 0]) + new_pos,
            np.matmul(new_R, np.array([0.05, 0, 0])) + new_pos,
            lineColorRGB=[1, 0, 1], lineWidth=1, lifeTime=1.5)
        bod_debug_line_y = p.addUserDebugLine(
            np.array([0, 0, 0]) + new_pos,
            np.matmul(new_R, np.array([0, 0.05, 0])) + new_pos,
            lineColorRGB=[1, 0, 1], lineWidth=1, lifeTime=1.5)
        bod_debug_line_z = p.addUserDebugLine(
            np.array([0, 0, 0]) + new_pos,
            np.matmul(new_R, np.array([0, 0, 0.05])) + new_pos,
            lineColorRGB=[1, 0, 1], lineWidth=1, lifeTime=1.5)'''
    
    p.stepSimulation()
    time.sleep(1. /24.)

    t += 1. / 24.
    i += 1
    k += 1
    if t >2 and t < 12:
       all_pos_sim.append(new_pos)
       #if t_hip > 80 or t_upper > 80 or t_lower > 80: continue
       ts.append(t-2)
       all_torques_hip.append(t_hip)
       all_torques_upper.append(t_upper)
       all_torques_lower.append(t_lower)

torques = np.zeros((len(all_torques_hip),4))
torques[:,0] = np.array(ts)
torques[:,1] = np.array(all_torques_hip)
torques[:,2] = np.array(all_torques_upper)
torques[:,3] = np.array(all_torques_lower)
np.savetxt('/home/dan/Desktop/Projects/GTDynamics/build/examples/example_a1_walking/torques_pyb.txt', torques)

pos, orn = p.getBasePositionAndOrientation(robot_id)
print("Final Base\n\tPos: {}\n\tOrn: {}".format(pos,
                                                p.getEulerFromQuaternion(orn)))

fig, axs = plt.subplots(6, 1, sharex=True)
fig.subplots_adjust(hspace=0.1)

axs[0].plot(ts, [p[0] for p in all_pos_sim])
#axs[0].axvline(x=t_f, color='k', linestyle='--')
axs[0].set_ylabel('x (m.)')
axs[0].set_title('Robot Body location Vs. Time')

axs[1].plot(ts, [p[1] for p in all_pos_sim])
#axs[1].axvline(x=t_f, color='k', linestyle='--')
axs[1].set_ylabel('y (m.)')
axs[1].set_ylim((-0.2,0.2))

axs[2].plot(ts, [p[2] for p in all_pos_sim])
#axs[2].axvline(x=t_f, color='k', linestyle='--')
axs[2].set_ylabel('z (m.)')
axs[2].set_ylim((0.2,0.45))

axs[3].plot(ts, [t for t in all_torques_hip])
#axs[2].axvline(x=t_f, color='k', linestyle='--')
axs[3].set_ylabel('tau hip')
#axs[3].set_ylim((-10,10))

axs[4].plot(ts, [t for t in all_torques_upper])
#axs[2].axvline(x=t_f, color='k', linestyle='--')
axs[4].set_ylabel('tau upper')
#axs[3].set_ylim((-10,10))

axs[5].plot(ts, [t for t in all_torques_lower])
#axs[2].axvline(x=t_f, color='k', linestyle='--')
axs[5].set_ylabel('tau lower')
#axs[3].set_ylim((-10,10))

plt.xlabel("time (s.)")

plt.show()
# uncomment to output pdflatex
#plt.rc('pgf', texsystem='pdflatex')
#plt.savefig('/home/dan/Desktop/my_papers/thesis/create_simulation/pyb_sim.pgf')

# i = 0
# while True:
    # i += 1
    # print(i)
    # p.stepSimulation()
    # time.sleep(1. / 240)
p.disconnect()
