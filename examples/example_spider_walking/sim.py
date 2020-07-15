"""Run kinematic motion planning using GTDynamics outputs."""
from typing import Dict

import time

import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import pandas as pd
import numpy as np

# pylint: disable=I1101, C0103

_ = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # To load plane SDF.
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
p.changeDynamics(planeId, -1, lateralFriction=1)
robot = p.loadSDF("spider.sdf")
robot_id = robot[0] # loadSDF returns a list of objects; the first is the integer ID.

# TODO (disha + stephanie): check whether this function is necessary/correct
#      for initially setting basePosition and baseOrientation
#      (in prior examples, this was done in the call to p.loadURDF().)
p.resetBasePositionAndOrientation(robot_id, [0, 0, 0.21], [0, 0, 0, 1])

joint_to_jid_map = {}
for i in range(p.getNumJoints(robot_id)):
    jinfo = p.getJointInfo(robot_id, i)
    joint_to_jid_map[jinfo[1].decode("utf-8")] = jinfo[0]

def set_joint_angles(joint_angles: Dict[str, float], joint_vels: Dict[str, float]):
    """Actuate to the supplied joint angles using PD control."""
    for jid in joint_to_jid_map.values():
        p.setJointMotorControl2(robot_id, jid, p.VELOCITY_CONTROL, force=5000)

    for k, v in joint_angles.items():
        p.setJointMotorControl2(bodyUniqueId=robot_id,
                                jointIndex=joint_to_jid_map[k],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=v,
                                targetVelocity=joint_vels[k + '.1'])

df = pd.read_csv('traj.csv')
print(df.columns)

input("Press ENTER to continue.")

pos, orn = p.getBasePositionAndOrientation(robot_id)

print("Init Base\n\tPos: {}\n\tOrn: {}".format(pos,
                                               p.getEulerFromQuaternion(orn)))

debug_iters = 20

max_traj_replays = 5
num_traj_replays = 0

t = 0
t_f = None
ts = []
all_pos_sim = []

i=0
while True:

    if num_traj_replays == max_traj_replays:
        break

    if (i == len(df) - 1):
        i = 0
        if num_traj_replays == 0:
            t_f = t
        num_traj_replays += 1

    jangles = df.loc[i][[str(i) for i in range(12)]]
    jvels = df.loc[i][[str(i) + '.1' for i in range(12)]]
    jaccels = df.loc[i][[str(i) + '.2' for i in range(12)]]
    jtorques = df.loc[i][[str(i) + '.3' for i in range(12)]]

    set_joint_angles(jangles, jvels)

    # Update body CoM coordinate frame.
    new_pos, new_orn = p.getBasePositionAndOrientation(robot_id)
    new_pos = np.array(new_pos)
    new_R = np.array(p.getMatrixFromQuaternion(new_orn)).reshape(3, 3)

    if (i % debug_iters) == 0:
        print("\tIter {} Base\n\t\tPos: {}\n\t\tOrn: {}".format(
            i, new_pos, p.getEulerFromQuaternion(new_orn)))

        if (num_traj_replays == 0):
            p.addUserDebugLine(pos, new_pos, lineColorRGB=[1, 0, 1], lineWidth=2.5)
        else:
            p.addUserDebugLine(pos, new_pos, lineColorRGB=[0, 1, 1], lineWidth=2.5)
        pos, orn = new_pos, new_orn

        bod_debug_line_x = p.addUserDebugLine(
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
            lineColorRGB=[1, 0, 1], lineWidth=1, lifeTime=1.5)

    p.stepSimulation()
    time.sleep(1. / 240.)

    ts.append(t)
    t += 1. / 240.
    all_pos_sim.append(new_pos)
    i+=1


pos, orn = p.getBasePositionAndOrientation(robot_id)
print("Final Base\n\tPos: {}\n\tOrn: {}".format(pos,
                                                p.getEulerFromQuaternion(orn)))

fig, axs = plt.subplots(3, 1, sharex=True)
fig.subplots_adjust(hspace=0)

axs[0].plot(ts, [p[0] for p in all_pos_sim])
axs[0].axvline(x=t_f, color='k', linestyle='--')
axs[0].set_ylabel('x (m.)')

axs[1].plot(ts, [p[1] for p in all_pos_sim])
axs[1].axvline(x=t_f, color='k', linestyle='--')
axs[1].set_ylabel('y (m.)')

axs[2].plot(ts, [p[2] for p in all_pos_sim])
axs[2].axvline(x=t_f, color='k', linestyle='--')
axs[2].set_ylabel('z (m.)')

plt.xlabel("time (s.)")

plt.show()

while True:
    p.stepSimulation()
    time.sleep(1. / 240)

p.disconnect()
