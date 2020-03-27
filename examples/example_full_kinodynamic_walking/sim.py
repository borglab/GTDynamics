"""Run kinematic motion planning using GTDynamics outputs."""
from typing import Dict

import time

import pybullet as p
import pybullet_data
import pandas as pd
import numpy as np

# pylint: disable=I1101, C0103

_ = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # To load plane SDF.
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
p.changeDynamics(planeId, -1, lateralFriction=2.0)
quad_id = p.loadURDF("vision60.urdf", [0, 0, 0.21], [0, 0, 0, 1], False,
                     False)

joint_to_jid_map = {}
for i in range(p.getNumJoints(quad_id)):
    jinfo = p.getJointInfo(quad_id, i)
    joint_to_jid_map[jinfo[1].decode("utf-8")] = jinfo[0]

def set_joint_angles(joint_angles: Dict[str, float], joint_vels: Dict[str, float]):
    """Actuate to the suppplied joint angles using PD control."""
    for jid in joint_to_jid_map.values():
        p.setJointMotorControl2(quad_id, jid, p.VELOCITY_CONTROL, force=500)

    for k, v in joint_angles.items():
        p.setJointMotorControl2(bodyUniqueId=quad_id,
                                jointIndex=joint_to_jid_map[k],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=v,
                                targetVelocity=joint_vels[k + '.1'])

df = pd.read_csv('traj.csv')
print(df.columns)

input("Press ENTER to continue.")

pos, orn = p.getBasePositionAndOrientation(quad_id)

print("Init Base\n\tPos: {}\n\tOrn: {}".format(pos,
                                               p.getEulerFromQuaternion(orn)))

debug_iters = 20
for i in range(len(df)):
    jangles = df.loc[i][[str(i) for i in range(12)]]
    jvels = df.loc[i][[str(i) + '.1' for i in range(12)]]
    jaccels = df.loc[i][[str(i) + '.2' for i in range(12)]]
    jtorques = df.loc[i][[str(i) + '.3' for i in range(12)]]

    set_joint_angles(jangles, jvels)

    # Update body CoM coordinate frame.
    new_pos, new_orn = p.getBasePositionAndOrientation(quad_id)
    new_pos = np.array(new_pos)
    new_R = np.array(p.getMatrixFromQuaternion(new_orn)).reshape(3, 3)

    if (i % debug_iters) == 0:
        print("\tIter {} Base\n\t\tPos: {}\n\t\tOrn: {}".format(
            i, new_pos, p.getEulerFromQuaternion(new_orn)))
        p.addUserDebugLine(pos, new_pos, lineColorRGB=[0, 1, 1], lineWidth=1)
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

pos, orn = p.getBasePositionAndOrientation(quad_id)
print("Final Base\n\tPos: {}\n\tOrn: {}".format(pos,
                                                p.getEulerFromQuaternion(orn)))

while True:
    p.stepSimulation()
    time.sleep(1. / 240)

p.disconnect()
