"""Run kinematic motion planning using GTDynamics outputs."""
from typing import Dict

import time

# import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import pandas as pd
import numpy as np

# pylint: disable=I1101, C0103

_ = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # To load plane SDF.
p.setGravity(0, 0, -9.8)
quad_id = p.loadURDF("inverted_pendulum.urdf", [0, 0, 0], [0, 0, 0, 1], False,
                     True)

# joint_to_jid_map = {}
# for i in range(p.getNumJoints(quad_id)):
#     jinfo = p.getJointInfo(quad_id, i)
#     joint_to_jid_map[jinfo[1].decode("utf-8")] = jinfo[0]

# def set_joint_angles(joint_angles: Dict[str, float], joint_vels: Dict[str, float]):
#     """Actuate to the suppplied joint angles using PD control."""
#     for jid in joint_to_jid_map.values():
#         p.setJointMotorControl2(quad_id, jid, p.VELOCITY_CONTROL, force=500)

#     for k, v in joint_angles.items():
#         p.setJointMotorControl2(bodyUniqueId=quad_id,
#                                 jointIndex=joint_to_jid_map[k],
#                                 controlMode=p.POSITION_CONTROL,
#                                 targetPosition=v,
#                                 targetVelocity=joint_vels[k + '.1'])

# df = pd.read_csv('traj.csv')
# print(df.columns)

# input("Press ENTER to continue.")

# pos, orn = p.getBasePositionAndOrientation(quad_id)

# print("Init Base\n\tPos: {}\n\tOrn: {}".format(pos,
#                                                p.getEulerFromQuaternion(orn)))

# # Store positions and times for analysis.
# t = 0
# ts = []
# all_pos_sim = []

# # To draw goal coordinate frames.
# goal_pos = None
# goal_orn = None
# debug_line_x, debug_line_y, debug_line_z = None, None, None

# debug_iters = 20
# for i in range(len(df)):
#     jangles = df.loc[i][[str(i) for i in range(12)]]
#     jvels = df.loc[i][[str(i) + '.1' for i in range(12)]]
#     jaccels = df.loc[i][[str(i) + '.2' for i in range(12)]]
#     jtorques = df.loc[i][[str(i) + '.3' for i in range(12)]]

#     new_goal_pos = df.loc[i][['gol_x', 'gol_y', 'gol_z']].tolist()
#     new_goal_pos[2] = new_goal_pos[2] + 0.21
#     new_goal_orn = df.loc[i][['gol_qx', 'gol_qy', 'gol_qz', 'gol_qw']].tolist()

#     set_joint_angles(jangles, jvels)

#     # Update body CoM coordinate frame.
#     new_pos, new_orn = p.getBasePositionAndOrientation(quad_id)
#     new_pos = np.array(new_pos)
#     new_R = np.array(p.getMatrixFromQuaternion(new_orn)).reshape(3, 3)

#     # Add goal pose coordinate frame.
#     if (new_goal_pos != goal_pos) or (new_goal_orn != goal_orn):

#         # Remove old debug items.
#         if debug_line_x != None:
#             p.removeUserDebugItem(debug_line_x)
#             p.removeUserDebugItem(debug_line_y)
#             p.removeUserDebugItem(debug_line_z)

#         t = np.array(new_goal_pos)
#         R = np.array(p.getMatrixFromQuaternion(new_goal_orn)).reshape(3, 3)

#         new_debug_line_x = p.addUserDebugLine(
#             np.array([0, 0, 0]) + t, np.matmul(R, np.array([0.1, 0, 0])) + t,
#             lineColorRGB=[1, 0, 0], lineWidth=3)
#         new_debug_line_y = p.addUserDebugLine(
#             np.array([0, 0, 0]) + t, np.matmul(R, np.array([0, 0.1, 0])) + t,
#             lineColorRGB=[0, 1, 0], lineWidth=3)
#         new_debug_line_z = p.addUserDebugLine(
#             np.array([0, 0, 0]) + t, np.matmul(R, np.array([0, 0, 0.1])) + t,
#             lineColorRGB=[0, 0, 1], lineWidth=3)

#         goal_pos = new_goal_pos
#         goal_orn = new_goal_orn
#         debug_line_x = new_debug_line_x
#         debug_line_y = new_debug_line_y
#         debug_line_z = new_debug_line_z

#     if (i % debug_iters) == 0:
#         print("\tIter {} Base\n\t\tPos: {}\n\t\tOrn: {}".format(
#             i, new_pos, p.getEulerFromQuaternion(new_orn)))
#         p.addUserDebugLine(pos, new_pos, lineColorRGB=[0, 1, 1], lineWidth=1)
#         pos, orn = new_pos, new_orn

#         bod_debug_line_x = p.addUserDebugLine(
#             np.array([0, 0, 0]) + new_pos,
#             np.matmul(new_R, np.array([0.05, 0, 0])) + new_pos,
#             lineColorRGB=[1, 0, 1], lineWidth=1, lifeTime=1.5)
#         bod_debug_line_y = p.addUserDebugLine(
#             np.array([0, 0, 0]) + new_pos,
#             np.matmul(new_R, np.array([0, 0.05, 0])) + new_pos,
#             lineColorRGB=[1, 0, 1], lineWidth=1, lifeTime=1.5)
#         bod_debug_line_z = p.addUserDebugLine(
#             np.array([0, 0, 0]) + new_pos,
#             np.matmul(new_R, np.array([0, 0, 0.05])) + new_pos,
#             lineColorRGB=[1, 0, 1], lineWidth=1, lifeTime=1.5)

#     ts.append(t)
#     all_pos_sim.append(new_pos)

#     p.stepSimulation()
#     time.sleep(1. / 240.)
#     t += 1. / 240.

# pos, orn = p.getBasePositionAndOrientation(quad_id)
# print("Final Base\n\tPos: {}\n\tOrn: {}".format(pos,
#                                                 p.getEulerFromQuaternion(orn)))


while True:
    p.stepSimulation()
    time.sleep(1. / 240)

p.disconnect()
