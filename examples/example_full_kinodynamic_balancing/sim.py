"""Run kinematic motion planning using GTDynamics outputs."""
from typing import Dict

import time

import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import pandas as pd

# pylint: disable=I1101, C0103

_ = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # To load plane SDF.
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
p.changeDynamics(planeId, -1, lateralFriction=1)
quad_id = p.loadURDF("vision60.urdf", [0, 0, 0.21], [0, 0, 0, 1], False,
                     False)

joint_to_jid_map = {}
for i in range(p.getNumJoints(quad_id)):
    jinfo = p.getJointInfo(quad_id, i)
    joint_to_jid_map[jinfo[1].decode("utf-8")] = jinfo[0]

def set_joint_angles(joint_angles: Dict[str, float]):
    """Actuate to the suppplied joint angles using PD control."""
    for jid in joint_to_jid_map.values():
        p.setJointMotorControl2(quad_id, jid, p.VELOCITY_CONTROL, force=500)

    for k, v in joint_angles.items():
        p.setJointMotorControl2(bodyUniqueId=quad_id,
                                jointIndex=joint_to_jid_map[k],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=v)

joint_angle_map = {
	0: 0.308376491461,
	1: 0.902111795473,
	10: -9.17104566809e-05,
	11: -9.56766350639e-05,
	2: 0.308376875101,
	3: 0.90211123062,
	4: 0.308378488313,
	5: 0.90211192244,
	6: 0.308378871977,
	7: 0.902111357571,
	8: -9.17105064605e-05,
	9: -9.56766725055e-05,
}
joint_angle_map = {str(k): v for k, v in joint_angle_map.items()}

while True:
    set_joint_angles(joint_angle_map)
    p.stepSimulation()
    time.sleep(1. / 240.)

# df = pd.read_csv('traj.csv')

# input("Press ENTER to continue.")

# pos_des = df.loc[0][['bodyx', 'bodyy', 'bodyz']].tolist()
# pos_des[2] = pos_des[2] + 0.2
# pos, orn = p.getBasePositionAndOrientation(quad_id)

# print("Init Base\n\tPos: {}\n\tOrn: {}".format(pos,
#                                                p.getEulerFromQuaternion(orn)))

# # Store positions and times for analysis.
# t = 0
# ts = []
# all_pos_sim = []
# all_pos_des = []

# debug_iters = 100
# for i in range(len(df)):
#     set_joint_angles(df.loc[i][[str(i) for i in range(12)]])

#     new_pos_des = df.loc[i][['bodyx', 'bodyy', 'bodyz']].tolist()
#     new_pos_des[2] = new_pos_des[2] + 0.2
#     new_pos, new_orn = p.getBasePositionAndOrientation(quad_id)

#     if (i % debug_iters) == 0:
#         print("\tIter {} Base\n\t\tPos: {}\n\t\tOrn: {}".format(
#             i, new_pos, p.getEulerFromQuaternion(new_orn)))
#         p.addUserDebugLine(pos, new_pos, lineColorRGB=[0, 1, 0], lineWidth=3)
#         p.addUserDebugLine(pos_des, new_pos_des, lineColorRGB=[1, 0, 0], lineWidth=3)
#         pos, orn = new_pos, new_orn
#         pos_des = new_pos_des

#     ts.append(t)
#     all_pos_sim.append(new_pos)
#     all_pos_des.append(new_pos_des)

#     p.stepSimulation()
#     time.sleep(1. / 240.)
#     t += 1. / 240.

# pos, orn = p.getBasePositionAndOrientation(quad_id)
# print("Final Base\n\tPos: {}\n\tOrn: {}".format(pos,
#                                                 p.getEulerFromQuaternion(orn)))

# fig, axs = plt.subplots(3, 1, sharex=True)
# fig.subplots_adjust(hspace=0)

# axs[0].plot(ts, [p[0] for p in all_pos_sim])
# axs[0].plot(ts, [p[0] for p in all_pos_des])
# axs[0].set_ylabel('x (m.)')

# axs[1].plot(ts, [p[1] for p in all_pos_sim])
# axs[1].plot(ts, [p[1] for p in all_pos_des])
# axs[1].set_ylabel('y (m.)')

# axs[2].plot(ts, [p[2] for p in all_pos_sim])
# axs[2].plot(ts, [p[2] for p in all_pos_des])
# axs[2].set_ylabel('z (m.)')

# plt.xlabel("time (s.)")

# plt.show()

# while True:
#     p.stepSimulation()
#     time.sleep(1. / 240)

p.disconnect()
