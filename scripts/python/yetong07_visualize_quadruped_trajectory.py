"""Run kinematic motion planning using GTDynamics outputs."""
import time

import numpy as np
import pandas as pd
import pybullet as p
import pybullet_data

import gtdynamics as gtd

scenario_name = "yetong07_ie_quadruped_jump"
optimizer_name = "manopt"
scenario_folder = "data/" + scenario_name + "/"
traj_file = scenario_folder + optimizer_name + "_traj_viz.csv"

ordered_joints = ["fl_upper", "fl_lower", "rl_upper", "rl_lower", "fr_upper", "fr_lower", "rr_upper", "rr_lower", "fl_hip", "rl_hip", "fr_hip", "rr_hip"]

# pylint: disable=I1101, C0103

_ = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # To load plane SDF.
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
p.changeDynamics(planeId, -1, lateralFriction=2.0)
quad_id = p.loadURDF("models/urdfs/vision60.urdf", [0, 0, 0.21], [0, 0, 0, 1], False,
                     False)

joint_to_jid_map = {}
for i in range(p.getNumJoints(quad_id)):
    jinfo = p.getJointInfo(quad_id, i)
    urdf_joint_name = jinfo[1].decode("utf-8")
    joint_name = ordered_joints[int(urdf_joint_name)]
    joint_to_jid_map[joint_name] = jinfo[0]

df = pd.read_csv(traj_file)
print(df.columns)

input("Press ENTER to continue.")

pos, orn = p.getBasePositionAndOrientation(quad_id)

print("Init Base\n\tPos: {}\n\tOrn: {}".format(pos,
                                               p.getEulerFromQuaternion(orn)))

# Store positions and times for analysis.
t = 0
ts = []
all_pos_sim = []
dt = 0.1

# To draw goal coordinate frames.
goal_pos = None
goal_orn = None
debug_line_x, debug_line_y, debug_line_z = None, None, None

debug_iters = 20
for i in range(len(df)):
    jangles = df.loc[i][[str(i) for i in ordered_joints]]
    jvels = df.loc[i][[str(i) + '.1' for i in ordered_joints]]
    jaccels = df.loc[i][[str(i) + '.2' for i in ordered_joints]]
    jtorques = df.loc[i][[str(i) + '.3' for i in ordered_joints]]

    # print(jangles)

    base_pos = df.loc[i][['base_x', 'base_y', 'base_z']].tolist()
    # base_pos[2] = base_pos[2] + 0.21
    base_orn = df.loc[i][['base_qx', 'base_qy', 'base_qz', 'base_qw']].tolist()

    p.resetBasePositionAndOrientation(quad_id, base_pos, base_orn)

    for joint, angle in jangles.items():
        target_velocity = jvels.get(joint + '.1', 0.0)
        p.resetJointState(quad_id,
                          jointIndex=joint_to_jid_map[joint],
                          targetValue=angle,
                          targetVelocity=target_velocity)


    gtd.sim.set_joint_angles(p, quad_id, joint_to_jid_map, jangles, jvels)

    ts.append(t)
    # all_pos_sim.append(new_pos)

    p.stepSimulation()
    time.sleep(dt)
    t += dt

time.sleep(10)

pos, orn = p.getBasePositionAndOrientation(quad_id)
print("Final Base\n\tPos: {}\n\tOrn: {}".format(pos,
                                                p.getEulerFromQuaternion(orn)))


p.disconnect()
