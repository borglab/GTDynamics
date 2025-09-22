"""Run kinematic motion planning using GTDynamics outputs."""

import time

import numpy as np
import pybullet as p
import pybullet_data

import gtdynamics as gtd

# pylint: disable=I1101, C0103

_ = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # To load plane SDF.
p.setGravity(0, 0, -9.8)
robot = p.loadURDF(gtd.URDF_PATH + "cart_pole.urdf", [0, 0, 0], [0, 0, 0, 1], False, True)
t = 0


def set_joint_angle(joint_id: float, joint_angle: float, joint_vel: float):
    """Actuate to the suppplied joint angles using PD control."""
    p.setJointMotorControl2(robot, joint_id, p.VELOCITY_CONTROL, force=500)

    p.setJointMotorControl2(bodyUniqueId=robot,
                            jointIndex=joint_id,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_angle,
                            targetVelocity=joint_vel)


while True:
    p.stepSimulation()
    set_joint_angle(0, np.sin(t), 1 - np.sin(t))
    set_joint_angle(1, np.pi * np.cos(t), np.cos(t))

    t += 1. / 240
    time.sleep(1. / 240)

p.disconnect()
