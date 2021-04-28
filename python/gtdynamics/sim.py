"""Utilities for PyBullet simulation."""

#pylint: disable=c-extension-no-member

from typing import Dict


def set_joint_angles(pyb,
                     robot: int,
                     joint_to_jid_map: Dict,
                     joint_angles: Dict[str, float],
                     joint_velocities: Dict[str, float],
                     force: float = 500):
    """
    Actuate to the suppplied joint angles using PD control.

    Args:
        pyb: Pybullet module.
        robot: Robot ID
        joint_id: Joint ID on the robot.
        joint_angle: Angle of the joint to set.
        joint_velocity: Velocity of the joint to set.
    """
    for joint_id in joint_to_jid_map.values():
        pyb.setJointMotorControl2(bodyUniqueId=robot,
                                  jointindex=joint_id,
                                  controlMode=pyb.VELOCITY_CONTROL,
                                  force=force)

    for joint, angle in joint_angles.items():
        target_velocity = joint_velocities.get(joint + '.1', 0.0)
        pyb.setJointMotorControl2(bodyUniqueId=robot,
                                  jointIndex=joint_to_jid_map[joint],
                                  controlMode=pyb.POSITION_CONTROL,
                                  targetPosition=angle,
                                  targetVelocity=target_velocity)
