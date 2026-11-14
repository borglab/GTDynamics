"""Joint names shared with ur_bridge (copied from UR10e-Trajectory-Validation).

ur_bridge publishes JOINT_NAMES[1:], the six UR10e joints; the rail joint is
first, as in that repo, so a later rail bridge can use the same tuple.
"""

JOINT_NAMES = (
    'linear_rail_joint',
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
)
NUM_JOINTS = len(JOINT_NAMES)
