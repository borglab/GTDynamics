import gtdynamics
import gtsam
import numpy as np

robot = gtdynamics.CreateRobotFromFile("/home/varun/borglab/GTDynamics/urdfs/biped.urdf",
                                       "biped")
# robot.print_();

robotLinks = []
queue = [robot.link("body")]
while queue:
    # print("current queue", queue)
    link = queue.pop(0)
    robotLinks.append(link)
    # print("link:", link.name())
    joints = link.getJoints()
    # print("\t\t", joints)
    for idx, j in enumerate(joints):
        joint = robot.joint(j.name())
        # print(idx, joint.childLink().name())

        if joint.childLink().name() == link.name():
            continue

        queue.append(joint.childLink())

# print(robotLinks)

# for link in robotLinks:
#     print(link.name(), link.getID())

# print(robot.joints())
joint_angles, joint_velocities = {}, {}
for joint in robot.joints():
    # print(joint.name(), joint.wTj())
    joint_angles[joint.name()] = 0
    joint_velocities[joint.name()] = 0

result = robot.forwardKinematics(joint_angles,
                                 joint_velocities,
                                 "lower0",
                                 gtsam.Pose3(),
                                 np.zeros(6))

link_poses, link_twists = result
for name, pose in link_poses.items():
    print(name, pose)
# graph = gtdynamics.DynamicsGraph()
# graph.linearDynamicsGraph(
#       robot, 0,
#       {const gtdynamics::JointValues &joint_angles},
#       const gtdynamics::JointValues &joint_vels,
#       const gtdynamics::FKResults &fk_results,
#       const boost::optional<gtsam::Vector3> &gravity,
#       const boost::optional<gtsam::Vector3> &planar_axis)
