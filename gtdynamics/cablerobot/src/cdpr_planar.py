import gtdynamics as gtd
import gtsam
from gtsam import Pose3, Rot3
import numpy as np

class Cdpr:
    def __init__(self):
        # construct links
        inertia = np.eye(3)
        base_pose = Pose3(Rot3(), (1.5, 0, 1.5))
        ee_pose = Pose3(Rot3(), (1.5, 0, 1.5))
        c0a_pose = Pose3(Rot3.Ry(0.0*np.pi), (0, 0, 0))
        c1a_pose = Pose3(Rot3.Ry(1.5*np.pi), (3, 0, 0))
        c2a_pose = Pose3(Rot3.Ry(1.0*np.pi), (3, 0, 3))
        c3a_pose = Pose3(Rot3.Ry(0.5*np.pi), (0, 0, 3))
        c0b_pose = Pose3(Rot3.Ry(0.0*np.pi), (1.35, 0, 1.35))
        c1b_pose = Pose3(Rot3.Ry(1.5*np.pi), (1.65, 0, 1.35))
        c2b_pose = Pose3(Rot3.Ry(1.0*np.pi), (1.65, 0, 1.65))
        c3b_pose = Pose3(Rot3.Ry(0.5*np.pi), (1.35, 0, 1.65))
        
        base = gtd.Link(0, "base", 99, inertia, base_pose, Pose3(), True)
        ee = gtd.Link(1, "ee", 1, inertia, ee_pose, Pose3())
        c0a = gtd.Link(2, "c0a", 1e-3, inertia, c0a_pose, Pose3())
        c1a = gtd.Link(3, "c1a", 1e-3, inertia, c1a_pose, Pose3())
        c2a = gtd.Link(4, "c2a", 1e-3, inertia, c2a_pose, Pose3())
        c3a = gtd.Link(5, "c3a", 1e-3, inertia, c3a_pose, Pose3())
        c0b = gtd.Link(6, "c0b", 1e-3, inertia, c0b_pose, Pose3())
        c1b = gtd.Link(7, "c1b", 1e-3, inertia, c1b_pose, Pose3())
        c2b = gtd.Link(8, "c2b", 1e-3, inertia, c2b_pose, Pose3())
        c3b = gtd.Link(9, "c3b", 1e-3, inertia, c3b_pose, Pose3())

        links = {link.name(): link for link in [base, ee,
            c0a, c0b, c1a, c1b, c2a, c2b, c3a, c3b]}

        # construct joints
        params = gtd.JointParams()
        j0a_pose = Pose3(Rot3(), (0, 0, 0))
        j0b_pose = Pose3(Rot3(), (0, 0, 0))
        j0c_pose = Pose3(Rot3(), (1.35, 0, 1.35))
        j1a_pose = Pose3(Rot3(), (3, 0, 0))
        j1b_pose = Pose3(Rot3(), (3, 0, 0))
        j1c_pose = Pose3(Rot3(), (1.65, 0, 1.35))
        j2a_pose = Pose3(Rot3(), (3, 0, 3))
        j2b_pose = Pose3(Rot3(), (3, 0, 3))
        j2c_pose = Pose3(Rot3(), (1.65, 0, 1.65))
        j3a_pose = Pose3(Rot3(), (0, 0, 3))
        j3b_pose = Pose3(Rot3(), (0, 0, 3))
        j3c_pose = Pose3(Rot3(), (1.35, 0, 1.65))


        axis = np.array([0, 1, 0])
        j0a = gtd.RevoluteJoint(00, 'r0a', j0a_pose, base, c0a, params, axis)
        j1a = gtd.RevoluteJoint(10, 'r1a', j1a_pose, base, c1a, params, axis)
        j2a = gtd.RevoluteJoint(20, 'r2a', j2a_pose, base, c2a, params, axis)
        j3a = gtd.RevoluteJoint(30, 'r3a', j3a_pose, base, c3a, params, axis)
        
        j0b = gtd.PrismaticJoint( 1, 'p0b', j0b_pose, c0a, c0b, params, [1, 0, 1])
        j1b = gtd.PrismaticJoint(11, 'p1b', j1b_pose, c1a, c1b, params, [-1, 0, 1])
        j2b = gtd.PrismaticJoint(21, 'p2b', j2b_pose, c2a, c2b, params, [-1, 0, -1])
        j3b = gtd.PrismaticJoint(31, 'p3b', j3b_pose, c3a, c3b, params, [1, 0, -1])

        j0c = gtd.RevoluteJoint( 2, 'r0c', j0c_pose, c0b, ee, params, axis)
        j1c = gtd.RevoluteJoint(12, 'r1c', j1c_pose, c1b, ee, params, axis)
        j2c = gtd.RevoluteJoint(22, 'r2c', j2c_pose, c2b, ee, params, axis)
        j3c = gtd.RevoluteJoint(32, 'r3c', j3c_pose, c3b, ee, params, axis)

        joints = {joint.name() : joint for joint in [j0a, j1a, j2a, j3a,
                                                    j0b, j1b, j2b, j3b,
                                                    j0c, j1c, j2c, j3c]}

        # connect links to joints
        # TODO(frank): non-functional. And not logical: why do links know about joints?
        for j in joints.values():
            j.parent().addJoint(j)
            j.child().addJoint(j)

        # construct robot
        self.robot = gtd.Robot(links, joints)

    def eelink(self):
        return self.robot.link('ee')
