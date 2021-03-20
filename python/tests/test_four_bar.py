"""Unit tests for inverse dynamics of a four bar linkage."""

# pylint: disable=no-member, no-name-in-module

import unittest

import gtsam
from gtsam import Pose3, Rot3
import numpy as np

import gtdynamics as gtd


class TestFourBar(unittest.TestCase):
    """Create a 4-bar linkage manually and test it."""

    def test_four_bar(self):
        """ Testing for four bar linkage. """

        # construct links
        inertia = np.eye(3)
        l1_pose = Pose3(Rot3.Rz(0), (0, 0, 0))
        l2_pose = Pose3(Rot3.Rz(np.pi / 2), (2, 0, 0))
        l3_pose = Pose3(Rot3.Rz(np.pi), (2, 2, 0))
        l4_pose = Pose3(Rot3.Rz(np.pi * 3 / 2), (0, 2, 0))
        com = Pose3(Rot3(), (1, 0, 0))

        link1 = gtd.Link(gtd.LinkParams("l1", 1, inertia, l1_pose, com))
        link2 = gtd.Link(gtd.LinkParams("l2", 1, inertia, l2_pose, com))
        link3 = gtd.Link(gtd.LinkParams("l3", 1, inertia, l3_pose, com))
        link4 = gtd.Link(gtd.LinkParams("l4", 1, inertia, l4_pose, com))
        # TODO(frank): setID? There lies madness!!!
        link1.setID(1)
        link2.setID(2)
        link3.setID(3)
        link4.setID(4)
        link4.fix()

        links = {"l1": link1, "l2": link2, "l3": link3, "l4": link4}

        # construct joints
        params = gtd.JointParams()
        axis = np.array([0, 0, 1])
        j1_pose = Pose3(Rot3.Rz(0), (2, 0, 0))
        j2_pose = Pose3(Rot3.Rz(0), (2, 2, 0))
        j3_pose = Pose3(Rot3.Rz(0), (0, 2, 0))
        j4_pose = Pose3(Rot3.Rz(0), (0, 0, 0))

        joint1 = gtd.RevoluteJoint(1, "j1", j1_pose, link1, link2, params, axis)
        joint2 = gtd.RevoluteJoint(2, "j2", j2_pose, link2, link3, params, axis)
        joint3 = gtd.RevoluteJoint(3, "j3", j3_pose, link3, link4, params, axis)
        joint4 = gtd.RevoluteJoint(4, "j4", j4_pose, link4, link1, params, axis)
        joints = {"j1": joint1, "j2": joint2, "j3": joint3, "j4": joint4}

        # connect links to joints
        # TODO(frank): non-functional. And not logical: why do links know about joints?
        link1.addJoint(joint4)
        link1.addJoint(joint1)
        link2.addJoint(joint1)
        link2.addJoint(joint2)
        link3.addJoint(joint2)
        link3.addJoint(joint3)
        link4.addJoint(joint3)
        link4.addJoint(joint4)

        # construct robot
        robot = gtd.Robot(links, joints)
        # print(robot)

        # construct dynamics graph
        opt_setting = gtd.OptimizerSetting()
        gravity = np.array([0, 0, 0])
        planar_axis = np.array([0, 0, 1])
        graph_builder = gtd.DynamicsGraph(opt_setting, gravity, planar_axis)

        graph = graph_builder.dynamicsFactorGraph(robot, 0, None, None)
        joint_angles = np.array([0, 0, 0, 0])
        joint_vels = np.array([0, 0, 0, 0])
        torques = np.array([1, 0, 0, 0])
        prior_graph = graph_builder.forwardDynamicsPriors(
            robot, 0, joint_angles, joint_vels, torques)
        graph.push_back(prior_graph)

        # construct init values and solve
        init_values = gtd.ZeroValues(robot, 0, 0)
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, init_values)
        result = optimizer.optimize()

        a1_key = gtd.JointAccelKey(1, 0).key()
        a1 = result.atDouble(a1_key)
        self.assertAlmostEqual(a1, 0.125, 5) # regression. Show work!


if __name__ == "__main__":
    unittest.main()
