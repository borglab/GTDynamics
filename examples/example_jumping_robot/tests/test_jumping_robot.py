"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_jumping_robot.py
 * @brief Unit test for jumping robot.
 * @author Yetong Zhang
"""

import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 

from src.jumping_robot import Actuator, JumpingRobot
from src.robot_graph_builder import RobotGraphBuilder
from src.actuation_graph_builder import ActuationGraphBuilder
from src.jr_graph_builder import JRGraphBuilder
from src.jr_simulator import JRSimulator
import unittest

import gtsam
import gtdynamics as gtd
import numpy as np

class TestJumpingRobot(unittest.TestCase):
    """ Tests for jumping robot. """
    def __init__(self, *args, **kwargs):
        """ Constructor. """
        super(TestJumpingRobot, self).__init__(*args, **kwargs)
        self.yaml_file_path = "examples/example_jumping_robot/yaml/robot_config.yaml"
        self.init_config = JumpingRobot.create_init_config()
        # self.jr_simulator = JRSimulator(self.yaml_file_path, self.init_config)
        self.jr = JumpingRobot(self.yaml_file_path, self.init_config)

    def test_links_joints(self):
        """ Test creating jumping robot. """
        
        self.assertEqual(self.jr.robot.numLinks(), 6)
        self.assertEqual(self.jr.robot.numJoints(), 6)

    def test_forward_kinematics(self):
        """ Test forward kinematics of jumping robot. """
        self.jr = JumpingRobot(self.yaml_file_path, self.init_config)

        values = gtsam.Values()
        k = 0
        theta = 0.
        qs = [-theta, 2 * theta, -theta, -theta, 2*theta, -theta]
        vs = [0., 0., 0., 0., 0., 0.]
        
        for joint in self.jr.robot.joints():
            j = joint.id()
            gtd.InsertJointAngleDouble(values, j, k, qs[j-1])
            gtd.InsertJointVelDouble(values, j, k , vs[j-1])

        fk_results = self.jr.robot.forwardKinematics(values, k, "torso")
        for link in self.jr.robot.links():
            i = link.id()
            pose = gtd.Pose(fk_results, i, k)
            print(link.name())
            print(pose)

        from src.jr_visualizer import visualize_jr
        visualize_jr(fk_results, self.jr, k)
        # print(jr.robot)
        # print(torso_pose)

    # def test_robot_forward_dynamics(self):
    #     """ Test forward dynamics of robot frame """
    #     # specify joint angles, joint vels, torques
    #     k = 0
        
    #     theta = np.pi/3
    #     qs = [-theta, 2 * theta, -theta, -theta, 2*theta, -theta]
    #     vs = [0., 0., 0., 0., 0., 0.]
    #     torques = [0., 0., 0., 0., 0., 0.]

    #     values = gtsam.Values()
    #     for acutator in self.jr_simulator.jr.actuators:
    #         j = acutator.j
    #         gtd.InsertTorqueDouble(values, j, k, 0.0)
    #         gtd.InsertJointAngleDouble(values, j, k, qs[j-1])
    #         gtd.InsertJointVelDouble(values, j, k ,0.0)
        
    #     i = self.jr_simulator.jr.robot.link("torso").id()
    #     torso_pose = gtsam.Pose3(gtsam.Rot3.Rx(np.pi), gtsam.Point3(0, 0, 0.55))
    #     gtd.InsertPose(values, i, k, torso_pose)
    #     gtd.InsertTwist(values, i, k, np.zeros(6))

    #     self.jr_simulator.step_robot_dynamics(k, values)
    #     joint_accels = gtd.DynamicsGraph.jointAccels(self.jr_simulator.jr.robot, values, k)
    #     np.testing.assert_array_almost_equal(joint_accels, np.zeros(6), decimal=5)

    # def test_actuation_forward_dynamics(self):
    #     """ test forward dynamics of actuator """
    #     controls = JumpingRobot.create_controls()
    #     values = self.jr_simulator.init_config_values(controls)
    #     k=0
    #     curr_time = 1.0
    #     self.jr_simulator.step_actuator_dynamics(k, values, curr_time)

    #     torques = []
    #     for actuator in jr_simulator.jr.actuators:
    #         j = actuator.j
    #         torque = gtd.TorqueDouble(values, j, k)
    #         torques.append(torque)
    #     print(torques)
    #     # TODO(yetong): check torques, pressures, etc



if __name__ == "__main__":
    unittest.main()
