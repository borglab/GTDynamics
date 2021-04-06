import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 

from src.jumping_robot import Actuator, JumpingRobot
from src.jr_graphs import JRGraphBuilder
from src.jr_simulator import JRSimulator
import unittest

import gtsam
import gtdynamics as gtd
import numpy as np

class TestJumpingRobot(unittest.TestCase):
    """ Tests for jumping robot. """
    def __init__(self, *args, **kwargs):
        """ Create a jumping robot
        """
        super(TestJumpingRobot, self).__init__(*args, **kwargs)
        self.yaml_file_path = "examples/example_jumping_robot/yaml/robot_config.yaml"
        self.init_config = JumpingRobot.create_init_config()
        self.jr_simulator = JRSimulator(self.yaml_file_path, self.init_config)

    def test_jumping_robot(self):
        """ Test creating jumping robot """
        jr = JumpingRobot(self.yaml_file_path, self.init_config)
        self.assertEqual(jr.robot.numLinks(), 6)
        self.assertEqual(jr.robot.numJoints(), 6)

    def test_robot_dynamics(self):
        """ test forward dynamics of robot frame """
        # specify joint angles, joint vels, torques
        k = 0
        phase = 0
        values = gtsam.Values()
        for acutator in self.jr_simulator.jr.actuators:
            j = acutator.j
            gtd.InsertTorqueDouble(values, j, k, 0.0)
            gtd.InsertJointAngleDouble(values, j, k, 0.0)
            gtd.InsertJointVelDouble(values, j, k ,0.0)
        self.jr_simulator.step_robot_dynamics(k, values, phase)
        joint_accels = gtd.DynamicsGraph.jointAccels(self.jr_simulator.jr.robot, values, k)
        np.testing.assert_array_almost_equal(joint_accels, np.zeros(6), decimal=5)

    def test_actuator_dynamics(self):
        """ test forward dynamics of actuator """
        controls = JumpingRobot.create_controls()
        values = self.jr_simulator.init_config_values(controls)
        k=0
        curr_time = 1.0
        self.jr_simulator.step_actuator_dynamics(k, values, curr_time)

        torques = []
        for actuator in jr_simulator.jr.actuators:
            j = actuator.j
            torque = gtd.TorqueDouble(values, j, k)
            torques.append(torque)
        print(torques)
        # TODO(yetong): check torques, pressures, etc


    def test_simulation(self):
        return
    
    def test_optimization(self):
        return

if __name__ == "__main__":
    unittest.main()
