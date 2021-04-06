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
    """Create a jumping robot and test it."""
    def __init__(self, *args, **kwargs):
        super(TestJumpingRobot, self).__init__(*args, **kwargs)
        self.yaml_file_path = "examples/example_jumping_robot/yaml/robot_config.yaml"
        self.jr = JumpingRobot(self.yaml_file_path, JumpingRobot.create_init_config())

    def test_jumping_robot(self):
        self.assertEqual(self.jr.robot.numLinks(), 6)
        self.assertEqual(self.jr.robot.numJoints(), 6)

    def test_forward_kinematics(self):
        init_config = JumpingRobot.create_init_config()
        controls = JumpingRobot.create_controls()
        jr_simulator = JRSimulator(self.yaml_file_path, init_config, controls)

        # specify joint angles, joint vels, torques
        k = 0
        phase = 0
        values = gtsam.Values()
        for acutator in jr_simulator.jr.actuators:
            j = acutator.j
            gtd.InsertTorqueDouble(values, j, k, 0.0)
            gtd.InsertJointAngleDouble(values, j, k, 0.0)
            gtd.InsertJointVelDouble(values, j, k ,0.0)
        jr_simulator.step_robot_dynamics(k, values, phase)
        joint_accels = gtd.DynamicsGraph.jointAccels(jr_simulator.jr.robot, values, k)
        np.testing.assert_array_almost_equal(joint_accels, np.zeros(6), decimal=5)

    def test_actuator_dynamics(self):
        init_config = JumpingRobot.create_init_config()
        controls = JumpingRobot.create_controls()
        jr_simulator = JRSimulator(self.yaml_file_path, init_config, controls)
        
        values = gtsam.Values()
        V_s_key = Actuator.SourceVolumeKey()
        values.insertDouble(V_s_key, jr_simulator.values.atDouble(V_s_key))
        k = 0
        curr_time = 1.0
        m_s_key = Actuator.SourceMassKey(k)
        values.insertDouble(m_s_key, 1e-6)
        for acutator in jr_simulator.jr.actuators:
            j = acutator.j
            gtd.InsertJointAngleDouble(values, j, k, 0.0)
            gtd.InsertJointVelDouble(values, j, k, 0.0)
            m_a_key = Actuator.MassKey(j, k)
            values.insertDouble(m_a_key, 1e-5)
            To_key = Actuator.ValveOpenTimeKey(j)
            Tc_key = Actuator.ValveCloseTimeKey(j)
            values.insertDouble(To_key, jr_simulator.values.atDouble(To_key))
            values.insertDouble(Tc_key, jr_simulator.values.atDouble(Tc_key))
        jr_simulator.step_actuator_dynamics(k, values, curr_time)

        P_s_key = Actuator.SourcePressureKey(k)
        P_s = values.atDouble(P_s_key)

        torques = []
        for actuator in jr_simulator.jr.actuators:
            j = actuator.j
            torque = gtd.TorqueDouble(values, j, k)
            torques.append(torque)
        self.assertAlmostEqual(torques, [0, 0, 0, 0], 5)


    def test_simulation(self):
        return
    
    def test_optimization(self):
        return

if __name__ == "__main__":
    unittest.main()
