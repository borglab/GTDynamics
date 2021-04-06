import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 

from src.jumping_robot import Actuator, JumpingRobot, JRGraphBuilder, JRSimulator
import unittest

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

    def test_actuator_dynamics(self):
        return
    
    def test_simulation(self):
        return
    
    def test_optimization(self):
        return

if __name__ == "__main__":
    unittest.main()
