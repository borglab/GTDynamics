"""RosRobot (ros2/scate_ur10e_hw/executor_node.py) against stand-in ROS modules.

ROS is not needed: rclpy and the message packages are replaced by minimal
stand-ins, so this checks the adapter's own logic (joint order by name,
status gating, command messages), not ROS itself.
"""

import os
import sys
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'ros2', 'scate_ur10e_hw'))


class Message:
    def __init__(self, **fields):
        self.__dict__.update(fields)


def install_stand_ins():
    """Stand-ins for the ROS modules, unless ROS 2 is sourced (then the real ones are used)."""
    try:
        import rclpy  # noqa: F401
        import sensor_msgs.msg  # noqa: F401
        import std_msgs.msg  # noqa: F401
        return
    except ImportError:
        pass
    modules = {}
    for name in ('rclpy', 'rclpy.executors', 'rclpy.node', 'rclpy.qos', 'sensor_msgs',
                 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg', 'rcl_interfaces', 'rcl_interfaces.msg'):
        modules[name] = sys.modules[name] = types.ModuleType(name)
    modules['rclpy.executors'].ExternalShutdownException = RuntimeError
    modules['rclpy.executors'].MultiThreadedExecutor = object
    modules['rclpy.node'].Node = object
    modules['rclpy.qos'].QoSProfile = lambda **kw: kw
    modules['rclpy.qos'].DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    modules['sensor_msgs.msg'].JointState = Message
    modules['std_msgs.msg'].Float64MultiArray = Message
    modules['std_msgs.msg'].String = Message
    modules['rcl_interfaces.msg'].ParameterDescriptor = Message


class FakeNode:
    def __init__(self):
        self.published, self.subscriptions = [], {}

    def create_publisher(self, kind, topic, qos):
        node = self
        return types.SimpleNamespace(publish=lambda message: node.published.append((topic, message)))

    def create_subscription(self, kind, topic, callback, qos):
        self.subscriptions[topic] = callback


class RosRobotTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        install_stand_ins()
        from scate_ur10e_hw import executor_node
        cls.module = executor_node

    def test_state_status_and_commands(self):
        node = FakeNode()
        robot = self.module.RosRobot(node)
        self.assertIsNone(robot.state())
        self.assertIn('no /ur/status', robot.status())
        # Joints arrive in any order (and with the rail first); they are read by name.
        names = ['wrist_3_joint', 'linear_rail_joint', 'shoulder_pan_joint', 'shoulder_lift_joint',
                 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint']
        node.subscriptions['/ur/joint_states'](Message(
            name=names, position=[6.0, 9.0, 1.0, 2.0, 3.0, 4.0, 5.0],
            velocity=[0.6, 0.9, 0.1, 0.2, 0.3, 0.4, 0.5]))
        state = robot.state()
        self.assertEqual(state.q, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        self.assertEqual(state.v, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        node.subscriptions['/ur/status'](Message(data='shadow mode (enable_commands:=false)'))
        self.assertTrue(robot.status().startswith('shadow'))
        self.assertEqual(self.module.RosRobot(FakeNode(), require_ready=False).status(),
                         'no /ur/status received (is ur_bridge running?)')
        node.subscriptions['/ur/status'](Message(data='ready'))
        self.assertEqual(robot.status(), 'ready')
        robot.command([0.1] * 6)
        robot.stop()
        self.assertEqual([topic for topic, _ in node.published], ['/ur/joint_velocity_command'] * 2)
        self.assertEqual(node.published[-1][1].data, [0.0] * 6)
        # A message missing a joint is ignored.
        node.subscriptions['/ur/joint_states'](Message(name=names[:3], position=[0, 0, 0], velocity=[]))
        self.assertEqual(robot.state().q, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_shadow_mode_does_not_wait_for_ready(self):
        node = FakeNode()
        robot = self.module.RosRobot(node, require_ready=False)
        node.subscriptions['/ur/status'](Message(data='shadow mode (enable_commands:=false)'))
        self.assertEqual(robot.status(), 'ready')


if __name__ == '__main__':
    unittest.main()
