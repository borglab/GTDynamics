#!/usr/bin/env python3
"""ROS 2 node 'ur_bridge' (fake): ur_bridge's topics around a simulated arm.

For rehearsing the ROS pipeline with no robot: it takes the same name and
topics as the real bridge (/ur/joint_velocity_command in, /ur/joint_states
and a latched /ur/status out) and moves scate_hw's SimRobot, which follows
commands with a short lag, stops when they stop, and faults above
max_joint_speed like the real bridge.

Parameters: scate_hw_path (as executor_node), q0 (six rad, the arm at power-on;
empty = the rig experiment's start), max_joint_speed, publish_hz.
"""

import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from scate_ur10e_hw.configurations import JOINT_NAMES
from scate_ur10e_hw.ros_params import declare

ARM_JOINT_NAMES = list(JOINT_NAMES[1:])
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
RIG_START = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]


class FakeUR(Node):

    def __init__(self):
        super().__init__('ur_bridge')
        root = declare(self, 'gtdynamics', os.environ.get('GTDYNAMICS_DIR',
                                                          os.path.expanduser('~/dev/GTDynamics')))
        hw_path = declare(self, 'scate_hw_path', '') or os.path.join(
            root, 'examples', 'example_scate_ur10e', 'hardware')
        if hw_path not in sys.path:
            sys.path.insert(0, hw_path)
        from scate_hw.robots import SimRobot
        q0 = declare(self, 'q0', '')
        q0 = [float(x) for x in q0.replace(',', ' ').split()] if q0 else RIG_START
        self.arm = SimRobot(q0, max_joint_speed=declare(self, 'max_joint_speed', 0.6))
        self.state_pub = self.create_publisher(JointState, '/ur/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/ur/status', LATCHED)
        self.create_subscription(Float64MultiArray, '/ur/joint_velocity_command', self.on_command, 10)
        self.create_timer(1.0 / declare(self, 'publish_hz', 125.0), self.publish)
        self.last_status = None
        self.get_logger().info('fake ur_bridge: simulated arm, nothing leaves this machine')

    def on_command(self, message):
        self.arm.command(list(message.data))

    def publish(self):
        state = self.arm.state()
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ARM_JOINT_NAMES
        message.position = state.q
        message.velocity = state.v
        self.state_pub.publish(message)
        status = self.arm.status()
        if status != self.last_status:
            self.status_pub.publish(String(data=status))
            self.last_status = status


def main(args=None):
    rclpy.init(args=args)
    node = FakeUR()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.arm.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
