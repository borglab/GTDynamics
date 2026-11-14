#!/usr/bin/env python3
"""ROS 2 node 'scate_executor': the SCATE hardware executor on ur_bridge.

The executor itself (scate_hw, plain Python) is the one the laptop
simulation uses; this node gives it a RosRobot:

  state     /ur/joint_states (sensor_msgs/JointState, six arm joints, from ur_bridge)
  status    /ur/status (std_msgs/String, 'ready' or why not, latched)
  command   /ur/joint_velocity_command (std_msgs/Float64MultiArray, six rad/s)

ur_bridge only accepts commands from nodes in its allowed_command_nodes,
which the launch file sets to this node's name. The node also serves the
executor's HTTP control API (default 127.0.0.1:8765) for the app.

Parameters:
  gtdynamics    GTDynamics checkout (default $GTDYNAMICS_DIR or ~/dev/GTDynamics);
                the three paths below default to places inside it
  scate_bin     scate_hardware executable (build/examples/example_scate_ur10e/)
  description   robot description (apps/scate/ur10e-reactive/public/robots/ur10e.json)
  scate_hw_path folder holding the scate_hw package (examples/example_scate_ur10e/hardware)
  host, port    control API address
  kp, rate_hz, command_cap, tracking_abort   executor settings (scate_hw/safety.py)
  require_ready refuse to begin unless /ur/status is 'ready' (false only for shadow mode)
"""

import os
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState as JointStateMsg
from std_msgs.msg import Float64MultiArray, String

from scate_ur10e_hw.configurations import JOINT_NAMES
from scate_ur10e_hw.ros_params import declare

ARM_JOINT_NAMES = list(JOINT_NAMES[1:])
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def import_scate_hw(path):
    if path and path not in sys.path:
        sys.path.insert(0, path)
    import scate_hw.executor  # noqa: F401
    import scate_hw.robots  # noqa: F401
    import scate_hw.safety  # noqa: F401
    import scate_hw.server  # noqa: F401
    return sys.modules['scate_hw']


class RosRobot:
    """scate_hw Robot interface over ur_bridge's topics."""

    kind = 'ros'

    def __init__(self, node, require_ready=True):
        from scate_hw.robots import JointState
        self.JointState = JointState
        self.node = node
        self.require_ready = require_ready
        self.lock = threading.Lock()
        self.latest = None
        self.bridge_status = None
        self.publisher = node.create_publisher(Float64MultiArray, '/ur/joint_velocity_command', 10)
        node.create_subscription(JointStateMsg, '/ur/joint_states', self.on_state, 10)
        node.create_subscription(String, '/ur/status', self.on_status, LATCHED)

    def on_state(self, message):
        try:
            index = [list(message.name).index(name) for name in ARM_JOINT_NAMES]
        except ValueError:
            return
        q = [message.position[i] for i in index]
        v = [message.velocity[i] for i in index] if len(message.velocity) == len(message.name) \
            else [0.0] * len(q)
        with self.lock:
            # Received time on this machine's monotonic clock: staleness is local.
            self.latest = self.JointState(time.monotonic(), q, v)

    def on_status(self, message):
        with self.lock:
            self.bridge_status = message.data

    def state(self):
        with self.lock:
            return self.latest

    def status(self):
        with self.lock:
            status = self.bridge_status
        if status is None:
            return 'no /ur/status received (is ur_bridge running?)'
        if status == 'ready' or not self.require_ready:
            return 'ready'
        return status

    def command(self, velocity):
        self.publisher.publish(Float64MultiArray(data=[float(x) for x in velocity]))

    def stop(self):
        self.command([0.0] * len(ARM_JOINT_NAMES))

    def close(self):
        pass


class ScateExecutorNode(Node):

    def __init__(self):
        super().__init__('scate_executor')
        root = declare(self, 'gtdynamics', os.environ.get('GTDYNAMICS_DIR',
                                                          os.path.expanduser('~/dev/GTDynamics')))
        scate_bin = declare(self, 'scate_bin', '') or os.path.join(
            root, 'build', 'examples', 'example_scate_ur10e', 'scate_hardware')
        description = declare(self, 'description', '') or os.path.join(
            root, 'apps', 'scate', 'ur10e-reactive', 'public', 'robots', 'ur10e.json')
        hw_path = declare(self, 'scate_hw_path', '') or os.path.join(
            root, 'examples', 'example_scate_ur10e', 'hardware')
        host = declare(self, 'host', '127.0.0.1')
        port = declare(self, 'port', 8765)
        require_ready = declare(self, 'require_ready', True)

        scate_hw = import_scate_hw(hw_path)
        settings = scate_hw.safety.Settings(
            kp=declare(self, 'kp', 1.5),
            rate_hz=declare(self, 'rate_hz', 50.0),
            command_cap=declare(self, 'command_cap', 0.55),
            tracking_abort=declare(self, 'tracking_abort', 0.08))
        self.robot = RosRobot(self, require_ready)
        self.executor = scate_hw.executor.Executor(self.robot, scate_bin, description, settings)
        self.server = scate_hw.server.make_server(self.executor, host, port)
        scate_hw.server.serve_in_thread(self.server)
        self.get_logger().info(f'control API on http://{host}:{port}; planner {scate_bin}; '
                               f'description {description}')

    def shutdown(self):
        self.executor.close()
        self.server.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ScateExecutorNode()
    spinner = MultiThreadedExecutor()
    spinner.add_node(node)
    try:
        spinner.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
