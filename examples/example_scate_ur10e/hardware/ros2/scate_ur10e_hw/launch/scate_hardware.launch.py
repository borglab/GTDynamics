"""SCATE hardware run on the UR10e: ur_bridge (or a fake arm) + the executor.

  ros2 launch scate_ur10e_hw scate_hardware.launch.py                    # fake arm
  ros2 launch scate_ur10e_hw scate_hardware.launch.py hardware:=real     # shadow mode
  ros2 launch scate_ur10e_hw scate_hardware.launch.py hardware:=real enable_commands:=true

  hardware:=fake   (default) fake_ur moves a simulated arm; nothing leaves this machine.
  hardware:=real   ur_bridge (UR10e realtime 30003, RTDE 30004). Commands are only
                   sent with enable_commands:=true; without it the bridge runs in
                   shadow mode (reads state, logs commands) and the executor is
                   told not to wait for /ur/status 'ready' -- the arm does not
                   move, so a run stops on tracking error within a second. Use
                   shadow mode to check the connections and the pre-start checks.

Then open the SCATE UR10e Base app (npm run dev in apps/) and use its
Hardware experiment panel; it talks to the executor at 127.0.0.1:8765.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ARGUMENTS = (
    ('hardware', 'fake', 'fake (simulated arm) or real (ur_bridge)'),
    ('enable_commands', 'false', 'real only: send commands to the arm (false = shadow mode)'),
    ('gtdynamics', os.environ.get('GTDYNAMICS_DIR', os.path.expanduser('~/dev/GTDynamics')),
     'GTDynamics checkout: build/ holds scate_hardware, apps/ the robot description'),
    ('ur_ip', '192.168.7.8', 'UR10e controller'),
    ('ur_stream', 'rtde', 'rtde, program or lines (see ur_bridge)'),
    ('ur_host_ip', '', "this PC's address on the robot network; empty = detect"),
    ('max_joint_speed', '0.6', 'rad/s: ur_bridge faults above this; above the executor cap 0.55'),
    ('speedj_acceleration', '2.0', 'rad/s^2: speedj acceleration, above the planner limit'),
    ('kp', '1.5', '1/s, executor position correction gain'),
    ('rate_hz', '50.0', 'executor command rate'),
    ('tracking_abort', '0.08', 'rad: stop when a joint lags the plan by more'),
    ('port', '8765', 'executor control API port (127.0.0.1)'),
)


def generate_launch_description():
    config = {name: LaunchConfiguration(name) for name, _, _ in ARGUMENTS}
    fake = LaunchConfigurationEquals('hardware', 'fake')
    real = LaunchConfigurationEquals('hardware', 'real')
    shadow = PythonExpression(["'", config['hardware'], "' == 'real' and '",
                               config['enable_commands'], "'.lower() != 'true'"])
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=text)
          for name, default, text in ARGUMENTS],
        Node(package='scate_ur10e_hw', executable='fake_ur', name='ur_bridge', output='screen',
             condition=fake, parameters=[{'gtdynamics': config['gtdynamics'],
                                          'max_joint_speed': config['max_joint_speed']}]),
        Node(package='scate_ur10e_hw', executable='ur_bridge', name='ur_bridge', output='screen',
             condition=real, parameters=[{
                 'robot_ip': config['ur_ip'],
                 'enable_commands': config['enable_commands'],
                 'stream': config['ur_stream'],
                 'host_ip': config['ur_host_ip'],
                 'max_joint_speed': config['max_joint_speed'],
                 'acceleration': config['speedj_acceleration'],
                 'allowed_command_nodes': 'scate_executor',
             }]),
        Node(package='scate_ur10e_hw', executable='scate_executor', name='scate_executor',
             output='screen', parameters=[{
                 'gtdynamics': config['gtdynamics'],
                 'kp': config['kp'],
                 'rate_hz': config['rate_hz'],
                 'tracking_abort': config['tracking_abort'],
                 'port': config['port'],
                 'require_ready': PythonExpression(['not (', shadow, ')']),
             }]),
    ])
