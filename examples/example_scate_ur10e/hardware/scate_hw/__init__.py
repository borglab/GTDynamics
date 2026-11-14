"""Executor of SCATE hardware runs on the UR10e.

The planner runs in its own process (scate_hardware, C++). This package
drives it: it streams the arm's measured state to the planner at each state
time, receives every new plan, checks it, and turns it into joint velocity
commands for the arm, the way plan_executor in UR10e-Trajectory-Validation
does (velocity feed-forward plus a position correction).

Pure Python standard library, so it runs on a Mac without ROS:

  kinematics  forward kinematics and wall/floor clearance from the live
              app's robot description (robots/ur10e.json)
  plan        a plan from the planner, evaluated at any time
  request     the bits of a live-app request the executor needs
  safety      plan checks and the command law
  planner     the scate_hardware child process
  robots      the arm interface and a simulated arm (SimRobot)
  executor    the experiment state machine and control loop
  server      the HTTP control API the app talks to

The ROS 2 package in ../ros2 adds RosRobot (ur_bridge topics) on top.
"""

__all__ = ['kinematics', 'plan', 'request', 'safety', 'planner', 'robots', 'executor', 'server']
