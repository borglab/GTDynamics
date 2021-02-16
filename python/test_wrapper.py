import gtsam
import gtdynamics as gtd
import numpy as np


# load example robot
SDF_PATH = "sdfs/"
simple_rr = gtd.CreateRobotFromFile(SDF_PATH + "/test/simple_rr.sdf", "simple_rr_sdf")

# check links and joints
assert simple_rr.numLinks() == 3
assert simple_rr.numJoints() == 2

# # forward dynamics test
graph_builder = gtd.DynamicsGraph()
# gravity = np.array([0, 0, 0])
# planar_axis = np.array([1, 0, 0])
t = int(0)
robot = gtd.Robot()
graph = graph_builder.dynamicsFactorGraph(simple_rr, t, None, None, None, None)
print(graph)