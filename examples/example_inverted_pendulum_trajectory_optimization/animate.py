"""Run kinematic motion planning using GTDynamics outputs."""
from typing import Dict

import time

# import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import pandas as pd
import numpy as np

# pylint: disable=I1101, C0103

_ = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # To load plane SDF.
p.setGravity(0, 0, -9.8)
quad_id = p.loadURDF("inverted_pendulum.urdf", [0, 0, 0], [0, 0, 0, 1], False,
                     True)

while True:
    p.stepSimulation()
    time.sleep(1. / 240)

p.disconnect()
