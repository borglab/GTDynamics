"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  cdpr_controller.py
@brief Optimal controller for a cable robot.  Solved by creating a factor graph and adding state
objectives and control costs, then optimizing
@author Frank Dellaert
@author Gerry Chen
"""

import gtsam
import gtdynamics as gtd
import numpy as np
import utils

class CdprControllerBase:
    """Interface for cable robot controllers
    """
    @property
    def update(self, values, t):
        """gives the new control input given current measurements

        Args:
            values (gtsam.Values): values object will contain at least the current Pose and Twist,
            but should often also include the current joint angles and velocities
            t (int): The current time index (discrete time index)

        Returns:
            gtsam.Values: A values object which contains the joint torques for this time step.

        Raises:
            NotImplementedError: Derived classes must override this function
        """
        raise NotImplementedError("CdprControllers need to implement the `update` function")
