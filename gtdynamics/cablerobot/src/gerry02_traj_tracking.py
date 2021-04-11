"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  gerry01_planar_tracking.py
@brief quick test of open-loop trajectory tracking for planar cdpr.
@author Frank Dellaert
@author Gerry Chen
"""

import gtdynamics as gtd
import gtsam
import matplotlib.pyplot as plt
import numpy as np
from gtsam import Pose3, Rot3

from cdpr_planar import Cdpr
from cdpr_controller_ilqr import CdprControllerIlqr
from cdpr_planar_sim import CdprSimulator
from paint_parse import ParseFile

import cProfile

def main(fname='data/iros_logo_2.h'):
    isPaints, colorinds, colorpalette, traj = ParseFile(fname)
    def paintString(isPaint, colori):
        return '{:d}, {:d}, {:d}'.format(*colorpalette[colori]) if isPaint else 'paint off'
    i = 0
    print(traj.shape)
    for isPaint, colori, point in zip(isPaints, colorinds, traj):
        print('{:5.2f}, {:5.2f}\t-\t{}'.format(*point, paintString(isPaint, colori)))
        if i > 100:
            break
        i += 1

if __name__ == '__main__':
    main()