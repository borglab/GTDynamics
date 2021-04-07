"""
 * GTDynamics Copyright 2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_print.py
 * @brief Test printing with DynamicsSymbol.
 * @author Gerry Chen
"""

import unittest

import gtdynamics as gtd
import gtsam

class Print(unittest.TestCase):
    def test_values(self):
        """Checks that printing Values uses the GTDKeyFormatter instead of gtsam's default"""
        v = gtd.Values()
        gtd.InsertJointAngleDouble(v, 0, 1, 2)
        self.assertTrue('q(0)1' in v.__repr__())

    def test_nonlinear_factor_graph(self):
        """Checks that printing NonlinearFactorGraph uses the GTDKeyFormatter"""
        fg = gtd.NonlinearFactorGraph()
        fg.push_back(gtd.MinTorqueFactor(gtd.internal.TorqueKey(0, 0).key(),
                                         gtsam.noiseModel.Unit.Create(1)))
        self.assertTrue('T(0)0' in fg.__repr__())

if __name__ == "__main__":
    unittest.main()
