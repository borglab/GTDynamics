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
from io import StringIO
from unittest.mock import patch

import gtsam

import gtdynamics as gtd


class TestPrint(unittest.TestCase):
    """Test printing of keys."""
    def test_values(self):
        """Checks that printing Values uses the GTDKeyFormatter instead of gtsam's default"""
        v = gtd.Values()
        gtd.InsertJointAngleDouble(v, 0, 1, 2)
        self.assertTrue('q(0)1' in v.__repr__())

    def test_nonlinear_factor_graph(self):
        """Checks that printing NonlinearFactorGraph uses the GTDKeyFormatter"""
        fg = gtd.NonlinearFactorGraph()
        fg.push_back(
            gtd.MinTorqueFactor(
                gtd.internal.TorqueKey(0, 0).key(),
                gtsam.noiseModel.Unit.Create(1)))
        self.assertTrue('T(0)0' in fg.__repr__())

    def test_key_formatter(self):
        """Tests print method with various key formatters"""
        torqueKey = gtd.internal.TorqueKey(0, 0).key()
        factor = gtd.MinTorqueFactor(torqueKey,
                                     gtsam.noiseModel.Unit.Create(1))
        with patch('sys.stdout', new=StringIO()) as fake_out:
            factor.print('factor: ', gtd.GTDKeyFormatter)
            self.assertTrue('factor: min torque factor' in fake_out.getvalue())
            self.assertTrue('keys = { T(0)0 }' in fake_out.getvalue())

        def myKeyFormatter(key):
            return 'this is my key formatter {}'.format(key)

        with patch('sys.stdout', new=StringIO()) as fake_out:
            factor.print('factor: ', myKeyFormatter)
            self.assertTrue('factor: min torque factor' in fake_out.getvalue())
            self.assertTrue('keys = {{ this is my key formatter {} }}'.format(
                torqueKey) in fake_out.getvalue())


if __name__ == "__main__":
    unittest.main()
