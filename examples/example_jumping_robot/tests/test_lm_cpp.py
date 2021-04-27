"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_lm_cpp.py
 * @brief Unit test that expose issue when using LM with QR option.
 * @author Yetong Zhang
"""


import gtsam
import gtdynamics as gtd
import unittest


class TestJRGraphBuilder(unittest.TestCase):
    def test_solving_actuator_cpp(self):
        actuator = gtd.PneumaticActuator()
        k = 0
        prior_values = actuator.priorValues()
        graph = gtsam.NonlinearFactorGraph()
        graph.push_back(actuator.actuatorFactorGraph(k))
        graph.push_back(actuator.actuatorPriorGraph(k, prior_values))

        init_values = gtsam.Values()
        init_values.insert(actuator.actuatorInitValues(k, prior_values))

        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosityLM("SUMMARY")
        params.setLinearSolverType("MULTIFRONTAL_QR")
        # params.setLinearSolverType("SEQUENTIAL_QR")
        optimizer = gtsam.LevenbergMarquardtOptimizer(
            graph, init_values, params)
        results = optimizer.optimize()


if __name__ == "__main__":
    unittest.main()
