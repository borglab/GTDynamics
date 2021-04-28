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

        results = gtd.optimize_LMQR(graph, init_values)

        # params = gtsam.LevenbergMarquardtParams()
        # params.setVerbosityLM("SUMMARY")
        # params.setLinearSolverType("MULTIFRONTAL_QR")
        # # params.setLinearSolverType("SEQUENTIAL_QR")
        # optimizer = gtsam.LevenbergMarquardtOptimizer(
        #     graph, init_values, params)
        # results = optimizer.optimize()

    # def test_solve_simple(self):
    #     graph = gtsam.NonlinearFactorGraph()
    #     values = gtsam.Values()
    #     model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)

    #     graph.add(gtd.PriorFactorDouble(1, 0.1, model))
    #     graph.add(gtd.PriorFactorDouble(2, 0.1, model))
    #     values.insertDouble(1, 0.1)
    #     values.insertDouble(2, 0.1)

    #     params = gtsam.LevenbergMarquardtParams()
    #     params.setVerbosityLM("SUMMARY")
    #     params.setLinearSolverType("MULTIFRONTAL_QR")
    #     # params.setLinearSolverType("SEQUENTIAL_QR")
    #     optimizer = gtsam.LevenbergMarquardtOptimizer(
    #         graph, values, params)
    #     results = optimizer.optimize()

if __name__ == "__main__":
    unittest.main()
