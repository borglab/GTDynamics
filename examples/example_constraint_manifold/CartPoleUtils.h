/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  CartPoleUtils.h
 * @brief Utility functions for cart pole experiments.
 * @author Yetong Zhang
 */

#pragma once

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtdynamics/manifold/ManifoldOptimizerType1.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtdynamics/optimizer/OptimizationBenchmark.h>

namespace gtsam{

class CartPole {
public:
  // Cart-pole dynamic planning scenario setting.
  gtdynamics::Robot robot =
      gtdynamics::CreateRobotFromFile(gtdynamics::kUrdfPath +
                                      std::string("cart_pole.urdf"))
          .fixLink("l0");
  int j0_id = robot.joint("j0")->id(), j1_id = robot.joint("j1")->id();
  Vector6 X_i = Vector6::Constant(6, 0);
  Vector6 X_T = (Vector(6) << 1, 0, 0, M_PI, 0, 0).finished();
  Vector3 gravity = Vector3(0, 0, -9.8);
  gtdynamics::CollocationScheme collocation_scheme = gtdynamics::CollocationScheme::Trapezoidal;

  double sigma_dynamics = 1e-5;
  double sigma_prior = 1e-7;
  double sigma_collocation = 1e-5;
  double sigma_pos_objective = 1e-5;
  double sigma_objectives = 5e-3;
  double sigma_min_torque = 2e1;
  SharedNoiseModel prior_model = noiseModel::Isotropic::Sigma(1, sigma_prior);           // prior constraints.
  SharedNoiseModel pos_objectives_model = noiseModel::Isotropic::Sigma(1, sigma_pos_objective);  // Pos objectives.
  SharedNoiseModel objectives_model = noiseModel::Isotropic::Sigma(1, sigma_objectives);  // Additional objectives.
  SharedNoiseModel control_model = noiseModel::Isotropic::Sigma(1, sigma_min_torque);       // Controls.

  gtdynamics::OptimizerSetting opt = getOptSetting();
  gtdynamics::DynamicsGraph graph_builder = gtdynamics::DynamicsGraph(opt, gravity);

public:
  /// Kinodynamic constraints
  NonlinearFactorGraph getDynamicsGraph(size_t num_steps) const;

  /// Set the 2nd joint to be unactuated
  NonlinearFactorGraph getUnactuatedGraph(size_t num_steps) const;

  /** Cost for planning, includes terminal state objectives, control costs.*/
  NonlinearFactorGraph minTorqueCosts(size_t num_steps) const;

  /// Priors on initial state.
  NonlinearFactorGraph initStateGraph() const;

  /// Priors on final state.
  NonlinearFactorGraph finalStateGraph(size_t num_steps) const;

  /** Collocation between each step, benchmark will be made by either treating
  * collocation as costs or constraints. */
  NonlinearFactorGraph getCollocation(size_t num_steps, double dt);

  /// Initial values for trajectory optimization.
  Values getInitValues(size_t num_steps, std::string option = "zero");

  /// Export trajectory to external file.
  void exprotTrajectory(const Values& results, size_t num_steps, double dt, std::string file_name = "traj.csv") const;

  /// Print joint angles for all steps.
  void printJointAngles(const Values& values, size_t num_steps) const;

  /// Return function that select basis keys for constraint manifolds.
  BasisKeyFunc getBasisKeyFunc(bool unactuated_as_constraint = true) const;

protected:
  /// Default optimizer setting.
  gtdynamics::OptimizerSetting getOptSetting() const;

  /// Initial values seet as rest pose for all steps.
  Values getInitValuesZero(size_t num_steps);

  /// Initial values seet as rest pose for all steps.
  Values getInitValuesInterp(size_t num_steps);

  /// Infeasible initial values.
  Values getInitValuesInfeasible(size_t num_steps);

};

}
