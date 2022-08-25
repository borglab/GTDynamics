/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Trajectory optimization for a legged robot with contacts.
 * @author Alejandro Escontrela
 */

#include <gtdynamics/utils/PointOnLink.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/OptimizerSetting.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/expressions.h>

#include <gtdynamics/factors/ContactDynamicsMomentFactor.h>
#include <gtdynamics/factors/ContactDynamicsFrictionConeFactor.h>
#include <gtdynamics/factors/WrenchEquivalenceFactor.h>
#include <gtdynamics/factors/WrenchFactor.h>
#include <gtdynamics/factors/TorqueFactor.h>
#include <gtdynamics/factors/PoseFactor.h>

#include "gtdynamics/factors/CollocationFactors.h"
#include "gtdynamics/factors/ContactPointFactor.h"
#include "gtdynamics/manifold/ConnectedComponent.h"
#include "gtdynamics/optimizer/ConstrainedOptimizer.h"
#include "gtdynamics/manifold/ConstraintManifold.h"
#include "gtdynamics/manifold/TspaceBasis.h"
#include "gtdynamics/utils/DynamicsSymbol.h"
#include "gtdynamics/utils/values.h"
#include "gtdynamics/optimizer/OptimizationBenchmark.h"

#include "gtdynamics/utils/QuadrupedUtils.h"

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

using namespace gtdynamics;
using namespace gtsam;

void TrajectoryOptimization() {
  /// Initialize vision60 robot
  Vision60Robot vision60;
  vision60.express_redundancy = false;

  /// Scenario
  size_t num_steps = 30;
  double dt = 0.1;
  Pose3 base_pose_init(Rot3::identity(), Point3(0, 0, vision60.nominal_height));
  Vector6 base_twist_init = Vector6::Zero();

  // std::vector<Pose3> des_poses {Pose3(Rot3::identity(), Point3(0, 0, vision60.nominal_height + 0.1)),
  // Pose3(Rot3::Rz(0.1), Point3(0, 0, vision60.nominal_height + 0.2)),
  // Pose3(Rot3::Rz(0.1), Point3(0, 0, vision60.nominal_height + 0.1))};
  
  // std::vector<double> des_poses_t {1.0, 2.0, 3.0};
  
  std::vector<Pose3> des_poses {Pose3(Rot3::identity(), Point3(0, 0, vision60.nominal_height + 0.1))};
  std::vector<double> des_poses_t {3.0};

  /// Get constraints, costs, and initial values.
  auto constraints_graph = vision60.getConstraintsGraphTrajectory(num_steps);
  auto constraints = ConstraintsFromGraph(constraints_graph);
  auto init_values = vision60.getInitValuesTrajectory(num_steps, dt, base_pose_init, des_poses, des_poses_t);
  NonlinearFactorGraph collocation_costs = vision60.collocationCosts(num_steps, dt);
  NonlinearFactorGraph boundary_costs = vision60.boundaryCosts(base_pose_init, base_twist_init, des_poses, des_poses_t, dt);
  NonlinearFactorGraph min_torque_costs = vision60.minTorqueCosts(num_steps);
  NonlinearFactorGraph friction_cone_costs = vision60.frictionConeCosts(num_steps);
  NonlinearFactorGraph costs;
  costs.add(collocation_costs);
  costs.add(boundary_costs);
  costs.add(min_torque_costs);
  costs.add(friction_cone_costs);
  auto EvaluateCosts = [=] (const Values& values) {
    std::cout << "collocation costs:\t" << collocation_costs.error(values) << "\n";
    std::cout << "boundary costs:\t" << boundary_costs.error(values) << "\n";
    std::cout << "min torque costs:\t" << min_torque_costs.error(values) << "\n";
    std::cout << "friction cone costs:\t" << friction_cone_costs.error(values) << "\n";
  };

  /// Export initial trajectory
  EvaluateCosts(init_values);
  vision60.exportTrajectory(init_values, num_steps, "/Users/yetongzhang/packages/GTDynamics/data/init_traj.csv");


  /// Construct problem
  auto problem = EqConsOptProblem(costs, constraints, init_values);
  std::ostringstream latex_os;
  LevenbergMarquardtParams lm_params;
  lm_params.minModelFidelity = 0.3;
  lm_params.setVerbosityLM("SUMMARY");
  lm_params.setlambdaUpperBound(1e10);
  // lm_params.setMaxIterations(10);

  // optimize soft constraints
  std::cout << "soft constraints:\n";
  auto soft_result =
      OptimizeSoftConstraints(problem, latex_os, lm_params, 1e2);
  EvaluateCosts(soft_result);
  vision60.exportTrajectory(soft_result, num_steps, "/Users/yetongzhang/packages/GTDynamics/data/soft_traj.csv");

  // // optimize penalty method
  // std::cout << "penalty method:\n";
  // auto penalty_result =
  //     OptimizePenaltyMethod(problem, latex_os);
  // EvaluateCosts(penalty_result);

  // // optimize augmented lagrangian
  // std::cout << "augmented lagrangian:\n";
  // auto augl_result =
  //     OptimizeAugmentedLagrangian(problem, latex_os);
  // EvaluateCosts(augl_result);

  // std::cout << "constraint manifold basis variables feasible:\n";
  auto mopt_params = DefaultMoptParamsSV();
  // mopt_params.cc_params->retract_params->setDynamics(true);
  // mopt_params.cc_params->retract_params->setProjection(true, 1.0, true);
  mopt_params.cc_params->retract_params->setUopt();
  mopt_params.cc_params->basis_key_func = vision60.getBasisKeyFunc();
  mopt_params.cc_params->retract_params->check_feasible = true;
  // auto cm_result =
  //       OptimizeConstraintManifold(problem, latex_os, mopt_params, lm_params, "Constraint Manifold (F)");
  // EvaluateCosts(cm_result);
  // vision60.exportTrajectory(cm_result, num_steps, "/Users/yetongzhang/packages/GTDynamics/data/cm_traj.csv");

  // std::cout << "constraint manifold basis variables infeasible:\n";
  // mopt_params.cc_params->retract_params->lm_params.setMaxIterations(10);
  // auto cm_infeas_result =
  //       OptimizeConstraintManifold(problem, latex_os, mopt_params, lm_params, "Constraint Manifold (I)");
  // EvaluateCosts(cm_infeas_result);
  // vision60.exportTrajectory(cm_infeas_result, num_steps, "/Users/yetongzhang/packages/GTDynamics/data/cm_infeas_traj.csv");


  std::cout << latex_os.str();
}


int main(int argc, char **argv) {
  TrajectoryOptimization();
  return 0;
}

