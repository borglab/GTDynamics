/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  cablerobot_manifold.cpp
 * @brief Kinematic trajectory planning problem of reaching a target pose with a
 * cable robot. Benchmarking dynamic factor graph, constraint manifold, manually
 * specified manifold.
 * @author Yetong Zhang
 * @author Gerry Chen
 */

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/JointLimitFactor.h>
#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtdynamics/cablerobot/factors/CableLengthFactor.h>
#include <gtdynamics/cablerobot/factors/CableVelocityFactor.h>
#include <gtdynamics/cablerobot/factors/CableTensionFactor.h>
#include <gtdynamics/optimizer/ManifoldOptimizerType1.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/universal_robot/SerialChain.h>
#include <gtsam/base/timing.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;
using namespace gtdynamics;

const size_t num_steps = 10;
auto joint_noise = noiseModel::Isotropic::Sigma(1, 1.0);
auto joint_limit_noise = noiseModel::Isotropic::Sigma(1, 1e-2);
auto target_noise = noiseModel::Isotropic::Sigma(6, 1e-2);

const double kCdprWidth = 3, kCdprHeight = 2;
const std::array<Point3, 4> kCdprFrameMountPoints = {
    Point3{kCdprWidth, 0, 0},  //
    Point3{kCdprWidth, kCdprHeight, 0},
    Point3{0, kCdprHeight, 0},
    Point3{0, 0, 0}};
const double kCdprEeWidth = 0.1, kCdprEeHeight = 0.1;
const std::array<Point3, 4> kCdprEeMountPoints = {
    Point3{kCdprEeWidth, 0, 0},
    Point3{kCdprEeWidth, kCdprEeHeight, 0},
    Point3{0, kCdprEeHeight, 0},
    Point3{0, 0, 0}};

const Key kEeId = 1;
const Pose3 target_pose = Pose3(Rot3(), Point3(0.2, 0.5, 0.0));
// const Pose3 base_pose;
double joint_limit_low = 0;
double joint_limit_high = 3.6;

/** compute error for joint limit error, to reproduce joint limit factor in
 * expressions. */
class JointLimitFunctor {
 protected:
  double low_, high_;

 public:
  JointLimitFunctor(const double& low, const double& high)
      : low_(low), high_(high) {}

  double operator()(const double& q,
                    OptionalJacobian<1, 1> H_q = boost::none) const {
    if (q < low_) {
      if (H_q) *H_q = -gtsam::I_1x1;
      return low_ - q;
    } else if (q <= high_) {
      if (H_q) *H_q = gtsam::Z_1x1;
      return 0.0;
    } else {
      if (H_q) *H_q = gtsam::I_1x1;
      return q - high_;
    }
  }
};

/** Kinematic trajectory optimization using dynamics factor graph. */
Values optimize_dynamics_graph(const NonlinearFactorGraph& constraints_graph,
                               const NonlinearFactorGraph& costs,
                               const Values& init_values) {
  NonlinearFactorGraph graph;
  graph.add(constraints_graph);
  graph.add(costs);

  LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  gttic_(dynamic_factor_graph);
  auto result = optimizer.optimize();
  gttoc_(dynamic_factor_graph);

  size_t graph_dim = 0;
  for (const auto& factor : graph) {
    graph_dim += factor->dim();
  }
  std::cout << "dimension: " << graph_dim << " x " << init_values.dim() << "\n";
  return result;
}

/** Kinematic trajectory optimization using constraint manifold. */
Values optimize_constraint_manifold(
    const NonlinearFactorGraph& constraints_graph,
    const NonlinearFactorGraph& costs, const Values& init_values) {
  auto constraints = ConstraintsFromGraph(constraints_graph);

  LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  // params.minModelFidelity = 0.1;
  std::cout << "building optimizer\n";
  ManifoldOptimizerType1 optimizer(costs, constraints, init_values, params);
  std::cout << "optimize\n";
  gttic_(constraint_manifold);
  auto result = optimizer.optimize();
  gttoc_(constraint_manifold);

  auto problem_dim = optimizer.problem_dimension();
  std::cout << "dimension: " << problem_dim.first << " x " << problem_dim.second
            << "\n";
  return result;
}

/** Kinematic trajectory optimization using manually defined serial chain
 * manifold. */
Values optimize_serial_chain_manifold(const NonlinearFactorGraph& costs,
                                      const Values& init_values) {
  LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  size_t graph_dim = 0;
  for (const auto& factor : costs) {
    graph_dim += factor->dim();
  }
  std::cout << "dimension: " << graph_dim << " x " << init_values.dim() << "\n";
  LevenbergMarquardtOptimizer optimizer(costs, init_values, params);
  gttic_(serial_chain_manifold);
  auto result = optimizer.optimize();
  gttoc_(serial_chain_manifold);
  return result;
}

/** Factor graph of all kinematic constraints. */
NonlinearFactorGraph get_constraints_graph() {
  NonlinearFactorGraph constraints_graph;
  auto graph_builder = DynamicsGraph(); // 
  // kinematics constraints at each step
  for (size_t k = 0; k <= num_steps; ++k) {
    for (size_t ci = 0; ci < 4; ++ci) {
      constraints_graph.emplace_shared<CableLengthFactor>(
          JointAngleKey(ci, k), PoseKey(kEeId, k),
          gtsam::noiseModel::Isotropic::Sigma(1, 0.001),
          kCdprFrameMountPoints[ci], kCdprEeMountPoints[ci]);
    }
  }
  // constraints_graph.print("constraints_graph", GTDKeyFormatter);
  // // prior constraints at first step
  constraints_graph.addPrior(PoseKey(kEeId, 0),
                             Pose3(Rot3(), Point3(1.5, 1.0, 0)),
                             gtsam::noiseModel::Isotropic::Sigma(6, 0.001));
  return constraints_graph;
}

/** Cost. */
NonlinearFactorGraph get_costs() {
  NonlinearFactorGraph costs;
  // rotation costs
  NonlinearFactorGraph rotation_costs;
  for (size_t k = 0; k < num_steps; k++) {
    for (size_t ci = 0; ci < 4; ++ci) {
      rotation_costs.emplace_shared<BetweenFactor<double>>(
          JointAngleKey(ci, k), JointAngleKey(ci, k + 1), 0.0, joint_noise);
    }
  }
  // joint limit costs;
  NonlinearFactorGraph joint_limit_costs;
  for (size_t k = 0; k <= num_steps; k++) {
    for (size_t ci = 0; ci < 4; ++ci) {
      joint_limit_costs.emplace_shared<JointLimitFactor>(
          JointAngleKey(ci, k), joint_limit_noise, joint_limit_low,
          joint_limit_high, 0.0);
    }
  }
  // target cost
  NonlinearFactorGraph target_costs;
  costs.addPrior(PoseKey(kEeId, num_steps), target_pose, target_noise);
  // Point3 point_com(0, 0, 0.1);
  // Point3 target_point(0.0, 0.7, 0.7);
  // target_costs.emplace_shared<PointGoalFactor>(
  //     PoseKey(ee_link->id(), num_steps), target_noise, point_com,
  //     target_point);
  costs.add(rotation_costs);
  costs.add(target_costs);
  costs.add(joint_limit_costs);
  return costs;
}

/** Initial values specifed by 0 for all joint angles. */
Values get_init_values() {
  Values init_values;
  for (size_t k = 0; k <= num_steps; k++) {
    for (size_t ci = 0; ci < 4; ++ci) {
      InsertJointAngle(&init_values, ci, k, 1.8);
    }
    InsertPose(&init_values, kEeId, k, Pose3(Rot3(), Point3(1.5, 1.0, 0)));
  }
  return init_values;
}

// /** Initial values of serial chain specifed by 0 for all joint angles. */
// Values get_init_values_sc() {
//   Values init_values;
//   auto shared_robot = boost::make_shared<Robot>(robot);
//   Values joint_angles;
//   for (const auto& joint : robot.joints()) {
//     gtdynamics::InsertJointAngle(&joint_angles, joint->id(), 0.0);
//   }
//   for (size_t k = 1; k <= num_steps; k++) {
//     Key sc_key = k;
//     init_values.insert(sc_key, SerialChain<7>(shared_robot, base_name,
//                                               base_pose, joint_angles));
//   }
//   return init_values;
// }

// NonlinearFactorGraph get_costs_sc() {
//   NonlinearFactorGraph costs;
//   // rotation costs
//   Expression<SerialChain<7>> state1(1);
//   for (const auto& joint : robot.joints()) {
//     auto joint_func = std::bind(&SerialChain<7>::joint, std::placeholders::_1,
//                                 joint->name(), std::placeholders::_2);
//     Expression<double> curr_joint(joint_func, state1);
//     costs.addExpressionFactor<double>(joint_noise, 0.0, curr_joint);
//   }
//   for (size_t k = 1; k < num_steps; k++) {
//     Key curr_key = k;
//     Key next_key = k + 1;
//     Expression<SerialChain<7>> curr_state(curr_key);
//     Expression<SerialChain<7>> next_state(next_key);
//     for (const auto& joint : robot.joints()) {
//       auto joint_func = std::bind(&SerialChain<7>::joint, std::placeholders::_1,
//                                   joint->name(), std::placeholders::_2);
//       Expression<double> curr_joint(joint_func, curr_state);
//       Expression<double> next_joint(joint_func, next_state);
//       Expression<double> joint_rotation_expr = next_joint - curr_joint;
//       costs.addExpressionFactor<double>(joint_noise, 0.0, joint_rotation_expr);
//     }
//   }

//   // joint limit costs
//   JointLimitFunctor joint_limit_functor(joint_limit_low, joint_limit_high);
//   for (size_t k = 1; k <= num_steps; k++) {
//     Key curr_key = k;
//     Expression<SerialChain<7>> curr_state(curr_key);
//     for (const auto& joint : robot.joints()) {
//       auto joint_func = std::bind(&SerialChain<7>::joint, std::placeholders::_1,
//                                   joint->name(), std::placeholders::_2);
//       Expression<double> curr_joint(joint_func, curr_state);
//       Expression<double> joint_limit_error(joint_limit_functor, curr_joint);
//       costs.addExpressionFactor<double>(joint_limit_noise, 0.0,
//                                         joint_limit_error);
//     }
//   }

//   // target costs
//   Key last_key = num_steps;
//   Expression<SerialChain<7>> last_state(last_key);
//   auto pose_func = std::bind(&SerialChain<7>::link_pose, std::placeholders::_1,
//                              ee_name, std::placeholders::_2);
//   Expression<Pose3> target_pose_expr(pose_func, last_state);
//   costs.addExpressionFactor<Pose3>(target_noise, target_pose, target_pose_expr);
//   return costs;
// }

/** Print joint angles for all steps. */
void print_joint_angles(const Values& values) {
  for (size_t k = 0; k <= num_steps; k++) {
    std::cout << "step " << k << ": ";
    for (size_t ci = 0; ci < 4; ++ci) {
      double angle = JointAngle(values, ci, k);
      std::cout << "\t" << angle;
    }
    std::cout << "\n";
  }
}

// /** Print joint angles of serial chain for all steps. */
// void print_joint_angles_sc(const Values& values) {
//   for (size_t k = 1; k <= num_steps; k++) {
//     Key state_key = k;
//     auto state = values.at<SerialChain<7>>(state_key);
//     std::cout << "step " << k << ":";
//     for (const auto& joint : robot.joints()) {
//       double angle = state.joint(joint->name());
//       std::cout << "\t" << angle;
//     }
//     std::cout << "\n";
//   }
// }

/** Compare simple kinematic planning tasks of a robot arm using (1) dynamics
 * factor graph (2) constraint manifold (3) manually specifed serial chain
 * manifold. */
void kinematic_planning() {
  // problem
  auto constraints_graph = get_constraints_graph();
  auto costs = get_costs();
  auto init_values = get_init_values();

  // optimize dynamics graph
  std::cout << "dynamics graph:\n";
  auto dfg_result =
      optimize_dynamics_graph(constraints_graph, costs, init_values);
  std::cout << "final cost: " << costs.error(dfg_result) << "\n";
  std::cout << "constraint violation: " << constraints_graph.error(dfg_result)
            << "\n";
  print_joint_angles(dfg_result);

  // optimize constraint manifold
  std::cout << "constraint manifold:\n";
  auto cm_result =
      optimize_constraint_manifold(constraints_graph, costs, init_values);
  std::cout << "final cost: " << costs.error(cm_result) << "\n";
  std::cout << "constraint violation: " << constraints_graph.error(cm_result)
            << "\n";
  print_joint_angles(cm_result);

  // // optimize serial chain manifold
  // std::cout << "serial chain manifold\n";
  // Values sc_init_values = get_init_values_sc();
  // auto sc_costs = get_costs_sc();
  // auto sc_result = optimize_serial_chain_manifold(sc_costs, sc_init_values);
  // std::cout << "final cost: " << sc_costs.error(sc_result) << "\n";
  // print_joint_angles_sc(sc_result);

  std::cout << "Timing results:\n";
  tictoc_finishedIteration_();
  tictoc_print_();
}

int main(int argc, char** argv) {
  kinematic_planning();
  return 0;
}
