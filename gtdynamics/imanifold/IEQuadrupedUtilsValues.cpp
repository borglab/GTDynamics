#include "utils/DynamicsSymbol.h"
#include "utils/values.h"
#include <gtdynamics/factors/CollocationFactors.h>
#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IEQuadrupedUtils.h>
#include <gtdynamics/manifold/GeneralPriorFactor.h>
#include <gtdynamics/utils/DebugUtils.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <stdexcept>

using namespace gtdynamics;

namespace gtsam {

/* ************************************************************************* */
Values IEVision60Robot::stepValues(
    const size_t k, const Values &init_values,
    const std::optional<KeyVector> &optional_known_keys) const {

  Values nominal_values_k = DynamicsValuesFromPrev(nominal_values, k);
  KeyVector known_keys =
      optional_known_keys ? *optional_known_keys : basisKeys(k);
  KeySet known_q_keys, known_v_keys, known_ad_keys;
  ClassifyKeysByLevel(known_keys, known_q_keys, known_v_keys, known_ad_keys);
  LevenbergMarquardtParams lm_params;
  Values known_values;

  // solve q level
  NonlinearFactorGraph graph_q = getConstraintsGraphStepQ(k);
  AddGeneralPriors(init_values, known_q_keys, params.tol_q, graph_q);
  Values init_values_q =
      PickValues(graph_q.keys(), init_values, nominal_values_k);
  LevenbergMarquardtOptimizer optimizer_q(graph_q, init_values_q, lm_params);
  auto results_q = optimizer_q.optimize();
  CheckFeasible(graph_q, results_q, "q-level ");
  known_values.insert(results_q);

  // solve v level
  NonlinearFactorGraph graph_v = getConstraintsGraphStepV(k);
  graph_v = ConstVarGraph(graph_v, known_values);
  AddGeneralPriors(init_values, known_v_keys, params.tol_v, graph_v);
  Values init_values_v =
      PickValues(graph_v.keys(), init_values, nominal_values_k);
  LevenbergMarquardtOptimizer optimizer_v(graph_v, init_values_v, lm_params);
  auto results_v = optimizer_v.optimize();
  CheckFeasible(graph_v, results_v, "v-level ");
  known_values.insert(results_v);

  // solve ad level
  NonlinearFactorGraph graph_ad = getConstraintsGraphStepAD(k);
  graph_ad = ConstVarGraph(graph_ad, known_values);
  AddGeneralPriors(init_values, known_ad_keys, params.tol_dynamics, graph_ad);
  Values init_values_ad =
      PickValues(graph_ad.keys(), init_values, nominal_values_k);
  LevenbergMarquardtOptimizer optimizer_ad(graph_ad, init_values_ad, lm_params);
  auto results_ad = optimizer_ad.optimize();
  CheckFeasible(graph_ad, results_ad, "ad-level ");
  known_values.insert(results_ad);

  return known_values;
}

/* ************************************************************************* */
Values IEVision60Robot::stepValuesByIntegration(
    const size_t k, const double dt, const Values &prev_values,
    const std::optional<KeyVector> &optional_intagration_keys) const {
  KeyVector integration_keys =
      optional_intagration_keys ? *optional_intagration_keys : basisKeys(k);
  KeySet integration_q_keys, integration_v_keys, integration_ad_keys;
  ClassifyKeysByLevel(integration_keys, integration_q_keys, integration_v_keys,
                      integration_ad_keys);

  Values values = DynamicsValuesFromPrev(prev_values);

  // integrate in q-level
  for (const Key &key : integration_q_keys) {
    DynamicsSymbol symb(key);
    if (symb.label() == "p") {
      Pose3 prev_pose = Pose(prev_values, symb.linkIdx(), k - 1);
      Vector6 prev_twist = Twist(prev_values, symb.linkIdx(), k - 1);
      Pose3 curr_pose = predictPose(prev_pose, dt * prev_twist);
      values.update(key, curr_pose);
    }
    if (symb.label() == "q") {
      double prev_q = JointAngle(prev_values, symb.jointIdx(), k - 1);
      double prev_v = JointVel(prev_values, symb.jointIdx(), k - 1);
      double curr_q = prev_q + dt * prev_v;
      values.update(key, curr_q);
    }
  }

  // integrate in v-level
  for (const Key &key : integration_v_keys) {
    DynamicsSymbol symb(key);
    if (symb.label() == "V") {
      Vector6 prev_twist = Twist(prev_values, symb.linkIdx(), k - 1);
      Vector6 prev_twistaccel = TwistAccel(prev_values, symb.linkIdx(), k - 1);
      Vector6 curr_twist = prev_twist + dt * prev_twistaccel;
      values.update(key, curr_twist);
    }
    if (symb.label() == "v") {
      double prev_v = JointVel(prev_values, symb.jointIdx(), k - 1);
      double prev_a = JointAccel(prev_values, symb.jointIdx(), k - 1);
      double curr_v = prev_v + dt * prev_a;
      values.update(key, curr_v);
    }
  }

  return values;
}

/* ************************************************************************* */
Values IEVision60Robot::getInitValuesStep(const size_t k,
                                          const Pose3 &base_pose,
                                          const Vector6 &base_twist,
                                          const Vector6 &base_accel,
                                          Values init_values_t) const {
  if (init_values_t.size() == 0) {
    Vector6 zero_vec6 = Vector6::Zero();
    for (auto &&joint : robot.joints()) {
      int j = joint->id();
      InsertJointAngle(&init_values_t, j, k, JointAngle(nominal_values, j));
      InsertJointVel(&init_values_t, j, k, JointVel(nominal_values, j));
      InsertJointAccel(&init_values_t, j, k, 0.0);
      InsertTorque(&init_values_t, j, k, 0.0);
      InsertWrench(&init_values_t, joint->parent()->id(), j, k, zero_vec6);
      InsertWrench(&init_values_t, joint->child()->id(), j, k, zero_vec6);
    }
    for (const auto &cp : contact_points) {
      int i = cp.link->id();
      init_values_t.insert(ContactWrenchKey(i, 0, k), zero_vec6);
    }
    for (auto &&link : robot.links()) {
      int i = link->id();
      InsertPose(&init_values_t, i, k, Pose(nominal_values, i));
      InsertTwist(&init_values_t, i, k, Twist(nominal_values, i));
      InsertTwistAccel(&init_values_t, i, k, zero_vec6);
    }
    if (params.express_redundancy) {
      init_values_t.insert(ContactRedundancyKey(k), zero_vec6);
    }
  }

  Values known_values;
  LevenbergMarquardtParams lm_params;
  // lm_params.setVerbosityLM("SUMMARY");

  // solve q level
  NonlinearFactorGraph graph_q = getConstraintsGraphStepQ(k);
  graph_q.addPrior<Pose3>(PoseKey(base_id, k), base_pose,
                          graph_builder.opt().p_cost_model);
  for (size_t i = 0; i < 4; i++) {
    if (!params.contact_indices.exists(i)) {
      Key hip_joint_key = JointAngleKey(legs[i].hip_joint_id, k);
      Key upper_joint_key = JointAngleKey(legs[i].upper_joint_id, k);
      Key lower_joint_key = JointAngleKey(legs[i].lower_joint_id, k);
      graph_q.addPrior<double>(hip_joint_key,
                               init_values_t.atDouble(hip_joint_key),
                               graph_builder.opt().prior_q_cost_model);
      graph_q.addPrior<double>(upper_joint_key,
                               init_values_t.atDouble(upper_joint_key),
                               graph_builder.opt().prior_q_cost_model);
      graph_q.addPrior<double>(lower_joint_key,
                               init_values_t.atDouble(lower_joint_key),
                               graph_builder.opt().prior_q_cost_model);
    }
  }

  Values init_values_q = SubValues(init_values_t, graph_q.keys());
  LevenbergMarquardtOptimizer optimizer_q(graph_q, init_values_q, lm_params);
  auto results_q = optimizer_q.optimize();
  if (graph_q.error(results_q) > 1e-5) {
    std::cout << "solving q fails! error: " << graph_q.error(results_q) << "\n";
    PrintGraphWithError(graph_q, init_values_q);
    PrintGraphWithError(graph_q, results_q);
  }
  known_values.insert(results_q);

  // solve v level
  NonlinearFactorGraph graph_v = getConstraintsGraphStepV(k);
  graph_v.addPrior<Vector6>(TwistKey(base_id, k), base_twist,
                            graph_builder.opt().v_cost_model);
  graph_v = ConstVarGraph(graph_v, known_values);
  Values init_values_v = SubValues(init_values_t, graph_v.keys());
  LevenbergMarquardtOptimizer optimizer_v(graph_v, init_values_v, lm_params);
  auto results_v = optimizer_v.optimize();
  if (graph_v.error(results_v) > 1e-5) {
    std::cout << "solving v fails! error: " << graph_v.error(results_v) << "\n";
    PrintGraphWithError(graph_v, init_values_v);
    PrintGraphWithError(graph_v, results_v);
  }
  known_values.insert(results_v);

  // solve a and dynamics level
  NonlinearFactorGraph graph_ad = getConstraintsGraphStepAD(k);
  // TODO: handle for boundary phases
  if (params.leaving_indices.size() < 4) {
    graph_ad.addPrior<Vector6>(TwistAccelKey(base_id, k), base_accel,
                               graph_builder.opt().a_cost_model);
  }

  Vector6 zero_vec6 = Vector6::Zero();
  if (params.express_redundancy) {
    graph_ad.addPrior<Vector6>(ContactRedundancyKey(k), zero_vec6,
                               redundancy_model);
  }
  graph_ad = ConstVarGraph(graph_ad, known_values);
  Values init_values_ad = SubValues(init_values_t, graph_ad.keys());
  LevenbergMarquardtOptimizer optimizer_ad(graph_ad, init_values_ad, lm_params);
  auto results_ad = optimizer_ad.optimize();
  if (graph_ad.error(results_ad) > 1e-5) {
    std::cout << "solving ad fails! error: " << graph_ad.error(results_ad)
              << "\n";
    PrintGraphWithError(graph_ad, init_values_ad);
    PrintGraphWithError(graph_ad, results_ad);
  }
  known_values.insert(results_ad);

  return known_values;
}

/* ************************************************************************* */
Values IEVision60Robot::getInitValuesTrajectory(
    const size_t num_steps, double dt, const Pose3 &base_pose_init,
    const std::vector<gtsam::Pose3> &des_poses,
    std::vector<double> &des_poses_t,
    const std::string initialization_technique) const {
  // Initialize solution.
  gtsam::Values init_vals;
  Initializer initializer;

  // solve 1 step value
  Values init_values_0 = getInitValuesStep(0, base_pose_init);

  // copy for all steps
  if (initialization_technique == "zero") {
    for (int t = 0; t <= num_steps; t++) {
      for (const Key &key : init_values_0.keys()) {
        init_vals.insert(key + t, init_values_0.at(key));
      }
    }
  } else if (initialization_technique == "interp") {
    /// interpolate base pose
    auto interp_values = initializer.InitializeSolutionInterpolationMultiPhase(
        robot, "body", base_pose_init, des_poses, des_poses_t, dt, 0.0,
        contact_points);
    Values base_link_values;
    for (int t = 0; t <= num_steps; t++) {
      base_link_values.insert(PoseKey(base_id, t),
                              interp_values.at(PoseKey(base_id, t)));
    }
    /// compute base twist
    Vector6 zero_vector6 = Vector6::Zero();
    base_link_values.insert(TwistKey(base_id, 0), zero_vector6);
    for (int t = 0; t < num_steps; t++) {
      Pose3 pose_prev = base_link_values.at<Pose3>(PoseKey(base_id, t));
      Pose3 pose_curr = base_link_values.at<Pose3>(PoseKey(base_id, t + 1));
      Pose3 pose_rel = pose_prev.inverse().compose(pose_curr);
      Vector6 twist_interval = Pose3::Logmap(pose_rel) / dt;
      Vector6 twist_prev = base_link_values.at<Vector6>(TwistKey(base_id, t));
      // Vector6 twist_curr = 2 * twist_interval - twist_prev;
      Vector6 twist_curr = twist_interval;
      base_link_values.insert(TwistKey(base_id, t + 1), twist_curr);
    }
    /// compute base accel
    base_link_values.insert(TwistAccelKey(base_id, 0), zero_vector6);
    for (int t = 0; t < num_steps; t++) {
      Vector6 twist_prev = base_link_values.at<Vector6>(TwistKey(base_id, t));
      Vector6 twist_curr =
          base_link_values.at<Vector6>(TwistKey(base_id, t + 1));
      Vector6 twist_rel = twist_curr - twist_prev;
      Vector6 accel_interval = twist_rel / dt;
      Vector6 accel_prev =
          base_link_values.at<Vector6>(TwistAccelKey(base_id, t));
      // Vector6 accel_curr = 2 * accel_interval - accel_prev;
      Vector6 accel_curr = accel_interval;
      base_link_values.insert(TwistAccelKey(base_id, t + 1), accel_curr);
    }
    /// solve kinodynamics for each step
    Values prev_values = init_values_0;
    init_vals = init_values_0;
    for (int t = 1; t <= num_steps; t++) {
      Values init_values_t;
      for (const Key &key : prev_values.keys()) {
        init_values_t.insert(key + 1, prev_values.at(key));
      }
      Values values_t = getInitValuesStep(
          t, Pose(base_link_values, base_id, t),
          Twist(base_link_values, base_id, t),
          TwistAccel(base_link_values, base_id, t), init_values_t);
      prev_values = values_t;
      init_vals.insert(values_t);

      // std::cout << "pose: " << Pose(init_vals, base_id, t) << "\n";
    }
  }

  return init_vals;
}

} // namespace gtsam
