#include "utils/DynamicsSymbol.h"
#include "utils/values.h"
#include <gtdynamics/factors/CollocationFactors.h>
#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/scenarios/IEQuadrupedUtils.h>
#include <gtdynamics/factors/GeneralPriorFactor.h>
#include <gtdynamics/utils/GraphUtils.h>
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

Values TrajectoryValuesVerticalJump(
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::vector<double> &phases_dt, const double torso_accel_z,
    const size_t ground_switch_k) {

  Values values;
  for (size_t phase_idx = 0; phase_idx < phases_dt.size(); phase_idx += 1) {
    values.insert(PhaseKey(phase_idx), phases_dt.at(phase_idx));
  }

  //// Phase on ground: part1: const accel of torso
  std::cout << "phase on ground part1\n";
  const IEVision60Robot &vision60_ground =
      vision60_multi_phase.phase_robots_[0];
  size_t num_steps_ground = vision60_multi_phase.phase_num_steps_[0];
  double dt_ground = phases_dt[0];

  double base_z = vision60_ground.nominal_height;
  double base_v = 0;
  Values prev_values;
  for (size_t k = 0; k <= ground_switch_k; k++) {
    // set a,v,q level for each step of base_link
    Vector6 base_accel = (Vector(6) << 0, 0, 0, 0, 0, torso_accel_z).finished();
    Vector6 base_twist = (Vector(6) << 0, 0, 0, 0, 0, base_v).finished();
    Pose3 base_pose(Rot3::Identity(), Point3(0, 0, base_z));
    base_z += base_v * dt_ground;
    base_v += torso_accel_z * dt_ground;

    /// solve kinodynamics
    Values init_values_k = k == 0 ? vision60_ground.nominal_values
                                  : DynamicsValuesFromPrev(prev_values);
    init_values_k.update(PoseKey(vision60_ground.base_id, k), base_pose);
    init_values_k.update(TwistKey(vision60_ground.base_id, k), base_twist);
    init_values_k.update(TwistAccelKey(vision60_ground.base_id, k), base_accel);
    KeyVector known_keys{PoseKey(vision60_ground.base_id, k),
                         TwistKey(vision60_ground.base_id, k),
                         TwistAccelKey(vision60_ground.base_id, k),
                         ContactRedundancyKey(k)};
    Values values_k = vision60_ground.stepValues(k, init_values_k, known_keys);
    values.insert(values_k);
    prev_values = values_k;
  }

  //// Phase on ground: part2: reduce contact force to 0
  std::cout << "phase on ground part2\n";
  std::map<uint8_t, Vector3> contact_forces;
  for (const auto &link_id : vision60_ground.contact_link_ids) {
    contact_forces.insert({link_id, prev_values.at<Vector3>(ContactForceKey(
                                        link_id, 0, ground_switch_k))});
  }

  size_t num_steps_ground2 = num_steps_ground - ground_switch_k;
  for (size_t k = ground_switch_k + 1; k <= num_steps_ground; k++) {
    // integrate prev values
    Values init_values_k =
        vision60_ground.stepValuesByIntegration(k, dt_ground, prev_values);

    // set contact force
    for (const auto &link_id : vision60_ground.contact_link_ids) {
      Vector3 boundary_contact_force = contact_forces.at(link_id);
      Vector3 contact_force =
          boundary_contact_force *
          (double(num_steps_ground - k) / num_steps_ground2);
      init_values_k.update(ContactForceKey(link_id, 0, k), contact_force);
    }

    // solve for ad level
    KeyVector basis_keys;
    basis_keys.push_back(PoseKey(vision60_ground.base_id, k));
    basis_keys.push_back(TwistKey(vision60_ground.base_id, k));
    for (const auto &link_id : vision60_ground.contact_link_ids) {
      basis_keys.push_back(ContactForceKey(link_id, 0, k));
    }
    init_values_k = vision60_ground.stepValues(k, init_values_k, basis_keys);
    values.insert(init_values_k);
    prev_values = init_values_k;
  }

  //// boundary step same as last step in ground phase

  //// Phase in air
  std::cout << "phase in air\n";
  const IEVision60Robot &vision60_air = vision60_multi_phase.phase_robots_[1];
  size_t num_steps_air = vision60_multi_phase.phase_num_steps_[1];
  double dt_air = phases_dt[1];

  std::map<uint8_t, double> prev_torques;
  for (const auto &joint : vision60_air.robot.joints()) {
    uint8_t j = joint->id();
    prev_torques.insert({j, Torque(prev_values, j, num_steps_ground)});
  }

  size_t num_steps = num_steps_ground + num_steps_air;
  for (size_t k = num_steps_ground + 1; k <= num_steps; k++) {
    // integrate prev values
    Values init_values_k =
        vision60_air.stepValuesByIntegration(k, dt_air, prev_values);

    // set joint torques
    for (const auto &joint : vision60_air.robot.joints()) {
      uint8_t j = joint->id();
      double prev_torque = prev_torques.at(j);
      double torque = prev_torque * (double(num_steps - k) / num_steps_air);
      init_values_k.update(TorqueKey(j, k), torque);
    }

    // solve for ad-level
    KeyVector basis_keys;
    basis_keys.push_back(PoseKey(vision60_air.base_id, k));
    basis_keys.push_back(TwistKey(vision60_air.base_id, k));
    for (const auto &joint : vision60_air.robot.orderedJoints()) {
      basis_keys.push_back(JointAngleKey(joint->id(), k));
      basis_keys.push_back(JointVelKey(joint->id(), k));
      basis_keys.push_back(TorqueKey(joint->id(), k));
    }
    init_values_k = vision60_air.stepValues(k, init_values_k, basis_keys);
    values.insert(init_values_k);
    prev_values = init_values_k;
  }

  return values;
}


Values TrajectoryValuesVerticalJumpDeprecated(
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::vector<double> &phases_dt, const double torso_accel_z){
  Values values;
  for (size_t phase_idx = 0; phase_idx < phases_dt.size(); phase_idx += 1) {
    values.insert(PhaseKey(phase_idx), phases_dt.at(phase_idx));
  }

  //// Phase on ground
  std::cout << "phase on ground\n";
  const IEVision60Robot &vision60_ground =
      vision60_multi_phase.phase_robots_[0];
  size_t num_steps_ground = vision60_multi_phase.phase_num_steps_[0];
  double dt_ground = phases_dt[0];
  double leave_height = 0.2;

  double base_z = vision60_ground.nominal_height;
  double base_v = 0;
  Values prev_values;
  for (size_t k = 0; k < num_steps_ground; k++) {
    // set a,v,q level for each step of base_link
    Vector6 base_accel = (Vector(6) << 0, 0, 0, 0, 0, torso_accel_z).finished();
    Vector6 base_twist = (Vector(6) << 0, 0, 0, 0, 0, base_v).finished();
    Pose3 base_pose(Rot3::Identity(), Point3(0, 0, base_z));
    base_z += base_v * dt_ground;
    base_v += torso_accel_z * dt_ground;

    /// solve kinodynamics
    Values init_values_k = k == 0 ? vision60_ground.nominal_values
                                  : DynamicsValuesFromPrev(prev_values);
    init_values_k.update(PoseKey(vision60_ground.base_id, k), base_pose);
    init_values_k.update(TwistKey(vision60_ground.base_id, k), base_twist);
    init_values_k.update(TwistAccelKey(vision60_ground.base_id, k), base_accel);
    KeyVector known_keys{PoseKey(vision60_ground.base_id, k),
                         TwistKey(vision60_ground.base_id, k),
                         TwistAccelKey(vision60_ground.base_id, k),
                         ContactRedundancyKey(k)};
    Values values_k = vision60_ground.stepValues(k, init_values_k, known_keys);
    values.insert(values_k);
    prev_values = values_k;
  }

  //// boundary step
  std::cout << "boundary step\n";
  size_t boundary_k = vision60_multi_phase.boundary_ks_[0];
  const IEVision60Robot &vision60_boundary =
      vision60_multi_phase.boundary_robots_[0];
  Values values_boundary;
  {
    values_boundary = vision60_ground.stepValuesByIntegration(
        boundary_k, dt_ground, prev_values);
    values_boundary = vision60_boundary.stepValues(boundary_k, values_boundary);
    values.insert(values_boundary);
  }

  //// Phase in air
  std::cout << "phase in air\n";
  const IEVision60Robot &vision60_air = vision60_multi_phase.phase_robots_[1];
  size_t num_steps_air = vision60_multi_phase.phase_num_steps_[1];
  double dt_air = phases_dt[1];

  // set joint constant a for air phase
  Values values_first_air = vision60_air.stepValuesByIntegration(
      boundary_k + 1, dt_air, values_boundary);
  Values joint_a_air_values;
  for (const auto &joint : vision60_air.robot.joints()) {
    uint8_t j = joint->id();
    double v = JointVel(values_first_air, j, boundary_k + 1);
    double a = -v / ((num_steps_air - 1) * dt_air);
    InsertJointAccel(&joint_a_air_values, j, boundary_k, a);
  }

  prev_values = values_boundary;
  for (size_t k = boundary_k + 1; k <= boundary_k + num_steps_air; k++) {
    // integrate prev values
    Values init_values_k =
        vision60_air.stepValuesByIntegration(k, dt_air, prev_values);

    // set joint accel
    joint_a_air_values = DynamicsValuesFromPrev(joint_a_air_values);
    init_values_k.update(joint_a_air_values);

    // solve for target a
    KeyVector basis_keys;
    basis_keys.push_back(PoseKey(vision60_air.base_id, k));
    basis_keys.push_back(TwistKey(vision60_air.base_id, k));
    for (const auto &joint : vision60_air.robot.orderedJoints()) {
      basis_keys.push_back(JointAngleKey(joint->id(), k));
      basis_keys.push_back(JointVelKey(joint->id(), k));
      basis_keys.push_back(JointAccelKey(joint->id(), k));
    }
    init_values_k = vision60_air.stepValues(k, init_values_k, basis_keys);
    values.insert(init_values_k);
    prev_values = init_values_k;
  }

  return values;
}



} // namespace gtsam
