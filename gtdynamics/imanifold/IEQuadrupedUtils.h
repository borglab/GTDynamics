/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  QuadrupedUtils.h
 * @brief Utilities for quadruped trajectory optimization experiments.
 * @author: Yetong Zhang
 */

#pragma once

// #include <examples/example_constraint_manifold/QuadrupedUtils.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/OptimizerSetting.h>
#include <gtdynamics/factors/ConstVarFactor.h>
#include <gtdynamics/manifold/ConnectedComponent.h>
#include <gtdynamics/manifold/ManifoldOptimizer.h>
#include <gtdynamics/manifold/MultiJacobian.h>
#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtdynamics/optimizer/MutableLMOptimizer.h>
#include <gtdynamics/optimizer/PenaltyMethodOptimizer.h>
#include <gtdynamics/universal_robot/Link.h>

#include <gtdynamics/factors/ContactPointFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtdynamics/utils/PointOnLink.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/expressions.h>

#include <gtdynamics/imanifold/IERetractor.h>

namespace gtsam {

/// Key to represent contact redundancy.
inline gtdynamics::DynamicsSymbol ContactRedundancyKey(int t = 0) {
  return gtdynamics::DynamicsSymbol::SimpleSymbol("CR", t);
}

/// Class of utilitis for Vision60 robot.
class IEVision60Robot {
public:
  struct Params {
    // environment
    gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
    double mu = 2.0;

    /// Noise sigma for costs
    double sigma_des_pose = 1e-2;
    double sigma_des_twist = 1e-2;
    double sigma_actuation = 1e1;
    double sigma_q_col = 1e-2;
    double sigma_v_col = 1e-2;

    // tolerance for e constraints
    double tol_q = 1e-2;        // tolerance of q-level constraints
    double tol_v = 1e-2;        // tolerance of v-level constraints
    double tol_a = 1e-2;        // tolerance of a-level constraints
    double tol_dynamics = 1e-2; // tolerance of d-level constraints

    // tolerance for i constraints
    double tol_jl = 0.1;
    double tol_tl = 0.1;
    double tol_fc = 0.1;
    double tol_ca = 0.1;

    /// Optimization settings
    bool express_redundancy = true;
    bool basis_using_torques = false;
    bool include_friction_cone = false;
    bool include_joint_limits = false;
    bool include_torque_limits = false;
    bool include_collision_avoidance = false;

    /// Joint and torque limits
    std::map<std::string, double> joint_lower_limits;
    std::map<std::string, double> joint_upper_limits;
    std::map<std::string, double> torque_lower_limits;
    std::map<std::string, double> torque_upper_limits;

    Params() = default;
  };

  /// Robot
  inline static gtdynamics::Robot robot = gtdynamics::CreateRobotFromFile(
      gtdynamics::kUrdfPath + std::string("vision60.urdf"));
  inline static gtsam::Point3 contact_in_com = Point3(0.14, 0, 0);

  /// Link ids
  inline static gtdynamics::LinkSharedPtr base_link = robot.link("body");
  inline static int base_id = base_link->id();
  inline static int lower0_id = robot.link("lower0")->id();
  inline static int lower1_id = robot.link("lower1")->id();
  inline static int lower2_id = robot.link("lower2")->id();
  inline static int lower3_id = robot.link("lower3")->id();
  inline static std::vector<std::string> hip_joint_names{"8", "9", "10", "11"};
  inline static std::vector<std::string> upper_joint_names{"0", "2", "4", "6"};
  inline static std::vector<std::string> lower_joint_names{"1", "3", "5", "7"};
  inline static std::vector<std::string> hip_link_names{"hip0", "hip1", "hip2",
                                                        "hip3"};
  inline static std::vector<std::string> upper_link_names{"upper0", "upper1",
                                                          "upper2", "upper3"};
  inline static std::vector<std::string> lower_link_names{"lower0", "lower1",
                                                          "lower2", "lower3"};

  /// Contact point info
  inline static gtdynamics::PointOnLinks contact_points{
      gtdynamics::PointOnLink(robot.link("lower0"), contact_in_com),
      gtdynamics::PointOnLink(robot.link("lower1"), contact_in_com),
      gtdynamics::PointOnLink(robot.link("lower2"), contact_in_com),
      gtdynamics::PointOnLink(robot.link("lower3"), contact_in_com)};
  inline static std::vector<int> contact_ids{lower0_id, lower1_id, lower2_id,
                                             lower3_id};

  /// Nominal configuration
  double nominal_height =
      gtdynamics::Pose(getNominalConfiguration(), lower0_id, 0)
          .transformFrom(contact_in_com)
          .z() *
      -1;
  Values nominal_values = getNominalConfiguration(nominal_height);
  std::vector<Point3> nominal_contact_in_world{
      gtdynamics::Pose(nominal_values, lower0_id, 0)
          .transformFrom(contact_in_com),
      gtdynamics::Pose(nominal_values, lower1_id, 0)
          .transformFrom(contact_in_com),
      gtdynamics::Pose(nominal_values, lower2_id, 0)
          .transformFrom(contact_in_com),
      gtdynamics::Pose(nominal_values, lower3_id, 0)
          .transformFrom(contact_in_com)};
  double nominal_a = 0.5 * (nominal_contact_in_world.at(0).x() -
                            nominal_contact_in_world.at(1).x());
  double nominal_b = 0.5 * (nominal_contact_in_world.at(0).y() -
                            nominal_contact_in_world.at(2).y());

  Params params_;
  gtdynamics::DynamicsGraph graph_builder;
  SharedNoiseModel des_pose_nm;
  SharedNoiseModel des_twist_nm;
  SharedNoiseModel min_torque_nm;
  SharedNoiseModel cpoint_cost_model;
  SharedNoiseModel redundancy_model;

protected:
  static gtdynamics::OptimizerSetting getOptSetting(const Params &params);

  Values getNominalConfiguration(const double height = 0) const;

  NonlinearFactorGraph getConstraintsGraphStepQ(const int t) const;

  NonlinearFactorGraph getConstraintsGraphStepV(const int t) const;

  NonlinearFactorGraph getConstraintsGraphStepAD(const int t) const;

  gtdynamics::InequalityConstraints
  frictionConeConstraints(const size_t k) const;

  gtdynamics::InequalityConstraints jointLimitConstraints(const size_t k) const;

  gtdynamics::InequalityConstraints
  torqueLimitConstraints(const size_t k) const;

  gtdynamics::InequalityConstraints
  collisionAvoidanceConstraints(const size_t k) const;

  /// Dynamcis factors without friction cone factors (as they are moved to
  /// inequality constraints).
  NonlinearFactorGraph DynamicsFactors(const size_t k) const;

public:
  IEVision60Robot(const Params &params)
      : params_(params), graph_builder(gtdynamics::DynamicsGraph(
                             getOptSetting(params), params_.gravity)) {
    des_pose_nm = noiseModel::Isotropic::Sigma(6, params.sigma_des_pose);
    des_twist_nm = noiseModel::Isotropic::Sigma(6, params.sigma_des_twist);
    min_torque_nm = noiseModel::Isotropic::Sigma(1, params.sigma_actuation);
    cpoint_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, params.tol_q);
    redundancy_model =
        gtsam::noiseModel::Isotropic::Sigma(6, params.tol_dynamics);
  }

  /** <================= Constraints and Costs =================> **/
  /// Kinodynamic constraints at the specified time step.
  gtdynamics::EqualityConstraints eConstraints(const size_t k) const;

  gtdynamics::InequalityConstraints iConstraints(const size_t k) const;

  gtdynamics::EqualityConstraints
  initStateConstraints(const Pose3 &init_pose, const Vector6 &init_twist) const;

  /// Costs for collocation across steps.
  NonlinearFactorGraph collocationCosts(const size_t num_steps,
                                        double dt) const;

  /// Costs for min torque objectives.
  NonlinearFactorGraph minTorqueCosts(const size_t num_steps) const;

  /// Costs for init condition and reaching target poses.
  NonlinearFactorGraph finalStateCosts(const Pose3 &des_pose,
                                       const Vector6 &des_twist,
                                       const size_t num_steps) const;

  /** <================= Value Initialize Functions =================> **/
  /// Return values of one step satisfying kinodynamic constraints.
  Values getInitValuesStep(const size_t k = 0,
                           const Pose3 &base_pose = Pose3(Rot3::Identity(),
                                                          Point3(0, 0, 0.0)),
                           const Vector6 &base_twist = Vector6::Zero(),
                           const Vector6 &base_accel = Vector6::Zero(),
                           Values init_values_t = Values()) const;

  /// Return values of trajectory satisfying kinodynamic constraints.
  Values getInitValuesTrajectory(
      const size_t num_steps, double dt, const Pose3 &base_pose_init,
      const std::vector<gtsam::Pose3> &des_poses,
      std::vector<double> &des_poses_t,
      const std::string initialization_technique = "interp") const;

  /** <================= Print and Export Functions =================> **/
  static void PrintValues(const Values &values, const size_t num_steps);

  static void PrintDelta(const VectorValues &values, const size_t num_steps);

  static void ExportValues(const Values &values, const size_t num_steps,
                           const std::string &file_path);

  static void ExportVector(const VectorValues &values, const size_t num_steps,
                           const std::string &file_path);

  /// Return optimizer setting.
  const gtdynamics::OptimizerSetting &opt() const {
    return graph_builder.opt();
  }

  /// Return function that select basis keys for constraint manifolds.
  BasisKeyFunc getBasisKeyFunc() const;
};

/** Retractor for quadruped manifold. */
class Vision60Retractor : public IERetractor {

public:
  struct Params {
    LevenbergMarquardtParams lm_params;
    double prior_sigma;
    bool use_basis_keys;
    bool check_feasible;
    double feasible_threshold;
  };

protected:
  const IEVision60Robot &robot_;
  const Params &params_;

  NonlinearFactorGraph merit_graph_;
  NonlinearFactorGraph graph_q_, graph_v_, graph_ad_;
  IndexSet i_indices_q_, i_indices_v_, i_indices_ad_;
  KeySet basis_q_keys_, basis_v_keys_, basis_ad_keys_;
  std::vector<std::shared_ptr<ConstVarFactor>> const_var_factors_v_,
      const_var_factors_ad_;

public:
  Vision60Retractor(const IEVision60Robot &robot,
                    const IEConstraintManifold &manifold, const Params &params);

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {}) const override;

protected:
  template <typename CONTAINER>
  static void classifyKeys(const CONTAINER &keys, KeySet &q_keys,
                           KeySet &v_keys, KeySet &ad_keys);

  void checkFeasible(const NonlinearFactorGraph &graph,
                     const Values &values) const;
};

class Vision60RetractorCreator : public IERetractorCreator {
protected:
  const IEVision60Robot &robot_;
  const Vision60Retractor::Params &params_;

public:
  Vision60RetractorCreator(const IEVision60Robot &robot,
                           const Vision60Retractor::Params &params)
      : robot_(robot), params_(params) {}

  virtual ~Vision60RetractorCreator() {}

  IERetractor::shared_ptr
  create(const IEConstraintManifold &manifold) const override;
};

} // namespace gtsam
