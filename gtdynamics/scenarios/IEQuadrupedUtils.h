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

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/imanifold/IERetractor.h>
#include <gtdynamics/manifold/ManifoldOptimizer.h>
#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtdynamics/universal_robot/Joint.h>
#include <gtdynamics/universal_robot/Link.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtdynamics/utils/PointOnLink.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

/// Key to represent contact redundancy.
inline gtdynamics::DynamicsSymbol ContactRedundancyKey(int t = 0) {
  return gtdynamics::DynamicsSymbol::SimpleSymbol("CR", t);
}

/**
 * Class of utilitis for Vision60 robot.
 */
class IEVision60Robot {
public:
  struct Leg {
    gtdynamics::JointSharedPtr hip_joint, upper_joint, lower_joint;
    gtdynamics::LinkSharedPtr hip_link, upper_link, lower_link;
    size_t hip_joint_id, upper_joint_id, lower_joint_id;
    size_t hip_link_id, upper_link_id, lower_link_id;
    std::vector<gtdynamics::JointSharedPtr> joints;
    std::vector<gtdynamics::LinkSharedPtr> links;

    Leg(const gtdynamics::Robot &robot, const std::string &hip_joint_name,
        const std::string &upper_joint_name,
        const std::string &lower_joint_name, const std::string &hip_link_name,
        const std::string &upper_link_name, const std::string &lower_link_name);
  };

  struct Params {
    // environment
    gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
    double mu = 2.0;
    gtdynamics::CollocationScheme collocation =
        gtdynamics::CollocationScheme::Trapezoidal;

    /// Noise sigma for costs
    double sigma_des_pose = 1e-2;
    double sigma_des_twist = 1e-2;
    double sigma_actuation = 1e1;
    double sigma_q_col = 1e-2;
    double sigma_v_col = 1e-2;
    double sigma_pose_col = 1e-2;
    double sigma_twist_col = 1e-2;

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
    double tol_phase_dt = 0.1;

    /// Optimization settings
    bool express_redundancy = true;
    bool express_contact_force = false;
    bool ad_basis_using_torques = false;
    bool include_friction_cone = false;
    bool include_joint_limits = false;
    bool include_torque_limits = false;
    bool include_collision_avoidance = false;

    /// Joint and torque limits
    std::map<std::string, double> joint_lower_limits;
    std::map<std::string, double> joint_upper_limits;
    std::map<std::string, double> torque_lower_limits;
    std::map<std::string, double> torque_upper_limits;

    // phase info
    IndexSet contact_indices;
    IndexSet leaving_indices;
    IndexSet landing_indices;

    Params() = default;

    void set4C();

    void setBackOnGround();

    void setFrontOnGround();

    void setInAir();

    void setBoundaryLeave(const Params &phase0_params,
                          const Params &phase1_params);

    void setBoundaryLand(const Params &phase0_params,
                         const Params &phase1_params);
  };

  /// Robot
  static gtdynamics::Robot getVision60Robot();

  static std::vector<Leg> getLegs(const gtdynamics::Robot &robot);

  inline static gtdynamics::Robot robot = getVision60Robot();
  inline static std::vector<Leg> legs = getLegs(robot);
  inline static gtsam::Point3 contact_in_com = Point3(0.14, 0, 0);

  /// Link ids
  inline static gtdynamics::LinkSharedPtr base_link = robot.link("body");
  inline static int base_id = base_link->id();

  std::vector<Point3> contact_in_world;

  /// Contact point info
  gtdynamics::PointOnLinks contact_points;
  std::vector<int> contact_link_ids;
  IndexSet leaving_link_indices;
  IndexSet landing_link_indices;

  /// Nominal configuration
  double nominal_height;
  Values nominal_values;
  std::vector<Point3> nominal_contact_in_world;
  double nominal_a;
  double nominal_b;

  Params params;
  gtdynamics::DynamicsGraph graph_builder;
  SharedNoiseModel des_pose_nm;
  SharedNoiseModel des_twist_nm;
  SharedNoiseModel des_q_nm;
  SharedNoiseModel des_v_nm;
  SharedNoiseModel min_torque_nm;
  SharedNoiseModel cpoint_cost_model;
  SharedNoiseModel c_force_model;
  SharedNoiseModel redundancy_model;

protected:
  static gtdynamics::OptimizerSetting getOptSetting(const Params &_params);

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

  NonlinearFactorGraph linkCollocationFactors(const uint8_t link_id,
                                              const size_t &k,
                                              const double &dt) const;

  NonlinearFactorGraph
  multiPhaseLinkCollocationFactors(const uint8_t link_id, const size_t &k,
                                   const Key &phase_key) const;

public:
  /// Constructor.
  IEVision60Robot(const Params &_params);

  /** <================= Constraints and Costs =================> **/
  /// Kinodynamic constraints at the specified time step.
  gtdynamics::EqualityConstraints eConstraints(const size_t k) const;

  gtdynamics::InequalityConstraints iConstraints(const size_t k) const;

  gtdynamics::EqualityConstraints
  initStateConstraints(const Pose3 &init_pose, const Vector6 &init_twist) const;

  NonlinearFactorGraph collocationCostsStep(const size_t k,
                                            const double dt) const;

  /// Costs for collocation across steps.
  NonlinearFactorGraph collocationCosts(const size_t num_steps,
                                        double dt) const;

  NonlinearFactorGraph
  multiPhaseCollocationCostsStep(const size_t k, const size_t phase_id) const;

  /// Cost for collocation that parameterize phase duration
  NonlinearFactorGraph multiPhaseCollocationCosts(const size_t start_step,
                                                  const size_t end_step,
                                                  const size_t phase_id) const;

  /// Costs for min torque objectives.
  NonlinearFactorGraph minTorqueCosts(const size_t num_steps) const;

  /// Costs for init condition and reaching target poses.
  NonlinearFactorGraph finalStateCosts(const Pose3 &des_pose,
                                       const Vector6 &des_twist,
                                       const size_t num_steps) const;

  /// Costs for init condition and reaching target poses.
  NonlinearFactorGraph
  stateCosts(const Values &values,
             const std::optional<KeyVector> &keys = {}) const;

  /** <================= Value Initialize Functions =================> **/
  /// Return values of one step satisfying kinodynamic constraints.
  Values getInitValuesStep(const size_t k = 0,
                           const Pose3 &base_pose = Pose3(Rot3::Identity(),
                                                          Point3(0, 0, 0.0)),
                           const Vector6 &base_twist = Vector6::Zero(),
                           const Vector6 &base_accel = Vector6::Zero(),
                           Values init_values_t = Values()) const;

  /// Return values of one step satisfying kinodynamic constraints. The
  /// variables specified by known_keys will remain unchanged. If known_keys not
  /// specified, basis_keys will be used.
  Values stepValues(const size_t k, const Values &init_values,
                    const std::optional<KeyVector> &known_keys = {}) const;

  /// Return values of one step by integration from previous step. The variables
  /// specified with integration_keys will be used for integration. Variables of
  /// a and d levels will be copied from the previous step.
  Values stepValuesByIntegration(
      const size_t k, const double dt, const Values &prev_values,
      const std::optional<KeyVector> &intagration_keys = {}) const;

  /// Return values of trajectory from start_step to end_step that satisfy all
  /// kinodynamic constraints and Euler collcation.
  Values trajectoryValuesByInterpolation(
      const size_t start_step, const size_t end_step, const double dt,
      const std::vector<Values> &boudnary_values,
      const std::string initialization_technique = "interp") const;

  // /// Return values of trajectory.
  // Values trajectoryValuesByIntegration(const size_t start_step,
  //                                      const size_t end_step, const double
  //                                      dt, const Values &prev_values) const;

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

  static void ExportValuesMultiPhase(const Values &values,
                                     const std::vector<size_t> &phase_num_steps,
                                     const std::string &file_path);

protected:
  static void PrintTorso(const Values &values, const size_t num_steps);

  static void PrintContactForces(const Values &values, const size_t num_steps);

  static void PrintJointValuesLevel(const Values &values,
                                    const size_t num_steps,
                                    const std::string level);

public:
  /** <===================== Utility Functions =====================> **/
  /** Transform contact wrench expressed in link frame to contact force
   * expressed in world frame. */
  static Vector3 GetContactForce(const Pose3 &pose, const Vector6 wrench,
                                 OptionalJacobian<3, 6> H_pose = {},
                                 OptionalJacobian<3, 6> H_wrench = {});

  /** Factor that enforce the relationship between contact force and contact
   * wernch. */
  NoiseModelFactor::shared_ptr contactForceFactor(const uint8_t link_id,
                                                  const size_t k) const;

  /** 6-dimensional vector that represents the redundancy in degress of freedom
   * in d-level with known acceleration. Only used in 4-leg contact case. */
  NoiseModelFactor::shared_ptr contactRedundancyFactor(const size_t k) const;

  /** Factor that enforce the q-level constraint at a known point contact. */
  NonlinearFactorGraph qPointContactFactors(const size_t k) const;

  /// Return optimizer setting.
  const gtdynamics::OptimizerSetting &opt() const {
    return graph_builder.opt();
  }

  /// Return function that select basis keys for constraint manifolds.
  BasisKeyFunc getBasisKeyFunc() const;

  KeyVector basisKeys(const size_t k,
                      bool include_init_state_constraints = false) const;
};

/**
 * Class of utilitis for Vision60 robot.
 */
class IEVision60RobotMultiPhase {
public:
  std::vector<IEVision60Robot> phase_robots_;
  std::vector<IEVision60Robot> boundary_robots_;
  std::vector<size_t> phase_num_steps_;
  std::vector<size_t> boundary_ks_;

public:
  IEVision60RobotMultiPhase(const std::vector<IEVision60Robot> &phase_robots,
                            const std::vector<IEVision60Robot> &boundary_robots,
                            const std::vector<size_t> &phase_num_steps);

  const IEVision60Robot &robotAtStep(const size_t k) const;

  NonlinearFactorGraph collocationCosts() const;

  /** Inequality constraints that limit the min phase durations. */
  gtdynamics::InequalityConstraints phaseMinDurationConstraints(
      const std::vector<double> &phases_min_dt) const;
};

/* ************************************************************************* */
/* <======================= Example Trajectories ==========================> */
/* ************************************************************************* */

/// Construct values of a vertical jumping trajectory. We pre-specified that all
/// feet leave the ground at the same time. The trajectory consists of two
/// phase: on-ground phase and in-air phase. In the on-ground phase, the robot
/// first accelerate its torso with constant acceleration, then reduce the
/// contact force uniformly to 0. In the in-air phase, the torques at all joints
/// are reduced uniformly to 0.
Values TrajectoryValuesVerticalJump(
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::vector<double> &phases_dt, const double torso_accel_z = 15,
    const size_t ground_switch_k = 5, bool use_trapezoidal = false);

Values TrajectoryValuesVerticalJumpDeprecated(
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::vector<double> &phases_dt, const double torso_accel_z = 15);

Values
TrajectoryWithTrapezoidal(const IEVision60RobotMultiPhase &vision60_multi_phase,
                          const std::vector<double> &phases_dt,
                          const Values &values);

/* ************************************************************************* */
/* <=================== Factory class for Retractor =======================> */
/* ************************************************************************* */

/** Hierarchical retractor creator for single phase. */
class Vision60HierarchicalRetractorCreator : public IERetractorCreator {
protected:
  const IEVision60Robot &robot_;
  const KinodynamicHierarchicalRetractor::Params &params_;
  bool use_basis_keys_;

public:
  Vision60HierarchicalRetractorCreator(
      const IEVision60Robot &robot,
      const KinodynamicHierarchicalRetractor::Params &params,
      bool use_basis_keys)
      : robot_(robot), params_(params), use_basis_keys_(use_basis_keys) {}

  virtual ~Vision60HierarchicalRetractorCreator() {}

  IERetractor::shared_ptr
  create(const IEConstraintManifold &manifold) const override;
};

/** Barrier retractor creator for single phase. */
class Vision60BarrierRetractorCreator : public IERetractorCreator {
protected:
  const IEVision60Robot &robot_;
  const BarrierRetractor::Params &params_;
  bool use_basis_keys_;

public:
  Vision60BarrierRetractorCreator(const IEVision60Robot &robot,
                                  const BarrierRetractor::Params &params,
                                  bool use_basis_keys)
      : robot_(robot), params_(params), use_basis_keys_(use_basis_keys) {}

  virtual ~Vision60BarrierRetractorCreator() {}

  IERetractor::shared_ptr
  create(const IEConstraintManifold &manifold) const override;
};

/** Hierarchical retractor creator for multiple phases. */
class Vision60MultiPhaseHierarchicalRetractorCreator
    : public IERetractorCreator {
protected:
  const IEVision60RobotMultiPhase &vision60_multi_phase_;
  const KinodynamicHierarchicalRetractor::Params &params_;
  bool use_basis_keys_;

public:
  Vision60MultiPhaseHierarchicalRetractorCreator(
      const IEVision60RobotMultiPhase &vision60_multi_phase,
      const KinodynamicHierarchicalRetractor::Params &params,
      bool use_basis_keys)
      : IERetractorCreator(params.use_varying_sigma, params.metric_sigmas),
        vision60_multi_phase_(vision60_multi_phase), params_(params),
        use_basis_keys_(use_basis_keys) {}

  virtual ~Vision60MultiPhaseHierarchicalRetractorCreator() {}

  IERetractor::shared_ptr
  create(const IEConstraintManifold &manifold) const override;
};

/** Barrier retractor creator for multiple phases. */
class Vision60MultiPhaseBarrierRetractorCreator : public IERetractorCreator {
protected:
  const IEVision60RobotMultiPhase &vision60_multi_phase_;
  const BarrierRetractor::Params &params_;
  bool use_basis_keys_;

public:
  Vision60MultiPhaseBarrierRetractorCreator(
      const IEVision60RobotMultiPhase &vision60_multi_phase,
      const BarrierRetractor::Params &params, bool use_basis_keys)
      : IERetractorCreator(params.use_varying_sigma, params.metric_sigmas),
        vision60_multi_phase_(vision60_multi_phase), params_(params),
        use_basis_keys_(use_basis_keys) {}

  virtual ~Vision60MultiPhaseBarrierRetractorCreator() {}

  IERetractor::shared_ptr
  create(const IEConstraintManifold &manifold) const override;
};

/** Tangent space creator for multiple phases. */
class Vision60MultiPhaseTspaceBasisCreator : public TspaceBasisCreator {
protected:
  const IEVision60RobotMultiPhase &vision60_multi_phase_;

public:
  Vision60MultiPhaseTspaceBasisCreator(
      const IEVision60RobotMultiPhase &vision60_multi_phase,
      const TspaceBasisParams::shared_ptr params =
          std::make_shared<TspaceBasisParams>(true))
      : TspaceBasisCreator(params),
        vision60_multi_phase_(vision60_multi_phase) {}

  TspaceBasis::shared_ptr create(const EqualityConstraints::shared_ptr constraints,
                                 const Values &values) const override;
};

} // namespace gtsam