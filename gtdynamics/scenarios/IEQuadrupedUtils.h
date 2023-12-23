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
#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/imanifold/IERetractor.h>
#include <gtdynamics/manifold/ManifoldOptimizer.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtdynamics/utils/PointOnLink.h>

#define ACTUATION_RMSE_TORQUE 0
#define ACTUATION_IMPULSE_SQR 1
#define ACTUATION_IMPULSE 2
#define ACTUATION_WORK_SQR 3
#define ACTUATION_WORK 4

#define JERK_AS_DIFF 0
#define JERK_DIV_DT 1

using gtdynamics::InequalityConstraints;

namespace gtsam {

/// Key to represent contact redundancy.
inline gtdynamics::DynamicsSymbol ContactRedundancyKey(int t = 0) {
  return gtdynamics::DynamicsSymbol::SimpleSymbol("CR", t);
}

/* ************************************************************************* */
/* <======================== Vision60 Single Phase ========================> */
/* ************************************************************************* */
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

  using TerrainHeightFunction =
      std::function<double(const Vector2 &point, OptionalJacobian<1, 2> H)>;

  // Function that models the flat terrain of constant height.
  static TerrainHeightFunction flatTerrainFunc(const double height);

  // Flat terrain with a hurdle modeled by a sinusoidal function.
  static TerrainHeightFunction sinHurdleTerrainFunc(const double center_x,
                                                    const double width,
                                                    const double height);

  struct Params {
    // environment
    gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
    double mu = 2.0;
    gtdynamics::CollocationScheme collocation =
        gtdynamics::CollocationScheme::Trapezoidal;

    /// Noise sigma for costs
    double sigma_q_col = 1e-2;
    double sigma_v_col = 1e-2;
    double sigma_pose_col = 1e-2;
    double sigma_twist_col = 1e-2;
    double sigma_actuation = 1e1;
    double sigma_jerk = 1e2;
    double sigma_des_pose = 1e-2;
    double sigma_des_twist = 1e-2;
    double sigma_des_point = 1e-2;
    double sigma_des_point_v = 1e-2;
    double sigma_phase_dt = 1e-4;
    double sigma_a_penalty = 1e2;
    double sigma_cf_jerk = 1e1;
    double sigma_symmetry = 1e-1;

    // tolerance for e constraints
    double tol_q = 1e-2;        // tolerance of q-level constraints
    double tol_v = 1e-2;        // tolerance of v-level constraints
    double tol_a = 1e-2;        // tolerance of a-level constraints
    double tol_dynamics = 1e-2; // tolerance of d-level constraints
    double tol_prior_q = 1e-2;
    double tol_prior_v = 1e-2;

    // tolerance for i constraints
    double tol_jl = 0.1;
    double tol_tl = 0.1;
    double tol_fc = 0.1;
    double tol_cf = 0.1;
    double tol_phase_dt = 0.1;

    /// Option for values
    bool express_redundancy = true;
    bool express_contact_force = false;
    bool ad_basis_using_torques = false;
    // Option for e-constraints
    bool include_state_constraints = true;
    Values state_constrianed_values;
    bool boundary_constrain_a = true;
    // Option for i-constraints
    bool include_joint_limits = false;
    bool include_collision_free_s = false;
    bool include_collision_free_h = false;
    bool include_collision_free_z = false;
    bool include_torque_limits = false;
    bool include_friction_cone = false;
    bool include_phase_duration_limits = false;
    bool i_constraints_symmetry = false;

    std::vector<double> phases_min_dt;
    std::map<std::string, double> joint_lower_limits;
    std::map<std::string, double> joint_upper_limits;
    std::map<std::string, double> torque_lower_limits;
    std::map<std::string, double> torque_upper_limits;
    std::vector<std::pair<std::string, Point3>> collision_checking_points_s;
    std::vector<std::pair<std::string, Point3>> collision_checking_points_h;
    std::vector<std::pair<std::string, Point3>> collision_checking_points_z;
    std::vector<std::pair<Point3, double>> sphere_obstacles;
    std::vector<std::pair<Point2, double>> hurdle_obstacles;
    TerrainHeightFunction terrain_height_function = flatTerrainFunc(0.0);
    double accel_panalty_threshold = 100.0;
    double cf_jerk_threshold = 100.0;

    // Option for costs
    bool include_collocation_costs = false;
    bool include_actuation_costs = false;
    bool include_jerk_costs = false;
    bool include_state_costs = false;
    bool include_accel_penalty = false;
    bool include_cf_jerk_costs = false;
    bool include_phase_duration_prior_costs = false;
    bool include_symmetry_costs = false;
    bool include_collision_free_z_inter_cost = false;
    bool collision_as_cost = false;
    bool joint_limits_as_cost = false;
    bool torque_limits_as_cost = false;
    bool friction_cone_as_cost = false;
    bool phase_duration_limit_as_cost = false;

    int actuation_cost_option = ACTUATION_RMSE_TORQUE;
    int jerk_cost_option = JERK_AS_DIFF;
    Values state_cost_values;
    std::vector<std::tuple<size_t, Point3, Point3, size_t>> state_cost_points;
    std::vector<std::tuple<size_t, Point3, Vector3, size_t>>
        state_cost_point_vels;
    std::vector<double> phase_prior_dt;
    double dt_threshold = 1.0;
    double step_div_ratio = 0.5;
    bool use_smooth_barrier_for_cost = false;
    double smooth_barrier_buffer_width = 1.0;

    // Option for logging
    bool eval_details = true;
    bool eval_collo_step = false;

    /// Constructor
    Params() = default;

    using shared_ptr = std::shared_ptr<Params>;
  };

  struct PhaseInfo {
    using shared_ptr = std::shared_ptr<PhaseInfo>;

    // phase info
    IndexSet contact_indices;
    IndexSet leaving_indices;
    IndexSet landing_indices;

    PhaseInfo(const IndexSet &_contact_indices,
              const IndexSet &_leaving_indices,
              const IndexSet &_landing_indices)
        : contact_indices(_contact_indices), leaving_indices(_leaving_indices),
          landing_indices(_landing_indices) {}

    void print() const;

    static shared_ptr Ground();

    static shared_ptr BackOnGround();

    static shared_ptr FrontOnGround();

    static shared_ptr InAir();

    static shared_ptr BoundaryLeave(const PhaseInfo &phase0_params,
                                    const PhaseInfo &phase1_params);

    static shared_ptr BoundaryLand(const PhaseInfo &phase0_params,
                                   const PhaseInfo &phase1_params);
  };

  friend class IEVision60RobotMultiPhase;

  /// Robot
  static gtdynamics::Robot getVision60Robot();
  static std::vector<Leg> getLegs(const gtdynamics::Robot &robot);
  static std::map<std::string, gtsam::Point3> getTorsoCorners();

  inline static gtdynamics::Robot robot = getVision60Robot();
  inline static std::vector<Leg> legs = getLegs(robot);
  inline static gtsam::Point3 contact_in_com = Point3(0.14, 0, 0);
  inline static std::map<std::string, gtsam::Point3> torso_corners_in_com =
      getTorsoCorners();

  /// Link ids
  inline static gtdynamics::LinkSharedPtr base_link = robot.link("body");
  inline static int base_id = base_link->id();

  static bool isLeft(const std::string &str);
  static bool isRight(const std::string &str);
  static std::string frontOrRear(const std::string &str);
  static bool isHip(const std::string &str);
  static std::string counterpart(const std::string &str);

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

  Params::shared_ptr params;
  PhaseInfo::shared_ptr phase_info;
  gtdynamics::DynamicsGraph graph_builder;
  SharedNoiseModel des_pose_nm;
  SharedNoiseModel des_twist_nm;
  SharedNoiseModel des_point_nm;
  SharedNoiseModel des_point_v_nm;
  SharedNoiseModel des_q_nm;
  SharedNoiseModel des_v_nm;
  SharedNoiseModel actuation_nm;
  SharedNoiseModel jerk_nm;
  SharedNoiseModel cf_jerk_nm;
  SharedNoiseModel cpoint_cost_model;
  SharedNoiseModel c_force_model;
  SharedNoiseModel redundancy_model;
  SharedNoiseModel symmetry_nm;

  static gtdynamics::OptimizerSetting getOptSetting(const Params &_params);

  Values getNominalConfiguration(const double height = 0) const;

public:
  /// Constructor.
  IEVision60Robot(const Params::shared_ptr &_params,
                  const PhaseInfo::shared_ptr &_phase_info);
  
  void moveContactPoints(const double forward_distance);

  /** <=================== Single Equality Constraint  ===================> **/
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

  /** <================= Single Inequality Constraint  ===================> **/

  /// Collision free with a spherical object.
  gtdynamics::DoubleExpressionInequality::shared_ptr
  obstacleCollisionFreeConstraint(const size_t link_idx, const size_t k,
                                  const Point3 &p_l, const Point3 &center,
                                  const double radius) const;

  /// Collision free with a spherical object.
  gtdynamics::DoubleExpressionInequality::shared_ptr
  hurdleCollisionFreeConstraint(const size_t link_idx, const size_t k,
                                const Point3 &p_l, const Point2 &center,
                                const double radius) const;

  /// Collision free with ground.
  gtdynamics::DoubleExpressionInequality::shared_ptr
  groundCollisionFreeConstraint(const std::string &link_name, const size_t k,
                                const Point3 &p_l) const;

  /// Collision free with ground.
  gtdynamics::DoubleExpressionInequality::shared_ptr
  groundCollisionFreeInterStepConstraint(const std::string &link_name,
                                         const size_t k, const double ratio,
                                         const Point3 &p_l) const;

  gtdynamics::DoubleExpressionInequality::shared_ptr
  jointUpperLimitConstraint(const std::string &j_name, const size_t k,
                            const double upper_limit) const;

  gtdynamics::DoubleExpressionInequality::shared_ptr
  jointLowerLimitConstraint(const std::string &j_name, const size_t k,
                            const double lower_limit) const;

  gtdynamics::DoubleExpressionInequality::shared_ptr
  torqueUpperLimitConstraint(const std::string &j_name, const size_t k,
                             const double upper_limit) const;

  gtdynamics::DoubleExpressionInequality::shared_ptr
  torqueLowerLimitConstraint(const std::string &j_name, const size_t k,
                             const double lower_limit) const;

  gtdynamics::DoubleExpressionInequality::shared_ptr
  frictionConeConstraint(const std::string &link_name, const size_t k) const;

  /** <======================= Single Cost Factor ========================> **/
  NoiseModelFactor::shared_ptr
  multiPhaseLinkPoseCollocationFactor(const uint8_t link_id, const size_t &k,
                                      const Key &phase_key) const;

  NoiseModelFactor::shared_ptr
  multiPhaseLinkTwistCollocationFactor(const uint8_t link_id, const size_t &k,
                                       const Key &phase_key) const;

  NoiseModelFactor::shared_ptr
  multiPhaseJointQCollocationFactor(const uint8_t joint_id, const size_t &k,
                                    const Key &phase_key) const;

  NoiseModelFactor::shared_ptr
  multiPhaseJointVCollocationFactor(const uint8_t joint_id, const size_t &k,
                                    const Key &phase_key) const;

  NonlinearFactorGraph linkCollocationFactors(const uint8_t link_id,
                                              const size_t &k,
                                              const double &dt) const;

  NoiseModelFactor::shared_ptr statePointCostFactor(const size_t link_id,
                                                    const Point3 &point_l,
                                                    const Point3 &point_w,
                                                    const size_t k) const;

  NoiseModelFactor::shared_ptr statePointVelCostFactor(const size_t link_id,
                                                       const Point3 &point_l,
                                                       const Vector3 &vel_w,
                                                       const size_t k) const;

  NoiseModelFactor::shared_ptr jerkCostFactor(const size_t joint_id,
                                              const size_t k,
                                              const Key &phase_key) const;

  NoiseModelFactor::shared_ptr
  contactForceJerkCostFactor(const size_t link_id, const size_t k,
                             const Key &phase_key) const;

  /** <==================== Step Equality Constraint  ====================> **/
  NonlinearFactorGraph getConstraintsGraphStepQ(const int t) const;

  NonlinearFactorGraph getConstraintsGraphStepV(const int t) const;

  NonlinearFactorGraph getConstraintsGraphStepAD(const int t) const;

  /// Dynamcis factors without friction cone factors (as they are moved to
  /// inequality constraints).
  NonlinearFactorGraph DynamicsFactors(const size_t k) const;

  /** <=================== Step Inequality Constraint  ===================> **/
  InequalityConstraints stepFrictionConeConstraints(const size_t k) const;

  InequalityConstraints stepJointLimitConstraints(const size_t k) const;

  InequalityConstraints stepTorqueLimitConstraints(const size_t k) const;

  /// Collision free with obstacles.
  InequalityConstraints
  stepObstacleCollisionFreeConstraints(const size_t k) const;

  InequalityConstraints
  stepHurdleCollisionFreeConstraints(const size_t k) const;

  /// Collision free with ground.
  InequalityConstraints
  stepGroundCollisionFreeConstraints(const size_t k) const;

  InequalityConstraints
  interStepGroundCollisionFreeConstraints(const size_t k) const;

  InequalityConstraints stepIConstraintsQ(const size_t k) const;

  InequalityConstraints stepIConstraintsV(const size_t k) const;

  InequalityConstraints stepIConstraintsAD(const size_t k) const;

  /** <=========================== Step Cost  ============================> **/
  NonlinearFactorGraph stepCollocationCosts(const size_t k,
                                            const double dt) const;

  NonlinearFactorGraph
  stepMultiPhaseCollocationCosts(const size_t k, const size_t phase_id) const;

  /// Cost based on the root-mean-square at step k.
  NonlinearFactorGraph stepActuationRmseTorqueCosts(const size_t k) const;

  /// Cost based on the impulse of all joints from step k to k+1.
  NonlinearFactorGraph stepActuationImpulseCosts(const size_t k,
                                                 const size_t phase_idx,
                                                 bool apply_sqrt = false) const;

  /// Cost based on the work done by all joints from step k to k+1.
  NonlinearFactorGraph stepActuationWorkCosts(const size_t k,
                                              bool apply_sqrt = false) const;

  NonlinearFactorGraph stepSymmetryCosts(const size_t k) const;

  NonlinearFactorGraph stepJerkCosts(const size_t k,
                                     const Key &phase_key) const;

  NonlinearFactorGraph stepContactForceJerkCosts(const size_t k,
                                                 const Key &phase_key) const;

public:
  /** <================= Constraints and Costs =================> **/
  /// Kinodynamic constraints at the specified time step.
  gtdynamics::EqualityConstraints eConstraints(const size_t k) const;

  InequalityConstraints iConstraints(const size_t k) const;

  gtdynamics::EqualityConstraints stateConstraints() const;

  /// Costs for collocation across steps.
  NonlinearFactorGraph collocationCosts(const size_t num_steps,
                                        double dt) const;

  /// Costs for min torque objectives.
  NonlinearFactorGraph actuationCosts(const size_t num_steps) const;

  /// Costs for minimize jerk in torques.
  NonlinearFactorGraph jerkCosts(const size_t num_steps) const;

  /// Penalty for large joint accelerations above threshold. In theory, the
  /// joint acceleration of a realistic trajectory can be infinitely large (at
  /// leg singularity configurations). However, due to discretization error in
  /// collocation, optimizers can take advantage of it to make abrupt change
  /// in joint velocity between consecutive time steps. Joint acceleration
  /// penalty is added to prevent the optimizer from doing such tricks.
  NonlinearFactorGraph accelPenaltyCosts(const size_t num_steps) const;

  /// Costs for init condition and reaching target poses.
  NonlinearFactorGraph stateCosts() const;

  /** <================= Value Initialize Functions =================> **/
  /// Return values of one step satisfying kinodynamic constraints.
  Values getInitValuesStep(const size_t k = 0,
                           const Pose3 &base_pose = Pose3(Rot3::Identity(),
                                                          Point3(0, 0, 0.0)),
                           const Vector6 &base_twist = Vector6::Zero(),
                           const Vector6 &base_accel = Vector6::Zero(),
                           Values init_values_t = Values()) const;

  /// Compute q-level values (poses, joint angles) of a single time step that
  /// satisfy all the equality constraints.
  Values stepValuesQ(const size_t k, const Values &init_values,
                     const KeyVector &known_q_keys = KeyVector(),
                     bool satisfy_i_constriants = false,
                     bool ensure_feasible = true) const;

  Values stepValuesV(const size_t k, const Values &init_values,
                     const Values &known_q_values,
                     const KeyVector &known_v_keys = KeyVector(),
                     bool satisfy_i_constriants = false,
                     bool ensure_feasible = true) const;

  Values stepValuesAD(const size_t k, const Values &init_values,
                      const Values &known_qv_values,
                      const KeyVector &known_ad_keys = KeyVector(),
                      bool satisfy_i_constriants = false,
                      bool ensure_feasible = true) const;

  Values stepValuesQV(const size_t k, const Values &init_values,
                      const std::optional<KeyVector> &known_keys = {},
                      bool satisfy_i_constriants = false,
                      bool ensure_feasible = true) const;

  /** Return values of one step satisfying kinodynamic constraints. The
   * variables specified by known_keys will remain unchanged. If known_keys not
   * specified, basis_keys will be used.
   * @param k time step
   * @param init_values init estimate of variables of the step
   * @param known_keys variables that can be used as priors
   * @param satisfy_i_constraints include i-constraints
   * @param ensure_feasible run a 2nd phase optimization without priors to
   * ensure constriant satisfaction
   */
  Values stepValues(const size_t k, const Values &init_values,
                    const std::optional<KeyVector> &known_keys = {},
                    bool satisfy_i_constriants = false,
                    bool ensure_feasible = true) const;

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

  /// Print pose, twist and acceleration of torso link over time steps.
  static void PrintTorso(const Values &values, const size_t num_steps);

  /// Print contact force over time steps.
  static void PrintContactForces(const Values &values, const size_t num_steps);

  /// Print joint angles(type="q"), velocity(type="v"), acceleration(type="a"),
  /// torque(type="T") over time steps.
  static void PrintJointValuesLevel(const Values &values,
                                    const size_t num_steps,
                                    const std::string type);

public:
  /** <===================== Utility Functions =====================> **/
  /// Return optimizer setting.
  const gtdynamics::OptimizerSetting &opt() const {
    return graph_builder.opt();
  }

  /// Return function that select basis keys for constraint manifolds.
  BasisKeyFunc getBasisKeyFunc() const;

  KeyVector basisKeys(const size_t k,
                      bool include_init_state_constraints = false) const;
};

/* ************************************************************************* */
/* <======================== Vision60 Multi Phase =========================> */
/* ************************************************************************* */
class IEVision60RobotMultiPhase {
public:
  std::vector<IEVision60Robot> phase_robots_;
  std::vector<IEVision60Robot> boundary_robots_;
  std::vector<size_t> phase_num_steps_;
  std::vector<size_t> boundary_ks_;
  using shared_ptr = std::shared_ptr<IEVision60RobotMultiPhase>;

public:
  IEVision60RobotMultiPhase(const std::vector<IEVision60Robot> &phase_robots,
                            const std::vector<IEVision60Robot> &boundary_robots,
                            const std::vector<size_t> &phase_num_steps);

  const IEVision60Robot::Params::shared_ptr &params() const {
    return phase_robots_.at(0).params;
  }

  size_t numSteps() const {
    return std::accumulate(phase_num_steps_.begin(), phase_num_steps_.end(), 0);
  }

  const IEVision60Robot &robotAtStep(const size_t k) const;

  /** <================= costs =================> **/
  NonlinearFactorGraph collocationCosts() const;

  NonlinearFactorGraph actuationCosts() const;

  NonlinearFactorGraph jerkCosts() const;

  NonlinearFactorGraph stateCosts() const;

  NonlinearFactorGraph phaseDurationPriorCosts() const;

  NonlinearFactorGraph accelPenaltyCosts() const;

  NonlinearFactorGraph contactForceJerkCosts() const;

  NonlinearFactorGraph symmetryCosts() const;

  /// Pair of cost with associated type.
  std::vector<std::pair<std::string, NonlinearFactorGraph>>
  classifiedCosts() const;

  /// Separate factor graphs containing collocation factors of each step.
  std::vector<NonlinearFactorGraph> collocationFactorsByStep() const;

  /// Costs of all types of all time steps.
  NonlinearFactorGraph costs() const;

  /** <================= inequality constraints =================> **/
  InequalityConstraints groundCollisionFreeConstraints() const;

  InequalityConstraints groundCollisionFreeInterStepConstraints() const;

  InequalityConstraints obstacleCollisionFreeConstraints() const;

  InequalityConstraints hurdleCollisionFreeConstraints() const;

  InequalityConstraints jointLimitConstraints() const;

  InequalityConstraints torqueLimitConstraints() const;

  InequalityConstraints frictionConeConstraints() const;

  /** Inequality constraints that limit the min phase durations. */
  InequalityConstraints phaseMinDurationConstraints() const;

  /// Pair of inequality constriants with associated type.
  std::vector<std::pair<std::string, InequalityConstraints>>
  classifiedIConstraints() const;

  /// Inequality constraints of all types of all time steps.
  InequalityConstraints iConstraints() const;

  /** <======================= equality constraints ======================> **/
  /// Equality constraints of all types of all time steps.
  EqualityConstraints eConstraints() const;

  /** <======================= evaluation functions ======================> **/
  gtdynamics::EqConsOptProblem::EvalFunc costsEvalFunc() const;

  /// evaluate the error of collocation of each time step, and accumulative
  /// error over the trajectory.
  void evaluateCollocation(const Values &values) const;

  // void exportEvaluation(const Values &values,
  //                       const std::string &file_path) const;
};

/* ************************************************************************* */
/* <=================== Factory class for Retractor =======================> */
/* ************************************************************************* */

/** Hierarchical retractor creator for single phase. */
class Vision60HierarchicalRetractorCreator : public IERetractorCreator {
protected:
  const IEVision60Robot &robot_;
  bool use_basis_keys_;

public:
  Vision60HierarchicalRetractorCreator(
      const IEVision60Robot &robot, const IERetractorParams::shared_ptr &params,
      bool use_basis_keys)
      : IERetractorCreator(params), robot_(robot),
        use_basis_keys_(use_basis_keys) {}

  virtual ~Vision60HierarchicalRetractorCreator() {}

  IERetractor::shared_ptr
  create(const IEConstraintManifold &manifold) const override;
};

/** Barrier retractor creator for single phase. */
class Vision60BarrierRetractorCreator : public IERetractorCreator {
protected:
  const IEVision60Robot &robot_;
  bool use_basis_keys_;

public:
  Vision60BarrierRetractorCreator(const IEVision60Robot &robot,
                                  const IERetractorParams::shared_ptr &params,
                                  bool use_basis_keys)
      : IERetractorCreator(params), robot_(robot),
        use_basis_keys_(use_basis_keys) {}

  virtual ~Vision60BarrierRetractorCreator() {}

  IERetractor::shared_ptr
  create(const IEConstraintManifold &manifold) const override;
};

/** Hierarchical retractor creator for multiple phases. */
class Vision60MultiPhaseHierarchicalRetractorCreator
    : public IERetractorCreator {
protected:
  IEVision60RobotMultiPhase::shared_ptr vision60_multi_phase_;
  bool use_basis_keys_;

public:
  Vision60MultiPhaseHierarchicalRetractorCreator(
      const IEVision60RobotMultiPhase::shared_ptr &vision60_multi_phase,
      const IERetractorParams::shared_ptr &params, bool use_basis_keys)
      : IERetractorCreator(params), vision60_multi_phase_(vision60_multi_phase),
        use_basis_keys_(use_basis_keys) {}

  virtual ~Vision60MultiPhaseHierarchicalRetractorCreator() {}

  IERetractor::shared_ptr
  create(const IEConstraintManifold &manifold) const override;
};

/** Barrier retractor creator for multiple phases. */
class Vision60MultiPhaseBarrierRetractorCreator : public IERetractorCreator {
protected:
  IEVision60RobotMultiPhase::shared_ptr vision60_multi_phase_;
  bool use_basis_keys_;

public:
  Vision60MultiPhaseBarrierRetractorCreator(
      const IEVision60RobotMultiPhase::shared_ptr &vision60_multi_phase,
      const IERetractorParams::shared_ptr &params, bool use_basis_keys)
      : IERetractorCreator(params), vision60_multi_phase_(vision60_multi_phase),
        use_basis_keys_(use_basis_keys) {}

  virtual ~Vision60MultiPhaseBarrierRetractorCreator() {}

  IERetractor::shared_ptr
  create(const IEConstraintManifold &manifold) const override;
};

/** Tangent space creator for multiple phases. */
class Vision60MultiPhaseTspaceBasisCreator : public TspaceBasisCreator {
protected:
  IEVision60RobotMultiPhase::shared_ptr vision60_multi_phase_;

public:
  Vision60MultiPhaseTspaceBasisCreator(
      const IEVision60RobotMultiPhase::shared_ptr &vision60_multi_phase,
      const TspaceBasisParams::shared_ptr params =
          std::make_shared<TspaceBasisParams>(true))
      : TspaceBasisCreator(params),
        vision60_multi_phase_(vision60_multi_phase) {}

  TspaceBasis::shared_ptr
  create(const EqualityConstraints::shared_ptr constraints,
         const Values &values) const override;
};

Values
TrajectoryWithTrapezoidal(const IEVision60RobotMultiPhase &vision60_multi_phase,
                          const std::vector<double> &phases_dt,
                          const Values &values);

/* ************************************************************************* */
/* <========================= Experiment Utils ============================> */
/* ************************************************************************* */
void EvaluateAndExportIELMResult(
    const IEConsOptProblem &problem,
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::pair<IEResultSummary, IELMItersDetails> &ielm_result,
    const std::string &scenario_folder, bool print_values = false,
    bool print_iter_details = false);

void EvaluateAndExportBarrierResult(
    const IEConsOptProblem &problem,
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::pair<IEResultSummary, BarrierItersDetail> &barrier_result,
    const std::string &scenario_folder, bool print_values = false);

void EvaluateAndExportInitValues(
    const IEConsOptProblem &problem,
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::string &scenario_folder, bool print_values = false);

void ExportOptimizationProgress(
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::string &scenario_folder, const IELMItersDetails &iters_details);

} // namespace gtsam

/* ************************************************************************* */
/* <========================= Example Scenarios ===========================> */
/* ************************************************************************* */

namespace quadruped_vertical_jump {
gtsam::IEVision60RobotMultiPhase::shared_ptr
GetVision60MultiPhase(const gtsam::IEVision60Robot::Params::shared_ptr &params,
                      const std::vector<size_t> &phase_num_steps);

/// Construct values of a vertical jumping trajectory. We pre-specified that all
/// feet leave the ground at the same time. The trajectory consists of two
/// phase: on-ground phase and in-air phase. In the on-ground phase, the robot
/// first accelerate its torso with constant acceleration, then reduce the
/// contact force uniformly to 0. In the in-air phase, the torques at all joints
/// are reduced uniformly to 0.
gtsam::Values InitValuesTrajectory(
    const gtsam::IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::vector<double> &phases_dt, const double torso_accel_z = 15,
    const size_t ground_switch_k = 5, bool use_trapezoidal = false);

/// Create an initial trajectory that is not dynamically feasible.
gtsam::Values InitValuesTrajectoryInfeasible(
    const gtsam::IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::vector<double> &phases_dt);

gtsam::Values InitValuesTrajectoryDeprecated(
    const gtsam::IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::vector<double> &phases_dt, const double torso_accel_z = 15);
} // namespace quadruped_vertical_jump

namespace quadruped_forward_jump {
gtsam::IEVision60RobotMultiPhase::shared_ptr
GetVision60MultiPhase(const gtsam::IEVision60Robot::Params::shared_ptr &params,
                      const std::vector<size_t> &phase_num_steps);

/// Construct values of a robot forward jump trajectory. Consisting of 5 phases:
/// on-ground -> back-contact -> in-air -> front-contact -> on-gorund.
gtsam::Values InitValuesTrajectory(
    const gtsam::IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::vector<double> &phases_dt, bool include_i_constriants,
    bool ensure_feasible, bool ignore_last_step = false);
} // namespace quadruped_forward_jump

namespace quadruped_forward_jump_land {
gtsam::IEVision60RobotMultiPhase::shared_ptr
GetVision60MultiPhase(const gtsam::IEVision60Robot::Params::shared_ptr &params,
                      const std::vector<size_t> &phase_num_steps,
                      const double jump_distance);

/// Construct values of a robot forward jump trajectory. Consisting of 5 phases:
/// on-ground -> back-contact -> in-air -> front-contact -> on-gorund.
gtsam::Values InitValuesTrajectory(
    const gtsam::IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::vector<double> &phases_dt, bool include_i_constriants,
    bool ensure_feasible);
} // namespace quadruped_forward_jump_land
