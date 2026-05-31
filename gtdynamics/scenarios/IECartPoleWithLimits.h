/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IECartPoleWithLimits.h
 * @brief Cart-pole trajectory optimization problem with cart and force limits.
 * @author Yetong Zhang
 */

#pragma once

#include <gtdynamics/cmcopt/IERetractor.h>
#include <gtdynamics/cmopt/ConstraintManifold.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>

#include <string>

namespace gtdynamics {
using namespace gtsam;

/**
 * Cart-pole problem with equality dynamics constraints and bound constraints on
 * cart position and cart force.
 */
class IECartPoleWithLimits {
 public:
  /// Robot model fixed at the base link.
  Robot robot_ = CreateRobotFromFile(kUrdfPath + std::string("cart_pole.urdf"))
                     .fixLink("l0");

  /// Prismatic cart joint id.
  size_t p_joint_id_ = robot_.joint("j0")->id();

  /// Revolute pole joint id.
  size_t r_joint_id_ = robot_.joint("j1")->id();

  /// Desired initial state `[x, xdot, xddot, theta, thetadot, thetaaccel]`.
  Vector6 initial_state_ = Vector6::Constant(6, 0);

  /// Desired terminal state `[x, xdot, xddot, theta, thetadot, thetaaccel]`.
  Vector6 target_state_ = (Vector(6) << 0, 0, 0, M_PI, 0, 0).finished();

  /// Gravity vector used by the dynamics graph.
  Vector3 gravity_ = Vector3(0, 0, -9.8);

  /// Whether the terminal cart position is constrained or only the pole state.
  bool constrain_final_x_ = false;

  /// Collocation scheme used for trajectory costs.
  CollocationScheme collocation_scheme_ = CollocationScheme::Trapezoidal;

  /// Lower cart position limit.
  double x_min_ = -.2;

  /// Upper cart position limit.
  double x_max_ = .2;

  /// Lower cart force limit.
  double f_min_ = -100;

  /// Upper cart force limit.
  double f_max_ = 100;

  /// Constraint tolerance.
  double tol_ = 1.0;

  /// Optimizer noise model settings.
  OptimizerSetting opt_ = getOptSetting();

  /// Dynamics graph builder configured for this problem.
  DynamicsGraph graph_builder_ = DynamicsGraph(opt_, gravity_);

  /// Sigma for terminal position objectives.
  double sigma_pos_objective_ = 1e-5;

  /// Sigma for other terminal objectives.
  double sigma_objectives_ = 5e-3;

  /// Sigma for cart force minimization.
  double sigma_min_torque_ = 2e1;

  /// Noise model for terminal position objectives.
  SharedNoiseModel pos_objectives_model_ =
      noiseModel::Isotropic::Sigma(1, sigma_pos_objective_);

  /// Noise model for terminal velocity objectives.
  SharedNoiseModel objectives_model_ = noiseModel::Isotropic::Sigma(
      1, sigma_objectives_);

  /// Noise model for cart force minimization.
  SharedNoiseModel control_model_ =
      noiseModel::Isotropic::Sigma(1, sigma_min_torque_);

  /** Return optimizer settings used by this cart-pole problem. */
  static OptimizerSetting getOptSetting() {
    auto opt = OptimizerSetting();
    double tol = 1.0;
    double sigma_collo = 1e-3;
    opt.bp_cost_model = noiseModel::Isotropic::Sigma(6, tol);
    opt.bv_cost_model = noiseModel::Isotropic::Sigma(6, tol);
    opt.ba_cost_model = noiseModel::Isotropic::Sigma(6, tol);
    opt.p_cost_model = noiseModel::Isotropic::Sigma(6, tol);
    opt.v_cost_model = noiseModel::Isotropic::Sigma(6, tol);
    opt.a_cost_model = noiseModel::Isotropic::Sigma(6, tol);
    opt.f_cost_model = noiseModel::Isotropic::Sigma(6, tol);
    opt.fa_cost_model = noiseModel::Isotropic::Sigma(6, tol);
    opt.t_cost_model = noiseModel::Isotropic::Sigma(1, tol);
    opt.cp_cost_model = noiseModel::Isotropic::Sigma(1, tol);
    opt.cfriction_cost_model = noiseModel::Isotropic::Sigma(1, tol);
    opt.cv_cost_model = noiseModel::Isotropic::Sigma(3, tol);
    opt.ca_cost_model = noiseModel::Isotropic::Sigma(3, tol);
    opt.planar_cost_model = noiseModel::Isotropic::Sigma(3, tol);
    opt.prior_q_cost_model = noiseModel::Isotropic::Sigma(1, tol);
    opt.prior_qv_cost_model = noiseModel::Isotropic::Sigma(1, tol);
    opt.prior_qa_cost_model = noiseModel::Isotropic::Sigma(1, tol);
    opt.prior_t_cost_model = noiseModel::Isotropic::Sigma(1, tol);

    opt.q_col_cost_model = noiseModel::Isotropic::Sigma(1, sigma_collo);
    opt.v_col_cost_model = noiseModel::Isotropic::Sigma(1, sigma_collo);
    return opt;
  }

  /** Constructor. */
  IECartPoleWithLimits() {}

  /** Return equality constraints for a single time step. */
  NonlinearEqualityConstraints eConstraints(const int k) const;

  /** Return position and force inequality constraints for a single time step. */
  NonlinearInequalityConstraints iConstraints(const int k) const;

  /** Return equality constraints that specify the initial joint state. */
  NonlinearEqualityConstraints initStateConstraints() const;

  /** Return equality constraints that specify the terminal joint state. */
  NonlinearEqualityConstraints finalStateConstraints(size_t num_steps) const;

  /** Return terminal state objective factors. */
  NonlinearFactorGraph finalStateCosts(size_t num_steps) const;

  /** Return cart force minimization factors. */
  NonlinearFactorGraph minTorqueCosts(size_t num_steps) const;

  /** Return trajectory collocation factors. */
  NonlinearFactorGraph collocationCosts(size_t num_steps, double dt) const;

  /** Compute a full dynamics-consistent state from cart and pole coordinates. */
  Values computeValues(const size_t &k, const double &x, const double &v,
                       const double &q, const double &w) const;

  /** Print trajectory values in a compact table. */
  static void PrintValues(const Values &values, const size_t num_steps);

  /** Print tangent-vector values in a compact table. */
  static void PrintDelta(const VectorValues &values, const size_t num_steps);

  /** Export trajectory values to a CSV file. */
  static void ExportValues(const Values &values, const size_t num_steps,
                           const std::string &file_path);

  // static void ExportVector(const VectorValues &values, const size_t
  // num_steps,
  //                          const std::string &file_path);

  /** Return initial values set as the rest pose for all steps. */
  Values getInitValuesZero(size_t num_steps) const;

  /** Return initial values interpolated between initial and target states. */
  Values getInitValuesInterp(size_t num_steps) const;

  /** Return a function that selects basis keys for constraint manifolds. */
  BasisKeyFunc getBasisKeyFunc() const;
};

/** Retraction rule that clamps cart position and force to their limits. */
class CartPoleWithLimitsRetractor : public IERetractor {
 protected:
  /// Cart-pole problem parameters.
  const IECartPoleWithLimits &cp_;

 public:
  /** Constructor. */
  CartPoleWithLimitsRetractor(const IECartPoleWithLimits &cp)
      : IERetractor(), cp_(cp) {}

  /** Retract a tangent update and return a feasible constrained manifold. */
  IEConstraintManifold retract(
      const IEConstraintManifold *manifold, const VectorValues &delta,
      const std::optional<IndexSet> &blocking_indices = {},
      IERetractInfo *retract_info = nullptr) const override;
};

}  // namespace gtdynamics
