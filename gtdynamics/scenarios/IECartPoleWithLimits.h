#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/imanifold/IERetractor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/manifold/ConstraintManifold.h>

namespace gtsam {

class IECartPoleWithLimits {
public:
  // robot and environmental settings
  gtdynamics::Robot robot =
      gtdynamics::CreateRobotFromFile(gtdynamics::kUrdfPath +
                                      std::string("cart_pole.urdf"))
          .fixLink("l0");
  size_t p_joint_id = robot.joint("j0")->id();
  size_t r_joint_id = robot.joint("j1")->id();
  Vector6 X_i = Vector6::Constant(6, 0);
  Vector6 X_T = (Vector(6) << 0, 0, 0, M_PI, 0, 0).finished();
  Vector3 gravity = Vector3(0, 0, -9.8);
  bool constrain_final_x = false;
  gtdynamics::CollocationScheme collocation_scheme = gtdynamics::CollocationScheme::Trapezoidal;

  // limits on x-range and force
  double x_min = -.2;
  double x_max = .2;
  double f_min = -100;
  double f_max = 100;

  // noise/tolerance settings for costs and constraints
  double tol = 1.0;
  gtdynamics::OptimizerSetting opt = getOptSetting();
  gtdynamics::DynamicsGraph graph_builder =
      gtdynamics::DynamicsGraph(opt, gravity);
  double sigma_pos_objective = 1e-5;
  double sigma_objectives = 5e-3;
  double sigma_min_torque = 2e1;
  SharedNoiseModel pos_objectives_model = noiseModel::Isotropic::Sigma(1, sigma_pos_objective);  // Pos objectives.
  SharedNoiseModel objectives_model = noiseModel::Isotropic::Sigma(1, sigma_objectives);  // Additional objectives.
  SharedNoiseModel control_model = noiseModel::Isotropic::Sigma(1, sigma_min_torque);       // Controls.
  


  static gtdynamics::OptimizerSetting getOptSetting() {
    auto opt = gtdynamics::OptimizerSetting();
    double tol = 1.0;
    double sigma_collo = 1e-3;
    opt.bp_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, tol);
    opt.bv_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, tol);
    opt.ba_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, tol);
    opt.p_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, tol);
    opt.v_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, tol);
    opt.a_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, tol);
    opt.f_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, tol);
    opt.fa_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, tol);
    opt.t_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, tol);
    opt.cp_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, tol);
    opt.cfriction_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, tol);
    opt.cv_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, tol);
    opt.ca_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, tol);
    opt.planar_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, tol);
    opt.prior_q_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, tol);
    opt.prior_qv_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, tol);
    opt.prior_qa_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, tol);
    opt.prior_t_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, tol);

    opt.q_col_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma_collo);
    opt.v_col_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma_collo);
    return opt;
  }

public:
  // Constructor.
  IECartPoleWithLimits() {}

  // Equality constriants include all dynamic constraints, and 0 torque
  // constraints.
  gtdynamics::EqualityConstraints eConstraints(const int k) const;

  // Inequality constraints include position limits and force limits
  // constraints.
  gtdynamics::InequalityConstraints iConstraints(const int k) const;

  // Equality constraints that specify q,v of initial state.
  gtdynamics::EqualityConstraints initStateConstraints() const;

  // Equality constriants that specify q,v of final state.
  gtdynamics::EqualityConstraints finalStateConstraints(size_t num_steps) const;

  // Cost for achieving q,v in final state.
  NonlinearFactorGraph finalStateCosts(size_t num_steps) const ;

  // Cost for minimizing accumulative force.
  NonlinearFactorGraph minTorqueCosts(size_t num_steps) const;

  // Cost for satisfying collocation.
  NonlinearFactorGraph collocationCosts(size_t num_steps, double dt) const;

  // Compute all values for a given timestep.
  Values computeValues(const size_t &k, const double &x, const double &v,
                       const double &q, const double &w) const;

  static void PrintValues(const Values &values, const size_t num_steps);

  static void PrintDelta(const VectorValues &values, const size_t num_steps);

  static void ExportValues(const Values &values, const size_t num_steps,
                           const std::string &file_path);

  // static void ExportVector(const VectorValues &values, const size_t
  // num_steps,
  //                          const std::string &file_path);

  /// Initial values seet as rest pose for all steps.
  Values getInitValuesZero(size_t num_steps) const;

  /// Initial values seet as rest pose for all steps.
  Values getInitValuesInterp(size_t num_steps) const;

  /// Return function that select basis keys for constraint manifolds.
  BasisKeyFunc getBasisKeyFunc() const;

};

class CartPoleWithLimitsRetractor : public IERetractor {
protected:
  const IECartPoleWithLimits &cp_;

public:
  CartPoleWithLimitsRetractor(const IECartPoleWithLimits &cp)
      : IERetractor(), cp_(cp) {}

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractInfo* retract_info = nullptr) const override;
};

} // namespace gtsam
