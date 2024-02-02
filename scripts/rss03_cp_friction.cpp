#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/scenarios/IECartPoleWithFriction.h>

using namespace gtsam;
using namespace gtdynamics;

std::string scenario = "rss03_cp_friction";
std::string scenario_folder = "../../data/" + scenario + "/";

IECartPoleWithFriction GetCP() {
  IECartPoleWithFriction cp;
  cp.include_torque_limits = true;
  return cp;
}

IECartPoleWithFriction cp = GetCP();
size_t num_steps = 20;
double dt = 0.05;

/* ************************************************************************* */
IERetractorParams::shared_ptr GetNominalRetractorParams() {
  auto retractor_params = std::make_shared<IERetractorParams>();
  retractor_params->lm_params = LevenbergMarquardtParams();
  retractor_params->lm_params.setlambdaUpperBound(1e10);
  retractor_params->lm_params.setAbsoluteErrorTol(1e-10);
  retractor_params->check_feasible = true;
  retractor_params->ensure_feasible = true;
  retractor_params->prior_sigma = 1e2;
  auto retract_barrier_params = std::make_shared<BarrierParameters>();
  retract_barrier_params->initial_mu = 1.0;
  retract_barrier_params->mu_increase_rate = 10.0;
  retract_barrier_params->num_iterations = 6;
  retractor_params->barrier_params = retract_barrier_params;
  return retractor_params;
}

/* ************************************************************************* */
IEConstraintManifold::Params::shared_ptr GetIECMParamsManual() {
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractor_creator =
      std::make_shared<UniversalIERetractorCreator>(
          std::make_shared<CPBarrierRetractor>(cp));
  iecm_params->e_basis_creator = std::make_shared<OrthonormalBasisCreator>();
  return iecm_params;
}

/* ************************************************************************* */
IEConstraintManifold::Params::shared_ptr GetIECMParamsSP() {
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractor_creator =
      std::make_shared<BarrierRetractorCreator>(GetNominalRetractorParams());
  iecm_params->e_basis_creator = std::make_shared<OrthonormalBasisCreator>();
  return iecm_params;
}

/* ************************************************************************* */
IEConstraintManifold::Params::shared_ptr GetIECMParamsCR() {
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  auto retractor_params_cr = GetNominalRetractorParams();
  retractor_params_cr->use_varying_sigma = true;
  retractor_params_cr->metric_sigmas = std::make_shared<VectorValues>();
  iecm_params->retractor_creator =
      std::make_shared<BarrierRetractorCreator>(retractor_params_cr);
  iecm_params->e_basis_creator = OrthonormalBasisCreator::CreateSparse();
  return iecm_params;
}

/* ************************************************************************* */
Values ComputeInitialValues() {
  Values values;
  for (size_t k = 0; k <= num_steps; k++) {
    double q = M_PI * k / num_steps;
    double v = 0;
    double a = 0;
    double tau = cp.computeTau(q, a);
    double fx = cp.computeFx(q, v, a);
    double fy = cp.computeFy(q, v, a);
    values.insert(QKey(k), q);
    values.insert(VKey(k), v);
    values.insert(AKey(k), a);
    values.insert(TauKey(k), tau);
    values.insert(FxKey(k), fx);
    values.insert(FyKey(k), fy);
  }
  return values;
}

/* ************************************************************************* */
IEConsOptProblem CreateProblem() {
  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    auto e_constraints_k = cp.eConstraints(k);
    auto i_constraints_k = cp.iConstraints(k);
    e_constraints.add(e_constraints_k);
    i_constraints.add(i_constraints_k);
  }
  e_constraints.emplace_shared<DoubleExpressionEquality>(
      Double_(QKey(0)) - Double_(0.0), 1.0);
  e_constraints.emplace_shared<DoubleExpressionEquality>(Double_(VKey(0)), 1.0);
  e_constraints.emplace_shared<DoubleExpressionEquality>(
      Double_(QKey(num_steps)) - Double_(M_PI), 1.0);
  e_constraints.emplace_shared<DoubleExpressionEquality>(
      Double_(VKey(num_steps)), 1.0);
  NonlinearFactorGraph graph;
  auto collo_model = noiseModel::Isotropic::Sigma(1, 1e-1);
  auto prior_model = noiseModel::Isotropic::Sigma(1, 1e-2);
  auto cost_model = noiseModel::Isotropic::Sigma(1, 1e0);
  // graph.addPrior<double>(QKey(0), M_PI_2, prior_model);
  // graph.addPrior<double>(VKey(0), 0.0, prior_model);
  // graph.addPrior<double>(QKey(num_steps), 0.0, prior_model);
  // graph.addPrior<double>(VKey(num_steps), 0.0, prior_model);
  for (size_t k = 0; k < num_steps; k++) {
    Double_ q0_expr(QKey(k));
    Double_ q1_expr(QKey(k + 1));
    Double_ v0_expr(VKey(k));
    Double_ v1_expr(VKey(k + 1));
    Double_ a0_expr(AKey(k));
    Double_ a1_expr(AKey(k + 1));
    graph.add(
        ExpressionFactor(collo_model, 0.0, q0_expr + dt * v0_expr - q1_expr));
    graph.add(
        ExpressionFactor(collo_model, 0.0, v0_expr + dt * a0_expr - v1_expr));
  }
  for (size_t k = 0; k <= num_steps; k++) {
    graph.addPrior<double>(QKey(k), M_PI, cost_model);
  }

  Values initial_values = ComputeInitialValues();
  IEConsOptProblem problem(graph, e_constraints, i_constraints, initial_values);
  return problem;
}

int main(int argc, char **argv) {
  auto problem = CreateProblem();

  // Parameters
  IELMParams ie_params;
  ie_params.boundary_approach_rate_threshold = 1e10;
  // ie_params.lm_params.setVerbosityLM("SUMMARY");

  // soft constraints
  std::cout << "optimize soft...\n";
  LevenbergMarquardtParams lm_params;
  // lm_params.setVerbosityLM("SUMMARY");
  auto soft_result = OptimizeSoftConstraints(problem, lm_params, 1e2);

  // penalty method
  std::cout << "optimize penalty...\n";
  auto penalty_params = std::make_shared<BarrierParameters>();
  penalty_params->initial_mu = 1e2;
  penalty_params->mu_increase_rate = 4;
  penalty_params->num_iterations = 8;
  auto penalty_result = OptimizePenaltyMethod(problem, penalty_params);

  // SQP method
  std::cout << "optimize SQP...\n";
  auto sqp_params = std::make_shared<SQPParams>();
  sqp_params->merit_e_l2_mu = 1e0;
  sqp_params->merit_i_l2_mu = 1e0;
  sqp_params->merit_e_l1_mu = 1e0;
  sqp_params->merit_i_l1_mu = 1e0;
  // sqp_params->lm_params.setVerbosityLM("SUMMARY");
  sqp_params->lm_params.setlambdaUpperBound(1e10);
  auto sqp_result = OptimizeSQP(problem, sqp_params);

  // ELM with penalty for i-constraints
  std::cout << "optimize CM-Opt(E-LM)...\n";
  auto elm_result = OptimizeELM(problem, ie_params, GetIECMParamsSP(), 1e2);

  // // IEGD method
  // std::cout << "optimize CMOpt(IE-GD)...\n";
  // GDParams gd_params;
  // auto iegd_result = OptimizeIEGD(problem, gd_params, GetIECMParamsSP());

  // IELM standard projection
  std::cout << "optimize CMC-Opt(IE-LM-SP)...\n";
  auto ielm_sp_result =
      OptimizeIELM(problem, ie_params, GetIECMParamsSP(), "CMC-Opt");

  // // IELM cost-aware projection
  // std::cout << "optimize CMOpt(IE-LM-CR)...\n";
  // auto ielm_cr_result =
  //     OptimizeIELM(problem, ie_params, GetIECMParamsCR(), "CMOpt(IE-LM-CR)");

  soft_result.first.printLatex(std::cout);
  penalty_result.first.printLatex(std::cout);
  sqp_result.first.printLatex(std::cout);
  elm_result.first.printLatex(std::cout);
  // iegd_result.first.printLatex(std::cout);
  ielm_sp_result.first.printLatex(std::cout);
  // ielm_cr_result.first.printLatex(std::cout);

  std::filesystem::create_directory(scenario_folder);
  soft_result.first.exportFile(scenario_folder + "soft_progress.csv");
  penalty_result.first.exportFile(scenario_folder + "penalty_progress.csv");
  sqp_result.first.exportFile(scenario_folder + "sqp_progress.csv");
  elm_result.first.exportFile(scenario_folder + "elm_progress.csv");
  // iegd_result.first.exportFile(scenario_folder + "iegd_progress.csv");
  ielm_sp_result.first.exportFile(scenario_folder + "ielm_sp_progress.csv");
  // ielm_cr_result.first.exportFile(scenario_folder + "ielm_cr_progress.csv");
  return 0;
}