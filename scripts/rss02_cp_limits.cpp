#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/scenarios/IECartPoleWithLimits.h>

using namespace gtsam;
using namespace gtdynamics;

std::string scenario = "rss02_cp_limits";
std::string scenario_folder = "../../data/" + scenario + "/";
IECartPoleWithLimits cp;

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
  iecm_params->ecm_params->basis_creator =
      std::make_shared<EliminationBasisCreator>(cp.getBasisKeyFunc());
  iecm_params->retractor_creator =
      std::make_shared<UniversalIERetractorCreator>(
          std::make_shared<CartPoleWithLimitsRetractor>(cp));
  return iecm_params;
}

/* ************************************************************************* */
IEConstraintManifold::Params::shared_ptr GetIECMParamsSP() {
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractor_creator =
      std::make_shared<BarrierRetractorCreator>(GetNominalRetractorParams());
  iecm_params->e_basis_creator = OrthonormalBasisCreator::CreateSparse();
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
IEConsOptProblem CreateProblem() {
  // Setting
  size_t num_steps = 20;
  double dt = 0.1;

  // Constraints
  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    auto e_constraints_k = cp.eConstraints(k);
    auto i_constraints_k = cp.iConstraints(k);
    e_constraints.add(e_constraints_k);
    i_constraints.add(i_constraints_k);
  }
  e_constraints.add(cp.initStateConstraints());

  // Costs
  NonlinearFactorGraph graph;
  NonlinearFactorGraph collocation_costs = cp.collocationCosts(num_steps, dt);
  NonlinearFactorGraph final_state_graph = cp.finalStateCosts(num_steps);
  NonlinearFactorGraph min_torque_costs = cp.minTorqueCosts(num_steps);
  graph.add(final_state_graph);
  graph.add(collocation_costs);
  graph.add(min_torque_costs);
  auto EvaluateCosts = [=](const Values &values) {
    std::cout << "collocation costs:\t" << collocation_costs.error(values)
              << "\n";
    std::cout << "final state costs:\t" << final_state_graph.error(values)
              << "\n";
    std::cout << "min torque costs:\t" << min_torque_costs.error(values)
              << "\n";
  };

  // Initial Values
  Values initial_values = cp.getInitValuesZero(num_steps);
  EvaluateCosts(initial_values);

  // Problem
  IEConsOptProblem problem(graph, e_constraints, i_constraints, initial_values);
  return problem;
}

/* ************************************************************************* */
int main(int argc, char **argv) {
  // Create problem
  auto problem = CreateProblem();

  // Parameters
  IELMParams ie_params;
  ie_params.boundary_approach_rate_threshold = 1e10;
  // ie_params.lm_params.setVerbosityLM("SUMMARY");

  // soft constraints
  std::cout << "optimize soft...\n";
  LevenbergMarquardtParams lm_params;
  // lm_params.setVerbosityLM("SUMMARY");
  auto soft_result = OptimizeSoftConstraints(problem, lm_params, 1e8);

  // penalty method
  std::cout << "optimize penalty...\n";
  auto penalty_params = std::make_shared<BarrierParameters>();
  penalty_params->initial_mu = 1e0;
  penalty_params->mu_increase_rate = 4;
  penalty_params->num_iterations = 20;
  auto penalty_result = OptimizePenaltyMethod(problem, penalty_params);

  // SQP method
  std::cout << "optimize SQP...\n";
  auto sqp_params = std::make_shared<SQPParams>();
  sqp_params->merit_e_l2_mu = 1e4;
  sqp_params->merit_i_l2_mu = 1e2;
  sqp_params->merit_e_l1_mu = 1e4;
  sqp_params->merit_i_l1_mu = 1e4;
  // sqp_params->lm_params.setVerbosityLM("SUMMARY");
  sqp_params->lm_params.setlambdaUpperBound(1e10);
  auto sqp_result = OptimizeSQP(problem, sqp_params);

  // ELM with penalty for i-constraints
  std::cout << "optimize CMOpt(E-LM)...\n";
  auto elm_result = OptimizeELM(problem, ie_params, GetIECMParamsSP(), 1e6);

  // IEGD method
  std::cout << "optimize CMOpt(IE-GD)...\n";
  GDParams gd_params;
  auto iegd_result = OptimizeIEGD(problem, gd_params, GetIECMParamsSP());

  // IELM standard projection
  std::cout << "optimize CMOpt(IE-LM-SP)...\n";
  auto ielm_sp_result =
      OptimizeIELM(problem, ie_params, GetIECMParamsSP(), "CMOpt(IE-LM-SP)");

  // IELM cost-aware projection
  std::cout << "optimize CMOpt(IE-LM-CR)...\n";
  auto ielm_cr_result =
      OptimizeIELM(problem, ie_params, GetIECMParamsCR(), "CMOpt(IE-LM-CR)");

  soft_result.first.printLatex(std::cout);
  penalty_result.first.printLatex(std::cout);
  sqp_result.first.printLatex(std::cout);
  elm_result.first.printLatex(std::cout);
  iegd_result.first.printLatex(std::cout);
  ielm_sp_result.first.printLatex(std::cout);
  ielm_cr_result.first.printLatex(std::cout);

  std::filesystem::create_directory(scenario_folder);
  soft_result.first.exportFile(scenario_folder + "soft_progress.csv");
  penalty_result.first.exportFile(scenario_folder + "penalty_progress.csv");
  sqp_result.first.exportFile(scenario_folder + "sqp_progress.csv");
  elm_result.first.exportFile(scenario_folder + "elm_progress.csv");
  iegd_result.first.exportFile(scenario_folder + "iegd_progress.csv");
  ielm_sp_result.first.exportFile(scenario_folder + "ielm_sp_progress.csv");
  ielm_cr_result.first.exportFile(scenario_folder + "ielm_cr_progress.csv");
  return 0;
}
