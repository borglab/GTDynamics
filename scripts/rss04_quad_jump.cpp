/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Trajectory optimization for a legged robot vertical jump.
 * @author Yetong Zhang
 */

#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/scenarios/IEQuadrupedUtils.h>

using namespace gtdynamics;
using namespace gtsam;
using namespace quadruped_forward_jump_land;

std::string scenario = "rss04_quad_jump";
std::string scenario_folder = "../../data/" + scenario + "/";
bool second_phase_opt = false;
bool evaluate_projected = false;
bool evaluate_cost_terms = true;

JumpParams GetJumpParams() {
  JumpParams params;
  params.phase_num_steps = std::vector<size_t>{20, 10, 20, 20};
  params.phases_dt = std::vector<double>{0.01, 0.02, 0.02, 0.02};
  params.vision60_params->phases_min_dt =
      std::vector<double>{0.01, 0.001, 0.01, 0.01};
  params.init_values_include_i_constraints = true;
  params.init_values_ensure_feasible = true;
  params.forward_distance = 1.5;

  params.vision60_params->eval_details = true;
  params.vision60_params->eval_collo_step = true;
  params.vision60_params->i_constraints_symmetry = true;
  params.vision60_params->terrain_height_function =
      IEVision60Robot::sinHurdleTerrainFunc(0.75, 0.3, 0.2);
  params.vision60_params->step_div_ratio = 0.25;
  params.vision60_params->state_cost_values =
      DesValues(params.phase_num_steps, Point3(params.forward_distance, 0, 0));

  /* <=========== costs ===========> */
  params.vision60_params->include_collocation_costs = true;
  params.vision60_params->include_actuation_costs = true;
  params.vision60_params->include_state_costs = true;
  params.vision60_params->include_jerk_costs = true;
  params.vision60_params->include_cf_jerk_costs = true;
  params.vision60_params->include_symmetry_costs = true;
  params.vision60_params->include_collision_free_z_inter_cost = true;

  params.vision60_params->sigma_des_pose = 2e-3;
  params.vision60_params->sigma_des_twist = 1e-2;
  params.vision60_params->sigma_actuation = 10;
  params.vision60_params->sigma_jerk = 3;
  params.vision60_params->sigma_q_col = 1e-2;
  params.vision60_params->sigma_v_col = 1e-2;
  params.vision60_params->sigma_twist_col = 2e-3;
  params.vision60_params->sigma_pose_col = 2e-3;
  params.vision60_params->sigma_cf_jerk = 1e1;
  params.vision60_params->sigma_symmetry = 1e-1;
  params.vision60_params->cf_jerk_threshold = 30;
  params.vision60_params->tol_cf = 1e-3;
  params.vision60_params->jerk_cost_option = JERK_DIV_DT;
  params.vision60_params->dt_threshold = 0.02;

  /* <=========== inequality constraints ===========> */
  params.vision60_params->include_phase_duration_limits = true;
  params.vision60_params->include_friction_cone = true;
  params.vision60_params->include_joint_limits = true;
  params.vision60_params->include_collision_free_z = true;

  return params;
}

JumpParams params = GetJumpParams();
IEVision60RobotMultiPhase::shared_ptr vision60_multi_phase =
    GetVision60MultiPhase(params.vision60_params, params.phase_num_steps,
                          params.forward_distance);

/* <=====================================================================> */
/* <========================== Create Problem ===========================> */
/* <=====================================================================> */
std::tuple<IEConsOptProblem, IEVision60RobotMultiPhase::shared_ptr, JumpParams>
CreateProblem() {
  std::filesystem::create_directory(scenario_folder);

  /* <=========== create problem ===========> */
  /// Constraints
  EqualityConstraints e_constraints = vision60_multi_phase->eConstraints();
  InequalityConstraints i_constraints = vision60_multi_phase->iConstraints();
  NonlinearFactorGraph costs = vision60_multi_phase->costs();
  // costs.addPrior(PhaseKey(3), 0.02, noiseModel::Isotropic::Sigma(1, 1e-3));

  /// Initial Values
  auto init_values =
      InitValuesTrajectory(*vision60_multi_phase, params.phases_dt,
                           params.init_values_include_i_constraints,
                           params.init_values_ensure_feasible);
  // auto init_values =
  //     LoadValuesFromFile(scenario_folder + "manopt_values_copy.dat");

  /// Problem
  IEConsOptProblem problem(costs, e_constraints, i_constraints, init_values);
  problem.eval_func = vision60_multi_phase->costsEvalFunc();
  return {problem, vision60_multi_phase, params};
}

/* ************************************************************************* */
std::vector<NonlinearFactorGraph> GetCostTerms(const IEVision60RobotMultiPhase& vision60_multi_phase) {
  NonlinearFactorGraph collo_cost;
  NonlinearFactorGraph state_cost;
  NonlinearFactorGraph actuation_cost;
  NonlinearFactorGraph jerk_cost;
  NonlinearFactorGraph total_cost;

  auto classified_costs = vision60_multi_phase.classifiedCosts();
  std::map<std::string, NonlinearFactorGraph> cost_map(classified_costs.begin(), classified_costs.end());
  collo_cost.add(cost_map.at("collocation"));
  actuation_cost.add(cost_map.at("actuation"));
  state_cost.add(cost_map.at("state"));
  jerk_cost.add(cost_map.at("jerk"));
  jerk_cost.add(cost_map.at("cf_jerk"));

  total_cost.add(collo_cost);
  total_cost.add(state_cost);
  total_cost.add(actuation_cost);
  total_cost.add(jerk_cost);

  return std::vector<NonlinearFactorGraph>{
      collo_cost, state_cost, actuation_cost, jerk_cost, total_cost};
}


/* ************************************************************************* */
IERetractorParams::shared_ptr GetNominalRetractorParams() {
  auto retractor_params = std::make_shared<IERetractorParams>();
  retractor_params->lm_params = LevenbergMarquardtParams();
  retractor_params->lm_params.setlambdaUpperBound(1e10);
  retractor_params->lm_params.setAbsoluteErrorTol(1e-10);
  retractor_params->check_feasible = true;
  retractor_params->ensure_feasible = true;
  retractor_params->feasible_threshold = 1e-5;
  retractor_params->prior_sigma = 1e-1;
  retractor_params->use_varying_sigma = false;

  auto barrier_params = std::make_shared<BarrierParameters>();
  // barrier_params->lm_params = params_->lm_params;
  barrier_params->initial_mu = 10.0;
  barrier_params->mu_increase_rate = 10.0;
  barrier_params->num_iterations = 2;
  auto lm_params1 = retractor_params->lm_params;
  auto lm_params2 = retractor_params->lm_params;
  // lm_params1.setMaxIterations(20);
  // lm_params1.setVerbosityLM("SUMMARY");
  barrier_params->iters_lm_params = std::vector<LevenbergMarquardtParams>();
  for (size_t i = 0; i < barrier_params->num_iterations - 1; i++) {
    barrier_params->iters_lm_params.push_back(lm_params1);
  }
  barrier_params->iters_lm_params.push_back(lm_params2);
  retractor_params->barrier_params = barrier_params;
  return retractor_params;
}

/* ************************************************************************* */
IEConstraintManifold::Params::shared_ptr GetIECMParamsSP() {
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->e_basis_build_from_scratch = false;
  iecm_params->retractor_creator =
      std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
          vision60_multi_phase, GetNominalRetractorParams(), false);
  iecm_params->e_basis_creator = OrthonormalBasisCreator::CreateSparse();
  return iecm_params;
}

/* ************************************************************************* */
IEConstraintManifold::Params::shared_ptr GetIECMParamsCR() {
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->e_basis_build_from_scratch = false;
  auto retractor_params_cr = GetNominalRetractorParams();
  retractor_params_cr->use_varying_sigma = true;
  retractor_params_cr->metric_sigmas = std::make_shared<VectorValues>();
  // retractor_params_cr->scale_varying_sigma = true;
  iecm_params->retractor_creator =
      std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
          vision60_multi_phase, retractor_params_cr, false);
  iecm_params->e_basis_creator = OrthonormalBasisCreator::CreateSparse();
  return iecm_params;
}

/* ************************************************************************* */
IELMParams NominalIELMParams() {
  IELMParams ie_params;
  ie_params.boundary_approach_rate_threshold = 1e10;
  ie_params.lm_params.setVerbosityLM("SUMMARY");
  // ie_params.lm_params.setMaxIterations(50);
  ie_params.lm_params.setLinearSolverType("SEQUENTIAL_QR");
  ie_params.lm_params.setlambdaUpperBound(1e10);
  // ie_params.lm_params.lambdaInitial = 1e-6;
  ie_params.iqp_max_iters = 100;
  ie_params.show_active_constraints = true;
  ie_params.active_constraints_group_as_categories = true;
  return ie_params;
}


/* ************************************************************************* */
std::pair<IEResultSummary, IELMItersDetails>
SecondPhaseOptimization(const Values values, std::string exp_name) {
  auto problem = std::get<0>(CreateProblem());
  problem.values_ = values;

  IELMParams ie_params = NominalIELMParams();
  ie_params.lm_params.setLinearSolverType("MULTIFRONTAL_QR");
  // ie_params.boundary_approach_rate_threshold = 10;
  return OptimizeIELM(problem, ie_params, GetIECMParamsCR(), exp_name, false);
}

/* ************************************************************************* */
void TrajectoryOptimization() {
  auto [problem, vision60_multi_phase, params] = CreateProblem();
  EvaluateAndExportInitValues(problem, *vision60_multi_phase, scenario_folder);

  // soft constraints
  std::cout << "optimize soft...\n";
  LevenbergMarquardtParams lm_params;
  lm_params.setlambdaUpperBound(1e10);
  lm_params.setVerbosityLM("SUMMARY");
  auto soft_result = OptimizeSoftConstraints(problem, lm_params, 1e4, evaluate_projected);

  // penalty method
  std::cout << "optimize penalty...\n";
  auto penalty_params = std::make_shared<BarrierParameters>();
  penalty_params->initial_mu = 1e-4;
  penalty_params->mu_increase_rate = 4;
  penalty_params->num_iterations = 16;
  penalty_params->lm_params.setVerbosityLM("SUMMARY");
  penalty_params->lm_params.setlambdaUpperBound(1e10);
  penalty_params->lm_params.setMaxIterations(30);
  auto penalty_result = OptimizePenaltyMethod(problem, penalty_params, evaluate_projected);

  // SQP method
  std::cout << "optimize SQP...\n";
  auto sqp_params = std::make_shared<SQPParams>();
  sqp_params->merit_e_l2_mu = 1e2;
  sqp_params->merit_i_l2_mu = 1e1;
  sqp_params->merit_e_l1_mu = 1e2;
  sqp_params->merit_i_l1_mu = 1e1;
  sqp_params->use_qp_constrained_mu = true;
  sqp_params->qp_constrained_mu = 1e8;
  sqp_params->lm_params.setVerbosityLM("SUMMARY");
  sqp_params->lm_params.setlambdaUpperBound(1e20);
  sqp_params->lm_params.linearSolverType =
      gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR;
  auto sqp_result = OptimizeSQP(problem, sqp_params, evaluate_projected);

  // ELM with penalty for i-constraints
  std::cout << "optimize CMOpt(E-LM)...\n";
  auto elm_result =
      OptimizeELM(problem, NominalIELMParams(), GetIECMParamsSP(), 1e4, evaluate_projected);

  // // IEGD method
  // std::cout << "optimize CMOpt(IE-GD)...\n";
  // GDParams gd_params;
  // gd_params.verbose = true;
  // gd_params.init_lambda = 1e-5;
  // gd_params.muLowerBound = 1e-15;
  // gd_params.maxIterations = 20;
  // auto iegd_result = OptimizeIEGD(problem, gd_params, GetIECMParamsSP(),
  // false);

  // IELM standard projection
  std::cout << "optimize CMOpt(IE-LM-SP)...\n";
  auto ielm_sp_result =
      OptimizeIELM(problem, NominalIELMParams(), GetIECMParamsSP(), "CMC-Opt", evaluate_projected);

  // // IELM cost-aware projection
  // std::cout << "optimize CMOpt(IE-LM-CR)...\n";
  // auto ielm_cr_result =
  //     OptimizeIELM(problem, NominalIELMParams(), GetIECMParamsCR(),
  //     "CMOpt(IE-LM-CR)", false);

  // soft_result.first.printLatex(std::cout);
  // elm_result.first.printLatex(std::cout);
  // sqp_result.first.printLatex(std::cout);
  // penalty_result.first.printLatex(std::cout);
  // ielm_sp_result.first.printLatex(std::cout);

  // std::filesystem::create_directory(scenario_folder);
  // soft_result.first.exportFile(scenario_folder + "soft_progress.csv");
  // penalty_result.first.exportFile(scenario_folder + "penalty_progress.csv");
  // sqp_result.first.exportFile(scenario_folder + "sqp_progress.csv");
  // elm_result.first.exportFile(scenario_folder + "elm_progress.csv");
  // iegd_result.first.exportFile(scenario_folder + "iegd_progress.csv");
  // ielm_sp_result.first.exportFile(scenario_folder + "ielm_sp_progress.csv");
  // ielm_cr_result.first.exportFile(scenario_folder + "ielm_cr_progress.csv");

  // Values penalty_projected_values = ProjectValues(problem,
  // penalty_result.second.back().values);

  // ExportValuesToFile(penalty_projected_values, scenario_folder +
  // "penalty_values.dat");
  // IEVision60Robot::ExportValuesMultiPhase(penalty_projected_values,
  //                                         vision60_multi_phase->phase_num_steps_,
  //                                         scenario_folder +
  //                                         "penalty_traj.csv");

  // EvaluateAndExportIELMResult(problem, *vision60_multi_phase, ielm_cr_result,
  //                             scenario_folder, false);
  // ExportOptimizationProgress(*vision60_multi_phase, scenario_folder,
  //                            ielm_cr_result.second);

  if (second_phase_opt) {
    // auto soft_continued = SecondPhaseOptimization(
    //     soft_result.first.projected_values, "soft-continued");
    // auto penalty_continued = SecondPhaseOptimization(
    //     penalty_result.first.projected_values, "penalty-continued");
    // auto sqp_continued = SecondPhaseOptimization(
    //     sqp_result.first.projected_values, "sqp-continued");
    // auto elm_continued = SecondPhaseOptimization(
    //     elm_result.first.projected_values, "CM-Opt-continued");

    // soft_continued.first.printLatex(std::cout);
    // elm_continued.first.printLatex(std::cout);
    // sqp_continued.first.printLatex(std::cout);
    // penalty_continued.first.printLatex(std::cout);

    // soft_continued.first.exportFile(scenario_folder + "soft_continued.csv");
    // elm_continued.first.exportFile(scenario_folder + "elm_continued.csv");
    // sqp_continued.first.exportFile(scenario_folder + "sqp_continued.csv");
    // penalty_continued.first.exportFile(scenario_folder +
    //                                    "penalty_continued.csv");
  }

  if (evaluate_cost_terms) {
    auto cost_terms = GetCostTerms(*vision60_multi_phase);
    EvaluateCostTerms(std::cout, cost_terms, problem.initValues(), problem.initValues(), "Init values");
    EvaluateCostTerms(std::cout, cost_terms, soft_result.first.values, soft_result.first.projected_values, "Soft");
    EvaluateCostTerms(std::cout, cost_terms, elm_result.first.values, elm_result.first.projected_values, "CM-Opt");
    EvaluateCostTerms(std::cout, cost_terms, sqp_result.first.values, sqp_result.first.projected_values, "SQP");
    EvaluateCostTerms(std::cout, cost_terms, penalty_result.first.values, penalty_result.first.projected_values, "Penalty");
    EvaluateCostTerms(std::cout, cost_terms, ielm_sp_result.first.values, ielm_sp_result.first.projected_values, "CMC-Opt");
  }

}

int main(int argc, char **argv) {
  TrajectoryOptimization();
  return 0;
}
