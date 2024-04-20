#include <gtdynamics/constrained_optimizer/ConstrainedOptBenchmarkIE.h>
#include <gtdynamics/scenarios/IEHalfSphere.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;
using namespace gtdynamics;

std::string scenario = "rss01_estimation";
std::string scenario_folder = "../../data/" + scenario + "/";
IEHalfSphere half_sphere;
size_t steps_per_edge = 10;
size_t num_steps = 3 * steps_per_edge;

bool display_result = true;
bool evaluate_cost_terms = false;
bool second_phase_opt = false;
bool log_progress = false;
bool log_values =false;
bool eval_proj_cost_progress = false;

/* ************************************************************************* */
void SaveValues(const std::string file_name, const Values &values) {
  {
    std::ofstream file;
    file.open(scenario_folder + file_name);
    file << "step,x,y,z\n";
    for (int k = 0; k < num_steps; k++) {
      Key point_key = gtsam::Symbol('p', k);
      Point3 point = values.at<Point3>(point_key);
      file << k << "," << point.x() << "," << point.y() << "," << point.z()
           << "\n";
    }
    file.close();
  }
}

/* ************************************************************************* */
IEConstraintManifold::Params::shared_ptr GetIECMParamsManual() {
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractor_creator =
      std::make_shared<UniversalIERetractorCreator>(
          std::make_shared<HalfSphereRetractor>(half_sphere));
  iecm_params->e_basis_creator = std::make_shared<OrthonormalBasisCreator>();
  return iecm_params;
}

/* ************************************************************************* */
IEConstraintManifold::Params::shared_ptr GetECMParamsManual() {
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractor_creator =
      std::make_shared<UniversalIERetractorCreator>(
          std::make_shared<SphereRetractor>(half_sphere));
  iecm_params->e_basis_creator = std::make_shared<OrthonormalBasisCreator>();
  return iecm_params;
}

// /* *************************************************************************
// */ IEConstraintManifold::Params::shared_ptr GetIECMParamsSP() {
//   auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
//   iecm_params->retractor_creator =
//       std::make_shared<UniversalIERetractorCreator>(
//           std::make_shared<HalfSphereRetractor>(half_sphere));
//   iecm_params->e_basis_creator = std::make_shared<OrthonormalBasisCreator>();
//   return iecm_params;
// }

/* ************************************************************************* */
IEConstraintManifold::Params::shared_ptr GetIECMParamsCR() {
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->e_basis_creator = std::make_shared<OrthonormalBasisCreator>();
  auto retractor_params = std::make_shared<IERetractorParams>();
  retractor_params->lm_params = LevenbergMarquardtParams();
  retractor_params->lm_params.setlambdaUpperBound(1e10);
  retractor_params->lm_params.setAbsoluteErrorTol(1e-10);
  retractor_params->check_feasible = true;
  retractor_params->ensure_feasible = true;
  retractor_params->feasible_threshold = 1e-5;
  retractor_params->use_varying_sigma = true;
  retractor_params->metric_sigmas = std::make_shared<VectorValues>();
  auto retract_penalty_params = std::make_shared<PenaltyParameters>();
  retract_penalty_params->initial_mu = 1.0;
  retract_penalty_params->mu_increase_rate = 10.0;
  retract_penalty_params->num_iterations = 3;
  retractor_params->penalty_params = retract_penalty_params;
  iecm_params->retractor_creator =
      std::make_shared<BarrierRetractorCreator>(retractor_params);
  return iecm_params;
}

/* ************************************************************************* */
IEConsOptProblem CreateProblem() {
  // ground-truth trajectory
  Values gt_values;
  for (size_t k = 0; k < steps_per_edge; k++) {
    double theta = (double)k / steps_per_edge * M_PI_2;
    gt_values.insert(PointKey(k), Point3(0, -sin(theta), cos(theta)));
    gt_values.insert(PointKey(k + steps_per_edge),
                     Point3(sin(theta), -cos(theta), 0));
    gt_values.insert(PointKey(k + 2 * steps_per_edge),
                     Point3(cos(theta), 0, sin(theta)));
  }
  SaveValues("gt_values.csv", gt_values);

  // generate gt odometry
  std::vector<Vector3> gt_odometrys;
  for (size_t k = 0; k < num_steps; k++) {
    size_t next_k = k == num_steps - 1 ? 0 : k + 1;
    Point3 x_curr = gt_values.at<Point3>(PointKey(k));
    Point3 x_next = gt_values.at<Point3>(PointKey(next_k));
    gt_odometrys.emplace_back(x_next - x_curr);
  }

  // inject odometry with noise
  auto odometry_noise = noiseModel::Isotropic::Sigma(3, 0.1);
  std::vector<Vector3> noisy_odometrys;
  uint_fast64_t rand_seed = 10;
  Sampler odo_sampler(odometry_noise, rand_seed);
  for (size_t k = 0; k < num_steps; k++) {
    noisy_odometrys.emplace_back(gt_odometrys.at(k) + odo_sampler.sample());
  }

  // construct graph
  NonlinearFactorGraph graph;
  auto prior_noise = noiseModel::Isotropic::Sigma(3, 0.1);
  graph.emplace_shared<PriorFactor<Point3>>(PointKey(0), Point3(0, 0, 1),
                                            prior_noise);
  for (size_t k = 0; k < num_steps; k++) {
    size_t next_k = k == num_steps - 1 ? 0 : k + 1;
    graph.emplace_shared<BetweenFactor<Point3>>(
        PointKey(k), PointKey(next_k), noisy_odometrys.at(k), odometry_noise);
  }

  // construct constraints
  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  for (size_t k = 0; k < num_steps; k++) {
    e_constraints.add(half_sphere.eConstraints(k));
    i_constraints.add(half_sphere.iConstraints(k));
  }

  // get initial values by integrate odo and then proj
  Values initial_values;
  Point3 point(0, 0, 1);
  for (size_t k = 0; k < num_steps; k++) {
    initial_values.insert(PointKey(k), half_sphere.project(point));
    point += noisy_odometrys.at(k);
  }

  // problem
  IEConsOptProblem problem(graph, e_constraints, i_constraints, initial_values);
  return problem;
}

/* ************************************************************************* */
std::pair<IEResultSummary, IELMItersDetails>
SecondPhaseOptimization(const Values values, std::string exp_name) {
  auto problem = CreateProblem();
  problem.values_ = values;

  IELMParams ie_params;
  ie_params.boundary_approach_rate_threshold = 1e10;
  return OptimizeIE_CMCOptLM(problem, ie_params, GetIECMParamsCR(), exp_name, false);
}

/* ************************************************************************* */
int main(int argc, char **argv) {
  auto problem = CreateProblem();

  IELMParams ie_params;
  // ie_params.lm_params.minModelFidelity = 0.5;

  // soft constraints
  std::cout << "optimize soft...\n";
  LevenbergMarquardtParams lm_params;
  auto soft_result = OptimizeIE_Soft(problem, lm_params, 1e2);

  // penalty method
  std::cout << "optimize penalty...\n";
  auto penalty_params = std::make_shared<PenaltyParameters>();
  penalty_params->initial_mu = 1;
  penalty_params->num_iterations = 4;
  penalty_params->mu_increase_rate = 10;
  penalty_params->store_iter_details = true;
  penalty_params->store_lm_details = true;
  penalty_params->lm_params.setVerbosityLM("SUMMARY");
  auto penalty_result = OptimizeIE_Penalty(problem, penalty_params);

  // augmented Lagrangian
  std::cout << "optimize augmented Lagrangian...\n";
  auto al_params = std::make_shared<AugmentedLagrangianParameters>();
  al_params->initial_mu_e = 10;
  al_params->store_iter_details = true;
  al_params->store_lm_details = true;
  auto al_result = OptimizeIE_AugmentedLagrangian(problem, al_params);

  // SQP method
  std::cout << "optimize SQP...\n";
  auto sqp_params = std::make_shared<SQPParams>();
  sqp_params->merit_e_l2_mu = 1e2;
  sqp_params->merit_i_l2_mu = 1e1;
  sqp_params->merit_e_l1_mu = 1e2;
  sqp_params->merit_i_l1_mu = 1e1;
  sqp_params->lm_params.setlambdaUpperBound(1e10);
  auto sqp_result = OptimizeIE_SQP(problem, sqp_params);

  // IP method
  std::cout << "optimize IP...\n";
  auto ip_result = OptimizeIE_IPOPT(problem);

  // ELM with penalty for i-constraints
  std::cout << "optimize CMOpt(E-LM)...\n";
  auto elm_result = OptimizeIE_CMOpt(problem, ie_params, GetECMParamsManual(), 1e2);

  // // // IEGD method
  // // std::cout << "optimize CMOpt(IE-GD)...\n";
  // // GDParams gd_params;
  // // // gd_params.muLowerBound = 1e-10;
  // // // gd_params.verbose = true;
  // // auto iegd_result = OptimizeIE_CMCOptGD(problem, gd_params, GetIECMParamsManual());

  // IELM standard projection
  std::cout << "optimize CMOpt(IE-LM)...\n";
  auto ielm_sp_result = OptimizeIE_CMCOptLM(problem, ie_params, GetIECMParamsManual(),
                                     "CMC-Opt");

  // // // IELM cost-aware projection
  // // std::cout << "optimize CMOpt(IE-LM-CR)...\n";
  // // // ie_params.lm_params.setVerbosityLM("SUMMARY");
  // // auto ielm_cr_result =
  // //     OptimizeIE_CMCOptLM(problem, ie_params, GetIECMParamsCR(), "CMOpt(IE-LM-CR)");

  if (display_result) {
    soft_result.first.printLatex(std::cout);
    elm_result.first.printLatex(std::cout);
    sqp_result.first.printLatex(std::cout);
    penalty_result.first.printLatex(std::cout);
    ip_result.first.printLatex(std::cout);
    al_result.first.printLatex(std::cout);
    ielm_sp_result.first.printLatex(std::cout);
  }

  if (log_progress) {
    std::filesystem::create_directory(scenario_folder);
    soft_result.first.exportFile(scenario_folder + "soft_progress.csv");
    penalty_result.first.exportFile(scenario_folder + "penalty_progress.csv");
    al_result.first.exportFile(scenario_folder + "al_progress.csv");
    sqp_result.first.exportFile(scenario_folder + "sqp_progress.csv");
    elm_result.first.exportFile(scenario_folder + "elm_progress.csv");
    // iegd_result.first.exportFile(scenario_folder + "iegd_progress.csv");
    ielm_sp_result.first.exportFile(scenario_folder + "ielm_sp_progress.csv");
    // ielm_cr_result.first.exportFile(scenario_folder + "ielm_cr_progress.csv");
  }


  if (log_values) {
    SaveValues("init_values.csv", problem.initValues());
    SaveValues("soft_values.csv", soft_result.second.back().values);
    SaveValues("penalty_values.csv", penalty_result.second.back().values);
    SaveValues("sqp_values.csv", sqp_result.second.back().state.values);
    SaveValues("elm_values.csv", elm_result.second.back().state.baseValues());
    // SaveValues("iegd_values.csv", iegd_result.second.back().state.baseValues());
    SaveValues("ielm_sp_values.csv",
              ielm_sp_result.second.back().state.baseValues());
    // SaveValues("ielm_cr_values.csv",
    //            ielm_cr_result.second.back().state.baseValues());
  }



  if (second_phase_opt) {
    auto soft_continued =
        SecondPhaseOptimization(soft_result.first.projected_values, "soft-continued");
    auto penalty_continued = SecondPhaseOptimization(
        penalty_result.first.projected_values, "penalty-continued");
    auto sqp_continued =
        SecondPhaseOptimization(sqp_result.first.projected_values, "sqp-continued");
    auto elm_continued =
        SecondPhaseOptimization(elm_result.first.projected_values, "CM-Opt-continued");

    soft_continued.first.printLatex(std::cout);
    elm_continued.first.printLatex(std::cout);
    sqp_continued.first.printLatex(std::cout);
    penalty_continued.first.printLatex(std::cout);

    soft_continued.first.exportFile(scenario_folder + "soft_continued.csv");
    elm_continued.first.exportFile(scenario_folder + "elm_continued.csv");
    sqp_continued.first.exportFile(scenario_folder + "sqp_continued.csv");
    penalty_continued.first.exportFile(scenario_folder +
                                       "penalty_continued.csv");
  }

  return 0;
}