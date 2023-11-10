

#include <gtdynamics/imanifold/IEGDOptimizer.h>
#include <gtdynamics/scenarios/IEHalfSphere.h>
#include <gtdynamics/imanifold/IELMOptimizer.h>
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/optimizer/BarrierOptimizer.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;
using namespace gtdynamics;

class RandEngine {
private:
  std::default_random_engine generator_;
  std::normal_distribution<double> distribution_;

public:
  RandEngine() : generator_(), distribution_(0.0, 1.0) {
    generator_.seed(
        std::chrono::system_clock::now().time_since_epoch().count());
  }

  RandEngine(const size_t seed) : generator_(), distribution_(0.0, 1.0) {
    generator_.seed(seed);
  }

  /** Generate a random variable with mean=0 and std=sigma. */
  double randn(const double &sigma) {
    return distribution_(generator_) * sigma;
  }

  /** Generate a random vector with mean=0 and std=sigmas. */
  gtsam::Vector rand_vec(const gtsam::Vector &sigmas) {
    gtsam::Vector vec = gtsam::Vector::Zero(sigmas.size());
    for (size_t i = 0; i < sigmas.size(); i++) {
      vec(i) = randn(sigmas(i));
    }
    return vec;
  }
};

Values ToyTrajectory() {
  Values values;
  values.insert(PointKey(0), Point3(0, 0, 0));
  values.insert(PointKey(1), Point3(0.5, 0.3, 0));
  values.insert(PointKey(2), Point3(1.0, 0, 0));
  values.insert(PointKey(3), Point3(0, 0, 0.5));
  values.insert(PointKey(4), Point3(0, 0, 1));
  values.insert(PointKey(5), Point3(0, 0.6, 0.8));
  return values;
}

std::vector<Vector3> ToyMeasurements() {
  std::vector<Vector3> measurements{
      Vector3(0.6, 0.3, 0), Vector3(0.6, -0.3, 0), Vector3(-1, 0.0, 0.5),
      Vector3(0.0, 0.0, 0.6), Vector3(0.0, 0.6, -0.2)};

  return measurements;
}

Values SimpleTrajectory(const IEHalfSphere &half_sphere) {
  double r = half_sphere.r;
  size_t num_steps_1 = 10;
  size_t num_steps_2 = 10;
  size_t num_steps_3 = 10;

  Values values;
  for (size_t k = 0; k <= num_steps_1; k++) {
    values.insert(PointKey(k), Point3(r * k / num_steps_1, 0, 0));
  }
  for (size_t k = 1; k <= num_steps_2; k++) {
    double theta = M_PI_2 * k / num_steps_2;
    values.insert(PointKey(k + num_steps_1),
                  Point3(r * cos(theta), 0, r * sin(theta)));
  }
  for (size_t k = 1; k <= num_steps_3; k++) {
    values.insert(PointKey(k + num_steps_1 + num_steps_2),
                  Point3(0, 0, r * (num_steps_3 - k) / num_steps_3));
  }
  return values;
}

/// Generate noisy measurements by injecting noise into the ground-truth
/// measurements
std::vector<Vector3>
GenerateMeasurements(const Values &values,
                     const gtsam::Vector &odometry_sigmas) {

  size_t num_steps = values.size() - 1;
  size_t random_seed = 100;
  RandEngine rand_engine(random_seed);

  std::vector<Vector3> measurements(num_steps);
  for (size_t k = 0; k < num_steps; k++) {
    Point3 p1 = values.at<Point3>(PointKey(k));
    Point3 p2 = values.at<Point3>(PointKey(k + 1));
    Point3 odometry = gtsam::traits<Point3>::Between(p1, p2);
    auto perturb_vec = rand_engine.rand_vec(odometry_sigmas);
    measurements[k] = gtsam::traits<Point3>::Retract(odometry, perturb_vec);
  }

  return measurements;
}

NonlinearFactorGraph GetCosts(const std::vector<Vector3> &measurements,
                              const gtsam::Point3 &init_point,
                              gtsam::SharedNoiseModel prior_noise,
                              gtsam::SharedNoiseModel between_noise) {
  NonlinearFactorGraph graph;
  graph.addPrior<Point3>(PointKey(0), init_point, prior_noise);
  for (int k = 0; k < measurements.size(); k++) {
    graph.emplace_shared<BetweenFactor<Point3>>(
        PointKey(k), PointKey(k + 1), measurements.at(k), between_noise);
  }
  return graph;
}

Values ZeroValues(const size_t num_steps) {
  Values values;
  for (size_t k = 0; k <= num_steps; k++) {
    values.insert(PointKey(k), Point3(0, 0, 0));
  }
  return values;
}

Values InitValuesByOdometry(const IEHalfSphere &half_sphere,
                            const Point3 &init_point,
                            const std::vector<Vector3> &odometry) {
  size_t num_steps = odometry.size();
  std::vector<Point3> points(num_steps + 1);
  points[0] = init_point;

  for (size_t k = 0; k < num_steps; k++) {
    const Vector3 &rel = odometry.at(k);
    points[k + 1] = points[k] + rel;
  }

  const double &r = half_sphere.r;

  Values values;
  for (size_t k = 0; k <= num_steps; k++) {
    Point3 modified_point = points[k];
    if (modified_point.z() < 0) {
      modified_point.z() = 0;
    }
    if (modified_point.norm() > r) {
      modified_point = r / modified_point.norm() * modified_point;
    }
    values.insert(PointKey(k), modified_point);
  }
  return values;
}

IEConsOptProblem ToyProblem(const IEHalfSphere &half_sphere) {
  // GT trajectory
  Values gt_values = ToyTrajectory();
  Point3 init_point = gt_values.at<Point3>(PointKey(0));
  size_t num_steps = gt_values.size() - 1;

  // measurements
  auto measurements = ToyMeasurements();

  // graph
  auto prior_noise = noiseModel::Isotropic::Sigma(3, 0.1);
  auto between_noise = noiseModel::Isotropic::Sigma(3, 0.1);
  NonlinearFactorGraph graph =
      GetCosts(measurements, init_point, prior_noise, between_noise);

  // constraints
  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    i_constraints.add(half_sphere.iDomeConstraints(k));
  }

  // values
  Values initial_values = ZeroValues(num_steps);
  return IEConsOptProblem(graph, e_constraints, i_constraints, initial_values);
}

IEConsOptProblem SimpleProblem(const IEHalfSphere &half_sphere) {
  // GT trajectory
  Values gt_values = SimpleTrajectory(half_sphere);
  Point3 init_point = gt_values.at<Point3>(PointKey(0));
  size_t num_steps = gt_values.size() - 1;

  // measurements
  auto prior_noise = noiseModel::Isotropic::Sigma(3, 0.01);
  auto between_noise = noiseModel::Isotropic::Sigma(3, 0.01);
  Vector odometry_sigmas = between_noise->sigmas();
  auto measurements = GenerateMeasurements(gt_values, odometry_sigmas);

  // graph
  NonlinearFactorGraph graph =
      GetCosts(measurements, init_point, prior_noise, between_noise);

  // constraints
  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    i_constraints.add(half_sphere.iDomeConstraints(k));
  }

  // values
  Values initial_values =
      InitValuesByOdometry(half_sphere, init_point, measurements);
  return IEConsOptProblem(graph, e_constraints, i_constraints, initial_values);
}

double TranslationError(const Values &gt_values, const Values &values) {
  size_t num_steps = gt_values.size() - 1;
  double error = 0;
  for (size_t k = 0; k <= num_steps; k++) {
    Point3 gt_point = gt_values.at<Point3>(PointKey(k));
    Point3 point = values.at<Point3>(PointKey(k));
    error += pow((gt_point - point).norm(), 2);
  }
  return sqrt(error);
}

int main(int argc, char **argv) {
  double radius = 1;
  IEHalfSphere half_sphere(radius);
  auto problem = SimpleProblem(half_sphere);

  problem.initValues().print();

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractor_creator =
      std::make_shared<UniversalIERetractorCreator>(
          std::make_shared<DomeRetractor>(half_sphere));
  iecm_params->e_basis_creator = std::make_shared<TspaceBasisCreator>(
      iecm_params->ecm_params->basis_params);

  LevenbergMarquardtParams lm_params;
  std::cout << "run soft...\n";
  auto soft_result = OptimizeSoftConstraints(problem, lm_params, 100);

  BarrierParameters barrier_params;
  barrier_params.num_iterations = 15;
  std::cout << "run barrier...\n";
  auto barrier_result = OptimizeBarrierMethod(problem, barrier_params);

  GDParams gd_params;
  std::cout << "run gd...\n";
  auto gd_result = OptimizeIEGD(problem, gd_params, iecm_params);

  IELMParams ie_params;
  std::cout << "run lm...\n";
  ie_params.lm_params.setVerbosityLM("SUMMARY");
  auto lm_result = OptimizeIELM(problem, ie_params, iecm_params);

  soft_result.first.printLatex(std::cout);
  barrier_result.first.printLatex(std::cout);
  gd_result.first.printLatex(std::cout);
  lm_result.first.printLatex(std::cout);

  Values gt = SimpleTrajectory(half_sphere);
  size_t num_steps = gt.size() - 1;

  std::string folder = "../../results/dome_estimation_simple/";
  std::filesystem::create_directory(folder);
  std::string folder_lm = folder + "lm/";
  std::filesystem::create_directory(folder_lm);
  IEHalfSphere::ExportValues(
      lm_result.second.back().state.baseValues(), num_steps,
      folder_lm + "values_final.txt");
  IEHalfSphere::ExportValues(gt, num_steps, folder_lm + "values_gt.txt");

  auto soft_error = TranslationError(gt, soft_result.second.back().values);
  auto barrier_error =
      TranslationError(gt, barrier_result.second.back().values);
  auto gd_error = TranslationError(
      gt, gd_result.second.back().state.manifolds.baseValues());
  auto lm_error = TranslationError(
      gt, lm_result.second.back().state.baseValues());
  std::cout << soft_error << "\t" << barrier_error << "\t" << gd_error << "\t"
            << lm_error << "\n";

  //   for (const auto &iter_details : details) {
  //     IEOptimizer::PrintIterDetails(iter_details, num_steps, false,
  //                                   IEHalfSphere::PrintValues,
  //                                   IEHalfSphere::PrintDelta);

  return 0;
}
