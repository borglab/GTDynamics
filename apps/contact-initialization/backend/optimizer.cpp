/**
 * @file optimizer.cpp
 * JSON adapter and repeated native benchmark for ContactInitializer.
 *
 * Reads one JSON request from stdin or a file and writes one JSON response.
 * dataset supplies frozen FK, selected frames, and explicit stance labels;
 * regularization selects the optional native priors. evaluationReference is
 * separate and is read only after every solve, outside all timed scopes.
 *
 * Timing repetitions construct fresh graphs without history. A separate solve
 * captures replay snapshots. Output reports phase timing samples, convergence,
 * factor-family costs, graph structure, and optional held-out pose errors. The
 * core has no JSON, reference-trajectory, or browser dependency.
 */
#include <gtdynamics/utils/ContactInitializer.h>
#include <gtsam/config.h>

#include <Eigen/Core>
#include <algorithm>
#include <boost/json/src.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <set>
#include <thread>

namespace json = boost::json;
using namespace gtdynamics::contact_init;
using gtsam::Point3;
using gtsam::Pose3;

namespace {
double number(const json::value& value) {
  return json::value_to<double>(value);
}
size_t integer(const json::value& value) {
  const double n = number(value);
  if (n < 0 || n != std::floor(n))
    throw std::invalid_argument("Expected a nonnegative integer.");
  return static_cast<size_t>(n);
}
Point3 point(const json::value& value) {
  const auto& a = value.as_array();
  if (a.size() != 3) throw std::invalid_argument("Expected a 3D point.");
  return {number(a[0]), number(a[1]), number(a[2])};
}
json::array vectorJson(const Point3& point) {
  return {point.x(), point.y(), point.z()};
}
// Serialize rotation as a row-major 3x3 array and translation in world metres.
json::object poseJson(const Pose3& pose) {
  json::array rotation;
  for (size_t i = 0; i < 3; ++i)
    for (size_t j = 0; j < 3; ++j)
      rotation.push_back(pose.rotation().matrix()(i, j));
  return {{"rotation", rotation},
          {"translation", vectorJson(pose.translation())}};
}
json::array posesJson(const std::vector<Pose3>& poses) {
  json::array array;
  for (const auto& pose : poses) array.push_back(poseJson(pose));
  return array;
}
json::array pointsJson(const std::vector<Point3>& points) {
  json::array array;
  for (const auto& p : points) array.push_back(vectorJson(p));
  return array;
}
template <class T>
json::array indicesJson(const std::vector<T>& values) {
  json::array array;
  for (const auto value : values) array.push_back(value);
  return array;
}
std::string text(const json::value& value) {
  return std::string(value.as_string());
}
// Select original samples and transform fixed sole points by local FK.
// Source quaternions use WXYZ; source world-root reference poses are not read.
std::vector<ContactMeasurements> datasetMeasurements(const json::object& data,
                                                     Model model) {
  const auto& robot = data.at("model").as_object();
  std::vector<Point3> sole;
  if (model == Model::Point)
    sole.push_back(point(robot.at("soleCenter")));
  else {
    const auto& corners = robot.at("soleCorners").as_array();
    for (size_t i = 0; i < 3; ++i) sole.push_back(point(corners[i]));
  }
  std::map<std::string, std::string> footByContact;
  for (const auto& contact : data.at("contacts").as_array()) {
    const auto& c = contact.as_object();
    footByContact[text(c.at("id"))] = text(c.at("foot"));
  }
  const auto& samples = data.at("samples").as_array();
  std::vector<ContactMeasurements> frames;
  for (const auto& source : data.at("frames").as_array()) {
    const auto& f = source.as_object();
    const size_t index = integer(f.at("sampleIndex"));
    const auto& sample = samples.at(index).as_object();
    ContactMeasurements frame{gtdynamics::Slice(index), number(f.at("t")), {}};
    for (const auto& idValue : f.at("contacts").as_array()) {
      const std::string id = text(idValue), foot = footByContact.at(id);
      const auto& fk = sample.at("feet").as_object().at(foot).as_object();
      const auto& q = fk.at("q").as_array();
      Eigen::Quaterniond quaternion(number(q[0]), number(q[1]), number(q[2]),
                                    number(q[3]));
      if (!(quaternion.norm() > 0) || !quaternion.coeffs().allFinite())
        throw std::invalid_argument(
            "Foot FK requires a finite nonzero quaternion.");
      quaternion.normalize();
      const Pose3 localFoot(gtsam::Rot3(quaternion), point(fk.at("p")));
      for (size_t i = 0; i < sole.size(); ++i)
        frame.observations.push_back(
            {id, foot, i, localFoot.transformFrom(sole[i])});
    }
    frames.push_back(std::move(frame));
  }
  return frames;
}
// Export contact-only connectivity separately from the graph with motion
// priors. Local geometric ranks describe contact observations, not full
// observability.
json::object graphJson(const Result& result, const json::object& data,
                       const Options& options) {
  const Model model = options.model;
  const auto& graph = result.graph;
  json::array frames, landmarks, contacts, components, contactComponents,
      betweenEdges, uprightPoses, groundLandmarks;
  size_t observations = 0;
  std::map<std::string, std::vector<size_t>> stanceLandmarks;
  for (size_t i = 0; i < graph.landmarks.size(); ++i) {
    const auto& p = graph.landmarks[i];
    landmarks.push_back({{"id", p.id},
                         {"contactId", p.stanceId},
                         {"foot", p.foot},
                         {"pointIndex", p.pointIndex}});
    stanceLandmarks[p.stanceId].push_back(i);
  }
  for (const auto& raw : data.at("contacts").as_array()) {
    auto contact = raw.as_object();
    const std::string id = text(contact.at("id"));
    if (stanceLandmarks.count(id)) {
      contact["landmarkIndices"] = indicesJson(stanceLandmarks.at(id));
      contacts.push_back(contact);
    }
  }
  for (size_t i = 0; i < graph.measurements.size(); ++i) {
    const auto& frame = graph.measurements[i];
    const auto& source = data.at("frames").as_array()[i].as_object();
    json::array measurement;
    for (const auto& o : graph.observations[i]) {
      measurement.push_back(
          {{"landmarkIndex", o.landmarkIndex},
           {"landmarkId", graph.landmarks[o.landmarkIndex].id},
           {"contactId", o.measurement.stanceId},
           {"foot", o.measurement.foot},
           {"local", vectorJson(o.measurement.localPoint)}});
      ++observations;
    }
    // Preserve the saved JSON schema: sampleIndex is the original Slice::k.
    frames.push_back(
        {{"id", "frame-" + std::to_string(i)},
         {"index", i},
         {"sampleIndex", frame.slice.k},
         {"time", frame.time},
         {"contacts", source.at("contacts")},
         {"starts", source.if_contains("starts") ? source.at("starts")
                                                 : json::value(json::array{})},
         {"ends", source.if_contains("ends") ? source.at("ends")
                                             : json::value(json::array{})},
         {"observations", measurement}});
  }
  for (const auto& component : graph.components)
    components.push_back(indicesJson(component));
  for (const auto& component : graph.contactComponents)
    contactComponents.push_back(indicesJson(component));
  if (options.addBetweenFactors)
    for (size_t i = 1; i < graph.measurements.size(); ++i)
      betweenEdges.push_back({i - 1, i});
  if (options.addUprightFactors)
    for (size_t i = 0; i < graph.measurements.size(); ++i)
      uprightPoses.push_back(i);
  if (!graph.groundFactors.empty())
    for (size_t i = 0; i < graph.landmarks.size(); ++i)
      groundLandmarks.push_back(i);
  const size_t poses = graph.measurements.size(),
               points = graph.landmarks.size();
  const bool ground = model == Model::Ground;
  const size_t gauge = (ground || options.addGroundFactors ? 3
                        : options.addUprightFactors        ? 4
                                                           : 6) *
                       graph.components.size();
  std::set<size_t> anchors(graph.anchors.begin(), graph.anchors.end());
  size_t localFree = 0;
  for (size_t i = 0; i < poses; ++i)
    if (!anchors.count(i)) localFree += 6 - graph.frameRanks[i];
  const long bound = static_cast<long>(6 * poses + (ground ? 2 : 3) * points) -
                     static_cast<long>(gauge + 3 * observations);
  const std::string explanation =
      model == Model::Point
          ? "Point contacts leave unobservable rotations. LM damping chooses "
            "one numerical solution; it adds no measurement information or "
            "trajectory prior. Low contact residual does not certify the path."
      : ground ? "A known level plane fixes all landmark heights. First-sole "
                 "FK sets initial tilt and floor height. Horizontal and "
                 "heading gauges remain in each component; sole rocking and "
                 "slip still cause trajectory error."
               : "Three noncollinear sole points transmit a full relative "
                 "pose. Each disconnected component has an independent SE(3) "
                 "gauge. The recorded G1 rocks its feet, so this rigid-sole "
                 "model can accumulate large drift.";
  json::object dimensions{
      {"poseVariables", 6 * poses},
      {"landmarkVariables", 3 * points},
      {"jointVariables", 0},
      {"totalVariables", 6 * poses + 3 * points},
      {"residuals", 3 * observations},
      {"observations", observations},
      {"floorConstraints", ground ? points : 0},
      {"effectiveLandmarkVariables", (ground ? 2 : 3) * points},
      {"optimizedScalars", result.graph.initialValues.dim()}};
  json::object observability{
      {"componentCount", graph.components.size()},
      {"components", components},
      {"gaugeDofs", gauge},
      {"frameRanks", indicesJson(graph.frameRanks)},
      {"locallyUnobservableDofs", localFree},
      {"freeDofsLowerBound",
       std::max<long>(localFree, std::max<long>(0, bound))},
      {"fullyConstrained",
       model != Model::Point &&
           std::all_of(graph.frameRanks.begin(), graph.frameRanks.end(),
                       [](int rank) { return rank == 6; })},
      {"explanation", explanation}};
  observability["contactComponentCount"] = graph.contactComponents.size();
  observability["contactComponents"] = contactComponents;
  if (options.addBetweenFactors || options.addUprightFactors ||
      options.addGroundFactors) {
    if (options.addBetweenFactors || options.addUprightFactors)
      observability["freeDofsLowerBound"] = 0;
    observability["explanation"] =
        explanation +
        " Local frame ranks describe contacts alone. Optional motion, upright "
        "and landmark-height factors add prior information; they are not "
        "contact measurements.";
  }
  observability["fullyConstrained"] =
      options.addBetweenFactors ||
      observability.at("fullyConstrained").as_bool();
  dimensions["betweenFactors"] = graph.betweenFactors.size();
  dimensions["uprightFactors"] = graph.uprightFactors.size();
  dimensions["groundFactors"] = graph.groundFactors.size();
  dimensions["residuals"] = 3 * observations + 6 * graph.betweenFactors.size() +
                            3 * graph.uprightFactors.size() +
                            graph.groundFactors.size();
  const double rotationVariance =
      options.betweenRotationSigma * options.betweenRotationSigma;
  const double translationVariance =
      options.betweenTranslationSigma * options.betweenTranslationSigma;
  json::object regularization{
      {"between", options.addBetweenFactors},
      {"upright", options.addUprightFactors},
      {"ground", options.addGroundFactors},
      {"betweenTranslationSigma", options.betweenTranslationSigma},
      {"betweenRotationSigma", options.betweenRotationSigma},
      {"uprightSigma", options.uprightSigma},
      {"betweenMeasurement", "identity"},
      {"betweenCovarianceDiagonal",
       {rotationVariance, rotationVariance, rotationVariance,
        translationVariance, translationVariance, translationVariance}},
      {"uprightResidual",
       "world-space body Z minus world Z; translation and heading remain "
       "free"}};
  regularization["groundSigma"] = options.groundSigma;
  regularization["groundHeight"] = graph.groundPriorHeight
                                       ? json::value(*graph.groundPriorHeight)
                                       : json::value(nullptr);
  regularization["groundAlreadyFixed"] = ground;
  regularization["groundResidual"] =
      "landmark Z minus ground height; horizontal coordinates remain free";
  regularization["groundHeightSource"] =
      options.groundPriorHeight
          ? "caller-supplied world height"
          : "first anchored stance mean height; no reference poses";
  json::object output{{"model", model == Model::Point ? "point" : "patch"},
                      {"noise", 0},
                      {"frames", frames},
                      {"contacts", contacts},
                      {"landmarks", landmarks},
                      {"anchorFrameIndices", indicesJson(graph.anchors)},
                      {"components", components},
                      {"contactComponents", contactComponents},
                      {"betweenEdges", betweenEdges},
                      {"uprightPoseIndices", uprightPoses},
                      {"regularization", regularization},
                      {"initialPoses", posesJson(graph.initialPoses)},
                      {"seedPoses", posesJson(graph.seedPoses)},
                      {"initialLandmarks", pointsJson(graph.initialLandmarks)},
                      {"dimensions", dimensions},
                      {"observability", observability}};
  if (graph.groundHeight)
    output["groundPlane"] = {{"enabled", true},
                             {"height", *graph.groundHeight},
                             {"normal", {0, 0, 1}},
                             {"derivedFromFrame", 0}};
  else
    output["groundPlane"] = nullptr;
  output["groundLandmarkIndices"] = groundLandmarks;
  return output;
}
// Report the median and nearest-rank p95 alongside every native timing sample.
json::object stats(const std::vector<double>& samples) {
  auto sorted = samples;
  std::sort(sorted.begin(), sorted.end());
  json::array values;
  for (double value : samples) values.push_back(value);
  const size_t middle = sorted.size() / 2;
  const double median = sorted.size() % 2
                            ? sorted[middle]
                            : 0.5 * (sorted[middle - 1] + sorted[middle]);
  return {{"samplesMs", values},
          {"medianMs", median},
          {"minMs", sorted.front()},
          {"p95Ms",
           sorted[static_cast<size_t>(std::ceil(0.95 * sorted.size())) - 1]}};
}
json::object environment() {
  std::string cpu = "unknown";
  std::ifstream cpuinfo("/proc/cpuinfo");
  for (std::string line; std::getline(cpuinfo, line);)
    if (line.rfind("model name", 0) == 0) {
      cpu = line.substr(line.find(':') + 2);
      break;
    }
  return {{"compiler", __VERSION__},
          {"gtsamVersion", GTSAM_VERSION_STRING},
          {"buildType", CONTACT_BUILD_TYPE},
          {"cpu", cpu},
          {"hardwareThreads", std::thread::hardware_concurrency()},
          {"eigenThreads", Eigen::nbThreads()},
          {"solver", "gtsam::LevenbergMarquardtOptimizer"},
          {"linearSolver", "MULTIFRONTAL_CHOLESKY"},
          {"diagonalDamping", true},
          {"landmarkParameterization", "Point3; Point2 for hard level plane"}};
}
// Evaluation-only gauge alignment at each graph component's first frame.
// Motion-connected variants have one alignment; disconnected variants have
// more.
std::vector<Pose3> alignedReferences(const Graph& graph,
                                     const json::object& reference) {
  const auto& source = reference.at("poses").as_array();
  std::vector<Pose3> poses;
  for (const auto& frame : graph.measurements) {
    const auto& value = source.at(frame.slice.k).as_object();
    const auto& q = value.at("q").as_array();
    Eigen::Quaterniond quaternion(number(q[0]), number(q[1]), number(q[2]),
                                  number(q[3]));
    if (!(quaternion.norm() > 0) || !quaternion.coeffs().allFinite())
      throw std::invalid_argument("Invalid evaluation quaternion.");
    poses.emplace_back(gtsam::Rot3(quaternion.normalized()),
                       point(value.at("p")));
  }
  for (const auto& component : graph.components) {
    const size_t anchor = component.front();
    const auto alignment =
        graph.initialPoses[anchor].compose(poses[anchor].inverse());
    for (size_t index : component)
      poses[index] = alignment.compose(poses[index]);
  }
  return poses;
}
// Root-position RMSE in metres and rotation-log RMSE in degrees, after
// alignment.
json::object referenceErrors(const std::vector<Pose3>& poses,
                             const std::vector<Pose3>& truth) {
  double position = 0, angle = 0;
  for (size_t i = 0; i < poses.size(); ++i) {
    position += (poses[i].translation() - truth[i].translation()).squaredNorm();
    angle +=
        gtsam::Rot3::Logmap(truth[i].rotation().between(poses[i].rotation()))
            .squaredNorm();
  }
  return {{"positionRmseMeters", std::sqrt(position / poses.size())},
          {"orientationRmseDegrees",
           180.0 / 3.141592653589793 * std::sqrt(angle / poses.size())}};
}
// Benchmark fresh native runs, capture replay separately, and finally evaluate.
json::object run(const json::object& request) {
  const auto& data = request.at("dataset").as_object();
  const std::string modelName =
      request.if_contains("model") ? text(request.at("model")) : "ground";
  if (modelName != "ground" && modelName != "patch" && modelName != "point")
    throw std::invalid_argument("model must be ground, patch or point");
  const Model model = modelName == "ground"  ? Model::Ground
                      : modelName == "patch" ? Model::Patch
                                             : Model::Point;
  Options options;
  options.model = model;
  options.maxIterations = request.if_contains("iterations")
                              ? integer(request.at("iterations"))
                              : 100;
  options.perturb =
      request.if_contains("perturb") && request.at("perturb").as_bool();
  if (const auto* regularization = request.if_contains("regularization")) {
    const auto& settings = regularization->as_object();
    options.addBetweenFactors =
        settings.if_contains("between") && settings.at("between").as_bool();
    options.addUprightFactors =
        settings.if_contains("upright") && settings.at("upright").as_bool();
    options.addGroundFactors =
        settings.if_contains("ground") && settings.at("ground").as_bool();
    if (settings.if_contains("betweenTranslationSigma"))
      options.betweenTranslationSigma =
          number(settings.at("betweenTranslationSigma"));
    if (settings.if_contains("betweenRotationSigma"))
      options.betweenRotationSigma =
          number(settings.at("betweenRotationSigma"));
    if (settings.if_contains("uprightSigma"))
      options.uprightSigma = number(settings.at("uprightSigma"));
    if (settings.if_contains("groundSigma"))
      options.groundSigma = number(settings.at("groundSigma"));
    if (settings.if_contains("groundHeight"))
      options.groundPriorHeight = number(settings.at("groundHeight"));
  }
  const size_t repetitions = request.if_contains("repetitions")
                                 ? integer(request.at("repetitions"))
                                 : 9;
  const size_t warmup =
      request.if_contains("warmup") ? integer(request.at("warmup")) : 2;
  if (!repetitions || repetitions > 1000 || warmup > 100 ||
      options.maxIterations > 1000)
    throw std::invalid_argument("Benchmark bounds exceeded.");
  // Source FK adaptation happens once, outside native core phase measurements.
  const auto measurements = datasetMeasurements(data, model);
  options.recordHistory = false;
  ContactInitializer initializer;
  for (size_t i = 0; i < warmup; ++i)
    initializer.Initialize(measurements, options);
  std::vector<double> optimization, build, seed, total;
  std::vector<size_t> iterationCounts;
  for (size_t i = 0; i < repetitions; ++i) {
    const auto result = initializer.Initialize(measurements, options);
    optimization.push_back(result.timings.optimizationMs);
    build.push_back(result.timings.graphBuildMs);
    seed.push_back(result.timings.seedMs);
    total.push_back(result.timings.totalMs);
    iterationCounts.push_back(result.iterations);
  }
  // History is captured in a separate run and excluded from benchmark samples.
  options.recordHistory = true;
  const auto result = initializer.Initialize(measurements, options);
  const auto graph = graphJson(result, data, options);
  // Reference evaluation happens after all solves and outside every timing
  // scope.
  std::vector<Pose3> reference;
  json::object evaluation;
  if (const auto* heldOut = request.if_contains("evaluationReference")) {
    reference = alignedReferences(result.graph, heldOut->as_object());
    evaluation = referenceErrors(result.poses, reference);
    evaluation["alignedReferencePoses"] = posesJson(reference);
    evaluation["componentCount"] = result.graph.components.size();
    evaluation["scope"] =
        "Held-out root poses, aligned once per connected graph component; "
        "never used for initialization or optimization";
  }
  json::array history;
  for (const auto& state : result.history) {
    json::object saved{{"iteration", state.iteration},
                       {"objective", state.objective},
                       {"rms", state.rms},
                       {"poses", posesJson(state.poses)},
                       {"landmarks", pointsJson(state.landmarks)}};
    if (!reference.empty())
      saved["referenceError"] = referenceErrors(state.poses, reference);
    history.push_back(saved);
  }
  auto benchmark = stats(optimization);
  benchmark["repetitions"] = repetitions;
  benchmark["warmup"] = warmup;
  benchmark["iterations"] = indicesJson(iterationCounts);
  benchmark["phases"] = {{"graphBuild", stats(build)},
                         {"seed", stats(seed)},
                         {"optimization", stats(optimization)},
                         {"total", stats(total)}};
  benchmark["scope"] =
      "Fresh native graph and FK/contact registration seed each run. "
      "Optimization samples include LM construction, linearization, damping "
      "and sparse solves; history snapshots disabled. No browser solver.";
  benchmark["excluded"] = {"file IO",
                           "JSON parsing",
                           "dataset-to-FK-event adaptation",
                           "reference evaluation",
                           "history capture",
                           "JSON serialization",
                           "network",
                           "browser rendering"};
  json::object outputResult{{"poses", posesJson(result.poses)},
                            {"landmarks", pointsJson(result.landmarks)},
                            {"history", history},
                            {"residualRms", result.residualRms},
                            {"objectiveInitial", result.objectiveInitial},
                            {"objectiveFinal", result.objectiveFinal},
                            {"iterations", result.iterations},
                            {"converged", result.converged},
                            {"timeMs", benchmark.at("medianMs")},
                            {"observability", graph.at("observability")},
                            {"dimensions", graph.at("dimensions")},
                            {"solver", "gtsam::LevenbergMarquardtOptimizer"},
                            {"backend", "native-cpp"}};
  outputResult["objectiveComponents"] = {{"contact", result.contactObjective},
                                         {"between", result.betweenObjective},
                                         {"upright", result.uprightObjective},
                                         {"ground", result.groundObjective}};
  return {{"schemaVersion", 1},           {"graph", graph},
          {"result", outputResult},       {"benchmark", benchmark},
          {"environment", environment()}, {"evaluation", evaluation}};
}
}  // namespace
int main(int argc, char** argv) {
  try {
    std::ifstream file;
    std::istream* input = &std::cin;
    if (argc > 1) {
      file.open(argv[1]);
      if (!file) throw std::runtime_error("Cannot read request file.");
      input = &file;
    }
    const std::string body{std::istreambuf_iterator<char>(*input),
                           std::istreambuf_iterator<char>()};
    const auto response = run(json::parse(body).as_object());
    std::cout << json::serialize(response) << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "contact_initializer: " << error.what() << '\n';
    return 1;
  }
}
