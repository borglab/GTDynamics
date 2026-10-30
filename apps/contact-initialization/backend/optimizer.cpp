// JSON adapter and reproducible benchmark for the reusable native initializer.
// The C++ core has no JSON, recorded reference, or browser dependency.
#include <gtdynamics/utils/ContactInitializer.h>
#include <boost/json/src.hpp>
#include <gtsam/config.h>
#include <Eigen/Core>

#include <algorithm>
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
double number(const json::value& value) { return json::value_to<double>(value); }
size_t integer(const json::value& value) { const double n = number(value); if (n < 0 || n != std::floor(n)) throw std::invalid_argument("Expected a nonnegative integer."); return static_cast<size_t>(n); }
Point3 point(const json::value& value) { const auto& a = value.as_array(); if (a.size() != 3) throw std::invalid_argument("Expected a 3D point."); return {number(a[0]), number(a[1]), number(a[2])}; }
json::array vectorJson(const Point3& point) { return {point.x(), point.y(), point.z()}; }
json::object poseJson(const Pose3& pose) {
  json::array rotation;
  for (size_t i = 0; i < 3; ++i) for (size_t j = 0; j < 3; ++j) rotation.push_back(pose.rotation().matrix()(i, j));
  return {{"rotation", rotation}, {"translation", vectorJson(pose.translation())}};
}
json::array posesJson(const std::vector<Pose3>& poses) { json::array array; for (const auto& pose : poses) array.push_back(poseJson(pose)); return array; }
json::array pointsJson(const std::vector<Point3>& points) { json::array array; for (const auto& p : points) array.push_back(vectorJson(p)); return array; }
template<class T> json::array indicesJson(const std::vector<T>& values) { json::array array; for (const auto value : values) array.push_back(value); return array; }
std::string text(const json::value& value) { return std::string(value.as_string()); }
std::vector<EventFrame> datasetFrames(const json::object& data, Model model) {
  const auto& robot = data.at("model").as_object();
  std::vector<Point3> sole;
  if (model == Model::Point) sole.push_back(point(robot.at("soleCenter")));
  else { const auto& corners = robot.at("soleCorners").as_array(); for (size_t i = 0; i < 3; ++i) sole.push_back(point(corners[i])); }
  std::map<std::string, std::string> footByContact;
  for (const auto& contact : data.at("contacts").as_array()) { const auto& c = contact.as_object(); footByContact[text(c.at("id"))] = text(c.at("foot")); }
  const auto& samples = data.at("samples").as_array();
  std::vector<EventFrame> frames;
  for (const auto& source : data.at("frames").as_array()) {
    const auto& f = source.as_object();
    const size_t index = integer(f.at("sampleIndex"));
    const auto& sample = samples.at(index).as_object();
    EventFrame frame{index, number(f.at("t")), {}};
    for (const auto& idValue : f.at("contacts").as_array()) {
      const std::string id = text(idValue), foot = footByContact.at(id);
      const auto& fk = sample.at("feet").as_object().at(foot).as_object();
      const auto& q = fk.at("q").as_array();
      Eigen::Quaterniond quaternion(number(q[0]), number(q[1]), number(q[2]), number(q[3]));
      if (!(quaternion.norm() > 0) || !quaternion.coeffs().allFinite()) throw std::invalid_argument("Foot FK requires a finite nonzero quaternion.");
      quaternion.normalize();
      const Pose3 localFoot(gtsam::Rot3(quaternion), point(fk.at("p")));
      for (size_t i = 0; i < sole.size(); ++i) frame.observations.push_back({id, foot, i, localFoot.transformFrom(sole[i])});
    }
    frames.push_back(std::move(frame));
  }
  return frames;
}
json::object graphJson(const Result& result, const json::object& data, Model model) {
  const auto& graph = result.graph;
  json::array frames, landmarks, contacts, components;
  size_t observations = 0;
  std::map<std::string, std::vector<size_t>> stanceLandmarks;
  for (size_t i = 0; i < graph.landmarks.size(); ++i) {
    const auto& p = graph.landmarks[i];
    landmarks.push_back({{"id", p.id}, {"contactId", p.stanceId}, {"foot", p.foot}, {"pointIndex", p.pointIndex}});
    stanceLandmarks[p.stanceId].push_back(i);
  }
  for (const auto& raw : data.at("contacts").as_array()) {
    auto contact = raw.as_object();
    const std::string id = text(contact.at("id"));
    if (stanceLandmarks.count(id)) { contact["landmarkIndices"] = indicesJson(stanceLandmarks.at(id)); contacts.push_back(contact); }
  }
  for (size_t i = 0; i < graph.frames.size(); ++i) {
    const auto& frame = graph.frames[i];
    const auto& source = data.at("frames").as_array()[i].as_object();
    json::array measurement;
    for (const auto& o : graph.observations[i]) {
      measurement.push_back({{"landmarkIndex", o.landmarkIndex}, {"landmarkId", graph.landmarks[o.landmarkIndex].id}, {"contactId", o.measurement.stanceId}, {"foot", o.measurement.foot}, {"local", vectorJson(o.measurement.localPoint)}});
      ++observations;
    }
    frames.push_back({{"id", "frame-" + std::to_string(i)}, {"index", i}, {"sampleIndex", frame.sampleIndex}, {"time", frame.time}, {"contacts", source.at("contacts")}, {"starts", source.if_contains("starts") ? source.at("starts") : json::value(json::array{})}, {"ends", source.if_contains("ends") ? source.at("ends") : json::value(json::array{})}, {"observations", measurement}});
  }
  for (const auto& component : graph.components) components.push_back(indicesJson(component));
  const size_t poses = graph.frames.size(), points = graph.landmarks.size();
  const bool ground = model == Model::Ground;
  const size_t gauge = (ground ? 3 : 6) * graph.components.size();
  std::set<size_t> anchors(graph.anchors.begin(), graph.anchors.end());
  size_t localFree = 0;
  for (size_t i = 0; i < poses; ++i) if (!anchors.count(i)) localFree += 6 - graph.frameRanks[i];
  const long bound = static_cast<long>(6 * poses + (ground ? 2 : 3) * points) - static_cast<long>(gauge + 3 * observations);
  const std::string explanation = model == Model::Point
    ? "Point contacts leave unobservable rotations. LM damping chooses one numerical solution; it adds no measurement information or trajectory prior. Low contact residual does not certify the path."
    : ground
      ? "A known level plane fixes all landmark heights. First-sole FK sets initial tilt and floor height. Horizontal and heading gauges remain in each component; sole rocking and slip still cause trajectory error."
      : "Three noncollinear sole points transmit a full relative pose. Each disconnected component has an independent SE(3) gauge. The recorded G1 rocks its feet, so this rigid-sole model can accumulate large drift.";
  json::object dimensions{{"poseVariables", 6 * poses}, {"landmarkVariables", 3 * points}, {"jointVariables", 0}, {"totalVariables", 6 * poses + 3 * points}, {"residuals", 3 * observations}, {"observations", observations}, {"floorConstraints", ground ? points : 0}, {"effectiveLandmarkVariables", (ground ? 2 : 3) * points}, {"optimizedScalars", result.graph.initialValues.dim()}};
  json::object observability{{"componentCount", graph.components.size()}, {"components", components}, {"gaugeDofs", gauge}, {"frameRanks", indicesJson(graph.frameRanks)}, {"locallyUnobservableDofs", localFree}, {"freeDofsLowerBound", std::max<long>(localFree, std::max<long>(0, bound))}, {"fullyConstrained", model != Model::Point && std::all_of(graph.frameRanks.begin(), graph.frameRanks.end(), [](int rank) { return rank == 6; })}, {"explanation", explanation}};
  json::object output{{"model", model == Model::Point ? "point" : "patch"}, {"noise", 0}, {"frames", frames}, {"contacts", contacts}, {"landmarks", landmarks}, {"anchorFrameIndices", indicesJson(graph.anchors)}, {"components", components}, {"initialPoses", posesJson(graph.initialPoses)}, {"seedPoses", posesJson(graph.seedPoses)}, {"initialLandmarks", pointsJson(graph.initialLandmarks)}, {"dimensions", dimensions}, {"observability", observability}};
  if (graph.groundHeight) output["groundPlane"] = { {"enabled", true}, {"height", *graph.groundHeight}, {"normal", {0, 0, 1}}, {"derivedFromFrame", 0} };
  else output["groundPlane"] = nullptr;
  return output;
}
json::object stats(const std::vector<double>& samples) {
  auto sorted = samples;
  std::sort(sorted.begin(), sorted.end());
  json::array values;
  for (double value : samples) values.push_back(value);
  const size_t middle = sorted.size() / 2;
  const double median = sorted.size() % 2 ? sorted[middle] : 0.5 * (sorted[middle - 1] + sorted[middle]);
  return {{"samplesMs", values}, {"medianMs", median}, {"minMs", sorted.front()}, {"p95Ms", sorted[static_cast<size_t>(std::ceil(0.95 * sorted.size())) - 1]}};
}
json::object environment() {
  std::string cpu = "unknown";
  std::ifstream cpuinfo("/proc/cpuinfo");
  for (std::string line; std::getline(cpuinfo, line);) if (line.rfind("model name", 0) == 0) { cpu = line.substr(line.find(':') + 2); break; }
  return {{"compiler", __VERSION__}, {"gtsamVersion", GTSAM_VERSION_STRING}, {"buildType", CONTACT_BUILD_TYPE}, {"cpu", cpu}, {"hardwareThreads", std::thread::hardware_concurrency()}, {"eigenThreads", Eigen::nbThreads()}, {"solver", "gtsam::LevenbergMarquardtOptimizer"}, {"linearSolver", "MULTIFRONTAL_CHOLESKY"}, {"diagonalDamping", true}, {"landmarkParameterization", "Point3; Point2 for hard level plane"}};
}
json::object run(const json::object& request) {
  const auto& data = request.at("dataset").as_object();
  const std::string modelName = request.if_contains("model") ? text(request.at("model")) : "ground";
  if (modelName != "ground" && modelName != "patch" && modelName != "point") throw std::invalid_argument("model must be ground, patch or point");
  const Model model = modelName == "ground" ? Model::Ground : modelName == "patch" ? Model::Patch : Model::Point;
  Options options;
  options.model = model;
  options.maxIterations = request.if_contains("iterations") ? integer(request.at("iterations")) : 100;
  options.perturb = request.if_contains("perturb") && request.at("perturb").as_bool();
  const size_t repetitions = request.if_contains("repetitions") ? integer(request.at("repetitions")) : 9;
  const size_t warmup = request.if_contains("warmup") ? integer(request.at("warmup")) : 2;
  if (!repetitions || repetitions > 1000 || warmup > 100 || options.maxIterations > 1000) throw std::invalid_argument("Benchmark bounds exceeded.");
  // Source FK adaptation happens once, outside native core phase measurements.
  const auto frames = datasetFrames(data, model);
  options.recordHistory = false;
  ContactInitializer initializer;
  for (size_t i = 0; i < warmup; ++i) initializer.Initialize(frames, options);
  std::vector<double> optimization, build, seed, total;
  std::vector<size_t> iterationCounts;
  for (size_t i = 0; i < repetitions; ++i) {
    const auto result = initializer.Initialize(frames, options);
    optimization.push_back(result.timings.optimizationMs);
    build.push_back(result.timings.graphBuildMs);
    seed.push_back(result.timings.seedMs);
    total.push_back(result.timings.totalMs);
    iterationCounts.push_back(result.iterations);
  }
  // History is captured in a separate run and excluded from benchmark samples.
  options.recordHistory = true;
  const auto result = initializer.Initialize(frames, options);
  const auto graph = graphJson(result, data, model);
  json::array history;
  for (const auto& state : result.history) history.push_back({{"iteration", state.iteration}, {"objective", state.objective}, {"rms", state.rms}, {"poses", posesJson(state.poses)}, {"landmarks", pointsJson(state.landmarks)}});
  auto benchmark = stats(optimization);
  benchmark["repetitions"] = repetitions;
  benchmark["warmup"] = warmup;
  benchmark["iterations"] = indicesJson(iterationCounts);
  benchmark["phases"] = {{"graphBuild", stats(build)}, {"seed", stats(seed)}, {"optimization", stats(optimization)}, {"total", stats(total)}};
  benchmark["scope"] = "Fresh native graph and FK/contact registration seed each run. Optimization samples include LM construction, linearization, damping and sparse solves; history snapshots disabled. No browser solver.";
  benchmark["excluded"] = {"file IO", "JSON parsing", "dataset-to-FK-event adaptation", "reference evaluation", "history capture", "JSON serialization", "network", "browser rendering"};
  json::object outputResult{{"poses", posesJson(result.poses)}, {"landmarks", pointsJson(result.landmarks)}, {"history", history}, {"residualRms", result.residualRms}, {"objectiveInitial", result.objectiveInitial}, {"objectiveFinal", result.objectiveFinal}, {"iterations", result.iterations}, {"converged", result.converged}, {"timeMs", benchmark.at("medianMs")}, {"observability", graph.at("observability")}, {"dimensions", graph.at("dimensions")}, {"solver", "gtsam::LevenbergMarquardtOptimizer"}, {"backend", "native-cpp"}};
  return {{"schemaVersion", 1}, {"graph", graph}, {"result", outputResult}, {"benchmark", benchmark}, {"environment", environment()}};
}
}  // namespace
int main(int argc, char** argv) {
  try {
    std::ifstream file;
    std::istream* input = &std::cin;
    if (argc > 1) { file.open(argv[1]); if (!file) throw std::runtime_error("Cannot read request file."); input = &file; }
    const std::string body{std::istreambuf_iterator<char>(*input), std::istreambuf_iterator<char>()};
    const auto response = run(json::parse(body).as_object());
    std::cout << json::serialize(response) << '\n';
    return 0;
  } catch (const std::exception& error) { std::cerr << "contact_initializer: " << error.what() << '\n'; return 1; }
}
