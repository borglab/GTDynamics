#include <gtdynamics/utils/ContactInitializer.h>
#include <gtdynamics/utils/ContactInitializationFactors.h>
#include <gtsam/base/numericalDerivative.h>

#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>

using namespace gtdynamics::contact_init;
using namespace gtsam;
void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
int main() {
  try {
    const Pose3 pose(Rot3::RzRyRx(0.3, -0.2, 0.7), Point3(0.4, -0.2, 0.8));
    const Point3 local(0.2, -0.1, -0.8), point(0.5, 0.1, -0.03);
    ContactFactor3 f3(Symbol('x', 0), Symbol('c', 0), local);
    Matrix Hp, Hc;
    f3.evaluateError(pose, point, &Hp, &Hc);
    auto fn3 = [&](const Pose3& x, const Point3& c) { return f3.evaluateError(x, c); };
    require((Hp - numericalDerivative21<Vector, Pose3, Point3>(fn3, pose, point)).norm() < 1e-7, "Pose3 factor Jacobian");
    require((Hc - numericalDerivative22<Vector, Pose3, Point3>(fn3, pose, point)).norm() < 1e-7, "Point3 factor Jacobian");
    ContactFactor2 f2(Symbol('x', 0), Symbol('c', 0), local, -0.7);
    const Point2 xy(0.5, 0.1);
    f2.evaluateError(pose, xy, &Hp, &Hc);
    auto fn2 = [&](const Pose3& x, const Point2& c) { return f2.evaluateError(x, c); };
    require((Hp - numericalDerivative21<Vector, Pose3, Point2>(fn2, pose, xy)).norm() < 1e-7, "Plane Pose3 Jacobian");
    require((Hc - numericalDerivative22<Vector, Pose3, Point2>(fn2, pose, xy)).norm() < 1e-7, "Plane Point2 Jacobian");
    AnchoredContactFactor3 a3(Symbol('c', 0), point);
    a3.evaluateError(point, &Hc);
    require((Hc - numericalDerivative11<Vector, Point3>([&](const Point3& c) { return a3.evaluateError(c); }, point)).norm() < 1e-7, "Anchored Point3 Jacobian");
    AnchoredContactFactor2 a2(Symbol('c', 0), point, -0.7);
    a2.evaluateError(xy, &Hc);
    require((Hc - numericalDerivative11<Vector, Point2>([&](const Point2& c) { return a2.evaluateError(c); }, xy)).norm() < 1e-7, "Anchored Point2 Jacobian");
    const std::vector<Point3> sole{{-0.06, -0.03, 0}, {0.09, -0.03, 0}, {0.09, 0.03, 0}};
    const std::vector<std::vector<std::string>> active{{"L0"}, {"L0", "R0"}, {"R0"}, {"R0", "L1"}, {"L1"}};
    const std::map<std::string, Point3> locations{{"L0", {0, -0.12, -0.8}}, {"R0", {0.23, 0.12, -0.8}}, {"L1", {0.46, -0.12, -0.8}}};
    std::vector<Pose3> truth;
    std::vector<EventFrame> frames;
    for (size_t i = 0; i < active.size(); ++i) {
      const Pose3 x(Rot3::RzRyRx(0.02 * i, -0.01 * i, 0.08 * i), Point3(0.1 * i, 0.012 * i, 0.02 * std::sin(i)));
      truth.push_back(x);
      EventFrame frame{i, i * 0.1, {}};
      for (const auto& id : active[i]) for (size_t j = 0; j < sole.size(); ++j) frame.observations.push_back({id, id[0] == 'L' ? "left" : "right", j, x.transformTo(Point3(locations.at(id) + sole[j]))});
      frames.push_back(frame);
    }
    for (const Model model : {Model::Patch, Model::Ground}) {
      Options options;
      options.model = model;
      options.perturb = true;
      const Result result = ContactInitializer().Initialize(frames, options);
      require(result.residualRms < 1e-6, "Synthetic contact reconstruction residual");
      require(result.objectiveFinal < result.objectiveInitial, "LM decreases the objective");
      for (size_t i = 0; i < frames.size(); ++i) require(result.poses[i].equals(truth[i], 1e-4), "Synthetic hidden pose recovery");
      require(result.values.exists(Symbol('x', 0)), "Result Values include the fixed anchor");
      require(!result.graph.initialValues.exists(Symbol('x', 0)), "Anchor is excluded from nonlinear optimization");
      for (size_t i = 1; i < result.history.size(); ++i) require(result.history[i].objective <= result.history[i - 1].objective + 1e-12, "LM history is monotonic");
      if (model == Model::Ground) for (const auto& p : result.landmarks) require(p.z() == *result.graph.groundHeight, "Ground plane must be exact");
    }
    Options mapped;
    mapped.model = Model::Patch;
    mapped.anchorPose = pose;
    mapped.poseKey = [](size_t i) { return Symbol('b', i + 1000); };
    mapped.landmarkKey = [](size_t i) { return Symbol('l', i + 2000); };
    const auto remapped = ContactInitializer().Initialize(frames, mapped);
    for (size_t i = 0; i < frames.size(); ++i) {
      require(remapped.poses[i].equals(pose.compose(truth[i]), 1e-7), "Caller world anchor propagates through contact registration");
      require(remapped.values.at<Pose3>(Symbol('b', i + 1000)).equals(remapped.poses[i], 1e-12), "Caller pose key convention preserved");
    }
    require(remapped.values.exists(Symbol('l', 2000)), "Caller landmark key convention preserved");
    Options unsupported;
    unsupported.anchorPose = pose;
    bool rejected = false;
    try { ContactInitializer().Initialize({EventFrame{0, 0, {}}}, unsupported); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Ground mode rejects unsupported anchors rather than dereferencing an empty contact set");
    std::vector<EventFrame> points = frames;
    for (auto& frame : points) {
      std::vector<Observation> reduced;
      for (const auto& o : frame.observations) if (o.pointIndex == 0) reduced.push_back(o);
      frame.observations = reduced;
    }
    Options pointOptions;
    pointOptions.model = Model::Point;
    pointOptions.perturb = true;
    const auto pointResult = ContactInitializer().Initialize(points, pointOptions);
    require(std::isfinite(pointResult.residualRms), "Ambiguous point graph solved with LM damping");
    require(pointResult.graph.frameRanks[0] == 3 && pointResult.graph.frameRanks[1] == 5, "Point rank caveat preserved");
    frames[3].observations.erase(frames[3].observations.begin(), frames[3].observations.begin() + 3);
    for (auto& o : frames[3].observations) o.stanceId = "L2";
    for (auto& o : frames[4].observations) o.stanceId = "L2";
    Options disconnected;
    disconnected.model = Model::Patch;
    require(ContactInitializer().Initialize(frames, disconnected).graph.components.size() == 2, "Flight must not invent a trajectory connection");
    std::cout << "Contact factor numerical Jacobians, synthetic SE3 reconstruction, fixed gauges, hard plane, LM descent, point ambiguity and disconnected support passed.\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
