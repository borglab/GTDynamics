#include <gtdynamics/utils/ContactInitializationFactors.h>
#include <gtdynamics/utils/ContactInitializer.h>
#include <gtsam/base/numericalDerivative.h>

#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>

using namespace gtdynamics::contact_init;
using namespace gtsam;
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
int main() {
  try {
    const Pose3 pose(Rot3::RzRyRx(0.3, -0.2, 0.7), Point3(0.4, -0.2, 0.8));
    const Point3 local(0.2, -0.1, -0.8), point(0.5, 0.1, -0.03);
    ContactFactor3 f3(Symbol('x', 0), Symbol('c', 0), local);
    Matrix Hp, Hc;
    f3.evaluateError(pose, point, &Hp, &Hc);
    auto fn3 = [&](const Pose3& x, const Point3& c) {
      return f3.evaluateError(x, c);
    };
    require(
        (Hp - numericalDerivative21<Vector, Pose3, Point3>(fn3, pose, point))
                .norm() < 1e-7,
        "Pose3 factor Jacobian");
    require(
        (Hc - numericalDerivative22<Vector, Pose3, Point3>(fn3, pose, point))
                .norm() < 1e-7,
        "Point3 factor Jacobian");
    ContactFactor2 f2(Symbol('x', 0), Symbol('c', 0), local, -0.7);
    const Point2 xy(0.5, 0.1);
    f2.evaluateError(pose, xy, &Hp, &Hc);
    auto fn2 = [&](const Pose3& x, const Point2& c) {
      return f2.evaluateError(x, c);
    };
    require((Hp - numericalDerivative21<Vector, Pose3, Point2>(fn2, pose, xy))
                    .norm() < 1e-7,
            "Plane Pose3 Jacobian");
    require((Hc - numericalDerivative22<Vector, Pose3, Point2>(fn2, pose, xy))
                    .norm() < 1e-7,
            "Plane Point2 Jacobian");
    AnchoredContactFactor3 a3(Symbol('c', 0), point);
    a3.evaluateError(point, &Hc);
    require(
        (Hc - numericalDerivative11<Vector, Point3>(
                  [&](const Point3& c) { return a3.evaluateError(c); }, point))
                .norm() < 1e-7,
        "Anchored Point3 Jacobian");
    AnchoredContactFactor2 a2(Symbol('c', 0), point, -0.7);
    a2.evaluateError(xy, &Hc);
    require((Hc - numericalDerivative11<Vector, Point2>(
                      [&](const Point2& c) { return a2.evaluateError(c); }, xy))
                    .norm() < 1e-7,
            "Anchored Point2 Jacobian");
    const UprightFactor upright(Symbol('x', 0), 0.25);
    upright.evaluateError(pose, &Hp);
    require((Hp - numericalDerivative11<Vector, Pose3>(
                      [&](const Pose3& x) { return upright.evaluateError(x); },
                      pose))
                    .norm() < 1e-7,
            "Upright pose Jacobian");
    require(
        upright.evaluateError(Pose3(Rot3::Rz(1.2), Point3(3, 4, 5))).norm() <
            1e-12,
        "Upright leaves yaw and translation free");
    require(upright.evaluateError(
                       Pose3(Rot3::Ry(3.141592653589793), Point3::Zero()))
                    .norm() > 1.9,
            "Upright penalizes an upside-down pose");
    const LandmarkGroundFactor groundFactor(Symbol('c', 0), -0.8, 0.02);
    groundFactor.evaluateError(point, &Hc);
    require((Hc -
             numericalDerivative11<Vector, Point3>(
                 [&](const Point3& c) { return groundFactor.evaluateError(c); },
                 point))
                    .norm() < 1e-7,
            "Ground height Jacobian");
    require(groundFactor.evaluateError(Point3(100, -200, -0.8)).norm() < 1e-12,
            "Ground factor leaves XY free");
    require(std::abs(groundFactor.evaluateError(Point3(0, 0, -0.7))[0] - 0.1) <
                1e-12,
            "Ground residual has metre units");
    const std::vector<Point3> sole{
        {-0.06, -0.03, 0}, {0.09, -0.03, 0}, {0.09, 0.03, 0}};
    const std::vector<std::vector<std::string>> active{
        {"L0"}, {"L0", "R0"}, {"R0"}, {"R0", "L1"}, {"L1"}};
    const std::map<std::string, Point3> locations{{"L0", {0, -0.12, -0.8}},
                                                  {"R0", {0.23, 0.12, -0.8}},
                                                  {"L1", {0.46, -0.12, -0.8}}};
    std::vector<Pose3> truth;
    std::vector<EventFrame> frames;
    for (size_t i = 0; i < active.size(); ++i) {
      const Pose3 x(Rot3::RzRyRx(0.02 * i, -0.01 * i, 0.08 * i),
                    Point3(0.1 * i, 0.012 * i, 0.02 * std::sin(i)));
      truth.push_back(x);
      EventFrame frame{i, i * 0.1, {}};
      for (const auto& id : active[i])
        for (size_t j = 0; j < sole.size(); ++j)
          frame.observations.push_back(
              {id, id[0] == 'L' ? "left" : "right", j,
               x.transformTo(Point3(locations.at(id) + sole[j]))});
      frames.push_back(frame);
    }
    for (const Model model : {Model::Patch, Model::Ground}) {
      Options options;
      options.model = model;
      options.perturb = true;
      const Result result = ContactInitializer().Initialize(frames, options);
      require(result.residualRms < 1e-6,
              "Synthetic contact reconstruction residual");
      require(result.objectiveFinal < result.objectiveInitial,
              "LM decreases the objective");
      for (size_t i = 0; i < frames.size(); ++i)
        require(result.poses[i].equals(truth[i], 1e-4),
                "Synthetic hidden pose recovery");
      require(result.values.exists(Symbol('x', 0)),
              "Result Values include the fixed anchor");
      require(!result.graph.initialValues.exists(Symbol('x', 0)),
              "Anchor is excluded from nonlinear optimization");
      for (size_t i = 1; i < result.history.size(); ++i)
        require(result.history[i].objective <=
                    result.history[i - 1].objective + 1e-12,
                "LM history is monotonic");
      if (model == Model::Ground)
        for (const auto& p : result.landmarks)
          require(p.z() == *result.graph.groundHeight,
                  "Ground plane must be exact");
    }
    Options mapped;
    mapped.model = Model::Patch;
    mapped.anchorPose = pose;
    mapped.poseKey = [](size_t i) { return Symbol('b', i + 1000); };
    mapped.landmarkKey = [](size_t i) { return Symbol('l', i + 2000); };
    const auto remapped = ContactInitializer().Initialize(frames, mapped);
    for (size_t i = 0; i < frames.size(); ++i) {
      require(remapped.poses[i].equals(pose.compose(truth[i]), 1e-7),
              "Caller world anchor propagates through contact registration");
      require(remapped.values.at<Pose3>(Symbol('b', i + 1000))
                  .equals(remapped.poses[i], 1e-12),
              "Caller pose key convention preserved");
    }
    require(remapped.values.exists(Symbol('l', 2000)),
            "Caller landmark key convention preserved");
    auto rocked = frames;
    for (auto& frame : rocked)
      for (auto& observation : frame.observations)
        if (observation.stanceId == "L1") observation.localPoint.z() += 0.15;
    Options heightOptions;
    heightOptions.model = Model::Patch;
    const auto withoutHeight =
        ContactInitializer().Initialize(rocked, heightOptions);
    heightOptions.addGroundFactors = true;
    const auto withHeight =
        ContactInitializer().Initialize(rocked, heightOptions);
    double beforeHeight = 0, afterHeight = 0;
    for (size_t i = 0; i < withHeight.landmarks.size(); ++i) {
      beforeHeight += std::pow(withoutHeight.landmarks[i].z() + 0.8, 2);
      afterHeight += std::pow(withHeight.landmarks[i].z() + 0.8, 2);
    }
    require(afterHeight < beforeHeight * 0.1,
            "Soft prior pulls elevated landmarks toward ground");
    require(
        !withHeight.graph.groundHeight &&
            withHeight.graph.initialValues.at<Point3>(Symbol('c', 0)).size() ==
                3,
        "Soft ground retains Point3 variables");
    require(std::abs(*withHeight.graph.groundPriorHeight + 0.8) < 1e-12,
            "Ground height derives from anchored FK");
    heightOptions.groundPriorHeight = -0.75;
    require(*ContactInitializer()
                    .Initialize(rocked, heightOptions)
                    .graph.groundPriorHeight == -0.75,
            "Caller can specify world ground height");
    heightOptions.model = Model::Ground;
    heightOptions.groundPriorHeight.reset();
    const auto exactHeight =
        ContactInitializer().Initialize(frames, heightOptions);
    require(exactHeight.graph.groundFactors.empty() &&
                exactHeight.groundObjective == 0,
            "Hard plane needs no redundant height factors");
    Options unsupported;
    unsupported.anchorPose = pose;
    bool rejected = false;
    try {
      ContactInitializer().Initialize({EventFrame{0, 0, {}}}, unsupported);
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    require(rejected,
            "Ground mode rejects unsupported anchors rather than dereferencing "
            "an empty contact set");
    std::vector<EventFrame> points = frames;
    for (auto& frame : points) {
      std::vector<Observation> reduced;
      for (const auto& o : frame.observations)
        if (o.pointIndex == 0) reduced.push_back(o);
      frame.observations = reduced;
    }
    Options pointOptions;
    pointOptions.model = Model::Point;
    pointOptions.perturb = true;
    const auto pointResult =
        ContactInitializer().Initialize(points, pointOptions);
    require(std::isfinite(pointResult.residualRms),
            "Ambiguous point graph solved with LM damping");
    require(pointResult.graph.frameRanks[0] == 3 &&
                pointResult.graph.frameRanks[1] == 5,
            "Point rank caveat preserved");
    frames[3].observations.erase(frames[3].observations.begin(),
                                 frames[3].observations.begin() + 3);
    for (auto& o : frames[3].observations) o.stanceId = "L2";
    for (auto& o : frames[4].observations) o.stanceId = "L2";
    Options disconnected;
    disconnected.model = Model::Patch;
    require(ContactInitializer()
                    .Initialize(frames, disconnected)
                    .graph.components.size() == 2,
            "Flight must not invent a trajectory connection");
    for (bool between : {false, true})
      for (bool up : {false, true})
        for (bool ground : {false, true}) {
          Options options = disconnected;
          options.addBetweenFactors = between;
          options.addUprightFactors = up;
          options.addGroundFactors = ground;
          const auto regularized =
              ContactInitializer().Initialize(frames, options);
          require(regularized.graph.contactComponents.size() == 2,
                  "Regularization preserves the measured contact components");
          require(regularized.graph.components.size() == (between ? 1 : 2),
                  "Only BetweenFactors connect support components");
          require(regularized.graph.anchors.size() == (between ? 1 : 2),
                  "BetweenFactors release the extra component gauge");
          require(regularized.graph.betweenFactors.size() ==
                      (between ? frames.size() - 1 : 0),
                  "BetweenFactor on each adjacent event pair");
          require(regularized.graph.uprightFactors.size() ==
                      (up ? frames.size() : 0),
                  "Upright cost on every pose including fixed anchors");
          require(regularized.graph.groundFactors.size() ==
                      (ground ? regularized.landmarks.size() : 0),
                  "One height factor per unique landmark, independent of "
                  "observation count");
          require(
              std::abs(
                  regularized.objectiveFinal - regularized.contactObjective -
                  regularized.betweenObjective - regularized.uprightObjective -
                  regularized.groundObjective) < 1e-8,
              "Reported total objective includes all selected factor families");
          double betweenCost = 0, uprightCost = 0, groundCost = 0;
          for (size_t i = 0; i < regularized.poses.size(); ++i) {
            if (up)
              uprightCost +=
                  (regularized.poses[i].rotation().rotate(Point3::UnitZ()) -
                   Point3::UnitZ())
                      .squaredNorm() /
                  (options.uprightSigma * options.uprightSigma);
            if (between && i > 0) {
              const Vector6 delta = Pose3::Logmap(
                  regularized.poses[i - 1].between(regularized.poses[i]));
              betweenCost += delta.head<3>().squaredNorm() /
                                 (options.betweenRotationSigma *
                                  options.betweenRotationSigma) +
                             delta.tail<3>().squaredNorm() /
                                 (options.betweenTranslationSigma *
                                  options.betweenTranslationSigma);
            }
          }
          require(std::abs(uprightCost - regularized.uprightObjective) < 1e-9,
                  "Upright cost matches independent world-up residuals");
          require(std::abs(betweenCost - regularized.betweenObjective) < 1e-9,
                  "Between cost matches independent whitened SE3 differences");
          if (ground)
            for (const auto& landmark : regularized.landmarks)
              groundCost += std::pow(
                  (landmark.z() - *regularized.graph.groundPriorHeight) /
                      options.groundSigma,
                  2);
          require(std::abs(groundCost - regularized.groundObjective) < 1e-9,
                  "Ground cost matches independent whitened landmark heights");
          size_t count = 0;
          for (const auto& frame : frames) count += frame.observations.size();
          require(std::abs(regularized.residualRms * regularized.residualRms *
                               count -
                           regularized.contactObjective) < 1e-10,
                  "Contact RMS excludes all prior penalties");
        }
    std::cout << "Contact factor numerical Jacobians, synthetic SE3 "
                 "reconstruction, fixed gauges, hard plane, LM descent, point "
                 "ambiguity and disconnected support passed.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
