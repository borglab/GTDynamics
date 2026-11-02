/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
#include <gtdynamics/utils/ContactMotion.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gtdynamics {

std::vector<ContactMotionSample> contactMotionLabels(
    const std::vector<double>& times,
    const std::vector<gtsam::Pose3>& worldTfoot,
    const std::vector<gtsam::Point3>& points,
    const ContactMotionParams& params) {
  const auto validPair = [](double low, double high) {
    return std::isfinite(low) && std::isfinite(high) && low > 0 && high > low;
  };
  if (times.empty() || times.size() != worldTfoot.size() || points.empty() ||
      !std::isfinite(params.windowSeconds) || params.windowSeconds <= 0 ||
      !validPair(params.stationarySpeed, params.movingSpeed) ||
      !validPair(params.stationaryExcursion, params.movingExcursion)) {
    throw std::invalid_argument(
        "contactMotionLabels: invalid inputs or thresholds");
  }
  for (size_t i = 0; i < times.size(); ++i) {
    if (!std::isfinite(times[i]) || (i && times[i] <= times[i - 1]) ||
        !worldTfoot[i].matrix().allFinite())
      throw std::invalid_argument(
          "contactMotionLabels: invalid times or poses");
  }
  for (const auto& point : points)
    if (!point.allFinite())
      throw std::invalid_argument(
          "contactMotionLabels: nonfinite material point");

  std::vector<std::vector<gtsam::Point3>> worldPoints(times.size());
  for (size_t i = 0; i < times.size(); ++i)
    for (const auto& point : points)
      worldPoints[i].push_back(worldTfoot[i].transformFrom(point));
  std::vector<ContactMotionSample> result(times.size());
  const double halfWindow = params.windowSeconds / 2;
  for (size_t i = 0; i < times.size(); ++i) {
    auto& sample = result[i];
    const double firstTime = times[i] - halfWindow;
    const double lastTime = times[i] + halfWindow;
    const size_t first =
        std::lower_bound(times.begin(), times.end(), firstTime - 1e-10) -
        times.begin();
    const size_t last =
        std::upper_bound(times.begin(), times.end(), lastTime + 1e-10) -
        times.begin() - 1;
    const double span = times[last] - times[first];
    sample.completeWindow = firstTime >= times.front() - 1e-10 &&
                            lastTime <= times.back() + 1e-10 && span > 0;
    for (size_t point = 0; point < points.size(); ++point) {
      const double speed =
          span > 0
              ? (worldPoints[last][point] - worldPoints[first][point]).norm() /
                    span
              : 0;
      double excursion = 0;
      for (size_t j = first; j <= last; ++j)
        excursion = std::max(
            excursion, (worldPoints[j][point] - worldPoints[i][point]).norm());
      sample.pointSpeeds.push_back(speed);
      sample.pointExcursions.push_back(excursion);
      sample.maxSpeed = std::max(sample.maxSpeed, speed);
      sample.maxExcursion = std::max(sample.maxExcursion, excursion);
    }
    sample.angularSpeed =
        span > 0 ? gtsam::Rot3::Logmap(worldTfoot[first].rotation().between(
                                           worldTfoot[last].rotation()))
                           .norm() /
                       span
                 : 0;
    if (!sample.completeWindow) continue;
    const double motion =
        std::max((sample.maxSpeed - params.stationarySpeed) /
                     (params.movingSpeed - params.stationarySpeed),
                 (sample.maxExcursion - params.stationaryExcursion) /
                     (params.movingExcursion - params.stationaryExcursion));
    sample.score = 1 - std::clamp(motion, 0.0, 1.0);
    sample.label = motion <= 0 ? 1 : motion >= 1 ? 0 : -1;
  }
  return result;
}

std::vector<ContactMotionSample> contactPoseMotionLabels(
    const std::vector<double>& times,
    const std::vector<gtsam::Pose3>& worldTcontact,
    const ContactPoseMotionParams& params) {
  const auto validPair = [](double low, double high) {
    return std::isfinite(low) && std::isfinite(high) && low > 0 && high > low;
  };
  if (!validPair(params.stationaryAngularSpeed, params.movingAngularSpeed) ||
      !validPair(params.stationaryAngularExcursion,
                 params.movingAngularExcursion))
    throw std::invalid_argument(
        "contactPoseMotionLabels: invalid angular thresholds");
  ContactMotionParams translation;
  translation.windowSeconds = params.windowSeconds;
  translation.stationarySpeed = params.stationarySpeed;
  translation.movingSpeed = params.movingSpeed;
  translation.stationaryExcursion = params.stationaryExcursion;
  translation.movingExcursion = params.movingExcursion;
  auto result = contactMotionLabels(times, worldTcontact,
                                    {gtsam::Point3::Zero()}, translation);
  for (size_t i = 0; i < times.size(); ++i) {
    auto& sample = result[i];
    const size_t first =
        std::lower_bound(times.begin(), times.end(),
                         times[i] - params.windowSeconds / 2 - 1e-10) -
        times.begin();
    const size_t last =
        std::upper_bound(times.begin(), times.end(),
                         times[i] + params.windowSeconds / 2 + 1e-10) -
        times.begin() - 1;
    for (size_t j = first; j <= last; ++j)
      sample.angularExcursion = std::max(
          sample.angularExcursion,
          gtsam::Rot3::Logmap(
              worldTcontact[i].rotation().between(worldTcontact[j].rotation()))
              .norm());
    if (!sample.completeWindow) continue;
    const double motion = std::max(
        {(sample.maxSpeed - params.stationarySpeed) /
             (params.movingSpeed - params.stationarySpeed),
         (sample.maxExcursion - params.stationaryExcursion) /
             (params.movingExcursion - params.stationaryExcursion),
         (sample.angularSpeed - params.stationaryAngularSpeed) /
             (params.movingAngularSpeed - params.stationaryAngularSpeed),
         (sample.angularExcursion - params.stationaryAngularExcursion) /
             (params.movingAngularExcursion -
              params.stationaryAngularExcursion)});
    sample.score = 1 - std::clamp(motion, 0.0, 1.0);
    sample.label = motion <= 0 ? 1 : motion >= 1 ? 0 : -1;
  }
  return result;
}
std::vector<int> walkingContactLabels(
    const std::vector<ContactMotionSample>& foot,
    const std::vector<ContactMotionSample>& otherFoot,
    const ContactPoseMotionParams& params) {
  if (foot.empty() || foot.size() != otherFoot.size() ||
      !std::isfinite(params.movingSpeed) || params.movingSpeed <= 0 ||
      !std::isfinite(params.movingExcursion) || params.movingExcursion <= 0)
    throw std::invalid_argument(
        "walkingContactLabels: invalid inputs or bounds");
  const auto valid = [](const ContactMotionSample& sample) {
    return sample.label >= -1 && sample.label <= 1 &&
           std::isfinite(sample.maxSpeed) && sample.maxSpeed >= 0 &&
           std::isfinite(sample.maxExcursion) && sample.maxExcursion >= 0;
  };
  const auto translating = [&params](const ContactMotionSample& sample) {
    return sample.maxSpeed >= params.movingSpeed ||
           sample.maxExcursion >= params.movingExcursion;
  };
  std::vector<int> result;
  result.reserve(foot.size());
  for (size_t i = 0; i < foot.size(); ++i) {
    const auto& sample = foot[i];
    const auto& other = otherFoot[i];
    if (!valid(sample) || !valid(other))
      throw std::invalid_argument("walkingContactLabels: invalid raw sample");
    const bool inferred = sample.completeWindow && other.completeWindow &&
                          sample.label != 1 && !translating(sample) &&
                          translating(other);
    result.push_back(inferred ? 2 : sample.label);
  }
  return result;
}
gtsam::DiscreteFactorGraph contactSupportGraph(
    const std::vector<double>& leftDisplacements,
    const std::vector<double>& rightDisplacements,
    const ContactSupportParams& params) {
  const size_t count = leftDisplacements.size();
  if (count < 2 || count != rightDisplacements.size() ||
      !std::isfinite(params.supportSigma) || params.supportSigma <= 0 ||
      !std::isfinite(params.swingSigma) ||
      params.swingSigma <= params.supportSigma ||
      !std::isfinite(params.noSupportWeight) || params.noSupportWeight <= 0 ||
      params.noSupportWeight > 1 || !std::isfinite(params.singleSliceWeight) ||
      params.singleSliceWeight <= 0 || params.singleSliceWeight > 1 ||
      !params.transition.allFinite() ||
      (params.transition.array() <= 0).any() ||
      (params.transition.rowwise().sum().array() - 1).abs().maxCoeff() > 1e-9)
    throw std::invalid_argument(
        "contactSupportGraph: invalid inputs or params");
  for (const auto* distances : {&leftDisplacements, &rightDisplacements})
    for (double distance : *distances)
      if (!std::isfinite(distance) || distance < 0)
        throw std::invalid_argument(
            "contactSupportGraph: invalid displacement");
  const auto logLikelihood = [](double displacement, double sigma) {
    const double ratio = std::min(displacement / sigma, 1e6);
    return -std::log(sigma) - .5 * ratio * ratio;
  };
  std::vector<double> transition;
  for (size_t previous = 0; previous < 4; ++previous)
    for (size_t next = 0; next < 4; ++next)
      transition.push_back(params.transition(previous, next));
  gtsam::DiscreteFactorGraph graph;
  graph.add(gtsam::DiscreteKeys{{0, 2}, {1, 2}}, std::vector<double>(4, .25));
  for (size_t k = 0; k < count; ++k)
    graph.add(gtsam::DiscreteKeys{{2 * k, 2}, {2 * k + 1, 2}},
              std::vector<double>{params.noSupportWeight, 1, 1, 1});
  for (size_t k = 0; k + 1 < count; ++k) {
    graph.add(
        gtsam::DiscreteKeys{
            {2 * k, 2}, {2 * k + 1, 2}, {2 * k + 2, 2}, {2 * k + 3, 2}},
        transition);
    const double previousWeight = k == 0 ? 1 : .5;
    const double nextWeight = k + 2 == count ? 1 : .5;
    for (size_t foot = 0; foot < 2; ++foot) {
      const auto& distances =
          foot == 0 ? leftDisplacements : rightDisplacements;
      std::vector<double> measurement;
      for (size_t previous = 0; previous < 2; ++previous)
        for (size_t next = 0; next < 2; ++next)
          measurement.push_back(
              previousWeight *
                  logLikelihood(distances[k], previous ? params.supportSigma
                                                       : params.swingSigma) +
              nextWeight *
                  logLikelihood(distances[k + 1], next ? params.supportSigma
                                                       : params.swingSigma));
      const double maximum =
          *std::max_element(measurement.begin(), measurement.end());
      for (double& value : measurement)
        value = std::exp(std::max(value - maximum, -700.0));
      graph.add(gtsam::DiscreteKeys{{2 * k + foot, 2}, {2 * (k + 1) + foot, 2}},
                measurement);
    }
  }
  // Row-major W/S triples: WWW, WWS, WSW, WSS, SWW, SWS, SSW, SSS.
  const std::vector<double> duration{
      1, 1, params.singleSliceWeight, 1, 1, params.singleSliceWeight, 1, 1};
  for (size_t k = 1; k + 1 < count; ++k)
    for (size_t foot = 0; foot < 2; ++foot)
      graph.add(gtsam::DiscreteKeys{{2 * (k - 1) + foot, 2},
                                    {2 * k + foot, 2},
                                    {2 * (k + 1) + foot, 2}},
                duration);

  return graph;
}
gtsam::DiscreteFactorGraph contactWalkingConstraints(size_t sliceCount) {
  if (sliceCount == 0)
    throw std::invalid_argument("contactWalkingConstraints: no slices");
  gtsam::DiscreteFactorGraph graph;
  for (size_t k = 0; k < sliceCount; ++k)
    graph.add(gtsam::DiscreteKeys{{2 * k, 2}, {2 * k + 1, 2}},
              std::vector<double>{0, 1, 1, 1});
  // Joint configurations WW, WS, SW, SS: only WS<->SW is forbidden here.
  // The per-slice factors above independently exclude WW.
  std::vector<double> exchange(16, 1);
  exchange[4 * 1 + 2] = 0;
  exchange[4 * 2 + 1] = 0;
  for (size_t k = 0; k + 1 < sliceCount; ++k)
    graph.add(
        gtsam::DiscreteKeys{
            {2 * k, 2}, {2 * k + 1, 2}, {2 * k + 2, 2}, {2 * k + 3, 2}},
        exchange);
  return graph;
}

gtsam::DiscreteFactorGraph contactVelocityFactors(
    const std::vector<double>& leftForwardVelocities,
    const std::vector<double>& rightForwardVelocities, double velocityScale,
    double minimumProbability) {
  if (leftForwardVelocities.empty() ||
      leftForwardVelocities.size() != rightForwardVelocities.size() ||
      !std::isfinite(velocityScale) || velocityScale <= 0 ||
      !std::isfinite(minimumProbability) || minimumProbability <= 0 ||
      minimumProbability >= .5)
    throw std::invalid_argument(
        "contactVelocityFactors: invalid inputs or params");
  const auto support = [velocityScale, minimumProbability](double velocity) {
    if (!std::isfinite(velocity))
      throw std::invalid_argument("contactVelocityFactors: nonfinite velocity");
    const double exponential = std::exp(-std::abs(velocity) / velocityScale);
    const double probability =
        velocity >= 0 ? exponential / (1 + exponential) : 1 / (1 + exponential);
    return std::clamp(probability, minimumProbability, 1 - minimumProbability);
  };
  gtsam::DiscreteFactorGraph graph;
  for (size_t i = 0; i < leftForwardVelocities.size(); ++i) {
    const double left = support(leftForwardVelocities[i]);
    const double right = support(rightForwardVelocities[i]);
    graph.add(gtsam::DiscreteKeys{{2 * i, 2}},
              std::vector<double>{1 - left, left});
    graph.add(gtsam::DiscreteKeys{{2 * i + 1, 2}},
              std::vector<double>{1 - right, right});
  }
  return graph;
}
}  // namespace gtdynamics
