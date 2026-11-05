/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file ContactInterpolationInitializer.cpp
 * @brief Fit contact poses and GP velocities without dynamics dependencies.
 * @author GTDynamics contributors
 */
#include <gtdynamics/kinematics/ContactInterpolationInitializer.h>
#include <gtdynamics/utils/values.h>

#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
namespace gtdynamics {
namespace {
using Clock = std::chrono::steady_clock;
double seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}
}  // namespace

ContactInterpolationInitializer::ContactInterpolationInitializer(
    const Robot& robot, const ContactInterpolationOptions& options)
    : robot_(robot), options_(options) {}

ContactInterpolationResult ContactInterpolationInitializer::run(
    const std::string& rootLinkName, const gtsam::Pose3& initialPose,
    const gtsam::Values& measuredJoints, const Trajectory& trajectory,
    const std::vector<Slice>& slices, const std::vector<double>& times,
    double dt, const std::vector<Slice>& selectedSlices) const {
  if (slices.size() < 2 || slices.size() != times.size())
    throw std::invalid_argument(
        "Contact interpolation requires >=2 source slices and matching times");
  std::map<size_t, double> sourceTimes;
  for (size_t i = 0; i < slices.size(); ++i) {
    if (!std::isfinite(times[i]) ||
        (i && (times[i] <= times[i - 1] || slices[i].k <= slices[i - 1].k)))
      throw std::invalid_argument(
          "Source indices and times must increase strictly");
    sourceTimes.emplace(slices[i].k, times[i]);
  }
  ContactInterpolationResult result;
  auto started = Clock::now();
  auto contactOptions = options_.contact;
  contactOptions.interpolateOutput = true;
  const auto configurations =
      PoseContactInitializer(robot_, contactOptions, options_.contactBuffer)
          .run(rootLinkName, initialPose, measuredJoints, trajectory, dt,
               &result.contact, selectedSlices);
  result.contactSeconds = seconds(started);

  started = Clock::now();
  constexpr double radiansPerDegree = 3.141592653589793 / 180.0;
  const double angular =
      options_.baseAngularProcessSigmaDegrees * radiansPerDegree;
  const double joint = options_.jointProcessSigmaDegrees * radiansPerDegree;
  const double encoder =
      options_.jointMeasurementSigmaDegrees * radiansPerDegree;
  gtsam::Vector6 basePsd;
  basePsd << angular * angular, angular * angular, angular * angular,
      std::pow(options_.baseLinearProcessSigma, 2),
      std::pow(options_.baseLinearProcessSigma, 2),
      std::pow(options_.baseLinearProcessSigma, 2);
  if (angular <= 0 || joint <= 0 || encoder <= 0 ||
      options_.baseLinearProcessSigma <= 0)
    throw std::invalid_argument("GP standard deviations must be positive");
  const auto joints = robot_.orderedJoints();
  auto angles = [&](size_t k) {
    gtsam::Vector q(joints.size());
    for (size_t j = 0; j < joints.size(); ++j)
      q(j) = JointAngle(measuredJoints, joints[j]->id(), k);
    return q;
  };
  FloatingBaseInterpolator problem(
      robot_, rootLinkName, basePsd,
      gtsam::Vector::Constant(joints.size(), joint * joint),
      gtsam::Vector::Constant(joints.size(), encoder));
  std::set<size_t> knots{slices.front().k, slices.back().k};
  for (const auto& slice : result.contact.selectedSlices) {
    if (!sourceTimes.count(slice.k))
      throw std::invalid_argument(
          "Every selected contact slice must be present in the output slices");
    knots.insert(slice.k);
  }
  for (size_t k : knots)
    problem.addKnot(sourceTimes.at(k),
                    Pose(configurations, robot_.link(rootLinkName)->id(), k),
                    angles(k));
  for (size_t i = 0; i < slices.size(); ++i)
    problem.addJointMeasurement(times[i], angles(slices[i].k));
  result.interpolation =
      std::make_shared<FloatingBaseInterpolation>(problem.solve());
  for (size_t i = 0; i < slices.size(); ++i)
    result.values.insert(result.interpolation->values(times[i], slices[i].k));
  result.interpolationSeconds = seconds(started);
  return result;
}
}  // namespace gtdynamics
