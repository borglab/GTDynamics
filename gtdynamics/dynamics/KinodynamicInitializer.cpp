/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file KinodynamicInitializer.cpp
 * @brief Compose kinematic initialization with an affine linear dynamics seed.
 * @author GTDynamics contributors
 */
#include <gtdynamics/dynamics/KinodynamicInitializer.h>
#include <gtdynamics/utils/values.h>

#include <chrono>
#include <cmath>
#include <stdexcept>
namespace gtdynamics {
gtsam::Values gpAccelerationPriors(
    const FloatingBaseInterpolation& interpolation, size_t rootId, double time,
    size_t k, double stepSeconds) {
  const auto knots = interpolation.knotTimes();
  if (!std::isfinite(stepSeconds) || stepSeconds <= 0 || !std::isfinite(time) ||
      time < knots.front() || time > knots.back())
    throw std::invalid_argument("Invalid acceleration differentiation stencil");
  const double before = std::max(knots.front(), time - stepSeconds);
  const double after = std::min(knots.back(), time + stepSeconds);
  if (after <= before)
    throw std::invalid_argument("Acceleration stencil has no duration");
  const auto first = interpolation.interpolate(before);
  const auto last = interpolation.interpolate(after);
  gtsam::Values priors;
  InsertTwistAccel(&priors, rootId, k,
                   (last.baseTwist - first.baseTwist) / (after - before));
  const gtsam::Vector accelerations =
      (last.jointVelocities - first.jointVelocities) / (after - before);
  const auto ids = interpolation.jointIds();
  for (size_t j = 0; j < ids.size(); ++j)
    InsertJointAccel(&priors, ids[j], k, accelerations(j));
  return priors;
}

KinodynamicInitializer::KinodynamicInitializer(
    const Robot& robot, const KinodynamicInitializationOptions& options)
    : robot_(robot), options_(options) {}

KinodynamicInitializationResult KinodynamicInitializer::run(
    const std::string& rootLinkName, const gtsam::Pose3& initialPose,
    const gtsam::Values& measuredJoints, const Trajectory& trajectory,
    const std::vector<Slice>& slices, const std::vector<double>& times,
    double dt, const std::vector<fixed_dynamics::Contact>& contacts,
    const gtsam::Vector& armature,
    const std::vector<Slice>& selectedSlices) const {
  using Clock = std::chrono::steady_clock;
  const auto started = Clock::now();
  KinodynamicInitializationResult result;
  result.kinematics =
      ContactInterpolationInitializer(robot_, options_.kinematics)
          .run(rootLinkName, initialPose, measuredJoints, trajectory, slices,
               times, dt, selectedSlices);
  result.values = result.kinematics.values;
  const auto dynamicsStarted = Clock::now();
  gtsam::Values accelerationPriors;
  const size_t rootId = robot_.link(rootLinkName)->id();
  for (size_t i = 0; i < slices.size(); ++i)
    accelerationPriors.insert(
        gpAccelerationPriors(*result.kinematics.interpolation, rootId, times[i],
                             slices[i].k, options_.accelerationStepSeconds));
  result.dynamics = fixedKinematicsDynamicsSeed(
      robot_, trajectory, slices, times, result.values, accelerationPriors,
      contacts, armature, options_.dynamics);
  result.values.insert(result.dynamics.values);
  for (const auto& slice : slices) {
    const auto pose = Pose(result.values, rootId, slice.k);
    const gtsam::Vector3 worldVelocity =
        pose.rotation() * Twist(result.values, rootId, slice.k).tail<3>();
    result.values.insert(LinearVelocityKey(rootId, slice.k),
                         gtsam::Vector(worldVelocity));
  }
  result.dynamicsSeconds =
      std::chrono::duration<double>(Clock::now() - dynamicsStarted).count();
  result.totalSeconds =
      std::chrono::duration<double>(Clock::now() - started).count();
  return result;
}
}  // namespace gtdynamics
