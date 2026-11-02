/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#include <gtdynamics/factors/GPLinearPriorFactor.h>
#include <gtdynamics/gpmp2/GPLinearInterpolator.h>
#include <gtdynamics/utils/FloatingBaseInterpolator.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/WnoaFactor.h>
#include <gtsam/nonlinear/WnoaInterpolator.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>

namespace gtdynamics {
namespace {
using namespace gtsam;

// Substitute zero increments for fixed configurations in an existing GP
// factor. This retains its exact whitening and avoids enormous artificial
// configuration priors or a nonlinear optimization over known poses.
void addVelocityPrior(const NonlinearFactor& factor, const Values& initial,
                      Key firstVelocity, Key secondVelocity,
                      GaussianFactorGraph* graph) {
  JacobianFactor linear(*factor.linearize(initial));
  const auto [matrix, rhs] = linear.jacobian();
  std::vector<std::pair<Key, Matrix>> terms;
  size_t column = 0;
  for (Key key : linear.keys()) {
    const size_t width = linear.getA(key).cols();
    if (key == firstVelocity || key == secondVelocity)
      terms.emplace_back(key, matrix.middleCols(column, width));
    column += width;
  }
  graph->emplace_shared<JacobianFactor>(terms, rhs);
}

Vector6 basePsd(const Vector6& psd) {
  if (!psd.allFinite() || (psd.array() <= 0).any())
    throw std::invalid_argument(
        "Base process spectral densities must be positive");
  return psd;
}
}  // namespace

FloatingBaseInterpolator::FloatingBaseInterpolator(
    const Robot& robot, const std::string& baseLink,
    const gtsam::Vector6& baseProcessSpectralDensity,
    const gtsam::Vector& jointProcessSpectralDensity,
    const gtsam::Vector& jointMeasurementSigmas)
    : robot_(robot),
      baseLink_(baseLink),
      joints_(robot.orderedJoints()),
      basePsd_(basePsd(baseProcessSpectralDensity)),
      jointPsd_(jointProcessSpectralDensity),
      jointSigmas_(jointMeasurementSigmas) {
  const auto root = robot_.link(baseLink_);
  for (const auto& link : robot_.links())
    if (link->isFixed())
      throw std::invalid_argument(
          "Interpolator requires a floating-base robot");
  if (jointPsd_.size() != joints_.size() ||
      jointSigmas_.size() != joints_.size() || !jointPsd_.allFinite() ||
      !jointSigmas_.allFinite() || (jointPsd_.array() <= 0).any() ||
      (jointSigmas_.array() <= 0).any())
    throw std::invalid_argument(
        "Joint noise must match the robot and be positive");
  // Count alone does not exclude a disconnected cycle plus an isolated root.
  if (robot_.links().size() != joints_.size() + 1)
    throw std::invalid_argument("Interpolator requires a connected robot tree");
  std::set<const Link*> reachable{root.get()};
  size_t previousSize = 0;
  while (previousSize != reachable.size()) {
    previousSize = reachable.size();
    for (const auto& joint : joints_) {
      const auto parent = joint->parent().get(), child = joint->child().get();
      if (reachable.count(parent)) reachable.insert(child);
      if (reachable.count(child)) reachable.insert(parent);
    }
  }
  if (reachable.size() != robot_.links().size())
    throw std::invalid_argument("Interpolator requires a connected robot tree");
}

void FloatingBaseInterpolator::checkAngles(const gtsam::Vector& angles) const {
  if (angles.size() != joints_.size() || !angles.allFinite())
    throw std::invalid_argument("Joint coordinates must match jointNames()");
}

void FloatingBaseInterpolator::addKnot(double time,
                                       const gtsam::Pose3& basePose,
                                       const gtsam::Vector& jointAngles) {
  checkAngles(jointAngles);
  if (!std::isfinite(time) || (!times_.empty() && time <= times_.back()) ||
      !basePose.matrix().allFinite())
    throw std::invalid_argument(
        "Knots require finite increasing times and poses");
  times_.push_back(time);
  poses_.push_back(basePose);
  angles_.push_back(jointAngles);
  solved_ = false;
}

void FloatingBaseInterpolator::addJointMeasurement(
    double time, const gtsam::Vector& jointAngles) {
  if (joints_.empty())
    throw std::invalid_argument("This robot has no joints to observe");
  checkAngles(jointAngles);
  interval(time);
  jointMeasurements_.push_back({time, jointAngles});
  solved_ = false;
}

void FloatingBaseInterpolator::addBaseVelocityMeasurement(
    double time, const gtsam::Vector3& worldLinearVelocity, double sigma) {
  if (!std::binary_search(times_.begin(), times_.end(), time) ||
      !worldLinearVelocity.allFinite() || !std::isfinite(sigma) || sigma <= 0)
    throw std::invalid_argument(
        "Base velocity needs an existing knot and sigma>0");
  velocityMeasurements_.push_back({time, worldLinearVelocity, sigma});
  solved_ = false;
}

size_t FloatingBaseInterpolator::interval(double time) const {
  if (times_.size() < 2 || !std::isfinite(time) || time < times_.front() ||
      time > times_.back())
    throw std::out_of_range(
        "Query must lie inside at least two existing knots");
  const auto upper = std::upper_bound(times_.begin(), times_.end(), time);
  return std::min(static_cast<size_t>(upper - times_.begin() - 1),
                  times_.size() - 2);
}

void FloatingBaseInterpolator::solve() {
  using namespace gtsam;
  const auto start = std::chrono::steady_clock::now();
  if (times_.size() < 2)
    throw std::invalid_argument("At least two knots are required");
  solved_ = false;
  Values initial;
  GaussianFactorGraph graph;
  const gtsam::SharedNoiseModel jointQc =
      joints_.empty() ? nullptr : noiseModel::Diagonal::Variances(jointPsd_);
  for (size_t i = 0; i < times_.size(); ++i) {
    initial.insert(Symbol('x', i), poses_[i]);
    initial.insert(Symbol('v', i), Vector6::Zero().eval());
    initial.insert(Symbol('q', i), angles_[i]);
    initial.insert(Symbol('r', i), Vector::Zero(joints_.size()).eval());
    if (!i) continue;
    const double dt = times_[i] - times_[i - 1];
    addVelocityPrior(
        WnoaMotionFactor<Pose3>(Symbol('x', i - 1), Symbol('v', i - 1),
                                Symbol('x', i), Symbol('v', i), dt, basePsd_),
        initial, Symbol('v', i - 1), Symbol('v', i), &graph);
    if (!joints_.empty())
      addVelocityPrior(
          GPLinearPrior(Symbol('q', i - 1), Symbol('r', i - 1), Symbol('q', i),
                        Symbol('r', i), dt, jointQc),
          initial, Symbol('r', i - 1), Symbol('r', i), &graph);
  }
  for (const auto& measurement : jointMeasurements_) {
    const size_t i = interval(measurement.time);
    const double dt = times_[i + 1] - times_[i];
    const double tau = measurement.time - times_[i];
    // Knot observations cannot change fixed configurations or velocities.
    if (tau == 0 || tau == dt) continue;
    GPLinearInterpolator interpolator(jointQc, dt, tau);
    Matrix firstRate, secondRate;
    const Vector zero = Vector::Zero(joints_.size());
    const Vector offset =
        interpolator.interpolatePose(angles_[i], zero, angles_[i + 1], zero,
                                     nullptr, &firstRate, nullptr, &secondRate);
    // WNOA conditional position covariance (integrated Brownian bridge).
    // Like WnoaInterpFactor, augment each measurement separately; cross-time
    // bridge correlations are not included in this approximation.
    const double bridgeVariance = std::pow(tau * (dt - tau) / dt, 3) / 3;
    const Vector variances =
        jointSigmas_.array().square().matrix() + bridgeVariance * jointPsd_;
    graph.emplace_shared<JacobianFactor>(
        Symbol('r', i), firstRate, Symbol('r', i + 1), secondRate,
        measurement.angles - offset,
        noiseModel::Diagonal::Variances(variances));
  }
  for (const auto& measurement : velocityMeasurements_) {
    const size_t i =
        std::lower_bound(times_.begin(), times_.end(), measurement.time) -
        times_.begin();
    Matrix36 selector = Matrix36::Zero();
    selector.rightCols<3>() = poses_[i].rotation().matrix();
    graph.emplace_shared<JacobianFactor>(
        Symbol('v', i), selector, measurement.velocity,
        noiseModel::Isotropic::Sigma(3, measurement.sigma));
  }
  const VectorValues solution = graph.optimize();
  twists_.clear();
  rates_.clear();
  for (size_t i = 0; i < times_.size(); ++i) {
    twists_.push_back(solution.at(Symbol('v', i)));
    rates_.push_back(joints_.empty() ? Vector() : solution.at(Symbol('r', i)));
    initial.update(Symbol('v', i), twists_.back());
    initial.update(Symbol('r', i), rates_.back());
  }
  processError_ = 0;
  for (size_t i = 1; i < times_.size(); ++i) {
    const double dt = times_[i] - times_[i - 1];
    processError_ +=
        WnoaMotionFactor<Pose3>(Symbol('x', i - 1), Symbol('v', i - 1),
                                Symbol('x', i), Symbol('v', i), dt, basePsd_)
            .error(initial);
    if (!joints_.empty())
      processError_ +=
          GPLinearPrior(Symbol('q', i - 1), Symbol('r', i - 1), Symbol('q', i),
                        Symbol('r', i), dt, jointQc)
              .error(initial);
  }
  solved_ = true;
  jointRms_ = 0;
  for (const auto& measurement : jointMeasurements_)
    jointRms_ +=
        (interpolate(measurement.time).jointAngles - measurement.angles)
            .squaredNorm();
  if (!jointMeasurements_.empty())
    jointRms_ =
        std::sqrt(jointRms_ / (jointMeasurements_.size() * joints_.size()));
  solveSeconds_ =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
}

FloatingBaseState FloatingBaseInterpolator::interpolate(double time) const {
  if (!solved_) throw std::logic_error("Call solve() after adding inputs");
  const size_t i = interval(time);
  if (time == times_[i]) return {poses_[i], twists_[i], angles_[i], rates_[i]};
  if (time == times_[i + 1])
    return {poses_[i + 1], twists_[i + 1], angles_[i + 1], rates_[i + 1]};
  const double dt = times_[i + 1] - times_[i], tau = time - times_[i];
  // GTSAM transports endpoint twists into the local pose chart and maps the
  // interpolated derivative back using the SE(3) right Jacobian. A raw linear
  // interpolation of twist coordinates would not differentiate the pose mean.
  gtsam::Interpolator<gtsam::Pose3> base(basePsd_);
  const auto baseState = base.interpolatePoseAndVelocity(
      gtsam::TimestampedPoseVelocity<gtsam::Pose3>(poses_[i], twists_[i],
                                                   times_[i]),
      gtsam::TimestampedPoseVelocity<gtsam::Pose3>(
          poses_[i + 1], twists_[i + 1], times_[i + 1]),
      time);
  if (joints_.empty()) return {baseState.pose, baseState.vel, {}, {}};
  GPLinearInterpolator joints(noiseModel::Diagonal::Variances(jointPsd_), dt,
                              tau);
  return {baseState.pose, baseState.vel,
          joints.interpolatePose(angles_[i], rates_[i], angles_[i + 1],
                                 rates_[i + 1]),
          joints.interpolateVelocity(angles_[i], rates_[i], angles_[i + 1],
                                     rates_[i + 1])};
}

gtsam::Values FloatingBaseInterpolator::values(double time, size_t k) const {
  const auto state = interpolate(time);
  gtsam::Values known;
  const auto root = robot_.link(baseLink_);
  InsertPose(&known, root->id(), k, state.basePose);
  InsertTwist(&known, root->id(), k, state.baseTwist);
  for (size_t j = 0; j < joints_.size(); ++j) {
    InsertJointAngle(&known, joints_[j]->id(), k, state.jointAngles[j]);
    InsertJointVel(&known, joints_[j]->id(), k, state.jointVelocities[j]);
  }
  return robot_.forwardKinematics(known, k, baseLink_);
}

std::vector<std::string> FloatingBaseInterpolator::jointNames() const {
  std::vector<std::string> result;
  for (const auto& joint : joints_) result.push_back(joint->name());
  return result;
}

std::vector<size_t> FloatingBaseInterpolator::jointIds() const {
  std::vector<size_t> result;
  for (const auto& joint : joints_) result.push_back(joint->id());
  return result;
}

double FloatingBaseInterpolator::jointMeasurementRms() const {
  if (!solved_)
    throw std::logic_error("Call solve() before requesting diagnostics");
  return jointRms_;
}

double FloatingBaseInterpolator::processError() const {
  if (!solved_)
    throw std::logic_error("Call solve() before requesting diagnostics");
  return processError_;
}
}  // namespace gtdynamics
