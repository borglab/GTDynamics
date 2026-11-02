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

double elapsedSeconds(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}

Vector6 basePsd(const Vector6& psd) {
  if (!psd.allFinite() || (psd.array() <= 0).any())
    throw std::invalid_argument(
        "Base process spectral densities must be positive");
  return psd;
}
size_t interval(const std::vector<double>& times, double time) {
  if (times.size() < 2 || !std::isfinite(time) || time < times.front() ||
      time > times.back())
    throw std::out_of_range(
        "Query must lie inside at least two existing knots");
  const auto upper = std::upper_bound(times.begin(), times.end(), time);
  return std::min(static_cast<size_t>(upper - times.begin() - 1),
                  times.size() - 2);
}

}  // namespace

FloatingBaseInterpolator::FloatingBaseInterpolator(
    const Robot& robot, const std::string& baseLink,
    const gtsam::Vector6& baseProcessSpectralDensity,
    const gtsam::Vector& jointProcessSpectralDensity,
    const gtsam::Vector& jointMeasurementSigmas)
    : robot_(std::make_shared<Robot>(robot)),
      baseLink_(baseLink),
      basePsd_(basePsd(baseProcessSpectralDensity)),
      jointPsd_(jointProcessSpectralDensity),
      jointSigmas_(jointMeasurementSigmas) {
  const auto root = robot_->link(baseLink_);
  const auto links = robot_->links();
  const auto joints = robot_->orderedJoints();
  for (const auto& link : links)
    if (link->isFixed())
      throw std::invalid_argument(
          "Interpolator requires a floating-base robot");
  if (jointPsd_.size() != joints.size() ||
      jointSigmas_.size() != joints.size() || !jointPsd_.allFinite() ||
      !jointSigmas_.allFinite() || (jointPsd_.array() <= 0).any() ||
      (jointSigmas_.array() <= 0).any())
    throw std::invalid_argument(
        "Joint noise must match the robot and be positive");
  // Count alone does not exclude a disconnected cycle plus an isolated root.
  if (links.size() != joints.size() + 1)
    throw std::invalid_argument("Interpolator requires a connected robot tree");
  std::set<const Link*> reachable{root.get()};
  size_t previousSize = 0;
  while (previousSize != reachable.size()) {
    previousSize = reachable.size();
    for (const auto& joint : joints) {
      const auto parent = joint->parent().get(), child = joint->child().get();
      if (reachable.count(parent)) reachable.insert(child);
      if (reachable.count(child)) reachable.insert(parent);
    }
  }
  if (reachable.size() != links.size())
    throw std::invalid_argument("Interpolator requires a connected robot tree");
}

void FloatingBaseInterpolator::checkAngles(const gtsam::Vector& angles) const {
  if (angles.size() != robot_->numJoints() || !angles.allFinite())
    throw std::invalid_argument(
        "Joint coordinates must match robot.orderedJoints()");
}

void FloatingBaseInterpolator::addKnot(double time,
                                       const gtsam::Pose3& basePose,
                                       const gtsam::Vector& jointAngles) {
  using namespace gtsam;
  const auto start = std::chrono::steady_clock::now();
  checkAngles(jointAngles);
  if (!std::isfinite(time) || (!times_.empty() && time <= times_.back()) ||
      !basePose.matrix().allFinite())
    throw std::invalid_argument(
        "Knots require finite increasing times and poses");
  const size_t i = times_.size();
  if (i != 0) {
    const size_t numJoints = robot_->numJoints();
    // Only the new interval is needed: fixed configurations are substituted
    // now, leaving binary factors in the two endpoint velocities.
    Values initial;
    initial.insert(Symbol('x', i - 1), poses_.back());
    initial.insert(Symbol('x', i), basePose);
    initial.insert(Symbol('v', i - 1), Vector6::Zero().eval());
    initial.insert(Symbol('v', i), Vector6::Zero().eval());
    const double dt = time - times_.back();
    processFactorIndices_.push_back(graph_.size());
    addVelocityPrior(
        WnoaMotionFactor<Pose3>(Symbol('x', i - 1), Symbol('v', i - 1),
                                Symbol('x', i), Symbol('v', i), dt, basePsd_),
        initial, Symbol('v', i - 1), Symbol('v', i), &graph_);
    if (numJoints != 0) {
      initial.insert(Symbol('q', i - 1), angles_.back());
      initial.insert(Symbol('q', i), jointAngles);
      initial.insert(Symbol('r', i - 1), Vector::Zero(numJoints).eval());
      initial.insert(Symbol('r', i), Vector::Zero(numJoints).eval());
      processFactorIndices_.push_back(graph_.size());
      addVelocityPrior(
          GPLinearPrior(Symbol('q', i - 1), Symbol('r', i - 1), Symbol('q', i),
                        Symbol('r', i), dt,
                        noiseModel::Diagonal::Variances(jointPsd_)),
          initial, Symbol('r', i - 1), Symbol('r', i), &graph_);
    }
  }
  times_.push_back(time);
  poses_.push_back(basePose);
  angles_.push_back(jointAngles);
  graphConstructionSeconds_ += elapsedSeconds(start);
}

void FloatingBaseInterpolator::addJointMeasurement(
    double time, const gtsam::Vector& jointAngles) {
  using namespace gtsam;
  const auto start = std::chrono::steady_clock::now();
  if (robot_->numJoints() == 0)
    throw std::invalid_argument("This robot has no joints to observe");
  checkAngles(jointAngles);
  const size_t i = interval(times_, time);
  const double dt = times_[i + 1] - times_[i], tau = time - times_[i];
  if (tau == 0 || tau == dt) {
    // Exact-knot observations carry no velocity information, but their fixed
    // residuals must still contribute to the physical-unit encoder RMS.
    fixedJointSquaredError_ +=
        (jointAngles - angles_[tau == 0 ? i : i + 1]).squaredNorm();
  } else {
    GPLinearInterpolator interpolator(
        noiseModel::Diagonal::Variances(jointPsd_), dt, tau);
    Matrix firstRate, secondRate;
    const Vector zero = Vector::Zero(robot_->numJoints());
    const Vector offset =
        interpolator.interpolatePose(angles_[i], zero, angles_[i + 1], zero,
                                     nullptr, &firstRate, nullptr, &secondRate);
    // WNOA conditional position covariance (integrated Brownian bridge).
    // As in WnoaInterpFactor, each measurement is augmented separately;
    // cross-time bridge correlations are not included in this approximation.
    const double bridgeVariance = std::pow(tau * (dt - tau) / dt, 3) / 3;
    const Vector variances =
        jointSigmas_.array().square().matrix() + bridgeVariance * jointPsd_;
    jointFactorIndices_.push_back(graph_.size());
    graph_.emplace_shared<JacobianFactor>(
        Symbol('r', i), firstRate, Symbol('r', i + 1), secondRate,
        jointAngles - offset, noiseModel::Diagonal::Variances(variances));
  }
  ++jointMeasurementCount_;
  graphConstructionSeconds_ += elapsedSeconds(start);
}

void FloatingBaseInterpolator::addBaseVelocityMeasurement(
    double time, const gtsam::Vector3& worldLinearVelocity, double sigma) {
  using namespace gtsam;
  const auto start = std::chrono::steady_clock::now();
  if (!std::binary_search(times_.begin(), times_.end(), time) ||
      !worldLinearVelocity.allFinite() || !std::isfinite(sigma) || sigma <= 0)
    throw std::invalid_argument(
        "Base velocity needs an existing knot and sigma>0");
  const size_t i =
      std::lower_bound(times_.begin(), times_.end(), time) - times_.begin();
  Matrix36 selector = Matrix36::Zero();
  selector.rightCols<3>() = poses_[i].rotation().matrix();
  graph_.emplace_shared<JacobianFactor>(Symbol('v', i), selector,
                                        worldLinearVelocity,
                                        noiseModel::Isotropic::Sigma(3, sigma));
  graphConstructionSeconds_ += elapsedSeconds(start);
}

FloatingBaseInterpolation FloatingBaseInterpolator::solve() const {
  using namespace gtsam;
  const auto start = std::chrono::steady_clock::now();
  if (times_.size() < 2)
    throw std::invalid_argument("At least two knots are required");
  const size_t numJoints = robot_->numJoints();
  const VectorValues solution = graph_.optimize();
  FloatingBaseInterpolation result(*this);
  result.twists_.reserve(times_.size());
  result.rates_.reserve(times_.size());
  for (size_t i = 0; i < times_.size(); ++i) {
    result.twists_.push_back(solution.at(Symbol('v', i)));
    result.rates_.push_back(numJoints == 0 ? Vector()
                                           : solution.at(Symbol('r', i)));
  }
  // With configurations fixed, the stored Gaussian process factors are exact
  // affine residuals, so they also supply the original GP objective values.
  for (size_t index : processFactorIndices_)
    result.processError_ += graph_[index]->error(solution);
  double jointSquaredError = fixedJointSquaredError_;
  for (size_t index : jointFactorIndices_)
    jointSquaredError += graph_.at<JacobianFactor>(index)
                             ->unweighted_error(solution)
                             .squaredNorm();
  if (jointMeasurementCount_ != 0)
    result.jointRms_ =
        std::sqrt(jointSquaredError / (jointMeasurementCount_ * numJoints));
  result.solveSeconds_ = elapsedSeconds(start);
  return result;
}

FloatingBaseInterpolation::FloatingBaseInterpolation(
    const FloatingBaseInterpolator& problem)
    : robot_(problem.robot_),
      baseLink_(problem.baseLink_),
      basePsd_(problem.basePsd_),
      jointPsd_(problem.jointPsd_),
      times_(problem.times_),
      poses_(problem.poses_),
      angles_(problem.angles_),
      graphConstructionSeconds_(problem.graphConstructionSeconds_) {}

FloatingBaseState FloatingBaseInterpolation::interpolate(double time) const {
  const size_t i = interval(times_, time);
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
  if (jointPsd_.size() == 0) return {baseState.pose, baseState.vel, {}, {}};
  GPLinearInterpolator joints(noiseModel::Diagonal::Variances(jointPsd_), dt,
                              tau);
  return {baseState.pose, baseState.vel,
          joints.interpolatePose(angles_[i], rates_[i], angles_[i + 1],
                                 rates_[i + 1]),
          joints.interpolateVelocity(angles_[i], rates_[i], angles_[i + 1],
                                     rates_[i + 1])};
}

gtsam::Values FloatingBaseInterpolation::values(double time, size_t k) const {
  const auto state = interpolate(time);
  gtsam::Values known;
  const auto root = robot_->link(baseLink_);
  InsertPose(&known, root->id(), k, state.basePose);
  InsertTwist(&known, root->id(), k, state.baseTwist);
  const auto joints = robot_->orderedJoints();
  for (size_t j = 0; j < joints.size(); ++j) {
    InsertJointAngle(&known, joints[j]->id(), k, state.jointAngles[j]);
    InsertJointVel(&known, joints[j]->id(), k, state.jointVelocities[j]);
  }
  return robot_->forwardKinematics(known, k, baseLink_);
}

std::vector<std::string> FloatingBaseInterpolation::jointNames() const {
  std::vector<std::string> result;
  for (const auto& joint : robot_->orderedJoints())
    result.push_back(joint->name());
  return result;
}

std::vector<size_t> FloatingBaseInterpolation::jointIds() const {
  std::vector<size_t> result;
  for (const auto& joint : robot_->orderedJoints())
    result.push_back(joint->id());
  return result;
}

}  // namespace gtdynamics
