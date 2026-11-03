/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_online.h
 * @author Harrison Jenkins
 *
 * SCATE online loop (paper Algorithm 1) for the joint-space manipulator graph.
 * Needs only GTSAM and the GTDynamics factors (no robot model): the robot's
 * collision geometry and sensor come in through the CollisionModel interface,
 * so this file can be built and tested with a toy robot. ur5_collision.h
 * implements CollisionModel for the UR5 with the GTDynamics factors
 * ObstacleSDFFactorGP and SelfCollisionSphereFactorGP.
 *
 * Per online step k (the robot has just executed u_{k-1} and reached x_k):
 *   1. remove the executed control's limit and effort factors      (paper: remove outdated factors)
 *   2. add encoder priors on q_k and v_k                            (paper eq. 11)
 *   3. if the obstacle is in sensor range, add a measurement factor
 *      on (q_k, l); the first one also creates the variable l       (paper eq. 12)
 *   4. move obstacle and self-collision factors to the future
 *      intervals only, built from the current estimate of l         (paper eqs. 18-20)
 *   5. re-solve with Levenberg-Marquardt from the previous solution
 *   6. if the solve moved the estimate of l, rebuild the obstacle
 *      factors from it and solve once more
 *
 * Deliberate differences from the paper (same choices as the 2D Python version):
 *   - one landmark variable l for the static obstacle, not one per step;
 *   - obstacle factors use the fused estimate of l as a constant (rebuilt
 *     each step), so the obstacle cost cannot drag l toward the trajectory;
 *   - the executed control's limit is removed (u_{k-1}), not u_k;
 *   - no separate feedback law (eq. 23): replanning every step plays that role;
 *   - the bearing/range measurement is used in Cartesian form: the obstacle
 *     center in the sensor frame (same information for small noise, and its
 *     Jacobian comes straight from Pose3::transformTo);
 *   - an optional weak control-effort prior (the paper has none), so the
 *     avoidance maneuver spreads over the remaining steps.
 */

#pragma once

#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "build_scate_graph.h"
#include "scate_keys.h"

namespace scate {

/// Key of the obstacle location variable l (one static obstacle).
inline gtsam::Key LKey() { return gtsam::Symbol('l', 0); }

// ============================================================
// 1. Interpolation between support states
// ============================================================

/**
 * Coefficients of q(tau) = a q1 + b v1 + c q2 + d v2: the cubic Hermite
 * spline through (q1, v1) at 0 and (q2, v2) at dt. These are the same
 * numbers GTDynamics' GPLinearInterpolator uses (calcInterpCoefficientsAccel
 * in gpmp2/GPUtils.h, first rows of Lambda and Psi). A cubic reproduces any
 * quadratic exactly, so on a constant-acceleration segment (our dynamics)
 * it returns the exact position.
 */
struct HermiteCoefficients {
  double a, b, c, d;
};

/// Hermite position coefficients at time tau (0 <= tau <= dt) into an interval.
inline HermiteCoefficients hermitePositionCoefficients(double dt, double tau) {
  if (!(dt > 0.0) || tau < 0.0 || tau > dt) {
    throw std::invalid_argument("hermitePositionCoefficients: need dt > 0, 0 <= tau <= dt");
  }
  const double s = tau / dt, s2 = s * s, s3 = s2 * s;
  return {1.0 - 3.0 * s2 + 2.0 * s3, dt * (s - 2.0 * s2 + s3),
          3.0 * s2 - 2.0 * s3, dt * (s3 - s2)};
}

/// Joint angles at time tau into the interval from (q1, v1) to (q2, v2).
inline Vector interpolatePosition(const Vector& q1, const Vector& v1, const Vector& q2,
                                  const Vector& v2, double dt, double tau) {
  const HermiteCoefficients h = hermitePositionCoefficients(dt, tau);
  return h.a * q1 + h.b * v1 + h.c * q2 + h.d * v2;
}

// ============================================================
// 2. What the planner needs from the robot
// ============================================================

/**
 * Collision geometry and sensor of a robot. One instance is shared by the
 * planner (which calls setObstacle when its estimate changes) and the
 * simulation (which only uses the const methods with the true obstacle).
 */
class CollisionModel {
 public:
  virtual ~CollisionModel() = default;

  /// Number of joints (length of q).
  virtual size_t dof() const = 0;

  /**
   * World pose of the sensor at joint angles q. If H is non-null it is set
   * to the 6 x dof Jacobian in the gtsam Pose3 tangent convention
   * (body-frame twist, rotation rows first).
   */
  virtual gtsam::Pose3 sensorPose(const Vector& q, gtsam::Matrix* H = nullptr) const = 0;

  /// Install the obstacle (a sphere) that obstacleFactor builds factors for.
  virtual void setObstacle(const gtsam::Point3& center, double radius) = 0;

  /**
   * Obstacle factor on the interval between states i and i+1, at time tau
   * (0 <= tau <= dt) after state i. Keys: QKey(i), VKey(i), QKey(i+1), VKey(i+1).
   */
  virtual gtsam::NonlinearFactor::shared_ptr obstacleFactor(int i, double tau) const = 0;

  /// Self-collision factor on the same interval, or null if the model has none.
  virtual gtsam::NonlinearFactor::shared_ptr selfCollisionFactor(int /*i*/,
                                                                 double /*tau*/) const {
    return gtsam::NonlinearFactor::shared_ptr();
  }

  /**
   * Smallest gap between the robot's collision spheres and the surface of a
   * spherical obstacle (negative = penetration). Used to score executions.
   */
  virtual double obstacleClearance(const Vector& q, const gtsam::Point3& center,
                                   double radius) const = 0;

  /// Smallest gap over the self-collision pairs (negative = penetration).
  virtual double selfClearance(const Vector& /*q*/) const {
    return std::numeric_limits<double>::infinity();
  }

  // ---- Drawing hooks, used by scate_replay.h. Optional: a model without
  // ---- them still plans, and the replay just has less geometry in it.

  /// World positions of the collision-sphere centers, one per column (3 x M).
  virtual gtsam::Matrix sphereCenters(const Vector& /*q*/) const { return gtsam::Matrix(3, 0); }

  /// Radius of each collision sphere (length M).
  virtual Vector sphereRadii() const { return Vector(0); }

  /// Index of the sphere whose path is drawn as "the tool", or -1.
  virtual int toolSphere() const { return -1; }

  /// Sphere index pairs checked for self collision.
  virtual std::vector<std::pair<size_t, size_t>> selfCollisionPairs() const { return {}; }

  /**
   * World poses of the robot's links in their URDF link frames (the frames
   * the URDF meshes are written in), with the link names.
   */
  virtual std::vector<std::pair<std::string, gtsam::Pose3>> linkFrames(const Vector& /*q*/) const {
    return {};
  }
};

// ============================================================
// 3. Obstacle measurement factor (paper eq. 12, Cartesian form)
// ============================================================

/**
 * z = (sensor pose at q)^-1 * l + noise: the obstacle center seen in the
 * sensor frame. Couples the joint angles at the time of the measurement with
 * the obstacle location, so l is estimated jointly with the trajectory.
 */
class ObstacleMeasurementFactor : public gtsam::NoiseModelFactorN<Vector, gtsam::Point3> {
  using Base = gtsam::NoiseModelFactorN<Vector, gtsam::Point3>;
  gtsam::Point3 measured_;
  std::shared_ptr<const CollisionModel> model_;

 public:
  /**
   * @param q_key joint angles at the time of the measurement
   * @param l_key obstacle location
   * @param measured obstacle center in the sensor frame
   * @param model provides the sensor pose and its Jacobian
   * @param noise 3D noise model in the sensor frame
   */
  ObstacleMeasurementFactor(gtsam::Key q_key, gtsam::Key l_key, const gtsam::Point3& measured,
                            std::shared_ptr<const CollisionModel> model,
                            const gtsam::SharedNoiseModel& noise)
      : Base(noise, q_key, l_key), measured_(measured), model_(std::move(model)) {
    if (!model_) throw std::invalid_argument("ObstacleMeasurementFactor: null model");
  }

  /// Predicted minus measured obstacle center, in the sensor frame.
  gtsam::Vector evaluateError(const Vector& q, const gtsam::Point3& l,
                              gtsam::OptionalMatrixType Hq = nullptr,
                              gtsam::OptionalMatrixType Hl = nullptr) const override {
    gtsam::Matrix H_sensor;
    const gtsam::Pose3 wTs = model_->sensorPose(q, Hq ? &H_sensor : nullptr);
    gtsam::Matrix36 H_pose;
    gtsam::Matrix3 H_point;
    const gtsam::Point3 predicted = wTs.transformTo(l, H_pose, H_point);
    if (Hq) *Hq = H_pose * H_sensor;
    if (Hl) *Hl = H_point;
    return predicted - measured_;
  }

  /// Deep copy of this factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new ObstacleMeasurementFactor(*this));
  }

  /// The measured obstacle center in the sensor frame.
  const gtsam::Point3& measured() const { return measured_; }
};

// ============================================================
// 4. Planner
// ============================================================

/// The fractions 0, 1/count, ..., (count-1)/count of an interval, for tau_fractions.
inline std::vector<double> evenlySpacedFractions(int count) {
  if (count < 1) throw std::invalid_argument("evenlySpacedFractions: count must be >= 1");
  std::vector<double> fractions;
  for (int c = 0; c < count; ++c) fractions.push_back(static_cast<double>(c) / count);
  return fractions;
}

/// Settings of the online planner.
struct OnlineConfig {
  // State measurements (paper eq. 11): encoder priors on q_k and v_k.
  double sigma_q_measurement = 1e-3;  ///< rad
  double sigma_v_measurement = 1e-2;  ///< rad/s
  /// Obstacle measurement (paper eq. 12), per axis, in the sensor frame (m).
  double sigma_l_measurement = 1e-2;
  /// Weak prior pulling each control toward zero (rad/s^2); 0 turns it off (paper).
  double sigma_effort = 1.0;
  /**
   * Per-joint sigmas of the effort prior (rad/s^2), overriding sigma_effort
   * when non-empty. A smaller sigma makes that joint's motion more expensive.
   */
  Vector effort_sigmas;
  /**
   * Obstacle and self-collision checks per interval, as fractions of dt in
   * [0, 1). A fraction of 1 would duplicate the next interval's 0; the final
   * state gets its own check at tau = dt on the last interval.
   */
  std::vector<double> tau_fractions = evenlySpacedFractions(3);
  bool self_collision = true;  ///< add self-collision factors
  /// Assumed obstacle radius (m); the paper estimates the location only.
  double obstacle_radius = 0.08;
  /// Rebuild obstacle factors and re-solve if l moved more than this (m).
  double l_rebuild_tolerance = 5e-3;
  int max_iterations = 100;  ///< Levenberg-Marquardt iteration cap
  /**
   * After each step, also compute the marginal covariance of l (for
   * plotting). Not included in the timing; skipped if the graph is singular.
   */
  bool compute_l_covariance = false;
};

/// What one call to planOffline/update did, for logging.
struct StepInfo {
  int k = 0;                   ///< state index the robot is at
  bool detected = false;       ///< an obstacle measurement was added this step
  bool have_obstacle = false;  ///< l exists in the graph
  gtsam::Point3 l_estimate = gtsam::Point3::Zero();  ///< estimate of l after the step
  int solves = 0;              ///< Levenberg-Marquardt solves this step
  double solve_ms = 0.0;       ///< total solve time this step
  double graph_error = 0.0;    ///< graph error after the last solve
  size_t num_factors = 0;      ///< graph size at the last solve
  /// The plan after this step: q and v of states k..N (plan_q[0] is state k).
  std::vector<Vector> plan_q, plan_v;
  /// Marginal covariance of l, if requested and l exists.
  std::optional<gtsam::Matrix3> l_covariance;
  /**
   * Gap between the goal configuration and the estimated obstacle (m), if l
   * exists. Negative: the goal itself is in collision, which no plan can fix
   * (the goal prior holds q_N there).
   */
  std::optional<double> goal_clearance;
};

/**
 * Holds the factor graph by name so factors can be removed and replaced
 * between solves, and the current solution as the warm start.
 */
class OnlinePlanner {
 public:
  /**
   * @param spec start, goal, limits, horizon and base noise of the graph
   * @param config online settings
   * @param model collision geometry and sensor of the robot
   * @param initial initial guess of every q_i, v_i and u_i
   */
  OnlinePlanner(const GraphSpec& spec, const OnlineConfig& config,
                std::shared_ptr<CollisionModel> model, const gtsam::Values& initial)
      : spec_(spec), config_(config), model_(std::move(model)), values_(initial) {
    validateSpec(spec_);
    if (!model_) throw std::invalid_argument("OnlinePlanner: null CollisionModel");
    if (model_->dof() != static_cast<size_t>(spec_.q_start.size())) {
      throw std::invalid_argument("OnlinePlanner: model dof does not match GraphSpec");
    }
    for (double fraction : config_.tau_fractions) {
      if (fraction < 0.0 || fraction >= 1.0) {
        throw std::invalid_argument("OnlinePlanner: tau fractions must be in [0, 1)");
      }
    }
    if (config_.effort_sigmas.size() != 0 &&
        config_.effort_sigmas.size() != spec_.q_start.size()) {
      throw std::invalid_argument("OnlinePlanner: effort_sigmas must be empty or one per joint");
    }
    addBaseFactors();
  }

  /// Number of intervals N (the last state index).
  int numIntervals() const { return spec_.num_poses - 1; }
  /// State index the robot is at.
  int currentIndex() const { return k_; }
  /// The graph specification.
  const GraphSpec& spec() const { return spec_; }
  /// The online settings.
  const OnlineConfig& config() const { return config_; }
  /// The current solution.
  const gtsam::Values& values() const { return values_; }
  /// Planned control u_i in the current solution.
  Vector control(int i) const { return values_.at<Vector>(UKey(i)); }

  /// Whether the obstacle variable l exists yet.
  bool haveObstacle() const { return values_.exists(LKey()); }
  /// Current estimate of the obstacle location l.
  gtsam::Point3 obstacleEstimate() const { return values_.at<gtsam::Point3>(LKey()); }

  /**
   * Paper Algorithm 1, "Generate Initial Plan": the obstacle is unknown, so
   * only self-collision factors are added on top of the base graph.
   */
  StepInfo planOffline() {
    if (k_ != 0 || planned_) throw std::logic_error("planOffline: call once, before update");
    planned_ = true;
    refreshIntervalFactors();
    StepInfo info;
    info.k = 0;
    solve(&info);
    return info;
  }

  /**
   * One online step: the robot has executed u_{k-1} and is now at state k,
   * with encoder readings and, if the obstacle is in range, its center in
   * the sensor frame.
   */
  StepInfo update(int k, const Vector& q_measured, const Vector& v_measured,
                  const std::optional<gtsam::Point3>& obstacle_measured) {
    if (!planned_) throw std::logic_error("update: call planOffline first");
    if (k != k_ + 1 || k > numIntervals()) {
      throw std::invalid_argument("update: expected state index " + std::to_string(k_ + 1));
    }
    k_ = k;

    // 1. The executed control is now history: drop its limit and effort cost.
    remove("control_limit", k - 1);
    remove("effort", k - 1);

    // 2-3. State measurement, then the obstacle measurement if there is one.
    addStateMeasurements(k, q_measured, v_measured);
    StepInfo info;
    info.k = k;
    if (obstacle_measured) {
      info.detected = true;
      addObstacleMeasurement(k, q_measured, *obstacle_measured);
    }

    // 4-5. Future-only collision factors from the current estimate, then solve.
    refreshIntervalFactors();
    solve(&info);

    // 6. If the fused estimate moved, plan against the new one.
    if (haveObstacle() &&
        (obstacleEstimate() - l_planned_).norm() > config_.l_rebuild_tolerance) {
      refreshIntervalFactors();
      solve(&info);
    }
    return info;
  }

  /// The current graph, assembled from the named factors.
  gtsam::NonlinearFactorGraph graph() const {
    gtsam::NonlinearFactorGraph graph;
    for (const auto& name_and_entry : factors_) graph.add(name_and_entry.second.factor);
    return graph;
  }

  /**
   * Number of factors in a group ("dynamics", "obstacle", "measurement_l",
   * ...), optionally only those whose index is below `below`.
   */
  size_t count(const std::string& group,
               int below = std::numeric_limits<int>::max()) const {
    size_t total = 0;
    for (const auto& name_and_entry : factors_) {
      const Entry& entry = name_and_entry.second;
      if (entry.group == group && entry.index < below) ++total;
    }
    return total;
  }

  /// Whether the graph holds a factor of this group at this index.
  bool has(const std::string& group, int index) const {
    for (const auto& name_and_entry : factors_) {
      const Entry& entry = name_and_entry.second;
      if (entry.group == group && entry.index == index) return true;
    }
    return false;
  }

 private:
  /// A named factor and the group and index it belongs to.
  struct Entry {
    std::string group;
    int index;  ///< state, control or interval index the factor belongs to
    gtsam::NonlinearFactor::shared_ptr factor;
  };

  GraphSpec spec_;
  OnlineConfig config_;
  std::shared_ptr<CollisionModel> model_;
  gtsam::Values values_;
  std::map<std::string, Entry> factors_;
  int k_ = 0;
  bool planned_ = false;
  int next_id_ = 0;  ///< keeps names unique within a group and index
  gtsam::Point3 l_planned_ = gtsam::Point3::Zero();

  /// Add a factor under a unique name; null factors are ignored.
  void add(const std::string& group, int index,
           const gtsam::NonlinearFactor::shared_ptr& factor) {
    if (!factor) return;
    factors_[group + "/" + std::to_string(index) + "/" + std::to_string(next_id_++)] =
        Entry{group, index, factor};
  }

  /// Remove every factor of a group at one index.
  void remove(const std::string& group, int index) {
    for (auto it = factors_.begin(); it != factors_.end();) {
      if (it->second.group == group && it->second.index == index) {
        it = factors_.erase(it);
      } else {
        ++it;
      }
    }
  }

  /// Remove every factor of a group.
  void removeGroup(const std::string& group) {
    for (auto it = factors_.begin(); it != factors_.end();) {
      if (it->second.group == group) {
        it = factors_.erase(it);
      } else {
        ++it;
      }
    }
  }

  /// Number of joints.
  int numJoints() const { return static_cast<int>(spec_.q_start.size()); }

  /**
   * The factors of buildScateGraph (start/goal priors, dynamics, control and
   * joint limits), by name, plus the optional control-effort priors.
   */
  void addBaseFactors() {
    addEndpointPriors();
    const Vector zero = Vector::Zero(numJoints());
    const gtsam::SharedNoiseModel effort_noise = effortNoise();
    for (int i = 0; i < numIntervals(); ++i) {
      add("dynamics", i, dynamicsFactor(spec_, i));
      add("control_limit", i, controlLimitFactor(spec_, i));
      if (effort_noise) {
        add("effort", i,
            gtsam::NonlinearFactor::shared_ptr(
                new gtsam::PriorFactor<Vector>(UKey(i), zero, effort_noise)));
      }
    }
    for (int i = 0; i <= numIntervals(); ++i) {
      add("joint_limit", i, jointLimitFactor(spec_, i));
      add("velocity_limit", i, velocityLimitFactor(spec_, i));
    }
  }

  /// Noise model of the effort prior: per-joint, isotropic, or null (off).
  gtsam::SharedNoiseModel effortNoise() const {
    if (config_.effort_sigmas.size() != 0) {
      return gtsam::noiseModel::Diagonal::Sigmas(config_.effort_sigmas);
    }
    if (config_.sigma_effort > 0.0) return isotropicNoise(numJoints(), config_.sigma_effort);
    return nullptr;
  }


  /// Start and goal priors on q and v (the arm at rest at both ends).
  void addEndpointPriors() {
    const int n = numJoints();
    const int last = numIntervals();
    const Vector zero = Vector::Zero(n);
    const auto noise = isotropicNoise(n, spec_.sigma_endpoint);
    using FactorPtr = gtsam::NonlinearFactor::shared_ptr;
    add("prior_q", 0, FactorPtr(new gtsam::PriorFactor<Vector>(QKey(0), spec_.q_start, noise)));
    add("prior_v", 0, FactorPtr(new gtsam::PriorFactor<Vector>(VKey(0), zero, noise)));
    add("goal_q", last, FactorPtr(new gtsam::PriorFactor<Vector>(QKey(last), spec_.q_goal, noise)));
    add("goal_v", last, FactorPtr(new gtsam::PriorFactor<Vector>(VKey(last), zero, noise)));
  }

  /// Encoder priors on q_k and v_k (paper eq. 11); also the new warm start.
  void addStateMeasurements(int k, const Vector& q_measured, const Vector& v_measured) {
    const int n = numJoints();
    add("measurement_q", k,
        gtsam::NonlinearFactor::shared_ptr(new gtsam::PriorFactor<Vector>(
            QKey(k), q_measured, isotropicNoise(n, config_.sigma_q_measurement))));
    add("measurement_v", k,
        gtsam::NonlinearFactor::shared_ptr(new gtsam::PriorFactor<Vector>(
            VKey(k), v_measured, isotropicNoise(n, config_.sigma_v_measurement))));
    values_.update(QKey(k), q_measured);
    values_.update(VKey(k), v_measured);
  }

  /**
   * Obstacle measurement on (q_k, l) (paper eq. 12). The first one also
   * creates l, initialized from this measurement.
   */
  void addObstacleMeasurement(int k, const Vector& q_measured,
                              const gtsam::Point3& obstacle_measured) {
    if (!haveObstacle()) {
      values_.insert(LKey(), model_->sensorPose(q_measured).transformFrom(obstacle_measured));
    }
    add("measurement_l", k,
        gtsam::NonlinearFactor::shared_ptr(new ObstacleMeasurementFactor(
            QKey(k), LKey(), obstacle_measured, model_,
            isotropicNoise(3, config_.sigma_l_measurement))));
  }

  /**
   * Rebuild the obstacle and self-collision factors on the intervals that
   * are still ahead (k_ .. N-1). The current state is measured, so its own
   * check (tau = 0 on interval k_) is skipped: it could only push on v.
   */
  void refreshIntervalFactors() {
    removeGroup("obstacle");
    removeGroup("self_collision");
    if (haveObstacle()) {
      l_planned_ = obstacleEstimate();
      model_->setObstacle(l_planned_, config_.obstacle_radius);
    }
    for (int i = k_; i < numIntervals(); ++i) {
      for (double fraction : config_.tau_fractions) {
        if (i == k_ && fraction == 0.0) continue;
        addCollisionChecks(i, fraction * spec_.dt);
      }
    }
    // The final state is the start of no interval, so check it at the end of
    // the last one.
    if (k_ < numIntervals()) addCollisionChecks(numIntervals() - 1, spec_.dt);
  }

  /// Obstacle (once l exists) and self-collision factors at time tau of interval i.
  void addCollisionChecks(int i, double tau) {
    if (haveObstacle()) add("obstacle", i, model_->obstacleFactor(i, tau));
    if (config_.self_collision) add("self_collision", i, model_->selfCollisionFactor(i, tau));
  }

  /// Re-solve from the current solution and record the result in info.
  void solve(StepInfo* info) {
    const gtsam::NonlinearFactorGraph current_graph = graph();
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(config_.max_iterations);
    const auto start_time = std::chrono::steady_clock::now();
    values_ = gtsam::LevenbergMarquardtOptimizer(current_graph, values_, params).optimize();
    const auto end_time = std::chrono::steady_clock::now();
    info->solves += 1;
    info->solve_ms += std::chrono::duration<double, std::milli>(end_time - start_time).count();
    info->graph_error = current_graph.error(values_);
    info->num_factors = current_graph.size();
    info->have_obstacle = haveObstacle();
    if (info->have_obstacle) {
      info->l_estimate = obstacleEstimate();
      info->goal_clearance =
          model_->obstacleClearance(spec_.q_goal, info->l_estimate, config_.obstacle_radius);
    }
    recordPlan(info);
    recordObstacleCovariance(current_graph, info);
  }

  /// Copy the planned q and v of states k..N into info.
  void recordPlan(StepInfo* info) const {
    info->plan_q.clear();
    info->plan_v.clear();
    for (int i = k_; i <= numIntervals(); ++i) {
      info->plan_q.push_back(values_.at<Vector>(QKey(i)));
      info->plan_v.push_back(values_.at<Vector>(VKey(i)));
    }
  }

  /// Marginal covariance of l, if requested and l exists (left out if singular).
  void recordObstacleCovariance(const gtsam::NonlinearFactorGraph& current_graph,
                                StepInfo* info) const {
    info->l_covariance.reset();
    if (!config_.compute_l_covariance || !info->have_obstacle) return;
    try {
      info->l_covariance = gtsam::Marginals(current_graph, values_).marginalCovariance(LKey());
    } catch (const std::exception&) {
      // Singular (e.g. no effort prior): leave it out of the replay.
    }
  }
};

// ============================================================
// 5. Closed-loop simulation
// ============================================================

/// Ground truth and noise of the closed-loop simulation.
struct SimulationConfig {
  gtsam::Point3 obstacle_center = gtsam::Point3::Zero();  ///< ground truth
  double obstacle_radius = 0.08;      ///< m
  double sensor_range = 0.4;          ///< m, sensor origin to obstacle center; <= 0 = blind
  double sigma_process_u = 1e-2;      ///< rad/s^2, noise added to each executed control
  double sigma_q_measurement = 1e-4;  ///< rad
  double sigma_v_measurement = 1e-3;  ///< rad/s
  double sigma_l_measurement = 5e-3;  ///< m, per axis in the sensor frame
  int clearance_samples = 10;         ///< samples per interval for scoring
  unsigned seed = 0;                  ///< random seed of all noise
};

/// Executed motion and scores of one closed-loop run.
struct ClosedLoopResult {
  std::vector<Vector> q, v;     ///< true states 0..N
  std::vector<Vector> u;        ///< executed controls 0..N-1 (planned + process noise)
  std::vector<StepInfo> steps;  ///< steps[0] is the offline plan
  std::vector<double> obstacle_clearance;  ///< per interval, minimum over samples
  double min_obstacle_clearance = std::numeric_limits<double>::infinity();
  double min_self_clearance = std::numeric_limits<double>::infinity();
  int first_detection = -1;  ///< state index of the first obstacle measurement
  double goal_error = 0.0;   ///< max |q_N - q_goal|
  double final_speed = 0.0;  ///< max |v_N|
  double max_solve_ms = 0.0; ///< slowest planner step
  /// Whether the robot hit the obstacle or itself.
  bool collided() const { return min_obstacle_clearance < 0.0 || min_self_clearance < 0.0; }
};

/**
 * Zero-mean Gaussian noise vectors from one seeded generator, identical on
 * every platform. std::mt19937's output is fixed by the C++ standard, but
 * std::normal_distribution is not (libc++ and libstdc++ give different
 * samples for the same seed), so the samples are made here with Box-Muller
 * from the raw 32-bit output. Every sample uses exactly two generator outputs.
 */
class GaussianNoise {
 public:
  /// Seed the generator.
  explicit GaussianNoise(unsigned seed) : generator_(seed) {}

  /// One standard normal sample.
  double standardNormal() {
    constexpr double kTwoPi = 6.283185307179586476925286766559;
    const double u1 = uniformOpenClosed();
    const double u2 = uniformOpenClosed();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(kTwoPi * u2);
  }

  /// A vector of n independent samples with standard deviation sigma.
  Vector sample(long n, double sigma) {
    Vector noise(n);
    for (long j = 0; j < n; ++j) noise(j) = sigma * standardNormal();
    return noise;
  }

 private:
  std::mt19937 generator_;

  /// Uniform in (0, 1], from one 32-bit output (never 0, so log() is finite).
  double uniformOpenClosed() {
    return (static_cast<double>(generator_()) + 1.0) / 4294967296.0;
  }
};

/**
 * Score one constant-acceleration interval from (q, v) under control u:
 * returns the smallest obstacle clearance over the samples, and lowers
 * result->min_self_clearance as needed.
 */
inline double scoreInterval(const CollisionModel& truth, const SimulationConfig& simulation,
                            const Vector& q, const Vector& v, const Vector& u, double dt,
                            ClosedLoopResult* result) {
  if (simulation.clearance_samples < 1) {
    throw std::invalid_argument(
        "scoreInterval: clearance_samples must be at least 1");
  }
  double interval_min = std::numeric_limits<double>::infinity();
  for (int s = 0; s <= simulation.clearance_samples; ++s) {
    const double t = dt * s / simulation.clearance_samples;
    const Vector q_at_t = q + t * v + 0.5 * t * t * u;
    interval_min = std::min(interval_min,
                            truth.obstacleClearance(q_at_t, simulation.obstacle_center,
                                                    simulation.obstacle_radius));
    result->min_self_clearance = std::min(result->min_self_clearance, truth.selfClearance(q_at_t));
  }
  return interval_min;
}

/// The obstacle center in the sensor frame, with noise, if it is in range.
inline std::optional<gtsam::Point3> measureObstacle(const CollisionModel& truth,
                                                    const SimulationConfig& simulation,
                                                    const Vector& q, GaussianNoise* noise) {
  const gtsam::Pose3 wTs = truth.sensorPose(q);
  if (simulation.sensor_range <= 0.0 ||
      (simulation.obstacle_center - wTs.translation()).norm() >
          simulation.sensor_range) {
    return std::nullopt;
  }
  return gtsam::Point3(wTs.transformTo(simulation.obstacle_center) +
                       noise->sample(3, simulation.sigma_l_measurement));
}

/**
 * Plan offline, then execute the plan one control at a time on a simulated
 * double integrator with process noise, replanning after every step.
 */
inline ClosedLoopResult runClosedLoop(OnlinePlanner& planner, const CollisionModel& truth,
                                      const SimulationConfig& simulation) {
  GaussianNoise noise(simulation.seed);
  const GraphSpec& spec = planner.spec();
  const long n = spec.q_start.size();
  const double dt = spec.dt;

  ClosedLoopResult result;
  result.steps.push_back(planner.planOffline());
  Vector q = spec.q_start, v = Vector::Zero(n);
  result.q.push_back(q);
  result.v.push_back(v);

  for (int k = 0; k < planner.numIntervals(); ++k) {
    const Vector u = planner.control(k) + noise.sample(n, simulation.sigma_process_u);
    result.u.push_back(u);

    const double interval_min = scoreInterval(truth, simulation, q, v, u, dt, &result);
    result.obstacle_clearance.push_back(interval_min);
    result.min_obstacle_clearance = std::min(result.min_obstacle_clearance, interval_min);

    q = q + dt * v + 0.5 * dt * dt * u;
    v = v + dt * u;
    result.q.push_back(q);
    result.v.push_back(v);

    const Vector q_measured = q + noise.sample(n, simulation.sigma_q_measurement);
    const Vector v_measured = v + noise.sample(n, simulation.sigma_v_measurement);
    const std::optional<gtsam::Point3> obstacle_measured =
        measureObstacle(truth, simulation, q, &noise);
    if (obstacle_measured && result.first_detection < 0) result.first_detection = k + 1;
    result.steps.push_back(planner.update(k + 1, q_measured, v_measured, obstacle_measured));
  }

  result.goal_error = (q - spec.q_goal).cwiseAbs().maxCoeff();
  result.final_speed = v.cwiseAbs().maxCoeff();
  for (const auto& step : result.steps) {
    result.max_solve_ms = std::max(result.max_solve_ms, step.solve_ms);
  }
  return result;
}

}  // namespace scate
