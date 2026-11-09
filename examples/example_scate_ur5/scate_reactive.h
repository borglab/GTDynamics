/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_reactive.h
 * @author Harrison Jenkins
 *
 * Reactive SCATE (paper eq. 7 and Algorithm 1) with motion-capture sensing,
 * for moving obstacles. A VICON-style system measures, in the world frame:
 *   - markers on the arm: their positions are FK(q) of points on the links,
 *     so each reading is a factor on q_k (paper eq. 11, phi^meas_x);
 *   - the center of every obstacle: a factor on that obstacle's location at
 *     that step, l_{j,k} (paper eq. 12, phi^meas_l, with a global sensor the
 *     measurement no longer involves the robot's state).
 *
 * Differences from scate_online.h (one static obstacle, wrist range sensor):
 *   - one location variable per obstacle per step, l_{j,k}, as in the paper,
 *     because the obstacles move;
 *   - reactive planning: after each step, every obstacle factor on the
 *     remaining intervals is rebuilt from the latest estimate l_{j,k}, i.e.
 *     the planner assumes the obstacles stay where they were last seen
 *     (paper Algorithm 1, "replace phi^obs_{i+1..N} with l_i^meas"). It does
 *     not predict their motion; that is predictive SCATE (paper eq. 8);
 *   - the arm's state is estimated from the markers (encoders are optional).
 *     Markers give positions only, so the executed control gets a prior
 *     equal to the commanded control, with the process noise as sigma:
 *     without it, past controls are free and the velocity is unobservable.
 *     The dynamics of an executed interval are exact (the uncertainty is in
 *     the executed control), so their factor is replaced by a tight one;
 *   - at k = 0 the measured start replaces the nominal start prior on q
 *     (paper Algorithm 1, "if i = 0 remove phi^start"); the arm is known to
 *     start at rest, so the prior v_0 = 0 stays. At k = N the goal priors are
 *     removed ("if i = N remove phi^goal"): otherwise their 1e-3 sigma
 *     outweighs the marker reading and the estimate of q_N sits at the goal
 *     whatever the arm did.
 *
 * Per online step k (the robot has executed u_{k-1}, or is at the start):
 *   1. if k > 0: drop the executed control's limit and effort factors, and
 *      add the executed-control prior on u_{k-1};
 *   2. add the marker factor on q_k (and encoder priors, if used);
 *   3. for each obstacle seen: add l_{j,k} with its measurement factor;
 *   4. rebuild obstacle and self-collision factors on the remaining
 *      intervals from the latest estimate of each obstacle;
 *   5. re-solve with Levenberg-Marquardt from the previous solution.
 *
 * Kept from scate_online.h: obstacle factors use the estimates as constants
 * (the collision cost cannot move an obstacle), no feedback law (re-planning
 * each step does that job), and an optional weak control-effort prior.
 */

#pragma once

#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "scate_online.h"

namespace scate {

/// Key of obstacle j's location at step k, l_{j,k} (j < 26).
inline gtsam::Key ObstacleKey(size_t j, int k) {
  if (j >= 26) throw std::invalid_argument("ObstacleKey: at most 26 obstacles");
  return gtsam::LabeledSymbol('o', static_cast<unsigned char>('a' + j), k);
}

// ============================================================
// 1. Motion-capture measurement factor on the arm
// ============================================================

/**
 * z = marker positions at q + noise: the stacked world positions of the
 * robot's motion-capture markers (paper eq. 11 for a manipulator: the
 * measured state is seen through forward kinematics).
 */
class MarkerMeasurementFactor : public gtsam::NoiseModelFactorN<Vector> {
  using Base = gtsam::NoiseModelFactorN<Vector>;
  Vector measured_;
  std::shared_ptr<const CollisionModel> model_;

 public:
  /**
   * @param q_key joint angles at the time of the reading
   * @param measured stacked world positions of the markers (3 per marker)
   * @param model provides the marker positions and their Jacobian
   * @param noise noise model of the stacked positions
   */
  MarkerMeasurementFactor(gtsam::Key q_key, const Vector& measured,
                          std::shared_ptr<const CollisionModel> model,
                          const gtsam::SharedNoiseModel& noise)
      : Base(noise, q_key), measured_(measured), model_(std::move(model)) {
    if (!model_) throw std::invalid_argument("MarkerMeasurementFactor: null model");
    if (measured_.size() != static_cast<long>(3 * model_->numMarkers())) {
      throw std::invalid_argument("MarkerMeasurementFactor: need 3 values per marker");
    }
  }

  /// Predicted minus measured marker positions.
  gtsam::Vector evaluateError(const Vector& q,
                              gtsam::OptionalMatrixType H = nullptr) const override {
    gtsam::Matrix jacobian;
    const Vector predicted = model_->markerPositions(q, H ? &jacobian : nullptr);
    if (H) *H = jacobian;
    return predicted - measured_;
  }

  /// Deep copy of this factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new MarkerMeasurementFactor(*this));
  }

  /// The measured marker positions.
  const Vector& measured() const { return measured_; }
};

// ============================================================
// 2. Reactive planner
// ============================================================

/// One motion-capture reading (and optional encoder reading) at a state.
struct ViconMeasurement {
  Vector markers;  ///< stacked world positions of the arm's markers (3 per marker)
  /// Obstacle centers in the world frame, one per obstacle; nullopt = not seen.
  std::vector<std::optional<gtsam::Point3>> obstacles;
  std::optional<Vector> q_encoder;  ///< joint encoders, if used
  std::optional<Vector> v_encoder;  ///< joint velocities, if used
};

/// Settings of the reactive planner.
struct ReactiveConfig {
  /// Known radius of each obstacle (m); its length is the number of obstacles.
  std::vector<double> obstacle_radii;
  double sigma_marker = 1e-3;    ///< m per axis, motion-capture markers on the arm
  double sigma_obstacle = 1e-3;  ///< m per axis, motion-capture obstacle centers
  double sigma_q_encoder = 1e-3;  ///< rad, if encoder readings are given
  double sigma_v_encoder = 1e-2;  ///< rad/s, if encoder readings are given
  /// Prior of an executed control on the commanded one (rad/s^2): the process noise.
  double sigma_executed_control = 1e-2;
  /**
   * Dynamics sigma of an executed interval. The motion over an executed
   * interval follows the double integrator exactly (the uncertainty is in
   * the executed control), so it is much tighter than the planning sigma.
   * With the planning sigma (1e-2) the dynamics could absorb 1 cm of position
   * error, and marker readings could not pin the velocity.
   */
  double sigma_executed_dynamics = 1e-5;
  /// Weak prior pulling each planned control toward zero (rad/s^2); 0 turns it off.
  double sigma_effort = 1.0;
  Vector effort_sigmas;  ///< per-joint effort sigmas, overriding sigma_effort if set
  /// Collision checks per interval, as fractions of dt in [0, 1).
  std::vector<double> tau_fractions = evenlySpacedFractions(3);
  bool self_collision = true;  ///< add self-collision factors
  int max_iterations = 100;    ///< Levenberg-Marquardt iteration cap
};

/**
 * Reactive SCATE: holds the graph by name (FactorRegistry) and the current
 * solution as the warm start, and re-plans after every motion-capture reading.
 */
class ReactivePlanner {
 public:
  /**
   * @param spec start, goal, limits, horizon and base noise of the graph
   * @param config sensing and planning settings, including the obstacle radii
   * @param model collision geometry and markers of the robot
   * @param initial initial guess of every q_i, v_i and u_i
   */
  ReactivePlanner(const GraphSpec& spec, const ReactiveConfig& config,
                  std::shared_ptr<CollisionModel> model, const gtsam::Values& initial)
      : spec_(spec), config_(config), model_(std::move(model)), values_(initial),
        latest_(config.obstacle_radii.size()) {
    validateSpec(spec_);
    if (!model_) throw std::invalid_argument("ReactivePlanner: null CollisionModel");
    if (model_->dof() != static_cast<size_t>(spec_.q_start.size())) {
      throw std::invalid_argument("ReactivePlanner: model dof does not match GraphSpec");
    }
    for (double fraction : config_.tau_fractions) {
      if (fraction < 0.0 || fraction >= 1.0) {
        throw std::invalid_argument("ReactivePlanner: tau fractions must be in [0, 1)");
      }
    }
    for (double radius : config_.obstacle_radii) {
      if (!(radius > 0.0)) throw std::invalid_argument("ReactivePlanner: radii must be > 0");
    }
    addPlanFactors(spec_,
                   effortNoise(numJoints(), config_.effort_sigmas, config_.sigma_effort),
                   &factors_);
  }

  /// Number of intervals N (the last state index).
  int numIntervals() const { return spec_.num_poses - 1; }
  /// State index of the last reading, or -1 before the first.
  int currentIndex() const { return k_; }
  /// The graph specification.
  const GraphSpec& spec() const { return spec_; }
  /// The settings.
  const ReactiveConfig& config() const { return config_; }
  /// The current solution.
  const gtsam::Values& values() const { return values_; }
  /// Planned control u_i in the current solution.
  Vector control(int i) const { return values_.at<Vector>(UKey(i)); }
  /// Number of obstacles.
  size_t numObstacles() const { return config_.obstacle_radii.size(); }
  /// The current graph, assembled from the named factors.
  gtsam::NonlinearFactorGraph graph() const { return factors_.graph(); }
  /// Number of factors in a group, optionally only those with index below `below`.
  size_t count(const std::string& group, int below = std::numeric_limits<int>::max()) const {
    return factors_.count(group, below);
  }
  /// Whether the graph holds a factor of this group at this index.
  bool has(const std::string& group, int index) const { return factors_.has(group, index); }

  /// Latest estimate of obstacle j's location, if it has been seen.
  std::optional<gtsam::Point3> obstacleEstimate(size_t j) const {
    if (!latest_.at(j)) return std::nullopt;
    return values_.at<gtsam::Point3>(ObstacleKey(j, *latest_[j]));
  }

  /**
   * Paper Algorithm 1, "Generate Initial Plan": no obstacle has been seen
   * (the empty obstacle set), so only self-collision factors are added.
   */
  StepInfo planOffline() {
    if (planned_) throw std::logic_error("planOffline: call once, before update");
    planned_ = true;
    refreshIntervalFactors(0);
    StepInfo info;
    info.k = 0;
    solve(0, &info);
    return info;
  }

  /**
   * One online step at state k (0 = the start, before any control is
   * executed; then after executing u_{k-1}): add the reading, rebuild the
   * future obstacle factors from the latest estimates, and re-solve.
   */
  StepInfo update(int k, const ViconMeasurement& reading) {
    if (!planned_) throw std::logic_error("update: call planOffline first");
    if (k != k_ + 1 || k > numIntervals()) {
      throw std::invalid_argument("update: expected state index " + std::to_string(k_ + 1));
    }
    if (reading.obstacles.size() != numObstacles()) {
      throw std::invalid_argument("update: need one (possibly empty) reading per obstacle");
    }
    k_ = k;
    StepInfo info;
    info.k = k;

    if (k > 0) recordExecutedControl(k - 1);
    else factors_.remove("prior_q", 0);  // the measured start replaces the nominal one
    if (k == numIntervals()) {
      // At the end the reading, not the goal, says where the arm is.
      factors_.remove("goal_q", k);
      factors_.remove("goal_v", k);
    }
    addArmMeasurements(k, reading);
    info.detected = addObstacleMeasurements(k, reading);

    refreshIntervalFactors(k);
    solve(k, &info);
    return info;
  }

 private:
  GraphSpec spec_;
  ReactiveConfig config_;
  std::shared_ptr<CollisionModel> model_;
  gtsam::Values values_;
  FactorRegistry factors_;
  int k_ = -1;
  bool planned_ = false;
  std::vector<std::optional<int>> latest_;  ///< step of each obstacle's latest reading

  /// Number of joints.
  int numJoints() const { return static_cast<int>(spec_.q_start.size()); }

  /**
   * u_i has been executed: it is history now, so drop its limit and effort
   * cost, tie it to the commanded value (which the warm start still holds)
   * with the process noise, and make the interval's dynamics exact.
   */
  void recordExecutedControl(int i) {
    factors_.remove("control_limit", i);
    factors_.remove("effort", i);
    factors_.remove("dynamics", i);
    GraphSpec executed = spec_;
    executed.sigma_dynamics = config_.sigma_executed_dynamics;
    factors_.add("executed_dynamics", i, dynamicsFactor(executed, i));
    factors_.add("executed_control", i,
                 gtsam::NonlinearFactor::shared_ptr(new gtsam::PriorFactor<Vector>(
                     UKey(i), control(i),
                     isotropicNoise(numJoints(), config_.sigma_executed_control))));
  }

  /// Marker factor on q_k (paper eq. 11), plus encoder priors if given.
  void addArmMeasurements(int k, const ViconMeasurement& reading) {
    using FactorPtr = gtsam::NonlinearFactor::shared_ptr;
    if (reading.markers.size() != 0) {
      factors_.add("measurement_markers", k,
                   FactorPtr(new MarkerMeasurementFactor(
                       QKey(k), reading.markers, model_,
                       isotropicNoise(reading.markers.size(), config_.sigma_marker))));
    }
    if (reading.q_encoder) {
      factors_.add("measurement_q", k,
                   FactorPtr(new gtsam::PriorFactor<Vector>(
                       QKey(k), *reading.q_encoder,
                       isotropicNoise(numJoints(), config_.sigma_q_encoder))));
      values_.update(QKey(k), *reading.q_encoder);
    }
    if (reading.v_encoder) {
      factors_.add("measurement_v", k,
                   FactorPtr(new gtsam::PriorFactor<Vector>(
                       VKey(k), *reading.v_encoder,
                       isotropicNoise(numJoints(), config_.sigma_v_encoder))));
      values_.update(VKey(k), *reading.v_encoder);
    }
  }

  /**
   * For each obstacle seen: the variable l_{j,k}, initialized at the reading,
   * and its measurement factor (paper eq. 12). Returns whether any was seen.
   */
  bool addObstacleMeasurements(int k, const ViconMeasurement& reading) {
    bool any = false;
    for (size_t j = 0; j < numObstacles(); ++j) {
      if (!reading.obstacles[j]) continue;
      any = true;
      values_.insert(ObstacleKey(j, k), *reading.obstacles[j]);
      factors_.add("measurement_l", k,
                   gtsam::NonlinearFactor::shared_ptr(new gtsam::PriorFactor<gtsam::Point3>(
                       ObstacleKey(j, k), *reading.obstacles[j],
                       isotropicNoise(3, config_.sigma_obstacle))));
      latest_[j] = k;
    }
    return any;
  }

  /// The obstacles as last seen (the reactive assumption: they stay there).
  std::vector<SphereObstacle> latestObstacles() const {
    std::vector<SphereObstacle> obstacles;
    for (size_t j = 0; j < numObstacles(); ++j) {
      if (const auto estimate = obstacleEstimate(j)) {
        obstacles.push_back({*estimate, config_.obstacle_radii[j]});
      }
    }
    return obstacles;
  }

  /**
   * Rebuild the obstacle and self-collision factors on the intervals still
   * ahead (k .. N-1) from the latest obstacle estimates (paper Algorithm 1,
   * reactive reassignment). The measured state's own check (tau = 0 on
   * interval k) is skipped after the first reading.
   */
  void refreshIntervalFactors(int k) {
    factors_.removeGroup("obstacle");
    factors_.removeGroup("self_collision");
    const std::vector<SphereObstacle> obstacles = latestObstacles();
    if (!obstacles.empty()) model_->setObstacles(obstacles);
    const bool have_obstacles = !obstacles.empty();
    for (int i = k; i < numIntervals(); ++i) {
      for (double fraction : config_.tau_fractions) {
        if (i == k && fraction == 0.0 && k_ >= 0) continue;
        addCollisionChecks(i, fraction * spec_.dt, have_obstacles);
      }
    }
    if (k < numIntervals()) addCollisionChecks(numIntervals() - 1, spec_.dt, have_obstacles);
  }

  /// Obstacle (if any obstacle is known) and self-collision factors at time tau of interval i.
  void addCollisionChecks(int i, double tau, bool have_obstacles) {
    if (have_obstacles) factors_.add("obstacle", i, model_->obstacleFactor(i, tau));
    if (config_.self_collision) {
      factors_.add("self_collision", i, model_->selfCollisionFactor(i, tau));
    }
  }

  /// Re-solve from the current solution and record the result in info.
  void solve(int k, StepInfo* info) {
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
    info->obstacle_estimates.clear();
    for (size_t j = 0; j < numObstacles(); ++j) {
      info->obstacle_estimates.push_back(obstacleEstimate(j));
    }
    recordFirstObstacle(info);
    info->plan_q.clear();
    info->plan_v.clear();
    for (int i = k; i <= numIntervals(); ++i) {
      info->plan_q.push_back(values_.at<Vector>(QKey(i)));
      info->plan_v.push_back(values_.at<Vector>(VKey(i)));
    }
  }

  /// The single-obstacle fields of StepInfo, from the first obstacle seen.
  void recordFirstObstacle(StepInfo* info) const {
    for (size_t j = 0; j < numObstacles(); ++j) {
      if (const auto estimate = info->obstacle_estimates[j]) {
        info->have_obstacle = true;
        info->l_estimate = *estimate;
        info->goal_clearance =
            model_->obstacleClearance(spec_.q_goal, *estimate, config_.obstacle_radii[j]);
        return;
      }
    }
  }
};

// ============================================================
// 3. Moving obstacles and the closed-loop simulation
// ============================================================

/// Which obstacle readings reach the planner (for the negative controls).
enum class ObstacleSensing {
  kEveryStep,      ///< reactive SCATE: every reading
  kFirstStepOnly,  ///< "frozen": the obstacles as seen at the start, never updated
  kNone,           ///< blind
};

/// Ground truth and noise of the motion-capture closed loop.
struct ViconSimulationConfig {
  std::vector<MovingObstacle> obstacles;  ///< ground truth
  ObstacleSensing sensing = ObstacleSensing::kEveryStep;
  bool use_encoders = false;          ///< also give joint encoder readings
  double sigma_process_u = 1e-2;      ///< rad/s^2, noise added to each executed control
  double sigma_marker = 5e-4;         ///< m per axis, arm markers
  double sigma_obstacle = 5e-4;       ///< m per axis, obstacle centers
  double sigma_q_encoder = 1e-4;      ///< rad
  double sigma_v_encoder = 1e-3;      ///< rad/s
  int clearance_samples = 10;         ///< samples per interval for scoring
  unsigned seed = 0;                  ///< random seed of all noise
};

/// Smallest gap between the robot at q and any obstacle at time t; updates per-obstacle minima.
inline double movingObstacleClearance(const CollisionModel& truth,
                                      const std::vector<MovingObstacle>& obstacles,
                                      const Vector& q, double t,
                                      std::vector<double>* per_obstacle) {
  double best = std::numeric_limits<double>::infinity();
  for (size_t j = 0; j < obstacles.size(); ++j) {
    const double gap = truth.obstacleClearance(q, obstacles[j].center(t), obstacles[j].radius);
    (*per_obstacle)[j] = std::min((*per_obstacle)[j], gap);
    best = std::min(best, gap);
  }
  return best;
}

/**
 * Score one constant-acceleration interval starting at time t0 from (q, v)
 * under control u, against the moving obstacles: returns the smallest
 * obstacle clearance and lowers the self and per-obstacle minima in result.
 */
inline double scoreMovingInterval(const CollisionModel& truth,
                                  const ViconSimulationConfig& simulation, const Vector& q,
                                  const Vector& v, const Vector& u, double t0, double dt,
                                  ClosedLoopResult* result) {
  if (simulation.clearance_samples < 1) {
    throw std::invalid_argument("scoreMovingInterval: clearance_samples must be at least 1");
  }
  double interval_min = std::numeric_limits<double>::infinity();
  for (int s = 0; s <= simulation.clearance_samples; ++s) {
    const double t = dt * s / simulation.clearance_samples;
    const Vector q_at_t = q + t * v + 0.5 * t * t * u;
    interval_min = std::min(interval_min,
                            movingObstacleClearance(truth, simulation.obstacles, q_at_t, t0 + t,
                                                    &result->min_clearance_per_obstacle));
    result->min_self_clearance = std::min(result->min_self_clearance, truth.selfClearance(q_at_t));
  }
  return interval_min;
}

/**
 * The motion-capture reading at state k (time t). Every noise sample is drawn
 * whether or not the reading is used, so runs with different sensing see
 * the same noise.
 */
inline ViconMeasurement readVicon(const CollisionModel& truth,
                                  const ViconSimulationConfig& simulation, int k, double t,
                                  const Vector& q, const Vector& v, GaussianNoise* noise) {
  ViconMeasurement reading;
  reading.markers = truth.markerPositions(q);
  reading.markers += noise->sample(reading.markers.size(), simulation.sigma_marker);
  const Vector q_noise = noise->sample(q.size(), simulation.sigma_q_encoder);
  const Vector v_noise = noise->sample(v.size(), simulation.sigma_v_encoder);
  if (simulation.use_encoders) {
    reading.q_encoder = q + q_noise;
    reading.v_encoder = v + v_noise;
  }
  const bool sees = simulation.sensing == ObstacleSensing::kEveryStep ||
                    (simulation.sensing == ObstacleSensing::kFirstStepOnly && k == 0);
  for (const MovingObstacle& obstacle : simulation.obstacles) {
    const gtsam::Point3 measured =
        obstacle.center(t) + gtsam::Point3(noise->sample(3, simulation.sigma_obstacle));
    reading.obstacles.push_back(sees ? std::optional<gtsam::Point3>(measured) : std::nullopt);
  }
  return reading;
}

/**
 * Reactive SCATE in closed loop (paper Algorithm 1): plan with no obstacle
 * known, then at every state k = 0..N read the motion-capture system,
 * update and re-plan, and (for k < N) execute one control with process
 * noise on a simulated double integrator while the obstacles move.
 * result.steps[k] is the plan after the reading at state k; result.q[k]
 * the true state; result.obstacle_centers[k] the true obstacle centers.
 */
inline ClosedLoopResult runReactiveClosedLoop(ReactivePlanner& planner,
                                              const CollisionModel& truth,
                                              const ViconSimulationConfig& simulation) {
  if (simulation.obstacles.size() != planner.numObstacles()) {
    throw std::invalid_argument("runReactiveClosedLoop: obstacle count differs from the planner");
  }
  GaussianNoise noise(simulation.seed);
  const GraphSpec& spec = planner.spec();
  const long n = spec.q_start.size();
  const double dt = spec.dt;

  ClosedLoopResult result;
  result.min_clearance_per_obstacle.assign(simulation.obstacles.size(),
                                           std::numeric_limits<double>::infinity());
  planner.planOffline();
  Vector q = spec.q_start, v = Vector::Zero(n);

  for (int k = 0; k <= planner.numIntervals(); ++k) {
    const double t = k * dt;
    result.q.push_back(q);
    result.v.push_back(v);
    std::vector<gtsam::Point3> centers;
    for (const MovingObstacle& obstacle : simulation.obstacles) centers.push_back(obstacle.center(t));
    result.obstacle_centers.push_back(centers);

    const ViconMeasurement reading = readVicon(truth, simulation, k, t, q, v, &noise);
    result.steps.push_back(planner.update(k, reading));
    if (result.steps.back().detected && result.first_detection < 0) result.first_detection = k;
    if (k == planner.numIntervals()) break;

    const Vector u = planner.control(k) + noise.sample(n, simulation.sigma_process_u);
    result.u.push_back(u);
    const double interval_min = scoreMovingInterval(truth, simulation, q, v, u, t, dt, &result);
    result.obstacle_clearance.push_back(interval_min);
    result.min_obstacle_clearance = std::min(result.min_obstacle_clearance, interval_min);
    q = q + dt * v + 0.5 * dt * dt * u;
    v = v + dt * u;
  }

  result.goal_error = (q - spec.q_goal).cwiseAbs().maxCoeff();
  result.final_speed = v.cwiseAbs().maxCoeff();
  for (const auto& step : result.steps) {
    result.max_solve_ms = std::max(result.max_solve_ms, step.solve_ms);
  }
  return result;
}

}  // namespace scate
