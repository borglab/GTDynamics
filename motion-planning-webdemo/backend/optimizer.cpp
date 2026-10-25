// Native GTDynamics/GTSAM trajectory optimizer. One NDJSON request per line.
// Batch LM motion planning over kinematic slices. Execution measurements
// are not incorporated into the graph.
#include "TrajectoryFactors.h"

#include <boost/json/src.hpp>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace json = boost::json;
using namespace gtdynamics;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Vector6;
using gtsam::Values;
using gtsam::noiseModel::Isotropic;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr size_t kMaxIterations = 80;
// Pose3 tangent coordinates: rotation in radians, translation in meters.
// A tenfold smaller sigma gives endpoint priors 100 times their former weight.
constexpr double kEndpointPoseSigma = 1e-5;
constexpr double kEndpointPositionTolerance = 1e-4;  // 0.1 mm, measured by FK(q)
constexpr double kEndpointRotationTolerance = 1e-3;  // radians

json::array vectorJson(const Eigen::Ref<const gtsam::Vector>& v) {
  json::array result;
  for (Eigen::Index i = 0; i < v.size(); ++i) result.push_back(v[i]);
  return result;
}
json::object poseJson(const Pose3& pose) {
  const auto quaternion = pose.rotation().toQuaternion();
  return {{"position", vectorJson(pose.translation())},
          {"quaternion", {quaternion.x(), quaternion.y(), quaternion.z(),
                          quaternion.w()}}};
}

double number(const json::value& v, const std::string& name) {
  if (!v.is_number()) throw std::invalid_argument(name + " must be a number");
  const double d = v.to_number<double>();
  if (!std::isfinite(d)) throw std::invalid_argument(name + " must be finite");
  return d;
}
Pose3 parsePose(const json::value& value) {
  if (!value.is_object()) throw std::invalid_argument("pose must be an object");
  const auto& object = value.as_object();
  const auto& position = object.at("position").as_array();
  const auto& orientation = object.at("quaternion").as_array();
  if (position.size() != 3 || orientation.size() != 4)
    throw std::invalid_argument("pose requires position[3] and quaternion[x,y,z,w]");
  gtsam::Point3 p;
  for (size_t i = 0; i < 3; ++i) p[i] = number(position[i], "position");
  if (p.norm() > 3.0) throw std::invalid_argument("target must be within 3 m of robot base");
  Eigen::Quaterniond q(number(orientation[3], "quaternion"),
                       number(orientation[0], "quaternion"),
                       number(orientation[1], "quaternion"),
                       number(orientation[2], "quaternion"));
  if (!std::isfinite(q.norm()) || q.norm() < 1e-8)
    throw std::invalid_argument("quaternion must have finite nonzero norm");
  q.normalize();
  return Pose3(Rot3(q), p);
}

class Demo {
  Robot robot_;
  std::vector<JointSharedPtr> joints_;
  std::vector<LinkSharedPtr> links_;
  Pose3 com_T_tcp_;
  Pose3 start_, goal_;
  const std::vector<double> home_{0.0, -0.55, 0.0, -2.25, 0.0, 1.75, kPi / 4};
  gtsam::NonlinearFactorGraph graph_;
  Values values_;
  std::unique_ptr<gtsam::LevenbergMarquardtOptimizer> optimizer_;
  json::object graph_info_;
  size_t num_states_ = 0, iteration_ = 0, start_factor_ = 0, goal_factor_ = 0;
  double duration_ = 4;
  bool converged_ = false, done_ = false;
  std::string stop_reason_;

  Values forward(const std::vector<double>& q, size_t k = 0) const {
    Values input;
    for (size_t j = 0; j < joints_.size(); ++j) {
      InsertJointAngle(&input, joints_[j]->id(), k, q[j]);
      InsertJointVel(&input, joints_[j]->id(), k, 0.0);
    }
    return robot_.forwardKinematics(input, k, "link0");
  }
  Pose3 tcp(const Values& values, size_t k) const {
    return Pose(values, links_.back()->id(), k).compose(com_T_tcp_);
  }
  std::vector<double> jointAngles(size_t k) const {
    std::vector<double> q;
    for (const auto& joint : joints_) q.push_back(JointAngle(values_, joint->id(), k));
    return q;
  }
  void resetOptimizer() {
    gtsam::LevenbergMarquardtParams params;
    params.maxIterations = kMaxIterations;
    params.relativeErrorTol = 1e-7;
    params.absoluteErrorTol = 1e-8;
    params.lambdaInitial = 1e-2;
    params.lambdaUpperBound = 1e12;
    params.setDiagonalDamping(false);
    params.setVerbosity("SILENT");
    params.setVerbosityLM("SILENT");
    optimizer_ = std::make_unique<gtsam::LevenbergMarquardtOptimizer>(graph_, values_, params);
    iteration_ = 0;
    converged_ = false;
    done_ = false;
    stop_reason_.clear();
  }
  void setEndpointFactors() {
    const auto model = Isotropic::Sigma(6, kEndpointPoseSigma);
    const auto link = links_.back();
    // Pose variables are COM poses, while the student's frames are TCP poses.
    graph_[start_factor_] = std::make_shared<gtsam::PriorFactor<Pose3>>(
        PoseKey(link->id(), 0), start_.compose(com_T_tcp_.inverse()), model);
    graph_[goal_factor_] = std::make_shared<gtsam::PriorFactor<Pose3>>(
        PoseKey(link->id(), num_states_ - 1), goal_.compose(com_T_tcp_.inverse()), model);
  }

 public:
  explicit Demo(const std::string& urdf)
      : robot_(CreateRobotFromFile(urdf).fixLink("link0")) {
    std::vector<std::string> joint_names, link_names;
    for (size_t i = 1; i <= 7; ++i) joint_names.push_back("joint" + std::to_string(i));
    for (size_t i = 0; i <= 7; ++i) link_names.push_back("link" + std::to_string(i));
    if (robot_.joints().size() != 7 || robot_.links().size() != 8)
      throw std::runtime_error("expected 7-joint, 8-link Panda URDF after fixed-link merging");
    robot_.reassignJoints(joint_names);
    robot_.reassignLinks(link_names);
    joints_ = robot_.orderedJoints();
    links_ = robot_.orderedLinks();
    // Menagerie hand is link7 * Tz(.107) * Rz(-pi/4). Its TCP lies .1034 m
    // along local +Z. Derive COM->link from the loaded (merged) inertial model.
    const Pose3 link7_T_tcp(Rot3::Rz(-kPi / 4), gtsam::Point3(0, 0, .2104));
    const auto end = links_.back();
    com_T_tcp_ = end->bMcom().between(end->bMlink()).compose(link7_T_tcp);
    start_ = tcp(forward(home_), 0);
    auto goal_q = home_;
    goal_q[0] = .55;
    goal_q[1] = -.30;
    goal_q[2] = -.30;
    goal_q[4] = .35;
    goal_q[5] = 1.95;
    goal_ = tcp(forward(goal_q), 0);
  }
  json::object describe() const {
    json::array names, link_names, home, limits;
    for (size_t j = 0; j < joints_.size(); ++j) {
      names.emplace_back(joints_[j]->name());
      home.push_back(home_[j]);
      const auto& l = joints_[j]->parameters().scalar_limits;
      limits.push_back(json::array{l.value_lower_limit, l.value_upper_limit});
    }
    for (const auto& link : links_) link_names.emplace_back(link->name());
    return {{"type", "description"}, {"joint_names", names}, {"link_names", link_names},
            {"home", home}, {"joint_limits", limits}, {"start", poseJson(start_)},
            {"goal", poseJson(goal_)}, {"tcp", json::object{
                {"parent", "link7"}, {"position", {0., 0., .2104}},
                {"quaternion", {0., 0., std::sin(-kPi / 8), std::cos(-kPi / 8)}}}},
            {"solver", "GTSAM Levenberg-Marquardt"},
            {"formulation", "GTDynamics kinematic slices with trapezoidal collocation"},
            {"coordinates", "Z-up world; xyzw quaternion; body twist [omega, linear] at COM"},
            {"endpoint_tolerance", json::object{
                {"position", kEndpointPositionTolerance},
                {"rotation", kEndpointRotationTolerance}}},
            {"min_intermediate_states", 1}, {"max_intermediate_states", 40}, {"max_iterations", kMaxIterations}};
  }
  json::object build(const json::object& request) {
    const double intermediate = number(request.at("intermediate_states"), "intermediate_states");
    const double duration = number(request.at("duration"), "duration");
    if (intermediate < 1 || intermediate > 40 || intermediate != std::floor(intermediate))
      throw std::invalid_argument("intermediate_states must be an integer from 1 to 40");
    if (duration < .5 || duration > 20)
      throw std::invalid_argument("duration must be between 0.5 and 20 seconds");
    const Pose3 start = parsePose(request.at("start"));
    const Pose3 goal = parsePose(request.at("goal"));
    num_states_ = static_cast<size_t>(intermediate) + 2;
    duration_ = duration;
    start_ = start;
    goal_ = goal;
    graph_.resize(0);
    values_.clear();
    const double dt = duration_ / (num_states_ - 1);
    KinematicsParameters parameters;
    parameters.p_cost_model = Isotropic::Sigma(6, 1e-3);
    parameters.bp_cost_model = Isotropic::Sigma(6, 1e-6);
    parameters.v_cost_model = Isotropic::Sigma(6, 1e-3);
    parameters.bv_cost_model = Isotropic::Sigma(6, 1e-6);
    Kinematics kinematics(parameters);
    for (size_t k = 0; k < num_states_; ++k) {
      const Slice slice(k);
      graph_.add(kinematics.qFactors(slice, robot_));
      graph_.add(kinematics.vFactors(slice, robot_));
      graph_.add(kinematics.jointAngleLimits(slice, robot_));
      // A deliberately unplanned but feasible joint seed makes LM's changing
      // trajectory visible. Every COM pose/twist is initialized using FK.
      auto q = home_;
      q[0] += .18 * std::sin(kPi * k / (num_states_ - 1));
      const Values fk = forward(q, k);
      for (size_t j = 0; j < joints_.size(); ++j) {
        const auto id = joints_[j]->id();
        InsertJointAngle(&values_, id, k, q[j]);
        InsertJointVel(&values_, id, k, 0.0);
        // Weak posture regularization chooses a stable redundant-arm solution.
        graph_.addPrior(JointAngleKey(id, k), home_[j],
                        Isotropic::Sigma(1, 4.0 * std::sqrt(num_states_)));
        if (k == 0 || k + 1 == num_states_)
          graph_.addPrior(JointVelKey(id, k), 0.0, Isotropic::Sigma(1, 1e-3));
      }
      for (const auto& link : links_) {
        InsertPose(&values_, link->id(), k, Pose(fk, link->id(), k));
        InsertTwist(&values_, link->id(), k, Twist(fk, link->id(), k));
      }
      if (k == 0) continue;
      for (const auto& joint : joints_) {
        const auto j = joint->id();
        graph_.emplace_shared<motion_planning_demo::JointTrapezoidalFactor>(
            JointAngleKey(j, k - 1), JointAngleKey(j, k), JointVelKey(j, k - 1),
            JointVelKey(j, k), dt, Isotropic::Sigma(1, .003));
        // Sum (v[k]-v[k-1])^2/dt approximates integral squared acceleration.
        graph_.emplace_shared<gtsam::BetweenFactor<double>>(
            JointVelKey(j, k - 1), JointVelKey(j, k), 0.0,
            Isotropic::Sigma(1, std::sqrt(dt)));
      }
      for (const auto& link : links_) {
        if (link->isFixed()) continue;
        graph_.emplace_shared<motion_planning_demo::PoseTrapezoidalFactor>(
            PoseKey(link->id(), k - 1), PoseKey(link->id(), k),
            TwistKey(link->id(), k - 1), TwistKey(link->id(), k), dt,
            Isotropic::Sigma(6, .02));
      }
    }
    start_factor_ = graph_.size();
    goal_factor_ = graph_.size() + 1;
    graph_.resize(graph_.size() + 2);
    setEndpointFactors();
    graph_info_ = {{"states", num_states_}, {"variables", values_.size()},
                   {"scalar_dimensions", num_states_ * 110}, {"factors", graph_.size()},
                   {"q_kinematic", num_states_ * 7}, {"v_kinematic", num_states_ * 7},
                   {"fixed_base", num_states_ * 2}, {"joint_limits", num_states_ * 7},
                   {"posture_priors", num_states_ * 7}, {"endpoint_priors", 2},
                   {"endpoint_pose_sigma", kEndpointPoseSigma},
                   {"stationary_endpoints", 14},
                   {"joint_collocation", (num_states_ - 1) * 7},
                   {"pose_collocation", (num_states_ - 1) * 7},
                   {"velocity_smoothness", (num_states_ - 1) * 7}, {"dt", dt}};
    resetOptimizer();
    return snapshot("built");
  }
  json::object targets(const json::object& request) {
    if (!optimizer_) throw std::logic_error("build the graph before updating targets");
    const Pose3 start = parsePose(request.at("start"));
    const Pose3 goal = parsePose(request.at("goal"));
    start_ = start;
    goal_ = goal;
    setEndpointFactors();
    resetOptimizer();  // values_ is the last accepted trajectory (warm start).
    return snapshot("targets");
  }
  json::object iterate() {
    if (!optimizer_) throw std::logic_error("build the graph before solving");
    if (!done_) {
      const double previous_error = optimizer_->error();
      optimizer_->iterate();  // One LM outer iteration, including damping retries.
      values_ = optimizer_->values();
      ++iteration_;
      const double error = optimizer_->error();
      if (!std::isfinite(error)) throw std::runtime_error("optimizer produced a non-finite error");
      if (error < 1e-9 || std::abs(previous_error - error) <= 1e-7 * std::max(1., previous_error)) {
        converged_ = true;
        done_ = true;
        stop_reason_ = "error_tolerance";
      } else if (iteration_ >= kMaxIterations) {
        done_ = true;
        stop_reason_ = "iteration_limit";
      }
    }
    return snapshot("iteration");
  }
  json::object snapshot(const std::string& type) const {
    json::array states;
    for (size_t k = 0; k < num_states_; ++k) {
      json::array q, v, links, com, twists;
      for (const auto& joint : joints_) {
        q.push_back(JointAngle(values_, joint->id(), k));
        v.push_back(JointVel(values_, joint->id(), k));
      }
      for (const auto& link : links_) {
        const auto pose = Pose(values_, link->id(), k);
        links.push_back(poseJson(pose.compose(link->bMcom().between(link->bMlink()))));
        com.push_back(poseJson(pose));
        twists.push_back(vectorJson(Twist(values_, link->id(), k)));
      }
      states.push_back(json::object{{"q", q}, {"v", v}, {"ee", poseJson(tcp(values_, k))},
                                   {"links", links}, {"com_poses", com}, {"twists", twists}});
    }
    // Report endpoint accuracy of the actual q trajectory that MuJoCo replays,
    // rather than only the near-exact priors on independent link variables.
    const auto actual_start = tcp(forward(jointAngles(0)), 0);
    const auto actual_goal = tcp(forward(jointAngles(num_states_ - 1)), 0);
    json::object errors{
        {"start_position", (actual_start.translation() - start_.translation()).norm()},
        {"start_rotation", Rot3::Logmap(start_.rotation().between(actual_start.rotation())).norm()},
        {"goal_position", (actual_goal.translation() - goal_.translation()).norm()},
        {"goal_rotation", Rot3::Logmap(goal_.rotation().between(actual_goal.rotation())).norm()}};
    const bool reached = errors.at("start_position").as_double() < kEndpointPositionTolerance &&
                         errors.at("goal_position").as_double() < kEndpointPositionTolerance &&
                         errors.at("start_rotation").as_double() < kEndpointRotationTolerance &&
                         errors.at("goal_rotation").as_double() < kEndpointRotationTolerance;
    return {{"type", type}, {"iteration", iteration_}, {"error", graph_.error(values_)},
            {"converged", converged_ && reached}, {"done", done_},
            {"stop_reason", converged_ && !reached ? "stalled" : stop_reason_},
            {"targets_reached", reached}, {"duration", duration_}, {"states", states},
            {"graph", graph_info_}, {"endpoint_error", errors}};
  }
};
}  // namespace

int main(int argc, char** argv) {
  // Library diagnostics must never corrupt the line-delimited JSON protocol.
  std::ostream protocol(std::cout.rdbuf());
  std::cout.rdbuf(std::cerr.rdbuf());
  try {
    Demo demo(argc > 1 ? argv[1] : MOTION_PLANNING_PANDA_URDF);
    std::string line;
    while (std::getline(std::cin, line)) {
      json::value request_id = nullptr;
      try {
        if (line.size() > 65536) throw std::invalid_argument("request exceeds 64 KiB");
        const auto request = json::parse(line).as_object();
        if (const auto* id = request.if_contains("request_id")) request_id = *id;
        const auto command = request.at("command").as_string();
        json::object response;
        if (command == "describe") response = demo.describe();
        else if (command == "build") response = demo.build(request);
        else if (command == "targets") response = demo.targets(request);
        else if (command == "iterate") response = demo.iterate();
        else throw std::invalid_argument("unknown command");
        response["request_id"] = request_id;
        protocol << json::serialize(response) << '\n' << std::flush;
      } catch (const std::exception& error) {
        protocol << json::serialize(json::object{{"type", "error"},
            {"request_id", request_id}, {"message", error.what()}}) << '\n' << std::flush;
      }
    }
  } catch (const std::exception& error) {
    std::cerr << "Motion planning backend initialization failed: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
