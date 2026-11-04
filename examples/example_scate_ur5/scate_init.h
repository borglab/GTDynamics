/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_init.h
 * @author Harrison Jenkins
 *
 * Step 1 of manipulator SCATE (GPMP2-style): robot, start/goal, and the initial
 * trajectory guess. Header-only so both main() (scate_init.cpp) and the unit
 * tests (testScateInit.cpp) can use the same functions.
 *
 * Robot: the UR5 (6 joints). The base and end-effector link names are DERIVED
 * from the robot's joint tree (root link / leaf link), so nothing here depends
 * on how a particular URDF names its links.
 *
 * Everything that touches GTDynamics uses only calls that appear in the
 * KinematicsSlice.cpp / testKinematicsSlice.cpp.
 */

#pragma once

#include <gtdynamics/config.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>  // CreateRobotFromFile
#include <gtdynamics/utils/Slice.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "build_scate_graph.h"
#include "scate_keys.h"
#include "trajectory_init.h"

namespace scate {

// ============================================================
// 1. Problem configuration
// ============================================================

inline constexpr int kNumPoses = 21;  ///< x_0 ... x_N
inline constexpr int kNumControls = kNumPoses - 1;
inline constexpr double kDt = 1.0;  ///< time between states (s)

/**
 * Sideways joint-space bump of the initial path (rad). There is no obstacle
 * yet, so there is no side to commit to; set this once we place one.
 */
inline constexpr double kBumpAmplitude = 0.0;

/**
 * Joint acceleration limit (rad/s^2) for the control-limit factors. This is a
 * planning choice, not read from the robot model; set it from the UR5's
 * datasheet or your safety margin.
 */
inline constexpr double kAccelerationLimit = 1.0;

/**
 * Joint velocity limit (rad/s) for the velocity-limit factors, capped by each
 * joint's limit in the robot model (3.15 rad/s on the UR5). Like
 * kAccelerationLimit, a planning choice: it bounds how far the arm can move
 * between two collision checks.
 */
inline constexpr double kVelocityLimit = 0.5;

/// How close IK must get to the goal pose (translation in m, rotation in rad).
inline constexpr double kIkTolerance = 1e-3;

/// Waypoints used when a direct IK solve from the start misses the goal.
inline constexpr int kIkContinuationSteps = 10;

/**
 * Joint-space start: the common UR5 "home" pose. Wrist 2 (q5) is at -pi/2, away
 * from the wrist singularity at q5 = 0. Verified against the standard UR5 DH
 * model for nonsingularity; CHECK it against your model's joint limits (the
 * code prints a warning per violated limit).
 */
inline Vector startJoints() {
  Vector q(6);
  q << 0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0;
  return q;
}

/**
 * The goal is a TASK-SPACE pose (what you would normally be given). To
 * guarantee it is reachable, we take the FK of a joint configuration we then
 * "forget"; IK has to recover a configuration that reaches it. A 6-joint arm has
 * no redundancy, so IK has a handful of discrete solutions, and seeding from the
 * start keeps us on the branch near it.
 */
inline Vector hiddenGoalJoints() {
  Vector q(6);
  q << 0.6, -1.2, 1.3, -1.9, -1.3, 0.4;
  return q;
}

/// The UR5 model that ships with GTDynamics (models/urdfs/ur5/ur5.urdf).
inline std::string ur5UrdfPath() {
  return std::string(gtdynamics::kUrdfPath) + "ur5/ur5.urdf";
}

// ============================================================
// 2. Robot structure, derived from the joint tree
// ============================================================

/// Names of links that are never the child of a joint (the base).
inline std::vector<std::string> rootLinks(const gtdynamics::Robot& robot) {
  std::set<size_t> children;
  for (auto&& joint : robot.joints()) children.insert(joint->child()->id());
  std::vector<std::string> names;
  for (auto&& link : robot.links()) {
    if (!children.count(link->id())) names.push_back(link->name());
  }
  return names;
}

/// Names of links that are never the parent of a joint (the tip).
inline std::vector<std::string> leafLinks(const gtdynamics::Robot& robot) {
  std::set<size_t> parents;
  for (auto&& joint : robot.joints()) parents.insert(joint->parent()->id());
  std::vector<std::string> names;
  for (auto&& link : robot.links()) {
    if (!parents.count(link->id())) names.push_back(link->name());
  }
  return names;
}

/// Name of the base link (the first root link).
inline std::string baseLinkName(const gtdynamics::Robot& robot) {
  const auto roots = rootLinks(robot);
  if (roots.empty()) throw std::runtime_error("robot has no root link");
  return roots.front();
}

/// Name of the end-effector link (the first leaf link).
inline std::string endEffectorName(const gtdynamics::Robot& robot) {
  const auto leaves = leafLinks(robot);
  if (leaves.empty()) throw std::runtime_error("robot has no leaf link");
  return leaves.front();
}

/**
 * Joint ids in chain order, base to tip. Position p in this list is the index
 * into q (so q[0] is the joint attached to the base), and it does NOT assume
 * anything about how GTDynamics numbers the joints. Throws, with details, if
 * the robot is not a single serial chain.
 */
inline std::vector<size_t> jointIdsInChainOrder(const gtdynamics::Robot& robot) {
  std::vector<size_t> order;
  size_t current = robot.link(baseLinkName(robot))->id();
  for (size_t guard = 0; guard <= robot.joints().size(); ++guard) {
    bool advanced = false;
    for (auto&& joint : robot.joints()) {
      if (joint->parent()->id() == current) {
        order.push_back(joint->id());
        current = joint->child()->id();
        advanced = true;
        break;
      }
    }
    if (!advanced) break;
  }
  if (order.size() != robot.joints().size()) {
    std::ostringstream message;
    message << "robot is not a single serial chain: found " << order.size()
            << " joints along the chain from the base, but the robot has "
            << robot.joints().size();
    throw std::runtime_error(message.str());
  }
  return order;
}

/// Position of a joint id within the chain order (its index into q).
inline size_t chainIndex(const std::vector<size_t>& order, size_t joint_id) {
  const auto it = std::find(order.begin(), order.end(), joint_id);
  if (it == order.end()) throw std::runtime_error("joint id not in chain");
  return static_cast<size_t>(it - order.begin());
}

// ============================================================
// 3. Robot and kinematics helpers
// ============================================================

/**
 * Check that robot_file names an existing .urdf or .sdf file. GTDynamics' own
 * message for a bad path ("Invalid file extension") does not say what path it
 * received, so this one does.
 */
inline void validateRobotFile(const std::string& robot_file) {
  namespace fs = std::filesystem;
  if (robot_file.empty()) {
    throw std::runtime_error(
        "robot file path is empty (the default is ur5UrdfPath())");
  }
  if (!fs::is_regular_file(robot_file)) {
    throw std::runtime_error("robot file path is not an existing file: '" +
                             robot_file + "'");
  }
  const std::string extension = fs::path(robot_file).extension().string();
  if (extension != ".urdf" && extension != ".sdf") {
    throw std::runtime_error("robot file '" + robot_file + "' has extension '" +
                             extension + "'; GTDynamics loads .urdf or .sdf");
  }
}

/// Print how q is indexed: one line per joint, in chain order.
inline void printJointChain(const gtdynamics::Robot& robot,
                            const std::vector<size_t>& order) {
  std::cout << "Joint chain (q index: joint id, parent -> child):\n";
  for (size_t p = 0; p < order.size(); ++p) {
    for (auto&& joint : robot.joints()) {
      if (joint->id() == order[p]) {
        std::cout << "  q[" << p << "]: id " << static_cast<int>(joint->id()) << ", "
                  << joint->parent()->name() << " -> " << joint->child()->name()
                  << "\n";
      }
    }
  }
}

/**
 * Load the robot and anchor its base so only the arm moves (as in the
 * GTDynamics tests). q is indexed in chain order (see jointIdsInChainOrder).
 */
inline gtdynamics::Robot loadRobot(const std::string& robot_file) {
  validateRobotFile(robot_file);
  const gtdynamics::Robot raw = gtdynamics::CreateRobotFromFile(robot_file);
  const gtdynamics::Robot robot = raw.fixLink(baseLinkName(raw));
  // Validates that it is a serial chain, and shows how q is indexed.
  printJointChain(robot, jointIdsInChainOrder(robot));
  return robot;
}

/**
 * Kinematics solver; AUGMENTED_LAGRANGIAN so the IK goal pose is a hard
 * constraint, as in the PandaIK / BarLab tests.
 */
inline gtdynamics::Kinematics makeKinematics() {
  gtdynamics::KinematicsParameters parameters;
  parameters.method = gtdynamics::OptimizationParameters::Method::AUGMENTED_LAGRANGIAN;
  return gtdynamics::Kinematics(parameters);
}

/**
 * Joint angles q as per-joint values at slice k. Index p of q is the p-th
 * joint along the chain from the base.
 */
inline gtsam::Values jointValues(const gtdynamics::Robot& robot, const Vector& q, size_t k) {
  const auto order = jointIdsInChainOrder(robot);
  gtsam::Values values;
  for (size_t p = 0; p < order.size(); ++p) {
    values.insert(gtdynamics::JointAngleKey(order[p], k), q(p));
  }
  return values;
}

/**
 * Pose (wTcom) of the end-effector link for joint angles q, via the same
 * FK-backed initialValues() call used in the GTDynamics tests.
 */
inline gtsam::Pose3 endEffectorPose(const gtdynamics::Kinematics& kinematics,
                                    const gtdynamics::Robot& robot, const Vector& q,
                                    size_t k = 0) {
  const gtsam::Values forward = kinematics.initialValues(
      gtdynamics::Slice(k), robot, 0.0, jointValues(robot, q, k), true);
  return forward.at<gtsam::Pose3>(
      gtdynamics::PoseKey(robot.link(endEffectorName(robot))->id(), k));
}

// ============================================================
// 4. Inverse kinematics
// ============================================================

/**
 * Pose a fraction t of the way from a to b along the constant-twist (screw)
 * motion: a * Exp(t * Log(a^-1 b)). t = 0 gives a and t = 1 gives b.
 */
inline gtsam::Pose3 interpolatePose(const gtsam::Pose3& a, const gtsam::Pose3& b,
                                    double t) {
  const gtsam::Vector6 twist = gtsam::Pose3::Logmap(a.between(b));
  const gtsam::Vector6 scaled = t * twist;
  return a.compose(gtsam::Pose3::Expmap(scaled));
}

/// Outcome of one IK attempt, with the achieved error measured by us.
struct IkResult {
  Vector q;                  ///< joint angles found
  double translation_error;  ///< m, end-effector position error
  double rotation_error;     ///< rad, end-effector orientation error
  /// Whether both errors are within kIkTolerance.
  bool ok() const {
    return translation_error <= kIkTolerance && rotation_error <= kIkTolerance;
  }
};

/**
 * One IK attempt, seeded (and regularized) toward q_seed. The goal is either a
 * hard equality constraint (augmented Lagrangian) or a soft prior (LM).
 */
inline IkResult tryIK(const gtdynamics::Kinematics& kinematics,
                      const gtdynamics::Robot& robot, const gtsam::Pose3& wTcom_goal,
                      const Vector& q_seed, bool goal_as_constraint) {
  const size_t k = 0;
  auto end_effector = robot.link(endEffectorName(robot));
  gtdynamics::PoseGoals pose_goals = {
      {k, gtdynamics::PoseGoal(end_effector, gtsam::Pose3(), wTcom_goal)}};

  const gtsam::Values result =
      kinematics.inverse(gtdynamics::Slice(k), robot, pose_goals,
                         jointValues(robot, q_seed, k), goal_as_constraint);

  const gtsam::Pose3 achieved =
      result.at<gtsam::Pose3>(gtdynamics::PoseKey(end_effector->id(), k));

  IkResult attempt;
  const auto order = jointIdsInChainOrder(robot);
  attempt.q = Vector(order.size());
  for (size_t p = 0; p < order.size(); ++p) {
    attempt.q(p) = result.at<double>(gtdynamics::JointAngleKey(order[p], k));
  }
  attempt.translation_error = (achieved.translation() - wTcom_goal.translation()).norm();
  attempt.rotation_error =
      gtsam::Rot3::Logmap(wTcom_goal.rotation().between(achieved.rotation())).norm();
  return attempt;
}

/**
 * Continuation IK: solve for a sequence of waypoints from the pose at q_start
 * to the goal, seeding each solve with the previous solution. Each step is a
 * small, easy IK problem (like a seed a fraction of a radian from the answer),
 * which is what the solver handles well. Throws, naming the failing waypoint,
 * if any step misses by more than kIkTolerance.
 */
inline Vector solveGoalIKContinuation(const gtdynamics::Kinematics& kinematics,
                                      const gtdynamics::Robot& robot,
                                      const gtsam::Pose3& wTcom_goal,
                                      const Vector& q_start, int steps) {
  const gtsam::Pose3 start_pose = endEffectorPose(kinematics, robot, q_start);
  Vector q = q_start;
  for (int step = 1; step <= steps; ++step) {
    const double t = static_cast<double>(step) / steps;
    const gtsam::Pose3 waypoint =
        (step == steps) ? wTcom_goal : interpolatePose(start_pose, wTcom_goal, t);
    const IkResult attempt = tryIK(kinematics, robot, waypoint, q, true);
    if (!attempt.ok()) {
      std::ostringstream message;
      message << "failed at waypoint " << step << "/" << steps << " (trans "
              << attempt.translation_error << " m, rot " << attempt.rotation_error
              << " rad)";
      throw std::runtime_error(message.str());
    }
    q = attempt.q;
  }
  return q;
}

/**
 * Solve IK for a goal pose: a direct solve from the seed first, then
 * continuation (kIkContinuationSteps waypoints, then 4x as many). Throws, with
 * every attempt's error in the message, if all of them miss.
 */
inline Vector solveGoalIK(const gtdynamics::Kinematics& kinematics,
                          const gtdynamics::Robot& robot,
                          const gtsam::Pose3& wTcom_goal, const Vector& q_seed) {
  const IkResult direct = tryIK(kinematics, robot, wTcom_goal, q_seed, true);
  if (direct.ok()) return direct.q;

  std::ostringstream message;
  message << "IK did not reach the goal within " << kIkTolerance
          << " (m / rad). Direct attempt: trans " << direct.translation_error
          << " m, rot " << direct.rotation_error << " rad.";
  for (int steps : {kIkContinuationSteps, 4 * kIkContinuationSteps}) {
    try {
      const Vector q =
          solveGoalIKContinuation(kinematics, robot, wTcom_goal, q_seed, steps);
      std::cout << "  note: direct IK missed the goal (trans "
                << direct.translation_error << " m, rot " << direct.rotation_error
                << " rad); continuation with " << steps
                << " waypoints succeeded.\n";
      return q;
    } catch (const std::runtime_error& e) {
      message << " Continuation (" << steps << " waypoints): " << e.what() << ".";
    }
  }
  throw std::runtime_error(message.str());
}

/// Prints a warning per violated joint limit and returns how many there were.
inline int checkJointLimits(const gtdynamics::Robot& robot, const std::string& label,
                            const Vector& q) {
  int violations = 0;
  const auto order = jointIdsInChainOrder(robot);
  for (auto&& joint : robot.joints()) {
    const auto& limits = joint->parameters().scalar_limits;
    const size_t p = chainIndex(order, joint->id());
    const double angle = q(p);
    if (angle < limits.value_lower_limit || angle > limits.value_upper_limit) {
      ++violations;
      std::cout << "  WARNING: " << label << " q[" << p << "] = "
                << angle << " outside [" << limits.value_lower_limit << ", "
                << limits.value_upper_limit << "]\n";
    }
  }
  return violations;
}

// ============================================================
// 5. Initialization (mirrors the Python version's initialize_trajectory)
// ============================================================

/// Start, goal and initial guess of the offline problem.
struct InitialProblem {
  Vector q_start;              ///< joint-space start
  Vector q_goal;               ///< IK solution for the goal pose
  gtsam::Pose3 wTcom_goal;     ///< task-space goal
  JointTrajectory trajectory;  ///< initial guess
  gtsam::Values values;        ///< q_i, v_i, u_i under QKey/VKey/UKey
};

/// A trajectory as Values under QKey/VKey/UKey.
inline gtsam::Values trajectoryValues(const JointTrajectory& trajectory) {
  gtsam::Values values;
  for (size_t i = 0; i < trajectory.q.size(); ++i) {
    values.insert(QKey(i), trajectory.q[i]);
    values.insert(VKey(i), trajectory.v[i]);
  }
  for (size_t i = 0; i < trajectory.u.size(); ++i) {
    values.insert(UKey(i), trajectory.u[i]);
  }
  return values;
}

/**
 * Start at q_start; the goal is the task-space pose of q_hidden_goal, which
 * IK then has to recover (seeded from the start).
 */
inline InitialProblem initializeProblem(const gtdynamics::Kinematics& kinematics,
                                        const gtdynamics::Robot& robot,
                                        const Vector& q_start,
                                        const Vector& q_hidden_goal) {
  InitialProblem problem;
  const size_t n = robot.joints().size();

  problem.q_start = q_start;
  if (problem.q_start.size() != static_cast<long>(n)) {
    throw std::runtime_error(
        "startJoints() has " + std::to_string(problem.q_start.size()) +
        " entries but the robot has " + std::to_string(n) +
        " joints (is this the UR5 model, and does it include extra joints?)");
  }

  // Task-space goal, then IK to get a joint-space goal near the start.
  problem.wTcom_goal = endEffectorPose(kinematics, robot, q_hidden_goal);
  problem.q_goal = solveGoalIK(kinematics, robot, problem.wTcom_goal, problem.q_start);

  std::cout << "Base link: " << baseLinkName(robot)
            << ", end-effector link: " << endEffectorName(robot) << "\n";
  std::cout << "Start joints: " << problem.q_start.transpose() << "\n";
  std::cout << "Goal  joints (IK): " << problem.q_goal.transpose() << "\n";
  checkJointLimits(robot, "start", problem.q_start);
  checkJointLimits(robot, "goal", problem.q_goal);

  // Joint-space trajectory: smooth rest-to-rest, consistent with the
  // double-integrator dynamics factor.
  TrajectoryConfig config;
  config.num_poses = kNumPoses;
  config.dt = kDt;
  config.bump_amplitude = kBumpAmplitude;
  problem.trajectory = makeInitialTrajectory(problem.q_start, problem.q_goal, config);
  problem.values = trajectoryValues(problem.trajectory);
  return problem;
}

/// The default problem: startJoints() to the pose of hiddenGoalJoints().
inline InitialProblem initializeProblem(const gtdynamics::Kinematics& kinematics,
                                        const gtdynamics::Robot& robot) {
  return initializeProblem(kinematics, robot, startJoints(), hiddenGoalJoints());
}

// ============================================================
// 5b. Robot -> GraphSpec (the bridge to the GTSAM-only graph code)
// ============================================================

/// Joint position limits from the robot model, indexed in chain order like q.
inline void jointPositionLimits(const gtdynamics::Robot& robot, Vector* lower,
                                Vector* upper) {
  const auto order = jointIdsInChainOrder(robot);
  *lower = Vector(order.size());
  *upper = Vector(order.size());
  for (auto&& joint : robot.joints()) {
    const size_t p = chainIndex(order, joint->id());
    const auto& limits = joint->parameters().scalar_limits;
    (*lower)(p) = limits.value_lower_limit;
    (*upper)(p) = limits.value_upper_limit;
  }
}

/**
 * Velocity limit per joint in chain order: kVelocityLimit, or the robot
 * model's limit where that is lower.
 */
inline Vector jointVelocityLimits(const gtdynamics::Robot& robot) {
  const auto order = jointIdsInChainOrder(robot);
  Vector limits = Vector::Constant(order.size(), kVelocityLimit);
  for (auto&& joint : robot.joints()) {
    const size_t p = chainIndex(order, joint->id());
    limits(p) = std::min(limits(p), joint->parameters().velocity_limit);
  }
  return limits;
}

/// Everything build_scate_graph.h needs, taken from the robot and the problem.
inline GraphSpec makeGraphSpec(const gtdynamics::Robot& robot,
                               const InitialProblem& problem) {
  GraphSpec spec;
  spec.q_start = problem.q_start;
  spec.q_goal = problem.q_goal;
  jointPositionLimits(robot, &spec.q_lower, &spec.q_upper);
  spec.u_limit = Vector::Constant(problem.q_start.size(), kAccelerationLimit);
  spec.v_limit = jointVelocityLimits(robot);
  spec.num_poses = kNumPoses;
  spec.dt = kDt;
  return spec;
}

// ============================================================
// 6. Inspect the result
// ============================================================

/// Write one CSV row per state: index, q, v and the end-effector position.
inline void writeTrajectoryCsv(const gtdynamics::Kinematics& kinematics,
                               const gtdynamics::Robot& robot,
                               const JointTrajectory& trajectory,
                               const std::string& csv_path) {
  const size_t n = robot.joints().size();
  std::ofstream csv(csv_path);
  csv << "i";
  for (size_t j = 0; j < n; ++j) csv << ",q" << j;
  for (size_t j = 0; j < n; ++j) csv << ",v" << j;
  csv << ",ee_x,ee_y,ee_z\n";
  for (size_t i = 0; i < trajectory.q.size(); ++i) {
    const gtsam::Pose3 pose = endEffectorPose(kinematics, robot, trajectory.q[i], i);
    csv << i;
    for (size_t j = 0; j < n; ++j) csv << "," << trajectory.q[i](j);
    for (size_t j = 0; j < n; ++j) csv << "," << trajectory.v[i](j);
    csv << "," << pose.x() << "," << pose.y() << "," << pose.z() << "\n";
  }
}

/**
 * Print a summary of the problem's trajectory (sizes, dynamics residual,
 * peak velocity and acceleration, end-effector start/end/goal) and write it
 * to csv_path so it can be plotted.
 */
inline void report(const gtdynamics::Kinematics& kinematics,
                   const gtdynamics::Robot& robot, const InitialProblem& problem,
                   const std::string& csv_path = "initial_trajectory.csv") {
  const size_t n = robot.joints().size();
  const auto dynamics = makeDoubleIntegrator(static_cast<int>(n), kDt);
  const JointTrajectory& trajectory = problem.trajectory;

  std::cout << "\n--- Initial guess ---\n";
  std::cout << "States: " << kNumPoses << " (q and v each), controls: "
            << kNumControls << ", total variables: " << problem.values.size() << "\n";
  std::cout << "Max dynamics residual of the guess: "
            << maxDynamicsResidual(trajectory, dynamics) << "\n";

  double max_u = 0.0, max_v = 0.0;
  for (const Vector& u : trajectory.u) max_u = std::max(max_u, u.cwiseAbs().maxCoeff());
  for (const Vector& v : trajectory.v) max_v = std::max(max_v, v.cwiseAbs().maxCoeff());
  std::cout << "Peak |qdot| = " << max_v << " rad/s, peak |qddot| = " << max_u
            << " rad/s^2\n";

  // End-effector (link CoM) path, written to CSV so you can plot it.
  writeTrajectoryCsv(kinematics, robot, trajectory, csv_path);

  const gtsam::Pose3 first = endEffectorPose(kinematics, robot, trajectory.q.front());
  const gtsam::Pose3 last = endEffectorPose(kinematics, robot, trajectory.q.back());
  std::cout << "EE start position: " << first.translation().transpose() << "\n";
  std::cout << "EE end position:   " << last.translation().transpose() << "\n";
  std::cout << "EE goal position:  "
            << problem.wTcom_goal.translation().transpose() << "\n";
  std::cout << "EE end vs goal pose error (norm of Logmap): "
            << gtsam::Pose3::Logmap(last.between(problem.wTcom_goal)).norm() << "\n";
  std::cout << "Wrote " << csv_path << "\n";
}

}  // namespace scate
