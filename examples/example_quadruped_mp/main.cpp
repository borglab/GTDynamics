/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Kinematic motion planning for the vision 60 quadruped.
 * @author Alejandro Escontrela
 */

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using std::cout;
using std::endl;

using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Vector3;

typedef std::vector<gtsam::Vector> CoeffVector;
typedef std::map<int, std::map<std::string, Pose3>> TargetFootholds;
typedef std::map<std::string, Pose3> TargetPoses;

using namespace gtdynamics;

#define GROUND_HEIGHT -0.2

/**
 * Compute cubic polynomial coefficients via Hermite parameterization.
 *
 * @param wTb_i initial pose
 * @param wTb_f final pose
 * @param x_0_p TODO(frank): no idea
 * @param x_1_p TODO(frank): no idea
 *
 * @return 3*4 matrix of coefficients
 *
 * Refer to this lecture for more info on the hermite parameterization
 * for cubic polynomials:
 * http://www.cs.cmu.edu/afs/cs/academic/class/15462-s10/www/lec-slides/lec06.pdf
 */
CoeffVector compute_spline_coefficients(const Pose3 &wTb_i, const Pose3 &wTb_f,
                                        const Vector3 &x_0_p,
                                        const Vector3 &x_1_p,
                                        const double horizon) {
  Vector3 x_0 = wTb_i.translation();
  Vector3 x_1 = wTb_f.translation();

  // Hermite parameterization.
  Vector3 a_0 = x_0, a_1 = x_0_p;
  Vector3 a_2 = -std::pow(horizon, -2) *
                (3 * (x_0 - x_1) + horizon * (2 * x_0_p + x_1_p));
  Vector3 a_3 =
      std::pow(horizon, -3) * (2 * (x_0 - x_1) + horizon * (x_0_p + x_1_p));

  std::vector<gtsam::Vector> coeffs;
  coeffs.push_back(a_0);
  coeffs.push_back(a_1);
  coeffs.push_back(a_2);
  coeffs.push_back(a_3);

  return coeffs;
}

/**
 * Compute the robot base pose as defined by the hermite spline.
 * TODO(frank): document better
 */
Pose3 compute_hermite_pose(const CoeffVector &coeffs, const Vector3 &x_0_p,
                           const double t, const Pose3 &wTb_i) {
  // The position.
  Point3 p(coeffs[0] + coeffs[1] * t + coeffs[2] * std::pow(t, 2) +
           coeffs[3] * std::pow(t, 3));

  // The rotation.
  Point3 dpdt_v3 =
      Point3(coeffs[1] + 2 * coeffs[2] * t + 3 * coeffs[3] * std::pow(t, 2));
  dpdt_v3 = dpdt_v3 / dpdt_v3.norm();
  Point3 x_0_p_point(x_0_p);
  x_0_p_point = x_0_p_point / x_0_p_point.norm();

  Point3 axis = x_0_p_point.cross(dpdt_v3);
  double angle = std::acos(x_0_p_point.dot(dpdt_v3));
  Rot3 R = wTb_i.rotation() * Rot3::AxisAngle(axis, angle);

  return Pose3(R, p);
}

/** Compute the target footholds for each support phase. */
TargetFootholds
compute_target_footholds(const CoeffVector &coeffs, const Vector3 &x_0_p,
                         const Pose3 &wTb_i, const double horizon,
                         const double t_support,
                         const std::map<std::string, Pose3> &bTfs) {
  TargetFootholds target_footholds;

  double t_swing = t_support / 4.0; // Time for each swing foot trajectory.
  int n_support_phases = horizon / t_support;

  for (int i = 0; i <= n_support_phases; i++) {
    Pose3 wTb =
        compute_hermite_pose(coeffs, x_0_p, i * t_support / horizon, wTb_i);
    std::map<std::string, Pose3> target_footholds_i;
    for (auto &&bTf : bTfs) {
      Pose3 wTf = wTb * bTf.second;
      Pose3 wTf_gh(wTf.rotation(), Point3(wTf.translation()[0],
                                          wTf.translation()[1], GROUND_HEIGHT));
      target_footholds_i.emplace(bTf.first, wTf_gh);
    }
    target_footholds.emplace(i, target_footholds_i);
  }
  return target_footholds;
}

/** Get base pose and foot positions at any time t. */
TargetPoses compute_target_poses(const TargetFootholds &targ_footholds,
                                 const double horizon, const double t_support,
                                 const double t,
                                 const std::vector<std::string> &swing_sequence,
                                 const CoeffVector &coeffs,
                                 const Vector3 &x_0_p, const Pose3 &wTb_i) {
  TargetPoses t_poses;
  // Compute the body pose.
  Pose3 wTb = compute_hermite_pose(coeffs, x_0_p, t / horizon, wTb_i);
  t_poses.emplace("body", wTb);

  const std::map<std::string, Pose3> &prev_targ_foothold =
      targ_footholds.at(static_cast<int>(std::floor(t / t_support)));
  const std::map<std::string, Pose3> &next_targ_foothold =
      targ_footholds.at(static_cast<int>(std::ceil(t / t_support)));

  // Time spent in current support phase.
  double t_in_support = std::fmod(t, t_support);
  double t_swing = t_support / 4.0; // Duration of swing phase.

  int swing_leg_idx;
  if (t_in_support <= t_swing)
    swing_leg_idx = 0;
  else if (t_in_support <= (2 * t_swing))
    swing_leg_idx = 1;
  else if (t_in_support <= (3 * t_swing))
    swing_leg_idx = 2;
  else
    swing_leg_idx = 3;

  // Normalized swing duration.
  double t_normed = (t_in_support - swing_leg_idx * t_swing) / t_swing;

  // Already completed swing phase in this support phase.
  for (int i = 0; i < swing_leg_idx; i++) {
    std::string leg_i = swing_sequence[i];
    t_poses.emplace(leg_i, next_targ_foothold.at(leg_i));
  }

  // Currently swinging.
  std::string swing_leg = swing_sequence[swing_leg_idx];
  auto prev_foot_pos = prev_targ_foothold.at(swing_leg).translation();
  auto next_foot_pos = next_targ_foothold.at(swing_leg).translation();
  Point3 curr_foot_pos =
      prev_foot_pos + (next_foot_pos - prev_foot_pos) * t_normed;
  double h = GROUND_HEIGHT +
             0.2 * std::pow(t_normed, 1.1) * std::pow(1 - t_normed, 0.7);

  t_poses.emplace(swing_leg,
                  Pose3(Rot3(), Point3(curr_foot_pos[0], curr_foot_pos[1], h)));

  // Yet to complete swing phase in this support phase.
  for (int i = swing_leg_idx + 1; i < 4; i++) {
    std::string leg_i = swing_sequence[i];
    t_poses.emplace(leg_i, prev_targ_foothold.at(leg_i));
  }
  return t_poses;
}

struct CsvWriter {
  std::ofstream pose_file;
  const std::vector<std::string> &swing_sequence;
  const Robot robot;

  CsvWriter(const std::string &filename,
            const std::vector<std::string> &swing_sequence, const Robot &robot)
      : swing_sequence(swing_sequence), robot(robot) {
    pose_file.open(filename);
  }
  ~CsvWriter() { pose_file.close(); }

  void writeheader() {
    pose_file.open("../traj.csv");
    pose_file << "bodyx,bodyy,bodyz";
    for (auto &&leg : swing_sequence)
      pose_file << "," << leg << "x"
                << "," << leg << "y"
                << "," << leg << "z";
    for (auto &&joint : robot.joints())
      pose_file << "," << joint->name();
    pose_file << "\n";
  }

  void writerow(const TargetPoses &tposes, const gtsam::Values &results,
                int ti) {
    auto body = tposes.at("body").translation();
    pose_file << body[0] << "," << body[1] << "," << body[2];
    for (auto &&leg_name : swing_sequence) {
      auto leg = tposes.at(leg_name).translation();
      pose_file << "," << leg[0] << "," << leg[1] << "," << leg[2];
    }
    for (auto &&joint : robot.joints())
      pose_file << "," << JointAngle(results, joint->id(), ti);
    pose_file << "\n";
  }
};

int main(int argc, char **argv) {
  // Load the quadruped. Based on the vision 60 quadruped by Ghost robotics:
  // https://youtu.be/wrBNJKZKg10
  Robot vision60 = CreateRobotFromFile("../vision60.urdf");

  // Coordinate system:
  //  z
  //  |    y
  //  |   /
  //  |  /
  //  | /
  //  |/
  //  ‾‾‾‾‾‾‾‾‾‾‾‾‾ x

  cout << "\033[1;32;7;4mParsed Robot:\033[0m" << endl;
  vision60.print();
  cout << "-------------" << endl;

  // Compute coefficients for cubic spline from current robot position
  // to final position using hermite parameterization.
  Pose3 wTb_i = vision60.link("body")->wTcom();
  Pose3 wTb_f = Pose3(Rot3(), Point3(3, 0, 0.1));
  Point3 x_0_p(1, 0, 0);
  Point3 x_0_p_traj(1, 0, 0.4);
  Point3 x_1_p_traj(1, 0, 0);
  auto coeffs =
      compute_spline_coefficients(wTb_i, wTb_f, x_0_p_traj, x_1_p_traj, 1);

  // Time horizon.
  double horizon = 72;

  //                       Gait pattern
  //               (Shaded indicates swing phase)
  //               ------------------------------
  //            LH |#######                     | left hind
  //            RH |       #######              | right hind
  //            RF |              #######       | right forward
  //            LF |                     #######| left forward
  //               ------------------------------
  //             t = 0   normalized time (t)  t = 1
  std::vector<std::string> swing_sequence{"lower0", "lower1", "lower2",
                                          "lower3"};
  double t_support = 8; // Duration of a support phase.

  // Offsets from base to foot.
  std::map<std::string, Pose3> bTfs;
  Pose3 comTfoot =
      Pose3(Rot3(), Point3(0.14, 0, 0)); // Foot is 14cm along X in COM
  Pose3 bTw_i = wTb_i.inverse();
  for (auto &&leg : swing_sequence) {
    const Pose3 bTfoot = bTw_i * vision60.link(leg)->wTcom() * comTfoot;
    bTfs.emplace(leg, bTfoot);
  }

  // Calculate foothold at the end of each support phase.
  TargetFootholds targ_footholds =
      compute_target_footholds(coeffs, x_0_p, wTb_i, horizon, t_support, bTfs);

  // Iteratively solve the inverse kinematics problem to obtain joint angles.
  double dt = 1. / 240., curr_t = 0.0;
  int ti = 0; // The time index.
  auto dgb = DynamicsGraph();

  // Initialize values.
  gtsam::Values values;
  for (auto &&link : vision60.links())
    InsertPose(&values, link->id(), link->wTcom());
  for (auto &&joint : vision60.joints())
    InsertJointAngle(&values, joint->id(), 0.0);

  // Write body,foot poses and joint angles to csv file.
  CsvWriter writer("../traj.csv", swing_sequence, vision60);
  writer.writeheader();

  while (curr_t < horizon) {
    const TargetPoses tposes =
        compute_target_poses(targ_footholds, horizon, t_support, curr_t,
                             swing_sequence, coeffs, x_0_p, wTb_i);

    // Create factor graph of kinematics constraints.
    gtsam::NonlinearFactorGraph kfg = dgb.qFactors(vision60, ti);

    // Constrain the base pose using trajectory value.
    kfg.addPrior(internal::PoseKey(vision60.link("body")->id(), ti),
                 tposes.at("body"), gtsam::noiseModel::Constrained::All(6));

    // Constrain the footholds.
    auto model3 = gtsam::noiseModel::Constrained::All(3);
    for (auto &&leg : swing_sequence) {
      kfg.add(PointGoalFactor(internal::PoseKey(vision60.link(leg)->id(), ti),
                              model3, comTfoot, tposes.at(leg).translation()));
    }

    gtsam::GaussNewtonOptimizer optimizer(kfg, values);
    gtsam::Values results = optimizer.optimize();

    if ((ti % 100) == 0)
      cout << "iter: " << ti << ", err: " << kfg.error(results) << endl;

    // Update the values for next iteration.
    values.clear();
    for (auto &&link : vision60.links())
      InsertPose(&values, link->id(), ti + 1, Pose(results, link->id(), ti));
    for (auto &&joint : vision60.joints())
      InsertJointAngle(&values, joint->id(), ti + 1,
                       JointAngle(results, joint->id(), ti));

    writer.writerow(tposes, results, ti);
    curr_t = curr_t + dt;
    ti = ti + 1;
  }

  return 0;
}
