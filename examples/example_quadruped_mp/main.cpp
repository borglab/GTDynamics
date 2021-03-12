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

typedef std::vector<gtsam::Vector> CoeffVector;
typedef std::map<int, std::map<std::string, gtsam::Pose3>> TargetFootholds;
typedef std::map<std::string, gtsam::Pose3> TargetPoses;

using namespace gtdynamics;

#define GROUND_HEIGHT -0.2

/** Compute cubic polynomial coefficients via Hermite parameterization.
 *
 *
 * Refer to this lecture for more info on the hermite parameterization
 * for cubic polynomials:
 * http://www.cs.cmu.edu/afs/cs/academic/class/15462-s10/www/lec-slides/lec06.pdf
 */
CoeffVector compute_spline_coefficients(const gtsam::Pose3 &wTb_i,
                                        const gtsam::Pose3 &wTb_f,
                                        const gtsam::Vector3 &x_0_p,
                                        const gtsam::Vector3 &x_1_p,
                                        const double &th) {
  gtsam::Vector3 x_0 = wTb_i.translation();
  gtsam::Vector3 x_1 = wTb_f.translation();

  // Hermite parameterization.
  gtsam::Vector3 a_0 = x_0, a_1 = x_0_p;
  gtsam::Vector3 a_2 =
      -std::pow(th, -2) * (3 * (x_0 - x_1) + th * (2 * x_0_p + x_1_p));
  gtsam::Vector3 a_3 =
      std::pow(th, -3) * (2 * (x_0 - x_1) + th * (x_0_p + x_1_p));

  std::vector<gtsam::Vector> coeffs;
  coeffs.push_back(a_0);
  coeffs.push_back(a_1);
  coeffs.push_back(a_2);
  coeffs.push_back(a_3);

  return coeffs;
}

/** Compute the robot base pose as defined by the hermite spline. */
gtsam::Pose3 compute_hermite_pose(const CoeffVector &coeffs,
                                  const gtsam::Vector3 &x_0_p, const double &t,
                                  const gtsam::Pose3 &wTb_i) {
  // The position.
  gtsam::Point3 p =
      gtsam::Point3(coeffs[0] + coeffs[1] * t + coeffs[2] * std::pow(t, 2) +
                    coeffs[3] * std::pow(t, 3));

  // The rotation.
  gtsam::Point3 dpdt_v3 = gtsam::Point3(coeffs[1] + 2 * coeffs[2] * t +
                                        3 * coeffs[3] * std::pow(t, 2));
  dpdt_v3 = dpdt_v3 / dpdt_v3.norm();
  gtsam::Point3 x_0_p_point(x_0_p);
  x_0_p_point = x_0_p_point / x_0_p_point.norm();

  gtsam::Point3 axis = x_0_p_point.cross(dpdt_v3);
  double angle = std::acos(x_0_p_point.dot(dpdt_v3));
  gtsam::Rot3 R = wTb_i.rotation() * gtsam::Rot3::AxisAngle(axis, angle);

  return gtsam::Pose3(R, p);
}

/** Compute the target footholds for each support phase. */
TargetFootholds compute_target_footholds(
    const CoeffVector &coeffs, const gtsam::Vector3 &x_0_p,
    const gtsam::Pose3 &wTb_i, const double &th, const double &t_support,
    const std::map<std::string, gtsam::Pose3> &bTfs) {
  TargetFootholds target_footholds;

  double t_swing = t_support / 4.0;  // Time for each swing foot trajectory.
  int n_support_phases = th / t_support;

  for (int i = 0; i <= n_support_phases; i++) {
    gtsam::Pose3 wTb =
        compute_hermite_pose(coeffs, x_0_p, i * t_support / th, wTb_i);
    std::map<std::string, gtsam::Pose3> target_footholds_i;
    for (auto &&bTf : bTfs) {
      gtsam::Pose3 wTf = wTb * bTf.second;
      gtsam::Pose3 wTf_gh = gtsam::Pose3(
          wTf.rotation(), gtsam::Point3(wTf.translation()[0],
                                        wTf.translation()[1], GROUND_HEIGHT));
      target_footholds_i.insert(std::make_pair(bTf.first, wTf_gh));
    }
    target_footholds.insert(std::make_pair(i, target_footholds_i));
  }
  return target_footholds;
}

/** Get base pose and foot positions at any time t. */
TargetPoses compute_target_poses(TargetFootholds targ_footholds,
                                 const double &th, const double &t_support,
                                 const double &t,
                                 std::vector<std::string> swing_sequence,
                                 const CoeffVector &coeffs,
                                 const gtsam::Vector3 &x_0_p,
                                 const gtsam::Pose3 &wTb_i) {
  TargetPoses t_poses;
  // Compute the body pose.
  gtsam::Pose3 wTb = compute_hermite_pose(coeffs, x_0_p, t / th, wTb_i);
  t_poses.insert(std::make_pair("body", wTb));

  std::map<std::string, gtsam::Pose3> prev_targ_foothold =
      targ_footholds[static_cast<int>(std::floor(t / t_support))];
  std::map<std::string, gtsam::Pose3> next_targ_foothold =
      targ_footholds[static_cast<int>(std::ceil(t / t_support))];

  // Time spent in current support phase.
  double t_in_support = std::fmod(t, t_support);
  double t_swing = t_support / 4.0;  // Duration of swing phase.

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
  for (int i = 0; i < swing_leg_idx; i++)
    t_poses.insert(std::make_pair(swing_sequence[i],
                                  next_targ_foothold[swing_sequence[i]]));

  // Currently swinging.
  auto prev_foot_pos =
      prev_targ_foothold[swing_sequence[swing_leg_idx]].translation();
  auto next_foot_pos =
      next_targ_foothold[swing_sequence[swing_leg_idx]].translation();
  gtsam::Point3 curr_foot_pos =
      prev_foot_pos + (next_foot_pos - prev_foot_pos) * t_normed;
  double h = GROUND_HEIGHT +
             0.2 * std::pow(t_normed, 1.1) * std::pow(1 - t_normed, 0.7);

  t_poses.insert(std::make_pair(
      swing_sequence[swing_leg_idx],
      gtsam::Pose3(gtsam::Rot3(),
                   gtsam::Point3(curr_foot_pos[0], curr_foot_pos[1], h))));

  // Yet to complete swing phase in this support phase.
  for (int i = swing_leg_idx + 1; i < 4; i++)
    t_poses.insert(std::make_pair(swing_sequence[i],
                                  prev_targ_foothold[swing_sequence[i]]));
  return t_poses;
}

int main(int argc, char **argv) {
  // Load the quadruped. Based on the vision 60 quadruped by Ghost robotics:
  // https://youtu.be/wrBNJKZKg10
  Robot vision60 = Robot("../vision60.urdf");

  // Coordinate system:
  //  z
  //  |    y
  //  |   /
  //  |  /
  //  | /
  //  |/
  //  ‾‾‾‾‾‾‾‾‾‾‾‾‾ x

  std::cout << "\033[1;32;7;4mParsed Robot:\033[0m" << std::endl;
  vision60.print();
  std::cout << "-------------" << std::endl;

  // Compute coefficients for cubic spline from current robot position
  // to final position using hermite parameterization.
  gtsam::Pose3 wTb_i = vision60.link("body")->wTcom();
  gtsam::Pose3 wTb_f = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 0, 0.1));
  gtsam::Vector3 x_0_p = (gtsam::Vector(3) << 1, 0, 0).finished();
  gtsam::Vector3 x_0_p_traj = (gtsam::Vector(3) << 1.0, 0, 0.4).finished();
  gtsam::Vector3 x_1_p_traj = (gtsam::Vector(3) << 1, 0, 0).finished();
  auto coeffs =
      compute_spline_coefficients(wTb_i, wTb_f, x_0_p_traj, x_1_p_traj, 1);

  // Time horizon.
  double th = 72;

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
  double t_support = 8;  // Duration of a support phase.

  // Offsets from base to foot.
  std::map<std::string, gtsam::Pose3> bTfs;
  gtsam::Pose3 comTc = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.14, 0, 0));
  for (auto &&leg : swing_sequence)
    bTfs.insert(std::make_pair(
        leg, wTb_i.inverse() * (vision60.link(leg)->wTcom() * comTc)));

  // Calculate foothold at the end of each support phase.
  TargetFootholds targ_footholds =
      compute_target_footholds(coeffs, x_0_p, wTb_i, th, t_support, bTfs);

  // Iteratively solve the inverse kinematics problem to obtain joint angles.
  double dt = 1. / 240., curr_t = 0.0;
  int ti = 0;  // The time index.
  auto dgb = DynamicsGraph();

  // Initialize values.
  gtsam::Values values;
  for (auto &&link : vision60.links())
    values.insert(PoseKey(link->id(), 0), link->wTcom());
  for (auto &&joint : vision60.joints())
    values.insert(JointAngleKey(joint->id(), 0), 0.0);

  // Write body,foot poses and joint angles to csv file.
  std::ofstream pose_file;
  pose_file.open("../traj.csv");
  pose_file << "bodyx,bodyy,bodyz";
  for (auto &&leg : swing_sequence)
    pose_file << "," << leg << "x"
              << "," << leg << "y"
              << "," << leg << "z";
  for (auto &&joint : vision60.joints()) pose_file << "," << joint->name();
  pose_file << "\n";

  while (curr_t < th) {
    TargetPoses tposes =
        compute_target_poses(targ_footholds, th, t_support, curr_t,
                             swing_sequence, coeffs, x_0_p, wTb_i);

    pose_file << tposes["body"].translation()[0] << ","
              << tposes["body"].translation()[1] << ","
              << tposes["body"].translation()[2];
    for (auto &&leg : swing_sequence)
      pose_file << "," << tposes[leg].translation()[0] << ","
                << tposes[leg].translation()[1] << ","
                << tposes[leg].translation()[2];

    // Create factor graph of kinematics constraints.
    gtsam::NonlinearFactorGraph kfg = dgb.qFactors(vision60, ti);

    // Constrain the base pose using trajectory value.
    kfg.add(gtsam::PriorFactor<gtsam::Pose3>(
        PoseKey(vision60.link("body")->id(), ti), tposes["body"],
        gtsam::noiseModel::Constrained::All(6)));

    // Constrain the footholds.
    for (auto &&leg : swing_sequence)
      kfg.add(PointGoalFactor(PoseKey(vision60.link(leg)->id(), ti),
                              gtsam::noiseModel::Constrained::All(3), comTc,
                              tposes[leg].translation()));

    gtsam::GaussNewtonOptimizer optimizer(kfg, values);
    gtsam::Values results = optimizer.optimize();

    if ((ti % 100) == 0)
      std::cout << "iter: " << ti << ", err: " << kfg.error(results)
                << std::endl;

    // Update the values for next iteration.
    values.clear();
    for (auto &&link : vision60.links())
      values.insert(PoseKey(link->id(), ti + 1),
                    results.at<gtsam::Pose3>(PoseKey(link->id(), ti)));
    for (auto &&joint : vision60.joints())
      values.insert(JointAngleKey(joint->id(), ti + 1),
                    results.atDouble(JointAngleKey(joint->id(), ti)));

    for (auto &&joint : vision60.joints())
      pose_file << "," << results.atDouble(JointAngleKey(joint->id(), ti));

    pose_file << "\n";
    curr_t = curr_t + dt;
    ti = ti + 1;
  }
  pose_file.close();

  return 0;
}
