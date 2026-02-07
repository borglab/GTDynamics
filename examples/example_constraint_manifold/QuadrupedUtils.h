/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  QuadrupedUtils.h
 * @brief Utilities for quadruped trajectory optimization experiments.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/cmopt/ManifoldOptimizer.h>
#include <gtdynamics/cmopt/MultiJacobian.h>
#include <gtdynamics/factors/ConstVarFactor.h>
#include <gtdynamics/factors/ContactPointFactor.h>
#include <gtdynamics/optimizer/MutableLMOptimizer.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtdynamics/utils/PointOnLink.h>
#include <gtsam/constrained/PenaltyOptimizer.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/expressions.h>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/dynamics/OptimizerSetting.h"

namespace gtdynamics {

/// Key to represent contact redundancy.
inline DynamicsSymbol ContactRedundancyKey(int t = 0) {
  return DynamicsSymbol::SimpleSymbol("CR", t);
}

/** For a quadruped in standing phase, provided with the configuration,
 * velocity, and accelerations, there are 6 degrees of freedom in dynamics.
 * The Contact redundancy constraint imposes additional constraints to make
 * the solution unique. */
gtsam::Vector6_ ContactRedundancyConstraint(int t,
                                            const std::vector<int> &contact_ids,
                                            const double &a, const double &b);
/** For a quadruped in standing phase, provided with the configuration,
 * velocity, and accelerations, there are 6 degrees of freedom in dynamics.
 * The Contact redundancy factor imposes additional costs to make the
 * optimal solution unique. */
gtsam::NoiseModelFactor::shared_ptr ContactRedundancyFactor(
    int t, const std::vector<int> &contact_ids, const double &a,
    const double &b, const gtsam::noiseModel::Base::shared_ptr &cost_model,
    bool express_redundancy = false);

/// q-level contact factors with fixed contact points.
NonlinearFactorGraph contact_q_factors(
    const int k, const PointOnLinks &contact_points,
    const std::vector<gtsam::Point3> &contact_in_world,
    const gtsam::noiseModel::Base::shared_ptr &cost_model);

/** Function to manually define the basis keys for each constraint manifold. */
KeyVector FindBasisKeys4C(const KeyVector &keys);

/** Function to manually define the basis keys for each constraint manifold. */
KeyVector findBasisKeysRedundancy(const KeyVector &keys);

/// Class of utilities for Vision60 robot.
class Vision60Robot {
 public:
  Robot robot = CreateRobotFromFile(kUrdfPath + std::string("vision60.urdf"));
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  double mu = 2.0;
  gtsam::Point3 contact_in_com = gtsam::Point3(0.14, 0, 0);
  PointOnLinks contact_points{
      PointOnLink(robot.link("lower0"), contact_in_com),
      PointOnLink(robot.link("lower1"), contact_in_com),
      PointOnLink(robot.link("lower2"), contact_in_com),
      PointOnLink(robot.link("lower3"), contact_in_com)};
  int lower0_id = robot.link("lower0")->id();
  int lower1_id = robot.link("lower1")->id();
  int lower2_id = robot.link("lower2")->id();
  int lower3_id = robot.link("lower3")->id();
  std::vector<int> contact_ids{lower0_id, lower1_id, lower2_id, lower3_id};
  LinkSharedPtr base_link = robot.link("body");
  int base_id = base_link->id();

  double nominal_height = Pose(getNominalConfiguration(), lower0_id, 0)
                              .transformFrom(contact_in_com)
                              .z() *
                          -1;
  Values nominal_values = getNominalConfiguration(nominal_height);

  std::vector<gtsam::Point3> contact_in_world{
      Pose(nominal_values, lower0_id, 0).transformFrom(contact_in_com),
      Pose(nominal_values, lower1_id, 0).transformFrom(contact_in_com),
      Pose(nominal_values, lower2_id, 0).transformFrom(contact_in_com),
      Pose(nominal_values, lower3_id, 0).transformFrom(contact_in_com)};
  double a = 0.5 * (contact_in_world.at(0).x() - contact_in_world.at(1).x());
  double b = 0.5 * (contact_in_world.at(0).y() - contact_in_world.at(2).y());

  double sigma_dynamics = 1e-2;   // Variance of dynamics constraints.
  double sigma_objectives = 1e0;  // Variance of additional objectives.
  gtsam::SharedNoiseModel des_pose_nm =
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  gtsam::SharedNoiseModel min_torque_nm =
      gtsam::noiseModel::Isotropic::Sigma(1, 1e1);
  gtsam::SharedNoiseModel cpoint_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics);
  gtsam::SharedNoiseModel redundancy_model =
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);

  DynamicsGraph graph_builder = DynamicsGraph(getOptSetting(), gravity);

  bool express_redundancy = false;

 protected:
  OptimizerSetting getOptSetting() const;

  Values getNominalConfiguration(const double height = 0) const;

  NonlinearFactorGraph getConstraintsGraphStepQ(const int t) const;

  NonlinearFactorGraph getConstraintsGraphStepV(const int t) const;

  NonlinearFactorGraph getConstraintsGraphStepAD(const int t) const;

 public:
  Vision60Robot() = default;

  /// Dynamics factors without friction cone factors (as they are moved to
  /// costs).
  NonlinearFactorGraph DynamicsFactors(
      const int k, const std::optional<PointOnLinks> &contact_points) const;

  /// Kinodynamic constraints at the specified time step.
  NonlinearFactorGraph getConstraintsGraphStep(const int t) const;

  /// The constraints only consist of the kinodynamic constraints at each time
  /// step.
  NonlinearFactorGraph getConstraintsGraphTrajectory(const int num_steps) const;

  /// Return values of one step satisfying kinodynamic constraints.
  Values getInitValuesStep(
      const int t = 0,
      const gtsam::Pose3 &base_pose = gtsam::Pose3(gtsam::Rot3::Identity(),
                                                   gtsam::Point3(0, 0, 0.0)),
      const gtsam::Vector6 &base_twist = gtsam::Vector6::Zero(),
      const gtsam::Vector6 &base_accel = gtsam::Vector6::Zero(),
      Values init_values_t = Values()) const;

  /// Return values of trajectory satisfying kinodynamic constraints.
  Values getInitValuesTrajectory(
      const size_t num_steps, double dt, const gtsam::Pose3 &base_pose_init,
      const std::vector<gtsam::Pose3> &des_poses,
      std::vector<double> &des_poses_t,
      const std::string initialization_technique = "interp") const;

  /// Costs for collocation across steps.
  NonlinearFactorGraph collocationCosts(const int num_steps, double dt) const;

  /// Costs for min torque objectives.
  NonlinearFactorGraph minTorqueCosts(const int num_steps) const;

  /// Costs for contact force within friction cone.
  NonlinearFactorGraph frictionConeCosts(const int num_steps) const;

  /// Costs for init condition and reaching target poses.
  NonlinearFactorGraph boundaryCosts(const gtsam::Pose3 &init_pose,
                                     const gtsam::Vector6 &init_twist,
                                     const std::vector<gtsam::Pose3> &des_poses,
                                     const std::vector<double> &des_poses_t,
                                     double dt) const;

  /// Print joint angles.
  void printJointAngles(const Values &values, int t = 0) const;

  /// Return optimizer setting.
  const OptimizerSetting &opt() const { return graph_builder.opt(); }

  /// Return function that select basis keys for constraint manifolds.
  BasisKeyFunc getBasisKeyFunc() const {
    if (express_redundancy) {
      return &findBasisKeysRedundancy;
    } else {
      return &FindBasisKeys4C;
    }
  }

  /// Save trajectory to file.
  void exportTrajectory(const Values &results, const size_t num_steps,
                        std::string file_path = "traj.csv");
};

}  // namespace gtdynamics
