/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WalkCycle.h
 * @brief Class to store walk cycle.
 * @author: Disha Das, Varun Agrawal
 */

#pragma once

#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <string>
#include <vector>

#include "gtdynamics/utils/Phase.h"
#include "gtdynamics/utils/initialize_solution_utils.h"

using gtsam::Pose3, gtsam::Vector3, gtsam::Vector6, gtsam::Vector,
    gtsam::Point3, gtsam::Rot3, gtsam::Sampler;

namespace gtdynamics {
/**
 * @class WalkCycle class stores the sequence of phases
 * in a walk cycle.
 */
class WalkCycle {
 protected:
  std::vector<gtdynamics::Phase> phases_;  ///< Phases in walk cycle
  ContactPoints contact_points_;  ///< All Contact points in the walk cycle

 public:
  /// Default Constructor
  WalkCycle() {}

  /**
   * @fn Adds phase in walk cycle
   * @param[in] phase      Phase object
   */
  void addPhase(const Phase& phase) {
    auto phase_contact_points = phase.getAllContactPoints();
    for (auto&& contact_point : phase_contact_points) {
      // If contact point is not present, add it
      if (contact_points_.find(contact_point.first) == contact_points_.end()) {
        contact_points_.emplace(contact_point.first, contact_point.second);
      } else {
        if (contact_points_[contact_point.first] != contact_point.second)
          throw std::runtime_error("Multiple Contact points for Link " +
                                   contact_point.first + " found!");
      }
    }
    phases_.push_back(phase);
  }

  /// Returns vector of phases in the walk cycle
  std::vector<Phase> phases() const { return phases_; }

  /// Returns count of phases in the walk cycle
  int numPhases() const { return phases_.size(); }

  /// Return all the contact points.
  ContactPoints allContactPoints() const { return contact_points_; }

  // gtsam::Values zeroValuesTrajectory(
  //     const Robot& robot, const int num_steps, const int num_phases,
  //     double gaussian_noise,
  //     const boost::optional<ContactPoints>& contact_points) {
  //   gtsam::Values z_values;
  //   for (int t = 0; t <= num_steps; t++)
  //     z_values.insert(ZeroValues(robot, t, gaussian_noise, contact_points));
  //   if (num_phases > 0) {
  //     for (int phase = 0; phase <= num_phases; phase++)
  //       z_values.insert(PhaseKey(phase), 0.0);
  //   }
  //   return z_values;
  // }

  // gtsam::Values multiPhaseZeroValuesTrajectory(
  //     const std::vector<Robot>& robots, const std::vector<int>& phase_steps,
  //     std::vector<gtsam::Values> transition_graph_init, double dt_i,
  //     double gaussian_noise,
  //     const std::vector<ContactPoints>& phase_contact_points =
  //         std::vector<ContactPoints>()) {
  //   gtsam::Values zero_values;
  //   int num_phases = robots.size();

  //   int t = 0;
  //   if (phase_contact_points.size() > 0) {
  //     zero_values.insert(
  //         ZeroValues(robots[0], t, gaussian_noise, phase_contact_points.at(0)));
  //   } else {
  //     zero_values.insert(ZeroValues(robots[0], t, gaussian_noise));
  //   }

  //   for (int phase = 0; phase < num_phases; phase++) {
  //     // in-phase
  //     for (int phase_step = 0; phase_step < phase_steps[phase] - 1;
  //          phase_step++) {
  //       if (phase_contact_points.size() > 0) {
  //         zero_values.insert(ZeroValues(robots[phase], ++t, gaussian_noise,
  //                                       phase_contact_points.at(phase)));
  //       } else {
  //         zero_values.insert(ZeroValues(robots[phase], ++t, gaussian_noise));
  //       }
  //     }

  //     if (phase == num_phases - 1) {
  //       if (phase_contact_points.size() > 0)
  //         zero_values.insert(ZeroValues(robots[phase], ++t, gaussian_noise,
  //                                       phase_contact_points.at(phase)));
  //       else
  //         zero_values.insert(ZeroValues(robots[phase], ++t, gaussian_noise));
  //     } else {
  //       t++;
  //       zero_values.insert(transition_graph_init[phase]);
  //     }
  //   }

  //   for (int phase = 0; phase < num_phases; phase++) {
  //     zero_values.insert(PhaseKey(phase), dt_i);
  //   }

  //   return zero_values;
  // }

  // // TODO(aescontrela3): Refactor this entire method to make use of
  // // InitializeSolutionInverseKinematics before merging.
  // gtsam::Values multiPhaseInverseKinematicsTrajectory(
  //     const std::vector<gtdynamics::Robot>& robots,
  //     const std::string& link_name, const std::vector<int>& phase_steps,
  //     const Pose3& wTl_i, const std::vector<Pose3>& wTl_t,
  //     const std::vector<double>& ts,
  //     std::vector<gtsam::Values> transition_graph_init, double dt_i,
  //     double gaussian_noise,
  //     const boost::optional<std::vector<gtdynamics::ContactPoints>>&
  //         phase_contact_points) {
  //   Point3 wPl_i = wTl_i.translation();  // Initial translation.
  //   Rot3 wRl_i = wTl_i.rotation();       // Initial rotation.
  //   int t_i = 0;                         // Time elapsed.
  //   Vector3 gravity(0, 0, -9.8);

  //   auto sampler_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
  //       Vector6::Constant(6, gaussian_noise));
  //   Sampler sampler(sampler_noise_model);

  //   std::vector<Pose3> wTl_dt = interpolatePoses(wTl_i, wTl_t, t_i, ts, dt_i);

  //   Pose3 wTl_i_processed;
  //   wTl_i_processed = addGaussianNoiseToPose(wTl_i, sampler);
  //   for (size_t i = 0; i < wTl_dt.size(); i++) {
  //     wTl_dt[i] = addGaussianNoiseToPose(wTl_dt[i], sampler);
  //   }

  //   // Iteratively solve the inverse kinematics problem while statisfying
  //   // the contact pose constraint.
  //   gtsam::Values init_vals, init_vals_t;

  //   // Initial pose and joint angles are known a priori.
  //   Robot::JointValues jangles, jvels;
  //   for (auto&& joint : robots[0].joints()) {
  //     jangles.insert(std::make_pair(joint->name(), sampler.sample()[0]));
  //     jvels.insert(std::make_pair(joint->name(), sampler.sample()[0]));
  //   }

  //   // Compute forward dynamics to obtain remaining link poses.
  //   auto fk_results =
  //       robots[0].forwardKinematics(jangles, jvels, link_name, wTl_i_processed);
  //   for (auto&& pose_result : fk_results.first)
  //     init_vals_t.insert(
  //         PoseKey(robots[0].getLinkByName(pose_result.first)->getID(), 0),
  //         pose_result.second);
  //   for (auto&& joint : robots[0].joints())
  //     init_vals_t.insert(JointAngleKey(joint->getID(), 0), sampler.sample()[0]);

  //   auto dgb = DynamicsGraph();

  //   int t = 0;
  //   int num_phases = robots.size();

  //   for (int phase = 0; phase < num_phases; phase++) {
  //     // In-phase.
  //     int curr_phase_steps = phase == (num_phases - 1) ? phase_steps[phase] + 1
  //                                                      : phase_steps[phase];
  //     for (int phase_step = 0; phase_step < curr_phase_steps; phase_step++) {
  //       gtsam::NonlinearFactorGraph kfg = dgb.qFactors(
  //           robots[phase], t, gravity, (*phase_contact_points)[phase]);

  //       kfg.add(gtsam::PriorFactor<Pose3>(
  //           PoseKey(robots[phase].getLinkByName(link_name)->getID(), t),
  //           wTl_dt[t], gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));

  //       gtsam::LevenbergMarquardtOptimizer optimizer(kfg, init_vals_t);
  //       gtsam::Values results = optimizer.optimize();

  //       init_vals.insert(results);

  //       // Update initial values for next timestep.
  //       init_vals_t.clear();
  //       for (auto&& link : robots[phase].links())
  //         init_vals_t.insert(
  //             gtdynamics::PoseKey(link->getID(), t + 1),
  //             results.at(gtdynamics::PoseKey(link->getID(), t)).cast<Pose3>());
  //       for (auto&& joint : robots[phase].joints())
  //         init_vals_t.insert(
  //             gtdynamics::JointAngleKey(joint->getID(), t + 1),
  //             results.atDouble(gtdynamics::JointAngleKey(joint->getID(), t)));
  //       t++;
  //     }
  //   }

  //   gtsam::Values zero_values = multiPhaseZeroValuesTrajectory(
  //       robots, phase_steps, transition_graph_init, dt_i, gaussian_noise,
  //       phase_contact_points);

  //   for (auto&& key_value_pair : zero_values)
  //     init_vals.tryInsert(key_value_pair.key, key_value_pair.value);

  //   return init_vals;
  // }
};
}  // namespace gtdynamics
