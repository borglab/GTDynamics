/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ObjectiveFactors.h
 * @brief Support for adding PriorFactors to specify objectives.
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/utils/values.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <iostream>

namespace gtdynamics {

/**
 * @brief Add an pose objective to graph for the link i at time k.
 *
 * @param graph to add to.
 * @param pose target pose.
 * @param model noise model used in factor.
 * @param i The link id.
 * @param k Time step index (default 0).
 */
void add_pose_objective(gtsam::NonlinearFactorGraph* graph,
                        const gtsam::Pose3& pose,
                        const gtsam::SharedNoiseModel& model, int i,
                        int k = 0) {
  graph->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      internal::PoseKey(i, k), pose, model);
}

/**
 * @brief Add pose/twist objective to graph for the link i at time k.
 *
 * @param graph to add to.
 * @param pose target pose.
 * @param pose_model noise model used for pose factor.
 * @param twist target twist.
 * @param twist_model noise model used for twist factor.
 * @param i The link id.
 * @param k Time step index (default 0).
 */
void add_link_objective(gtsam::NonlinearFactorGraph* graph,
                        const gtsam::Pose3& pose,
                        const gtsam::SharedNoiseModel& pose_model,
                        const gtsam::Vector6& twist,
                        const gtsam::SharedNoiseModel& twist_model, int i,
                        int k = 0) {
  add_pose_objective(graph, pose, pose_model, i, k);
  graph->emplace_shared<gtsam::PriorFactor<gtsam::Vector6>>(
      internal::TwistKey(i, k), twist, twist_model);
}

/**
 * @brief Add twist+derivatives objective to graph for the link i at time k.
 *
 * @param graph to add to.
 * @param twist target twist.
 * @param twist_model noise model used for twist.
 * @param twist_acceleration target twist acceleration.
 * @param twist_acceleration_model noise model used for twist acceleration.
 * @param i The link id.
 * @param k Time step index (default 0).
 */
void add_twist_objective(
    gtsam::NonlinearFactorGraph* graph, const gtsam::Vector6& twist,
    const gtsam::SharedNoiseModel& twist_model,  //
    const gtsam::Vector6& twist_acceleration,
    const gtsam::SharedNoiseModel& twist_acceleration_model,  //
    int i, int k = 0) {
  graph->emplace_shared<gtsam::PriorFactor<gtsam::Vector6>>(
      internal::TwistKey(i, k), twist, twist_model);
  graph->emplace_shared<gtsam::PriorFactor<gtsam::Vector6>>(
      internal::TwistAccelKey(i, k), twist_acceleration,
      twist_acceleration_model);
}
}  // namespace gtdynamics
