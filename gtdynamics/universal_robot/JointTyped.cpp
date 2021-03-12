/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  JointTyped.cpp
 * @brief Implementation of JointTyped factors functions
 * @author Frank Dellaert, Mandy Xie, Alejandro Escontrela, Yetong Zhang
 */

#include "gtdynamics/universal_robot/JointTyped.h"

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "gtdynamics/factors/PoseFactor.h"
#include "gtdynamics/factors/TorqueFactor.h"
#include "gtdynamics/factors/TwistAccelFactor.h"
#include "gtdynamics/factors/TwistFactor.h"
#include "gtdynamics/factors/WrenchEquivalenceFactor.h"
#include "gtdynamics/factors/WrenchPlanarFactor.h"
#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/RobotTypes.h"

namespace gtdynamics {

gtsam::NonlinearFactorGraph JointTyped::qFactors(
    size_t t, const OptimizerSetting &opt) const {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<PoseFactor>(
      PoseKey(parent_link_->id(), t), PoseKey(child_link_->id(), t),
      JointAngleKey(id(), t), opt.p_cost_model, shared());
  return graph;
}

gtsam::NonlinearFactorGraph JointTyped::vFactors(
    size_t t, const OptimizerSetting &opt) const {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<TwistFactor>(
      TwistKey(parent_link_->id(), t), TwistKey(child_link_->id(), t),
      JointAngleKey(id(), t), JointVelKey(id(), t), opt.v_cost_model,
      shared());

  return graph;
}

gtsam::NonlinearFactorGraph JointTyped::aFactors(
    size_t t, const OptimizerSetting &opt) const {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<TwistAccelFactor>(
      TwistKey(child_link_->id(), t), TwistAccelKey(parent_link_->id(), t),
      TwistAccelKey(child_link_->id(), t), JointAngleKey(id(), t),
      JointVelKey(id(), t), JointAccelKey(id(), t), opt.a_cost_model,
      boost::static_pointer_cast<const This>(shared()));

  return graph;
}

gtsam::NonlinearFactorGraph JointTyped::dynamicsFactors(
    size_t t, const OptimizerSetting &opt,
    const boost::optional<gtsam::Vector3> &planar_axis) const {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<WrenchEquivalenceFactor>(
      WrenchKey(parent_link_->id(), id(), t),
      WrenchKey(child_link_->id(), id(), t), JointAngleKey(id(), t),
      opt.f_cost_model,
      boost::static_pointer_cast<const This>(shared()));
  graph.emplace_shared<TorqueFactor>(
      WrenchKey(child_link_->id(), id(), t), TorqueKey(id(), t),
      opt.t_cost_model,
      boost::static_pointer_cast<const This>(shared()));
  if (planar_axis)
    graph.emplace_shared<WrenchPlanarFactor>(
        WrenchKey(child_link_->id(), id(), t), opt.planar_cost_model,
        *planar_axis);
  return graph;
}

}  // namespace gtdynamics
