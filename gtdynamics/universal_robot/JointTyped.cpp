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
#include "gtdynamics/utils/values.h"

namespace gtdynamics {

gtsam::NonlinearFactorGraph
JointTyped::qFactors(size_t t, const OptimizerSetting &opt) const {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<PoseFactor>(internal::PoseKey(parent_link_->id(), t),
                                   internal::PoseKey(child_link_->id(), t),
                                   internal::JointAngleKey(id(), t),
                                   opt.p_cost_model, shared());
  return graph;
}

gtsam::NonlinearFactorGraph
JointTyped::vFactors(size_t t, const OptimizerSetting &opt) const {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<TwistFactor>(internal::TwistKey(parent_link_->id(), t),
                                    internal::TwistKey(child_link_->id(), t),
                                    internal::JointAngleKey(id(), t),
                                    internal::JointVelKey(id(), t),
                                    opt.v_cost_model, shared());

  return graph;
}

gtsam::NonlinearFactorGraph
JointTyped::aFactors(size_t t, const OptimizerSetting &opt) const {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<TwistAccelFactor>(
      internal::TwistKey(child_link_->id(), t),
      internal::TwistAccelKey(parent_link_->id(), t),
      internal::TwistAccelKey(child_link_->id(), t),
      internal::JointAngleKey(id(), t), internal::JointVelKey(id(), t),
      internal::JointAccelKey(id(), t), opt.a_cost_model,
      boost::static_pointer_cast<const This>(shared()));

  return graph;
}

gtsam::NonlinearFactorGraph JointTyped::dynamicsFactors(
    size_t t, const OptimizerSetting &opt,
    const boost::optional<gtsam::Vector3> &planar_axis) const {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<WrenchEquivalenceFactor>(
      internal::WrenchKey(parent_link_->id(), id(), t),
      internal::WrenchKey(child_link_->id(), id(), t),
      internal::JointAngleKey(id(), t), opt.f_cost_model,
      boost::static_pointer_cast<const This>(shared()));
  graph.emplace_shared<TorqueFactor>(
      internal::WrenchKey(child_link_->id(), id(), t),
      internal::TorqueKey(id(), t), opt.t_cost_model,
      boost::static_pointer_cast<const This>(shared()));
  if (planar_axis)
    graph.emplace_shared<WrenchPlanarFactor>(
        internal::WrenchKey(child_link_->id(), id(), t), opt.planar_cost_model,
        *planar_axis);
  return graph;
}

} // namespace gtdynamics
