/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file CartPoleWithLimitsRetractor.cpp
 * @brief CartPoleWithLimitsRetractor implementation.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/scenarios/CartPoleWithLimitsRetractor.h>
#include <gtdynamics/scenarios/IECartPoleWithLimits.h>
#include <gtdynamics/utils/Initializer.h>

#include <iomanip>

namespace gtdynamics {
using namespace gtsam;

IEConstraintManifold CartPoleWithLimitsRetractor::retract(
    const IEConstraintManifold *manifold, const VectorValues &delta,
    const std::optional<IndexSet> &blocking_indices,
    IERetractionInfo *retract_info) const {
  Values new_values = manifold->values().retract(delta);

  // delta.print("==============================delta============================\n",
  // GTDKeyFormatter);

  size_t k = DynamicsSymbol(new_values.keys().front()).time();
  IndexSet active_indices;

  // limit x to bounded values
  double x = new_values.atDouble(JointAngleKey(cp_.p_joint_id_, k));
  if (x < cp_.x_min_ || blocking_indices && blocking_indices->exists(0)) {
    x = cp_.x_min_;
    active_indices.insert(0);
  }
  if (x > cp_.x_max_ || blocking_indices && blocking_indices->exists(1)) {
    x = cp_.x_max_;
    active_indices.insert(1);
  }

  // limit f to bounded values
  double f = new_values.atDouble(TorqueKey(cp_.p_joint_id_, k));
  if (f < cp_.f_min_ || blocking_indices && blocking_indices->exists(2)) {
    f = cp_.f_min_;
    active_indices.insert(2);
  }
  if (f > cp_.f_max_ || blocking_indices && blocking_indices->exists(3)) {
    f = cp_.f_max_;
    active_indices.insert(3);
  }

  // compute rest values
  double q = new_values.atDouble(JointAngleKey(cp_.r_joint_id_, k));
  double w = new_values.atDouble(JointVelKey(cp_.r_joint_id_, k));
  double v = new_values.atDouble(JointVelKey(cp_.p_joint_id_, k));
  Values known_values;
  InsertJointAngle(&known_values, cp_.p_joint_id_, k, x);
  InsertJointAngle(&known_values, cp_.r_joint_id_, k, q);
  InsertJointVel(&known_values, cp_.p_joint_id_, k, v);
  InsertJointVel(&known_values, cp_.r_joint_id_, k, w);
  known_values = cp_.robot_.forwardKinematics(known_values, k);
  InsertTorque(&known_values, cp_.p_joint_id_, k, f);
  InsertTorque(&known_values, cp_.r_joint_id_, k, 0.0);
  Values result = cp_.graph_builder_.linearSolveFD(cp_.robot_, k, known_values);

  // result.print("==============================result============================\n",
  // GTDKeyFormatter);

  return manifold->createWithNewValues(result, active_indices);
}

} // namespace gtdynamics
