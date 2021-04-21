/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PneumaticActuatorFactor.h
 * @brief Pneumatic factors.
 * @Author: Yetong Zhang
 */

// TODO(yetong): split into different files

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/nonlinear/ExpressionFactor.h>

#include <boost/optional.hpp>
#include <iostream>
#include <string>

namespace gtdynamics {

double multDouble1(const double& d1, const double& d2,
                   gtsam::OptionalJacobian<1, 1> H1,
                   gtsam::OptionalJacobian<1, 1> H2) {
  if (H1) *H1 = gtsam::I_1x1 * d2;
  if (H2) *H2 = gtsam::I_1x1 * d1;
  return d1 * d2;
}

void AddSourceMassCollocationFactor(
    gtsam::NonlinearFactorGraph& graph,
    const std::vector<gtsam::Key>& mdot_prev_keys,
    const std::vector<gtsam::Key>& mdot_curr_keys,
    gtsam::Key source_mass_key_prev, gtsam::Key source_mass_key_curr,
    gtsam::Key dt_key, bool isEuler,
    const gtsam::noiseModel::Base::shared_ptr& cost_model) {
  gtsam::Double_ expr0(source_mass_key_prev);
  gtsam::Double_ expr1(source_mass_key_curr);
  gtsam::Double_ expr_dt(dt_key);
  std::vector<gtsam::Double_> mdot0dt_vec;
  std::vector<gtsam::Double_> mdot1dt_vec;
  for (int idx = 0; idx < mdot_prev_keys.size(); idx++) {
    gtsam::Double_ mdot_prev(mdot_prev_keys[idx]);
    gtsam::Double_ mdot_curr(mdot_curr_keys[idx]);
    mdot0dt_vec.push_back(gtsam::Double_(multDouble1, expr_dt, mdot_prev));
    mdot1dt_vec.push_back(gtsam::Double_(multDouble1, expr_dt, mdot_curr));
  }

  if (isEuler) {
        graph.add(gtsam::ExpressionFactor(cost_model, 0.0, expr0 - mdot0dt_vec[0] - mdot0dt_vec[1] - mdot0dt_vec[2] - mdot0dt_vec[3] - expr1));
  } else {
        graph.add(gtsam::ExpressionFactor(cost_model, 0.0, expr0 - 0.5 * mdot1dt_vec[0] - 0.5 * mdot1dt_vec[1] - 0.5 * mdot1dt_vec[2] - 0.5 * mdot1dt_vec[3]
                                        - 0.5 * mdot1dt_vec[0] - 0.5 * mdot1dt_vec[1] - 0.5 * mdot1dt_vec[2] - 0.5 * mdot1dt_vec[3] - expr1));
  }
}

void AddTimeCollocationFactor(
  gtsam::NonlinearFactorGraph& graph,
  gtsam::Key t_prev_key, gtsam::Key t_curr_key, gtsam::Key dt_key,
  const gtsam::noiseModel::Base::shared_ptr& cost_model
) {
    gtsam::Double_ t_curr_expr(t_curr_key);
    gtsam::Double_ t_prev_expr(t_prev_key);
    gtsam::Double_ dt_expr(dt_key);
    gtsam::Double_ expr = t_curr_expr + dt_expr - t_prev_expr;
    graph.add(gtsam::ExpressionFactor(cost_model, 0.0, expr));
}

}