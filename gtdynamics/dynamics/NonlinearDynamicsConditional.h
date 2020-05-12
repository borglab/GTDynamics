/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearDynamicsConditional.h
 * @brief   Nonlinear conditional base class
 * @author  Mandy Xie
 */

#pragma once

#include <gtdynamics/factors/TorqueFactor.h>
#include <gtsam/global_includes.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>

namespace gtdynamics {
using gtsam::Conditional;
using gtsam::DefaultKeyFormatter;
using gtsam::KeyFormatter;
using gtsam::NoiseModelFactor;

/**
 * A non-linear conditional functions as the node in a Bayes network
 * It has a set of parents y,z, etc. and implements a probability density on x.
 */
class NonlinearDynamicsConditional
    : public TorqueFactor,
      public Conditional<TorqueFactor, NonlinearDynamicsConditional> {
 public:
  using This = NonlinearDynamicsConditional;   ///< typedef to this class
  using shared_ptr = boost::shared_ptr<This>;  ///< shared_ptr to this class
  using BaseFactor = TorqueFactor;  ///< typedef to our factor base class
  using BaseConditional =
      Conditional<BaseFactor, This>;  ///< Typedef to our conditional base class

  /** default constructor needed for serialization */
  NonlinearDynamicsConditional() {}

  /** constructor from factor */
  NonlinearDynamicsConditional(
      gtsam::Key wrench_key, gtsam::Key torque_key,
      const gtsam::noiseModel::Base::shared_ptr& cost_model,
      const gtsam::Vector6& screw_axis)
      : BaseFactor(wrench_key, torque_key, cost_model, screw_axis),
        BaseConditional(1) {}

  /** print */
  void print(const std::string& = "NonlinearDynamicsConditional",
             const KeyFormatter& formatter = DefaultKeyFormatter) const;

  /** equals function */
  bool equals(const TorqueFactor& cg, double tol = 1e-9) const;

};  // NonlinearDynamicsConditional

}  // namespace gtdynamics
