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
    : public NoiseModelFactor,
      public Conditional<NoiseModelFactor, NonlinearDynamicsConditional> {
 public:
  typedef NonlinearDynamicsConditional This;   ///< Typedef to this class
  typedef boost::shared_ptr<This> shared_ptr;  ///< shared_ptr to this class
  typedef NoiseModelFactor BaseFactor;  ///< Typedef to our factor base class
  typedef Conditional<BaseFactor, This>
      BaseConditional;  ///< Typedef to our conditional base class

  /** default constructor needed for serialization */
  NonlinearDynamicsConditional() {}

  /** print */
  void print(const std::string& = "NonlinearDynamicsConditional",
             const KeyFormatter& formatter = DefaultKeyFormatter) const;

  /** equals function */
  bool equals(const NoiseModelFactor& cg, double tol = 1e-9) const;

};  // NonlinearDynamicsConditional

}  // namespace gtdynamics
