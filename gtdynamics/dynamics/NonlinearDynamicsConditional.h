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

/**
 * A non-linear conditional functions as the node in a Bayes network
 * It has a set of parents y,z, etc. and implements a probability density on x.
 */
class NonlinearDynamicsConditional
    : public TorqueFactor,
      public Conditional<TorqueFactor, NonlinearDynamicsConditional> {
 public:
  typedef NonlinearDynamicsConditional This;           ///< Typedef to this class
  typedef boost::shared_ptr<This> shared_ptr;  ///< shared_ptr to this class
  typedef TorqueFactor BaseFactor;  ///< Typedef to our factor base class
  typedef Conditional<BaseFactor, This>
      BaseConditional;  ///< Typedef to our conditional base class

  /** default constructor needed for serialization */
  NonlinearDynamicsConditional() {}

};  // NonlinearDynamicsConditional

}  // namespace gtdynamics
