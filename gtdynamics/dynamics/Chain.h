/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Chain.h
 * @brief Create a serial kinematic chain.
 * @author Dan Barladeanu, Frank Dellaert.
 */


#pragma once

//#include <gtsam/linear/NoiseModel.h>
//#include <gtsam/nonlinear/NonlinearFactorGraph.h>
//#include <gtsam/nonlinear/Values.h>
#include <boost/optional.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

using gtsam::Pose3;
using gtsam::Matrix;
using gtsam::Vector;

namespace gtdynamics {

/**
 * Chain is a class used to create a serial kinematic chain, which 
 * helps in creating a LeanDynamicsGraph
 */
class Chain {
 private:
  Pose3 sMb_; //rest pose of "body" with respect to "spatial" frame
  Matrix axes_; //

 public:
  /// Constructor
  Chain(Pose3 &sMb, Matrix &axes) : sMb_(sMb), axes_(axes) {}

  static void monoidOperation(const Pose3 &aTb, const Matrix &bAj, const Pose3 &bTc,
                       const Matrix &cAk, Pose3 &aTc, Matrix &cAjk);

  static Chain compose(std::vector<Chain> &chain_vector);

  inline const Pose3 &sMb() const { return sMb_; }

  inline const Matrix &axes() const { return axes_; }

  Pose3 poe(const Vector &q, boost::optional<Pose3 &> fTe = boost::none,
            boost::optional<Matrix &> J = boost::none);
};

}