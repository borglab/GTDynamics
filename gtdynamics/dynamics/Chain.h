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
  Pose3 sMb_; //rest pose of "body" with respect to "spatial" frame.
  Matrix axes_; //screw axes of all joints in the chain expressed in body frame.

 public:
  /// Constructor
  Chain(Pose3 &sMb, Matrix &axes) : sMb_(sMb), axes_(axes) {}
  
  /**
   * This function performs the monoid operation on pose + jacobian pairs.
   * The monoid operation returns the composed chain pose and jacobian.
   * Detailed explanation in readme (chain.md)
   * @param aTb ............. chain 1 pose
   * @param bAj ............. chain 1 screw axes
   * @param bTc ............. chain 2 pose
   * @param cAk ............. chain 2 screw axes
   * @param aTc ............. composed chain pose
   * @param cAjk ............ composed chain jacobian
   */
  static void monoidOperation(const Pose3 &aTb, const Matrix &bAj, const Pose3 &bTc,
                       const Matrix &cAk, Pose3 &aTc, Matrix &cAjk);

  /**
   * This function returns a chain object composed from all chains in 
   * the input vector
   * @param chain_vector ........... Vector containing chains to compose 
   * @return ....................... Composed chain
   */
  static Chain compose(std::vector<Chain> &chain_vector);

  // Return sMb.
  inline const Pose3 &sMb() const { return sMb_; }

  // Return screw axes.
  inline const Matrix &axes() const { return axes_; }


  /**
   * Perform forward kinematics given q, return Pose of end-effector.
   * @param q ........... Input angles for all joints
   * @param fTe ......... The end-effector pose with respect to final link (Optional)
   * @param(out) J....... Empty matrix to be filled with the calculated Jacobian (Optional) 
   * @return ............ Pose of the end-effector calculated using Product of Exponentials
   */
  Pose3 poe(const Vector &q, boost::optional<Pose3 &> fTe = boost::none,
            boost::optional<Matrix &> J = boost::none);
};

}