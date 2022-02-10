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

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#include <boost/optional.hpp>

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/utils/values.h"

using gtsam::Matrix;
using gtsam::Pose3;
using gtsam::Vector;

namespace gtdynamics {

/**
 * Chain is a class used to create a serial kinematic chain, which
 * helps in creating a LeanDynamicsGraph
 */
class Chain {
 private:
  Pose3 sMb_;  // rest pose of "body" with respect to "spatial" frame.
  Matrix
      axes_;  // screw axes of all joints in the chain expressed in body frame.

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Default Constructor
  Chain() : sMb_(Pose3()), axes_(Matrix(6, 0)) {}

  /// Constructor
  Chain(const Pose3 &sMb, const Matrix &axes)
      : sMb_(sMb),
        axes_(axes.rows() == 6 ? axes
                               : throw std::runtime_error(
                                     "Jacobians should have 6 rows in SE(3)")) {
  }

  /// Construct only with sMb, set axes_ to empty Matrix
  Chain(const Pose3 &sMb) : sMb_(sMb), axes_(Matrix(6, 0)) {}

  /**
   * The * operator here composes two chains using the monoid operation on pose
   * + jacobian pairs. The monoid operation returns the composed chain pose and
   * jacobian. Detailed explanation in readme (chain.md)
   * @param chainA ............. chain 1 to compose
   * @param chainB ............. chain 2 to compose
   * @return ....................... Composed chain
   */
  friend Chain operator*(const Chain &chainA, const Chain &chainB);

  /**
   * This function returns a chain object composed from all chains in
   * the input vector
   * @param chain_vector ........... Vector containing chains to compose
   * @return ....................... Composed chain
   */
  static Chain compose(std::vector<Chain> &chains);

  // Return sMb.
  inline const Pose3 &sMb() const { return sMb_; }

  // Return screw axes.
  inline const Matrix &axes() const { return axes_; }

  // Return number of columns in axes_ matrix
  const size_t length() const { return axes_.cols(); }

  /**
   * Perform forward kinematics given q, return Pose of end-effector and
   * optionally the Jacobian.
   * @param q ........... Input angles for all joints
   * @param fTe ......... The end-effector pose with respect to final link
   * (Optional)
   * @param(out) J....... Empty matrix to be filled with the calculated Jacobian
   * (Optional)
   * @return ............ Pose of the end-effector calculated using Product of
   * Exponentials
   */
  Pose3 poe(const Vector &q, boost::optional<Pose3 &> fTe = boost::none,
            gtsam::OptionalJacobian<-1, -1> J = boost::none);

  /**
   * This function implements the Dynamical dependency between the
   * joint torques and the wrench applied on the body FOR A 3-LINK CHAIN (tau =
   * J*F) The input wrench is the wrench applied on the body by the joint
   * closest to the body. This equation is true in the case of massless links in
   * the chain. Detailed explanation in readme (chain.md)
   *
   * @param wrench .................. Wrench applied on the body by the joint
   * closest to it in the chain.
   * @param angles .................. Angles of the joints in the chain.
   * @param torques ................. Torques applied by the joints.
   * @return ........................ Vector of difference.
   */
  Vector DynamicalEquality3(const Vector &wrench, const Vector &angles,
                            const Vector &torques);

  // Helper function to create expression with a vector, used in
  // ChainConstraint3.
  Vector MakeVector(const double value0, const double value1,
                    const double value2) {
    Vector q(3);
    q << value0, value1, value2;
    return q;
  }

  /**
   * This function creates a gtsam expression of the Chain constraint FOR A
   * 3-LINK CHAIN.
   *
   * @param joints ............... Vector of joints in the kinematic chain, FROM
   * END-EFFECTOR TO BODY (first element in the vector is the joint whos child
   * is the end-effector).
   * @param wrench_key ........... Key of the wrench applied on the body by the
   * joint closest to the body.
   * @param k .................... Time slice.
   * @return ..................... GTSAM expression of the chain constraint.
   */
  gtsam::Vector3_ ChainConstraint3(const std::vector<JointSharedPtr> &joints,
                                   const gtsam::Key wrench_key, size_t k);
};

}  // namespace gtdynamics
