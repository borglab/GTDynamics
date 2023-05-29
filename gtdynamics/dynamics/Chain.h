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
#include <gtsam/slam/expressions.h>

#include <optional>

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/utils/utils.h"
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
  Pose3 poe(const Vector &q, std::optional<Pose3> fTe = {},
            gtsam::OptionalJacobian<-1, -1> J = {}) const;

  /**
   * This function implements the dynamic dependency between the
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
  gtsam::Vector3 DynamicalEquality3(
      const gtsam::Vector6 &wrench, const gtsam::Vector3 &angles,
      const gtsam::Vector3 &torques,
      gtsam::OptionalJacobian<3, 6> H_wrench = {},
      gtsam::OptionalJacobian<3, 3> H_angles = {},
      gtsam::OptionalJacobian<3, 3> H_torques = {}) const;

  /**
   * This function creates a gtsam expression of the Chain constraint FOR A
   * 3-LINK CHAIN.
   *
   * @param joints ............... Vector of joints in the kinematic chain,FROM
   *  BODY TO END-EFFECTOR.
   * @param wrench_key ........... Key of the wrench applied on the body by the
   * joint closest to the body.
   * @param k .................... Time slice.
   * @return ..................... GTSAM expression of the chain constraint.
   */
  gtsam::Vector3_ ChainConstraint3(const std::vector<JointSharedPtr> &joints,
                                   const gtsam::Key wrench_key, size_t k) const;

  /**
   * This function creates a gtsam expression of the End-Effector wrench using
   * a Chain under massless leg assumption and Product of Exponentials.
   *
   * @param joints ............... Vector of joints in the kinematic chain, FROM
   *  BODY TO END-EFFECTOR.
   * @param wrench_key ........... Key of the wrench applied on the body by the
   * joint closest to the body.
   * @param k .................... Time slice.
   * @return ..................... GTSAM expression of the chain constraint.
   */
  gtsam::Vector6_ AdjointWrenchConstraint3(
      const std::vector<JointSharedPtr> &joints,
      const gtsam::Key body_wrench_key, size_t k) const;

  /**
   *  This function calculates the end-effector wrench using POE and chain.
   *
   *
   *
   * @param angles .............. angles of the joints in the chain FROM
   *  BODY TO END-EFFECTOR.
   * @param wrench_body ......... wrench applied by the first joint in the chain
   * on the body link
   * @return ...................  gtsam expression of the  end-effector wrench
   */
  gtsam::Vector6 AdjointWrenchEquality3(
      const gtsam::Vector3 &angles, const gtsam::Vector6 &wrench_body,
      gtsam::OptionalJacobian<6, 3> H_angles = {},
      gtsam::OptionalJacobian<6, 6> H_wrench_body = {}) const;

  /**
   * This function creates a gtsam expression factor of the End-Effector pose
   * using Product of Exponentials and Chain (forward kinematics).
   *
   * @param joints ............... Vector of joints in the kinematic chain, FROM
   *  BODY TO END-EFFECTOR.
   * @param wTb_key............... Key of body link pose
   * @param wTe_key............... Key of end-effector pose
   * @param cost_model............ cost model for factor
   * @param k .................... Time slice.
   * @return ..................... GTSAM expression of the chain constraint.
   */
  gtsam::Vector6_ Poe3Factor(const std::vector<JointSharedPtr> &joints,
                             const gtsam::Key wTb_key, const gtsam::Key wTe_key,
                             size_t k) const;

  /**
   *  This function calculates the end-effector pose using POE and chain
   * (forward kinematics).
   *
   * @param angles .............. angles of the joints in the chain FROM
   *  BODY TO END-EFFECTOR.
   * @return ...................  gtsam expression of the  end-effector wrench
   */
  gtsam::Pose3 PoeEquality3(
      const gtsam::Vector3 &angles,
      gtsam::OptionalJacobian<6, 3> H_angles = {}) const;

};  // Chain class

// Helper function to create expression with a vector, used in
// ChainConstraint3.
inline gtsam::Vector3 MakeVector3(const double &value0, const double &value1,
                           const double &value2,
                           gtsam::OptionalJacobian<3, 1> J0 = {},
                           gtsam::OptionalJacobian<3, 1> J1 = {},
                           gtsam::OptionalJacobian<3, 1> J2 = {}) {
  gtsam::Vector3 q;
  q << value0, value1, value2;
  if (J0) {
    *J0 << 1.0, 0.0, 0.0;
  }
  if (J1) {
    *J1 << 0.0, 1.0, 0.0;
  }
  if (J2) {
    *J2 << 0.0, 0.0, 1.0;
  }
  return q;
}

}  // namespace gtdynamics
