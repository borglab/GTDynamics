/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Chain.cpp
 * @brief Create a serial kinematic chain.
 * @author Dan Barladeanu, Frank Dellaert.
 */

#include <gtdynamics/dynamics/Chain.h>

namespace gtdynamics {

Chain operator*(const Chain &chainA, const Chain &chainB) {
  // Compose poses:
  Pose3 aTb = chainA.sMb();
  Pose3 bTc = chainB.sMb();
  Pose3 aTc = aTb.compose(bTc);

  // Adjoint the first Jacobian to the new end-effector frame C:
  Matrix c_Ad_b = bTc.inverse().AdjointMap();

  // Create Matrix with correct number of columns:
  size_t length = chainA.length() + chainB.length();
  Matrix axes(6, length);

  // Fill Matrix:
  axes << c_Ad_b * chainA.axes(), chainB.axes();

  return Chain(aTc, axes);
}

Chain Chain::compose(std::vector<Chain> &chains) {
  // Initialize total chain
  Chain total_chain;

  // Perform monoid operations
  for (auto &&chain : chains) {
    total_chain = total_chain * chain;
  }
  return total_chain;
}

Pose3 Chain::poe(const Vector &q, boost::optional<Pose3 &> fTe,
                 gtsam::OptionalJacobian<-1, -1> J) const{
  // Check that input has good size
  if (q.size() != length()) {
    throw std::runtime_error(
        "number of angles in q different from number of cols in axes");
  }

  // Build vector of exponential maps to use in poe
  std::vector<Pose3> exp;
  for (int i = 0; i < q.size(); ++i) {
    exp.push_back(Pose3::Expmap(axes_.col(i) * q(i)));
  }

  Pose3 poe;
  if (!J) {
    // If J is not given, compute regular poe
    poe = sMb_;
    for (auto &expmap : exp) {
      poe = poe.compose(expmap);
    }
    if (fTe) {
      // compose end-effector pose
      poe = poe.compose(*fTe);
    }
  } else {
    // Compute FK + Jacobian with monoid compose.
    Chain chain(sMb_);
    for (int j = 0; j < q.size(); ++j) {
      chain = chain * Chain(exp[j], axes_.col(j));
    }
    if (fTe) {
      // compose end-effector pose
      chain = chain * Chain(*fTe);
    }
    poe = chain.sMb();
    *J = chain.axes();
  }
  return poe;
}

gtsam::Vector3 Chain::DynamicalEquality3(
    const gtsam::Vector6 &wrench, const gtsam::Vector3 &angles,
    const gtsam::Vector3 &torques, gtsam::OptionalJacobian<3, 6> H_wrench,
    gtsam::OptionalJacobian<3, 3> H_angles,
    gtsam::OptionalJacobian<3, 3> H_torques) const {
  Matrix J;
  poe(angles, boost::none, J);
  if (H_wrench) {
    // derivative of difference with respect to wrench
    *H_wrench = J.transpose();
  }
  if (H_angles) {
    // derivative of difference with respect to angles

    // The manipulator Jacobian is built such that in every step of chain
    // composition we multiply the existing screw axes with the adjoint map of
    // the inverse pose of the new chain, and then stack the screw axes of the
    // new chain horizontally in the axes matrix.
    // This means that the first angle actually doesn't get into the matrix at
    // all. The last column of the jacobian actually doesn't depend on the
    // angles at all, the second column depends only on the third angle, and the
    // first column depends on the second and third angles.
    // This means that the 3*3 jacobian has an upper triangular structure.
    Matrix A = gtsam::Z_3x3;

    // Calculate the Adjoint and take its derivative in relation to angles
    auto ad_J_angles1 = AdjointMapJacobianQ(angles(1), Pose3(), axes_.col(1));
    auto ad_J_angles2 = AdjointMapJacobianQ(angles(2), Pose3(), axes_.col(2));

    // Calculate the invers adjoint maps of the Poses, as we do in * operator
    Pose3 p1 = Pose3::Expmap(axes_.col(1) * angles(1));
    Pose3 p2 = Pose3::Expmap(axes_.col(2) * angles(2));
    auto ad_inv_p1 = p1.inverse().AdjointMap();
    auto ad_inv_p2 = p2.inverse().AdjointMap();

    // calculate the non-zero terms
    A(1, 2) = (ad_J_angles2 * axes_.col(1)).transpose() * wrench;
    A(0, 2) = (ad_J_angles2 * ad_inv_p1 * axes_.col(0)).transpose() * wrench;
    A(0, 1) = (ad_inv_p2 * ad_J_angles1 * axes_.col(0)).transpose() * wrench;

    *H_angles = A;
  }
  if (H_torques) {
    // derivative of difference with respect to torques
    *H_torques = -gtsam::I_3x3;
  }

  return (J.transpose() * wrench - torques);
}

gtsam::Vector3_ Chain::ChainConstraint3(
    const std::vector<JointSharedPtr> &joints, const gtsam::Key wrench_key,
    size_t k) const {
  // Get Expression for wrench
  gtsam::Vector6_ wrench_0_T(wrench_key);

  // The constraint is true for the wrench exerted on the end-effector frame, so
  // we need to adjoint from base to end-effector
  gtsam::Vector6_ wrench_0_H =
      (-1) * joints[0]->childAdjointWrench(wrench_0_T, k);
  gtsam::Vector6_ wrench_1_U =
      (-1) * joints[1]->childAdjointWrench(wrench_0_H, k);
  gtsam::Vector6_ wrench_2_L =
      (-1) * joints[2]->childAdjointWrench(wrench_1_U, k);

  // Get expression for joint angles as a column vector of size 3.
  gtsam::Double_ angle0(JointAngleKey(joints[0]->id(), k)),
      angle1(JointAngleKey(joints[1]->id(), k)),
      angle2(JointAngleKey(joints[2]->id(), k));
  gtsam::Vector3_ angles(MakeVector3, angle0, angle1, angle2);

  // Get expression for joint torques as a column vector of size 3.
  gtsam::Double_ torque0(TorqueKey(joints[0]->id(), k)),
      torque1(TorqueKey(joints[1]->id(), k)),
      torque2(TorqueKey(joints[2]->id(), k));
  gtsam::Vector3_ torques(MakeVector3, torque0, torque1, torque2);

  // Get expression of the dynamical equality
  gtsam::Vector3_ torque_diff(
      std::bind(&Chain::DynamicalEquality3, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4, std::placeholders::_5,
                std::placeholders::_6),
      wrench_2_L, angles, torques);

  return torque_diff;
}

}  // namespace gtdynamics
