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
                 gtsam::OptionalJacobian<-1, -1> J) {
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

gtsam::Vector3 Chain::DynamicalEquality3(const gtsam::Vector6 &wrench,
                                         const gtsam::Vector3 &angles,
                                         const gtsam::Vector3 &torques,
                                         gtsam::OptionalJacobian<3, 6> J0,
                                         gtsam::OptionalJacobian<3, 3> J1,
                                         gtsam::OptionalJacobian<3, 3> J2) {
  Matrix J;
  poe(angles, boost::none, J);
  if (J0) {
    // derivative of difference with respect to wrench
    *J0 = J.transpose();
  }
  if (J1) {
    // derivative of difference with respect to angles
    // NOT COMPLETE
    *J1 = gtsam::I_3x3;
  }
  if (J2) {
    // derivative of difference with respect to torques
    *J2 = -gtsam::I_3x3;
  }

  return (J.transpose() * wrench - torques);
}

// Helper function to create expression with a vector, used in
// ChainConstraint3.
gtsam::Vector3 MakeVector3(const double &value0, const double &value1,
                           const double &value2,
                           gtsam::OptionalJacobian<3, 1> J0 = boost::none,
                           gtsam::OptionalJacobian<3, 1> J1 = boost::none,
                           gtsam::OptionalJacobian<3, 1> J2 = boost::none) {
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

gtsam::Vector3_ Chain::ChainConstraint3(
    const std::vector<JointSharedPtr> &joints, const gtsam::Key wrench_key,
    size_t k) {
  // Get Expression for wrench
  gtsam::Vector6_ wrench(wrench_key);

  // Get expression for joint angles as a column vector of size 3.
  gtsam::Vector3_ angles(MakeVector3,
                         gtsam::Double_(JointAngleKey(joints[0]->id(), k)),
                         gtsam::Double_(JointAngleKey(joints[1]->id(), k)),
                         gtsam::Double_(JointAngleKey(joints[2]->id(), k)));

  // Get expression for joint torques as a column vector of size 3.
  gtsam::Vector3_ torques(MakeVector3,
                          gtsam::Double_(TorqueKey(joints[0]->id(), k)),
                          gtsam::Double_(TorqueKey(joints[1]->id(), k)),
                          gtsam::Double_(TorqueKey(joints[2]->id(), k)));

  // Get expression of the dynamical equality
  gtsam::Vector3_ torque_diff(
      std::bind(&Chain::DynamicalEquality3, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4, std::placeholders::_5,
                std::placeholders::_6),
      wrench, angles, torques);

  return torque_diff;
}

}  // namespace gtdynamics
