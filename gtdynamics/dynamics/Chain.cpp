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
  // expect 6 rows in jacobians
  if (chainA.axes().rows() != 6 || chainB.axes().rows() != 6) {
    throw std::runtime_error("Jacobians should have 6 rows in SE(3)");
  }

  // Compose poses:
  Pose3 aTb = chainA.sMb();
  Pose3 bTc = chainB.sMb();
  Pose3 aTc = aTb.compose(bTc);

  // Adjoint the first Jacobian to the new end-effector frame C:
  Matrix c_Ad_b = bTc.inverse().AdjointMap();

  // Multiply with first chain screw axes:
  Matrix mult = c_Ad_b * chainA.axes();

  // Create Matrix with correct number of columns:
  Matrix out(mult.rows(), mult.cols() + chainB.axes().cols());

  // Fill Matrix:
  out << mult, chainB.axes();

  return Chain(aTc, out);
}

Chain Chain::compose(std::vector<Chain> &chain_vector) {
  // Initialize total chain
  Matrix emptyMat(6, 0);
  Pose3 emptyPose;
  Chain chain_total(emptyPose, emptyMat);

  // Perform monoid operations
  for (auto &&chain : chain_vector) {
    chain_total = chain_total * chain;
  }
  return chain_total;
}

Pose3 Chain::poe(const Vector &q, boost::optional<Pose3 &> fTe,
                 gtsam::OptionalJacobian<-1, -1> J) {
  // Check that input has good size
  if (q.size() != axes_.cols()) {
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
    Matrix Empty(6, 0);
    Chain chain_total(sMb_, Empty);
    for (int j = 0; j < q.size(); ++j) {
      // compose chain pose and screw axes to build poe and Jacobian
      Matrix axes_it(axes_.col(j));
      Pose3 pose_it(exp[j]);
      Chain chain_it(pose_it, axes_it);
      chain_total = chain_total * chain_it;
    }
    if (fTe) {
      // compose end-effector pose
      chain_total = chain_total * Chain(*fTe, Empty);
    }
    poe = chain_total.sMb();
    *J = chain_total.axes();
  }
  return poe;
}
}  // namespace gtdynamics
