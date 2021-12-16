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
}  // namespace gtdynamics
