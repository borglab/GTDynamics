/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TspaceBasis.h
 * @brief Basis for tangent space of constraint manifold. Detailed definition of
 * tangent space and basis are available at Boumal20book Sec.8.4.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/optimizer/ConnectedComponent.h>
#include <gtdynamics/optimizer/MultiJacobian.h>
#include <gtdynamics/optimizer/MutableLMOptimizer.h>
#include <gtdynamics/optimizer/ConstVarFactor.h>
#include <gtdynamics/optimizer/PenaltyMethodOptimizer.h>

namespace gtsam {

/** Perform retraction for constraint manifold. */
class Retractor {
 public:
  using shared_ptr = boost::shared_ptr<Retractor>;

  /// Default constructor.
  Retractor() {}

  virtual Values retract(const Values& values) = 0;
};

class UoptRetractor : public Retractor {
 protected:
  MutableLMOptimizer optimizer_;

 public:
  UoptRetractor(
      const ConnectedComponent::shared_ptr& cc,
      const LevenbergMarquardtParams& lm_params = LevenbergMarquardtParams());

  Values retract(const Values& values) override;
};

class ProjRetractor : public Retractor {
 protected:
  gtdynamics::PenaltyMethodOptimizer optimizer_;
  const ConnectedComponent::shared_ptr& cc_;
 public:
  ProjRetractor(
      const ConnectedComponent::shared_ptr& cc,
      const LevenbergMarquardtParams& lm_params = LevenbergMarquardtParams());

  Values retract(const Values& values) override;
};

class BasisRetractor : public Retractor {
 protected:
  MutableLMOptimizer optimizer_;
  KeyVector basis_keys_;
  std::vector<boost::shared_ptr<ConstVarFactor>> factors_with_fixed_vars_;
 public:
  BasisRetractor(
      const ConnectedComponent::shared_ptr& cc, const KeyVector& basis_keys,
      const LevenbergMarquardtParams& lm_params = LevenbergMarquardtParams());

  Values retract(const Values& values) override;
 
 protected:
  void constructGraph(const ConnectedComponent::shared_ptr& cc, const KeyVector& basis_keys);
};

}  // namespace gtsam
