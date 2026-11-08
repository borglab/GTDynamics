/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file KinodynamicHierarchicalRetractor.h
 * @brief KinodynamicHierarchicalRetractor declaration.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/IERetractor.h>

namespace gtdynamics {

/**
 * Retractor that hierarchically solves the retraction probelm. It first solves
 * q-level by only considering constraints involving poses and joint angles.
 * Then, it solves for link twists and joint velocities using the v-level
 * constraints and the known q-level values. Finally, it solves for all a-level
 * and dynamics level variables as remainig ad-level factors are linear given
 * the qv-level variables.
 */
class KinodynamicHierarchicalRetractor : public IERetractor {
 public:
  /// Isolated copy used by runtime manifold types.
  IERetractor::shared_ptr clone() const override {
    auto copy = std::make_shared<KinodynamicHierarchicalRetractor>(*this);
    detachFixedFactors(copy->graph_v_, copy->const_var_factors_v_);
    detachFixedFactors(copy->graph_ad_, copy->const_var_factors_ad_);
    return copy;
  }

public:
  using Params = IERetractorParams;

protected:
  KeyVector basis_keys_;
  NonlinearFactorGraph merit_graph_;
  NonlinearFactorGraph graph_q_, graph_v_, graph_ad_;
  IndexSet i_indices_q_, i_indices_v_, i_indices_ad_;
  KeySet basis_q_keys_, basis_v_keys_, basis_ad_keys_;
  std::vector<std::shared_ptr<ConstVarFactor>> const_var_factors_v_,
      const_var_factors_ad_;

public:
  KinodynamicHierarchicalRetractor(
      const CMPoint &point, const Params::shared_ptr &params,
      const std::optional<KeyVector> &basis_keys = {});

  CMPoint
  retract(const CMPoint *point, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractionInfo *retract_info = nullptr) const override;
};

} // namespace gtdynamics
