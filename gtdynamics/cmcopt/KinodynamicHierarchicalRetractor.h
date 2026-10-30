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
      const IEConstraintManifold &manifold, const Params::shared_ptr &params,
      const std::optional<KeyVector> &basis_keys = {});

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractionInfo *retract_info = nullptr) const override;
};

} // namespace gtdynamics
