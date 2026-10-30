/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file BarrierRetractor.h
 * @brief BarrierRetractor declaration.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/IERetractor.h>

namespace gtdynamics {

/// Penalty-based approximation to thesis Eq. (4.26); with blocking indices it
/// implements the on-corner retraction in thesis Eq. (4.46).
class BarrierRetractor : public IERetractor {

protected:
  /** Penalty solve and active-face cleanup in exact affine coordinates. */
  IEConstraintManifold retractReduced(const IEConstraintManifold &manifold,
                                      const VectorValues &delta,
                                      const IndexSet &blocking,
                                      IERetractionInfo *info) const;

  bool use_basis_keys_;
  KeyVector basis_keys_;

public:
  BarrierRetractor(const IERetractorParams::shared_ptr &params,
                   const std::optional<KeyVector> &basis_keys = {})
      : IERetractor(params) {
    if (basis_keys) {
      use_basis_keys_ = true;
      basis_keys_ = *basis_keys;
    } else {
      use_basis_keys_ = false;
    }
  }

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractionInfo *retract_info = nullptr) const override;

  /// Solve the special case delta = 0 while forcing the requested inequalities
  /// to become active.
  IEConstraintManifold
  moveToBoundary(const IEConstraintManifold *manifold,
                 const IndexSet &blocking_indices,
                 IERetractionInfo *retract_info = nullptr) const override;
};

class BarrierRetractorCreator : public UniversalIERetractorCreator {

public:
  BarrierRetractorCreator(IERetractorParams::shared_ptr params =
                              std::make_shared<IERetractorParams>())
      : UniversalIERetractorCreator(
            std::make_shared<BarrierRetractor>(params)) {}

  virtual ~BarrierRetractorCreator() {}
};

} // namespace gtdynamics
