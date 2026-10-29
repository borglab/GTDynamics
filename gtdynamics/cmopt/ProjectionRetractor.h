/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ProjectionRetractor.h
 * @brief ProjectionRetractor declaration.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmopt/Retractor.h>

namespace gtdynamics {

/**
 * Retractor implementing projection-style updates with optional priors.
 *
 * @see README.md#retraction
 */
class ProjectionRetractor : public Retractor {
protected:
  NonlinearFactorGraph merit_graph_;
  KeyVector basis_keys_;

public:
  /**
   * Constructor.
   * @param constraints Equality constraints for the component.
   * @param params Retraction parameters.
   * @param basis_keys Optional basis keys used when `useBasisKeys` is true.
   */
  ProjectionRetractor(
      const NonlinearEqualityConstraints::shared_ptr &constraints,
      const RetractionParams::shared_ptr &params,
      std::optional<const KeyVector> basis_keys = {});

  /**
   * Retract with projection-style retraction.
   * @param values Base values composing the constraint manifold.
   * @param delta Tangent update.
   * @return Retraction result projected toward constraint manifold.
   */
  Values retract(const Values &values, const VectorValues &delta) override;

  Values retractConstraints(const Values &values) override;
};

/**
 * Factory for `ProjectionRetractor`.
 *
 * @see README.md#retraction
 */
class ProjectionRetractorCreator : public RetractorCreator {
public:
  ProjectionRetractorCreator(RetractionParams::shared_ptr params =
                                 std::make_shared<RetractionParams>())
      : RetractorCreator(params) {}

  virtual ~ProjectionRetractorCreator() {}

  Retractor::shared_ptr create(const NonlinearEqualityConstraints::shared_ptr
                                   constraints) const override {
    return std::make_shared<ProjectionRetractor>(constraints, params_);
  }
};

} // namespace gtdynamics
