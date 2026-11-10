/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file ArticulatedRetractor.h
 * @brief Forward-kinematics recovery followed by affine mechanics projection.
 */
#pragma once
#include <gtdynamics/cmcopt/ReducedALRetractor.h>
#include <gtdynamics/universal_robot/Robot.h>

namespace gtdynamics {
/** Retract one floating articulated tree at one time slice.
 *
 * Root pose/twist and joint angles/rates are free coordinates. FK recovers the
 * dependent link state; remaining equalities must be affine once q/v is fixed.
 * Inequalities may involve only acceleration, wrench and torque variables.
 * A root world-linear-velocity variable is recovered if present. Additional
 * q/v constraints not recovered by FK, cross-slice components, fixed links,
 * and q/v inequalities are unsupported. They are rejected, not ignored.
 *
 * The caller supplies independent acceleration/wrench keys for elimination,
 * a projection metric through IERetractorParams, and the existing AL settings.
 * There are no robot-specific scales or contact assumptions. Robot ownership
 * is retained; its links/joints must remain immutable during optimization.
 * Each instance owns its mutable fixed-variable factors and metric parameters.
 */
class ArticulatedRetractor : public IERetractor {
 public:
  /// Isolated copy used by runtime manifold types.
  IERetractor::shared_ptr clone() const override {
    auto copy = std::make_shared<ArticulatedRetractor>(*this);
    detachFixedFactors(copy->adGraph_, copy->fixedFactors_);
    return copy;
  }

  std::shared_ptr<const Robot> robot_;
  LinkSharedPtr root_;
  size_t slice_;
  KeySet qvKeys_, independentAD_;
  KeyVector priorKeys_;
  NonlinearFactorGraph adGraph_, qvGraph_;
  std::vector<std::shared_ptr<ConstVarFactor>> fixedFactors_;
  ReducedALRetractor::Settings settings_;

 public:
  ArticulatedRetractor(const IECMPoint& point,
                       const std::shared_ptr<const Robot>& robot,
                       const std::string& rootName, const KeySet& independentAD,
                       const IERetractorParams::shared_ptr& params,
                       const ReducedALRetractor::Settings& settings = {});

  IECMPoint retract(const IECMPoint* point,
                               const VectorValues& delta,
                               const std::optional<IndexSet>& blocking = {},
                               IERetractionInfo* info = nullptr) const override;

 private:
  friend class ArticulatedRetractorCreator;
  ArticulatedRetractor(const IECMPoint& point,
                       const std::shared_ptr<const Robot>& robot,
                       const LinkSharedPtr& root, const KeySet& independentAD,
                       const IERetractorParams::shared_ptr& params,
                       const ReducedALRetractor::Settings& settings);

};

/** Create a separate projection cache for each constraint component.
 * The projection metric is the supplied fixed metric, not the outer LM cost
 * metric. Components copy it at construction, including the sigma map.
 */
class ArticulatedRetractorCreator : public IERetractorCreator {
 public:
  /// Isolated copy used by runtime manifold types.
  IERetractorCreator::shared_ptr clone() const override {
    auto copy = std::make_shared<ArticulatedRetractorCreator>(*this);
    copy->projectionParams_ = IERetractorParams::snapshot(projectionParams_);
    return copy;
  }

  std::shared_ptr<const Robot> robot_;
  std::string rootName_;
  KeySet independentAD_;
  IERetractorParams::shared_ptr projectionParams_;
  ReducedALRetractor::Settings settings_;

 public:
  /// Freeze only this component's metric and independent-key selection.
  IERetractorCreator::shared_ptr cloneFor(const KeySet& keys) const override;
  ArticulatedRetractorCreator(
      const std::shared_ptr<const Robot>& robot, const std::string& rootName,
      const KeySet& independentAD, const IERetractorParams::shared_ptr& params,
      const ReducedALRetractor::Settings& settings = {});
  virtual ~ArticulatedRetractorCreator() = default;
  IERetractor::shared_ptr create(
      const IECMPoint& point) const override;

 private:
  LinkSharedPtr root_;

};
}  // namespace gtdynamics
