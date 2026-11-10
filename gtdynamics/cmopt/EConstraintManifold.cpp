/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file EConstraintManifold.cpp
 * @brief Equality-only factories retain their runtime specialization.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */
#include <gtdynamics/cmopt/ECMPoint.h>
#include <gtdynamics/cmopt/UnconstrainedOptimizationRetractor.h>

namespace gtdynamics {
namespace {
EConstraintManifold::Params equalityDefaults(
    EConstraintManifold::Params params) {
  if (!params.retractorCreator)
    params.retractorCreator =
        std::make_shared<UnconstrainedOptimizationRetractorCreator>();
  return params;
}
}  // namespace
EConstraintManifold::EConstraintManifold(
    const NonlinearEqualityConstraints& equalities, const Values& prototype,
    const Params& params)
    : IEConstraintManifold(equalities, {}, prototype, equalityDefaults(params),
                            EqualityLayout::EqualityOnly) {}

EConstraintManifold::EConstraintManifold(
    const NonlinearEqualityConstraints& equalities, const Values& prototype)
    : EConstraintManifold(equalities, prototype, Params()) {}

ECMPoint EConstraintManifold::point(
    const Bindings& bindings, const Values& values,
    const std::optional<IndexSet>& active) const {
  return ECMPoint(IEConstraintManifold::point(bindings, values, active));
}
ECMPoint EConstraintManifold::point(
    const Values& values, const std::optional<IndexSet>& active) const {
  return ECMPoint(IEConstraintManifold::point(values, active));
}
EConstraintManifold::shared_ptr EConstraintManifold::condition(
    const Values& fixed) const {
  return std::static_pointer_cast<const EConstraintManifold>(
      IEConstraintManifold::condition(fixed));
}
std::shared_ptr<IEConstraintManifold> EConstraintManifold::cloneDefinition()
    const {
  if (typeid(*this) != typeid(EConstraintManifold))
    throw std::invalid_argument(
        "Native conditioning belongs to its recovery policy");
  return std::make_shared<EConstraintManifold>(*this);
}
IndexSet EConstraintManifold::identifyActive(
    const IECMPoint&, const std::optional<IndexSet>& supplied) const {
  if (supplied && !supplied->empty())
    throw std::invalid_argument(
        "Equality points have no active inequality blocks");
  return {};
}
Vector EConstraintManifold::inequalityResidual(const IECMPoint&,
                                                size_t) const {
  throw std::out_of_range("Equality types have no inequality blocks");
}
EConstraintManifold::EConstraintManifold(const IEConstraintManifold& source)
    : IEConstraintManifold(source) {
  equalityOnlyLayout_ = true;
  inequalities_ = std::make_shared<NonlinearInequalityConstraints>();
  params_.equalityBasisCreator = params_.equalityBasisCreator->clone();
  params_.retractorCreator =
      std::make_shared<UnconstrainedOptimizationRetractorCreator>();
  prepareEqualities();
}
EConstraintManifold::shared_ptr EConstraintManifold::fromEqualityView(
    const IEConstraintManifold& source) {
  if (typeid(source) != typeid(IEConstraintManifold) &&
      typeid(source) != typeid(EConstraintManifold))
    throw std::invalid_argument(
        "A native equality view requires an explicit compiled equality chart");
  return shared_ptr(new EConstraintManifold(source));
}
}  // namespace gtdynamics
