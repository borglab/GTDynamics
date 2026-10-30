/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file AffineProjection.cpp
 * @brief Share affine projection geometry, constraints, and feasibility checks.
 * @author Frank Dellaert
 */

#include <algorithm>
#include <cmath>
#include <gtdynamics/cmcopt/AffineProjection.h>
#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <limits>
#include <stdexcept>

namespace gtdynamics {
namespace {
constexpr Key kReduced = AffineProjection::kReduced;

// Values::retract is addition only for these Euclidean types. In particular,
// accepting a Pose3 because it has six tangent coordinates would be incorrect.
template <int N>
const GenericValue<Eigen::Matrix<double, N, 1>> *fixedVector(const Value &v) {
  return dynamic_cast<const GenericValue<Eigen::Matrix<double, N, 1>> *>(&v);
}

Vector euclideanVector(const Value &v) {
  if (auto p = dynamic_cast<const GenericValue<double> *>(&v))
    return Vector1(p->value());
  if (auto p = dynamic_cast<const GenericValue<Vector> *>(&v))
    return p->value();
  if (auto p = fixedVector<1>(v))
    return p->value();
  if (auto p = fixedVector<2>(v))
    return p->value();
  if (auto p = fixedVector<3>(v))
    return p->value();
  if (auto p = fixedVector<4>(v))
    return p->value();
  if (auto p = fixedVector<5>(v))
    return p->value();
  if (auto p = fixedVector<6>(v))
    return p->value();
  throw std::invalid_argument(
      "Affine projection requires Euclidean scalar/vector Values");
}

double infinityNorm(const Vector &v) {
  if (!v.allFinite())
    return std::numeric_limits<double>::infinity();
  return v.size() ? v.lpNorm<Eigen::Infinity>() : 0.0;
}
// Pull back one original scalar constraint. Only its own keys are recovered;
// evaluating a contact constraint does not copy the whole dynamics state.
class ReducedExpression {
public:
  ReducedExpression(const NonlinearInequalityConstraint::shared_ptr &original,
                    const Values &anchor, const std::map<Key, Matrix> &maps,
                    const Vector &target, double lengthScale, bool normalize)
      : original_(original), target_(target) {
    if (original->dim() != 1)
      throw std::invalid_argument(
          "Affine projection requires scalar inequalities");
    sigma_ = original->noiseModel()->sigmas()(0);
    if (!(sigma_ > 0) || !std::isfinite(sigma_))
      throw std::invalid_argument("Invalid original inequality sigma");
    for (Key key : original->keys()) {
      anchor_.insert(key, anchor.at(key));
      maps_.push_back(lengthScale * maps.at(key));
    }
    // Positive, fixed scaling preserves the original function and boundary.
    // Use gradients at the anchor and target; no polynomial surrogate or
    // second derivatives are required. Linear rows get unit gradient norm.
    if (!normalize) {
      scale = 1.0;
      return;
    }
    Matrix anchorJacobian, targetJacobian;
    evaluate(-target_, &anchorJacobian);
    evaluate(Vector::Zero(target_.size()), &targetJacobian);
    scale =
        normalize
            ? std::max({anchorJacobian.norm(), targetJacobian.norm(), 1e-12})
            : 1.0;
  }

  Vector evaluate(const Vector &v, Matrix *H = nullptr) const {
    const Vector u = v + target_;
    VectorValues delta;
    const auto &keys = original_->keys();
    for (size_t i = 0; i < keys.size(); ++i)
      delta.insert(keys[i], maps_[i] * u);
    const Values values = anchor_.retract(delta);
    std::vector<Matrix> jacobians(keys.size());
    const Vector residual =
        original_->unwhitenedExpr(values, H ? &jacobians : nullptr) / sigma_;
    if (residual.size() != 1 || !residual.allFinite())
      throw std::invalid_argument("Invalid reduced inequality residual");
    if (H) {
      *H = Matrix::Zero(1, target_.size());
      for (size_t i = 0; i < keys.size(); ++i) {
        if (jacobians[i].rows() != 1 ||
            jacobians[i].cols() != maps_[i].rows() || !jacobians[i].allFinite())
          throw std::invalid_argument("Invalid original inequality Jacobian");
        H->noalias() += jacobians[i] * maps_[i] / sigma_;
      }
      if (!H->allFinite())
        throw std::invalid_argument("Nonfinite reduced inequality Jacobian");
    }
    return residual;
  }

  double scale;

private:
  NonlinearInequalityConstraint::shared_ptr original_;
  Values anchor_;
  std::vector<Matrix> maps_;
  Vector target_;
  double sigma_;
};

class ReducedEquality : public NonlinearEqualityConstraint {
public:
  explicit ReducedEquality(std::shared_ptr<const ReducedExpression> expression)
      : NonlinearEqualityConstraint(
            constrainedNoise(Vector1(expression->scale)), KeyVector{kReduced}),
        expression_(std::move(expression)) {}
  Vector unwhitenedError(const Values &values,
                         OptionalMatrixVecType H = nullptr) const override {
    return expression_->evaluate(values.at<Vector>(kReduced),
                                 H ? &(*H)[0] : nullptr);
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<ReducedEquality>(*this);
  }

private:
  std::shared_ptr<const ReducedExpression> expression_;
};

class ReducedInequality : public NonlinearInequalityConstraint {
public:
  explicit ReducedInequality(
      std::shared_ptr<const ReducedExpression> expression)
      : NonlinearInequalityConstraint(
            constrainedNoise(Vector1(expression->scale)), KeyVector{kReduced}),
        expression_(std::move(expression)) {}
  Vector unwhitenedExpr(const Values &values,
                        OptionalMatrixVecType H = nullptr) const override {
    return expression_->evaluate(values.at<Vector>(kReduced),
                                 H ? &(*H)[0] : nullptr);
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<ReducedInequality>(*this);
  }
  NonlinearEqualityConstraint::shared_ptr
  createEqualityConstraint() const override {
    return std::make_shared<ReducedEquality>(expression_);
  }

private:
  std::shared_ptr<const ReducedExpression> expression_;
};
} // namespace

AffineProjection::AffineProjection(const IEConstraintManifold &manifold,
                                   const VectorValues &delta,
                                   const IERetractorParams &params,
                                   const std::optional<KeyVector> &priorKeys,
                                   const IndexSet &blocking,
                                   double feasibilityTolerance,
                                   double activeTolerance, bool normalize)
    : manifold_(manifold), feasibility_tolerance_(feasibilityTolerance),
      active_tolerance_(activeTolerance), normalize_(normalize) {
  if (!(feasibilityTolerance > 0) || !std::isfinite(feasibilityTolerance) ||
      activeTolerance < feasibilityTolerance || !std::isfinite(activeTolerance))
    throw std::invalid_argument("Invalid affine projection tolerances");
  for (size_t index : blocking)
    if (index >= manifold.iConstraints()->size())
      throw std::invalid_argument("Invalid affine projection blocking index");
  const auto &anchor = manifold.values();
  if (infinityNorm(manifold.eConstraints()->violationVector(anchor)) >
      feasibilityTolerance)
    throw std::invalid_argument(
        "Affine projection requires an equality-feasible anchor");
  trial_ = anchor.retract(delta);
  for (const auto &kv : anchor)
    if (!euclideanVector(kv.value).allFinite() ||
        !euclideanVector(trial_.at(kv.key)).allFinite())
      throw std::invalid_argument(
          "Nonfinite affine projection anchor or trial");
  metric_ = manifold.projectionMetric(params, priorKeys);
  const Vector projected = metric_->project(delta);
  length_scale_ =
      normalize ? std::max(1.0, (metric_->costJacobian() * projected).norm())
                : 1.0;
  target_ = projected / length_scale_;
}

Values AffineProjection::recover(const Vector &v) const {
  const Vector u = v + target_;
  return manifold_.values().retract(metric_->recover(length_scale_ * u));
}

Values AffineProjection::initialValues(bool atAnchor) const {
  Values initial;
  const Vector v = atAnchor ? Vector(-target_) : Vector::Zero(target_.size());
  initial.insert<Vector>(kReduced, v);
  return initial;
}

std::optional<AffineProjection::Result>
AffineProjection::feasibleProjection(const IndexSet &blocking,
                                     IERetractionInfo *info) const {
  if (const auto active = feasibleResult(trial_, blocking)) {
    if (info)
      ++info->numFeasibleTrials;
    return Result{trial_, *active};
  }
  Values projected = recover(Vector::Zero(target_.size()));
  // Validate finiteness even if an unconstrained problem has no residuals.
  for (const auto &kv : projected)
    if (!euclideanVector(kv.value).allFinite())
      throw std::invalid_argument("Nonfinite equality projection");
  if (const auto active = feasibleResult(projected, blocking)) {
    if (info)
      ++info->numFeasibleProjections;
    return Result{std::move(projected), *active};
  }
  return {};
}

ConstrainedOptProblem
AffineProjection::buildProblem(const IndexSet &blocking) const {
  // v = u - target centers the objective without subtracting large quadratics.
  NonlinearFactorGraph costs;
  costs.emplace_shared<PriorFactor<Vector>>(
      kReduced, Vector::Zero(target_.size()),
      noiseModel::Gaussian::SqrtInformation(metric_->costJacobian()));
  NonlinearEqualityConstraints equalities;
  NonlinearInequalityConstraints inequalities;
  for (size_t i = 0; i < manifold_.iConstraints()->size(); ++i) {
    auto expression = std::make_shared<ReducedExpression>(
        manifold_.iConstraints()->at(i), manifold_.values(), metric_->maps(),
        target_, length_scale_, normalize_);
    auto constraint = std::make_shared<ReducedInequality>(expression);
    if (blocking.count(i))
      equalities.push_back(constraint->createEqualityConstraint());
    else
      inequalities.push_back(constraint);
  }
  return ConstrainedOptProblem(costs, equalities, inequalities);
}

std::optional<IndexSet>
AffineProjection::feasibleResult(const Values &values,
                                 const IndexSet &blocking) const {
  const Vector h = manifold_.eConstraints()->violationVector(values);
  if (!h.allFinite())
    throw std::invalid_argument("Nonfinite original equality residual");
  if (infinityNorm(h) > feasibility_tolerance_)
    return {};
  IndexSet active = blocking;
  for (size_t i = 0; i < manifold_.iConstraints()->size(); ++i) {
    const Vector g = manifold_.iConstraints()->at(i)->whitenedExpr(values);
    if (!g.allFinite() || g.size() != 1)
      throw std::invalid_argument("Invalid original inequality residual");
    if (g(0) > feasibility_tolerance_ ||
        (blocking.count(i) && std::abs(g(0)) > feasibility_tolerance_))
      return {};
    if (g(0) >= -active_tolerance_)
      active.insert(i);
  }
  return active;
}

IndexSet AffineProjection::validateResult(const Values &values,
                                          const IndexSet &blocking) const {
  for (const auto &kv : values)
    if (!euclideanVector(kv.value).allFinite())
      throw std::runtime_error("Nonfinite recovered projection result");
  const auto active = feasibleResult(values, blocking);
  if (!active)
    throw std::runtime_error("Reduced projection result violates original "
                             "constraints or blocking faces");
  return *active;
}

} // namespace gtdynamics
