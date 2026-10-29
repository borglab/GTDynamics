/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ReducedALRetractor.cpp
 * @brief Implement reduced augmented Lagrangian retraction with augmented
 * Lagrangian.
 * @author Frank Dellaert
 */

#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmcopt/ReducedALRetractor.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace gtdynamics {
namespace {
constexpr Key kReduced = 0;

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
  if (auto p = fixedVector<1>(v)) return p->value();
  if (auto p = fixedVector<2>(v)) return p->value();
  if (auto p = fixedVector<3>(v)) return p->value();
  if (auto p = fixedVector<4>(v)) return p->value();
  if (auto p = fixedVector<5>(v)) return p->value();
  if (auto p = fixedVector<6>(v)) return p->value();
  throw std::invalid_argument(
      "ReducedALRetractor requires Euclidean scalar/vector Values");
}

double infinityNorm(const Vector &v) {
  if (!v.allFinite()) return std::numeric_limits<double>::infinity();
  return v.size() ? v.lpNorm<Eigen::Infinity>() : 0.0;
}
// Pull back one original scalar constraint. Only its own keys are recovered;
// evaluating a contact constraint does not copy the whole dynamics state.
class ReducedExpression {
 public:
  ReducedExpression(const NonlinearInequalityConstraint::shared_ptr &original,
                    const Values &anchor, const std::map<Key, Matrix> &maps,
                    const Vector &target, double lengthScale)
      : original_(original), target_(target) {
    if (original->dim() != 1)
      throw std::invalid_argument("Reduced AL requires scalar inequalities");
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
    Matrix anchorJacobian, targetJacobian;
    evaluate(-target_, &anchorJacobian);
    evaluate(Vector::Zero(target_.size()), &targetJacobian);
    scale = std::max({anchorJacobian.norm(), targetJacobian.norm(), 1e-12});
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
  NonlinearEqualityConstraint::shared_ptr createEqualityConstraint()
      const override {
    return std::make_shared<ReducedEquality>(expression_);
  }

 private:
  std::shared_ptr<const ReducedExpression> expression_;
};
}  // namespace

ReducedALRetractor::Settings::Settings() {
  al.maxIterations = 80;
  al.absoluteViolationTolerance = 1e-9;
  al.absoluteStationarityTolerance = 1e-7;
  // Tighten the BCL inner target gradually: a tenfold tightening per dual
  // update can request sub-machine-precision objective decreases before the
  // projection reaches its primal tolerance, even for scalar linear bounds.
  al.bclInitialPenalty = 10.0;
  al.bclBetaOmega = 0.1;
  al.lmParams.maxIterations = 200;
  al.lmParams.absoluteErrorTol = al.lmParams.relativeErrorTol = 0.0;
  al.lmParams.lambdaUpperBound = 1e12;
}

ReducedALRetractor::ReducedALRetractor(
    const IERetractorParams::shared_ptr &params,
    const Settings &settings, const std::optional<KeyVector> &priorKeys)
    : IERetractor(params), settings_(settings), priorKeys_(priorKeys) {
  if (!params_ || !(settings_.feasibilityTolerance > 0) ||
      !std::isfinite(settings_.feasibilityTolerance) ||
      settings_.activeTolerance < settings_.feasibilityTolerance ||
      !std::isfinite(settings_.activeTolerance))
    throw std::invalid_argument("Invalid ReducedALRetractor configuration");
  if (params_->useVaryingSigma && !params_->metricSigmas)
    throw std::invalid_argument("Missing reduced projection metric sigmas");
  if (settings_.al.updatePolicy != AugmentedLagrangianUpdatePolicy::BCL)
    throw std::invalid_argument(
        "Reduced AL retraction requires BCL AL updates");
}

ConstrainedOptProblem ReducedALRetractor::buildProblem(
    const IEConstraintManifold &manifold, const Reduction &reduction,
    const IndexSet &blocking) const {
  // v = u - target centers the objective without subtracting large quadratics.
  NonlinearFactorGraph costs;
  costs.emplace_shared<PriorFactor<Vector>>(
      kReduced, Vector::Zero(reduction.target.size()),
      noiseModel::Unit::Create(reduction.target.size()));
  NonlinearEqualityConstraints equalities;
  NonlinearInequalityConstraints inequalities;
  for (size_t i = 0; i < manifold.iConstraints()->size(); ++i) {
    auto expression = std::make_shared<ReducedExpression>(
        manifold.iConstraints()->at(i), manifold.values(), reduction.metric->maps(),
        reduction.target, reduction.lengthScale);
    auto constraint = std::make_shared<ReducedInequality>(expression);
    if (blocking.count(i))
      equalities.push_back(constraint->createEqualityConstraint());
    else
      inequalities.push_back(constraint);
  }
  return ConstrainedOptProblem(costs, equalities, inequalities);
}

Vector ReducedALRetractor::solveProblem(const ConstrainedOptProblem &problem,
                                        const Reduction &reduction,
                                        IERetractionInfo *info) const {
  if (info) ++info->numALSolveCalls;
  auto al = std::make_shared<AugmentedLagrangianParams>(settings_.al);
  // Each reduced expression already includes the original sigma. Its noise
  // sigma is only the additional normalization, so this bound implies the
  // requested ORIGINAL whitened feasibility tolerance.
  double largestSigma = 1.0;
  for (const auto &constraint : problem.eConstraints())
    largestSigma =
        std::max(largestSigma, constraint->noiseModel()->sigmas()(0));
  for (const auto &constraint : problem.iConstraints())
    largestSigma =
        std::max(largestSigma, constraint->noiseModel()->sigmas()(0));
  al->absoluteViolationTolerance =
      std::min(al->absoluteViolationTolerance,
               0.5 * settings_.feasibilityTolerance / largestSigma);
  Values initial;
  const Vector initialVector = params_->initValuesAsX
                                   ? Vector(-reduction.target)
                                   : Vector::Zero(reduction.target.size());
  initial.insert<Vector>(kReduced, initialVector);
  AugmentedLagrangianOptimizer optimizer(problem, initial, al);
  AugmentedLagrangianState state(0, initial, problem);
  double penaltyEq = al->bclInitialPenalty, penaltyIneq = penaltyEq;
  // Use GTSAM's public AL iteration (including its dual/penalty updates).
  // Bound its shrinking inner target by the requested final stationarity:
  // solving the inner LM more accurately than that can stall before a useful
  // dual update. This floor does not relax either final convergence test.
  for (size_t iteration = 0; iteration < al->maxIterations; ++iteration) {
    if (state.bclOmega > 0)
      state.bclOmega =
          std::max(state.bclOmega, al->absoluteStationarityTolerance);
    std::tie(state, penaltyEq, penaltyIneq) =
        optimizer.iterate(state, penaltyEq, penaltyIneq);
    state.converged =
        state.innerConverged &&
        state.augmentedLagrangianStationarity <=
            al->absoluteStationarityTolerance &&
        state.generalizedConstraintViolation <= al->absoluteViolationTolerance;
    if (state.converged || !state.innerConverged) break;
  }
  if (info) info->numLMIterations += state.totalUnconstrainedIterations;
  if (!state.converged ||
      !std::isfinite(state.augmentedLagrangianStationarity) ||
      !std::isfinite(state.generalizedConstraintViolation) ||
      state.augmentedLagrangianStationarity >
          al->absoluteStationarityTolerance ||
      state.generalizedConstraintViolation > al->absoluteViolationTolerance) {
    std::ostringstream message;
    message << "Reduced AL failed: iterations=" << state.iteration
            << ", stationarity=" << state.augmentedLagrangianStationarity
            << ", violation=" << state.generalizedConstraintViolation
            << ", inner_converged=" << state.innerConverged
            << ", omega=" << state.bclOmega
            << ", lm_iterations=" << state.totalUnconstrainedIterations;
    throw std::runtime_error(message.str());
  }
  const Vector u = state.values.at<Vector>(kReduced) + reduction.target;
  if (!u.allFinite()) throw std::runtime_error("Nonfinite reduced AL result");
  return u;
}

std::optional<IndexSet> ReducedALRetractor::feasibleResult(
    const IEConstraintManifold &manifold, const Values &values,
    const IndexSet &blocking) const {
  const Vector h = manifold.eConstraints()->violationVector(values);
  if (!h.allFinite())
    throw std::invalid_argument("Nonfinite original equality residual");
  if (infinityNorm(h) > settings_.feasibilityTolerance) return {};
  IndexSet active = blocking;
  for (size_t i = 0; i < manifold.iConstraints()->size(); ++i) {
    const Vector g = manifold.iConstraints()->at(i)->whitenedExpr(values);
    if (!g.allFinite() || g.size() != 1)
      throw std::invalid_argument("Invalid original inequality residual");
    if (g(0) > settings_.feasibilityTolerance ||
        (blocking.count(i) && std::abs(g(0)) > settings_.feasibilityTolerance))
      return {};
    if (g(0) >= -settings_.activeTolerance) active.insert(i);
  }
  return active;
}

IndexSet ReducedALRetractor::validateResult(
    const IEConstraintManifold &manifold, const Values &values,
    const IndexSet &blocking) const {
  for (const auto &kv : values)
    if (!euclideanVector(kv.value).allFinite())
      throw std::runtime_error("Nonfinite recovered AL result");
  const auto active = feasibleResult(manifold, values, blocking);
  if (!active)
    throw std::runtime_error("Reduced AL result violates original constraints or blocking faces");
  return *active;
}

IEConstraintManifold ReducedALRetractor::retract(
    const IEConstraintManifold *manifold, const VectorValues &delta,
    const std::optional<IndexSet> &blockingIndices,
    IERetractionInfo *info) const {
  if (!manifold)
    throw std::invalid_argument("Null reduced AL retraction manifold");
  const IndexSet blocking = blockingIndices.value_or(IndexSet{});
  for (size_t index : blocking)
    if (index >= manifold->iConstraints()->size())
      throw std::invalid_argument(
          "Invalid reduced AL retraction blocking index");
  const Values &anchor = manifold->values();
  if (infinityNorm(manifold->eConstraints()->violationVector(anchor)) >
      settings_.feasibilityTolerance)
    throw std::invalid_argument("Reduced AL requires an equality-feasible affine anchor");
  const Values target = anchor.retract(delta);
  for (const auto &kv : anchor)
    if (!euclideanVector(kv.value).allFinite() ||
        !euclideanVector(target.at(kv.key)).allFinite())
      throw std::invalid_argument("Nonfinite reduced AL anchor or trial");

  // Geometry belongs to the manifold. This also checks that even a partial
  // prior metric measures every independent direction before taking a shortcut.
  const auto metric = manifold->projectionMetric(*params_, priorKeys_);
  if (const auto active = feasibleResult(*manifold, target, blocking)) {
    // Zero projection cost: the requested target is already the minimizer.
    if (info) ++info->numFeasibleTrials;
    return manifold->createWithNewValues(target, *active);
  }

  Reduction reduction;
  reduction.metric = metric;
  const Vector projected = metric->project(delta);
  Values result = anchor.retract(metric->recover(projected));
  for (const auto &kv : result)
    if (!euclideanVector(kv.value).allFinite())
      throw std::invalid_argument("Nonfinite equality projection");
  if (const auto active = feasibleResult(*manifold, result, blocking)) {
    // The unconstrained metric projection onto the affine equalities is also
    // inequality-feasible, hence solves the complete projection problem.
    if (info) ++info->numFeasibleProjections;
    return manifold->createWithNewValues(result, *active);
  }
  if (projected.size()) {
    // Only this trial-dependent normalization remains in the retractor.
    reduction.lengthScale = std::max(1.0, projected.norm());
    reduction.target = projected / reduction.lengthScale;
    const auto problem = buildProblem(*manifold, reduction, blocking);
    const Vector u = solveProblem(problem, reduction, info);
    result = anchor.retract(metric->recover(reduction.lengthScale * u));
  }
  const IndexSet active = validateResult(*manifold, result, blocking);
  return manifold->createWithNewValues(result, active);
}

}  // namespace gtdynamics
