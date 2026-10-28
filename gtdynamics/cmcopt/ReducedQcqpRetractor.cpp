/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ReducedQcqpRetractor.cpp
 * @brief Implement reduced QCQP retraction with augmented Lagrangian.
 * @author Frank Dellaert
 */

#include <gtdynamics/cmcopt/ReducedQcqpRetractor.h>

#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>

#include <Eigen/Cholesky>
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
const GenericValue<Eigen::Matrix<double, N, 1>>* fixedVector(const Value& v) {
  return dynamic_cast<const GenericValue<Eigen::Matrix<double, N, 1>>*>(&v);
}

Vector euclideanVector(const Value& v) {
  if (auto p = dynamic_cast<const GenericValue<double>*>(&v))
    return Vector1(p->value());
  if (auto p = dynamic_cast<const GenericValue<Vector>*>(&v)) return p->value();
  if (auto p = fixedVector<1>(v)) return p->value();
  if (auto p = fixedVector<2>(v)) return p->value();
  if (auto p = fixedVector<3>(v)) return p->value();
  if (auto p = fixedVector<4>(v)) return p->value();
  if (auto p = fixedVector<5>(v)) return p->value();
  if (auto p = fixedVector<6>(v)) return p->value();
  throw std::invalid_argument(
      "ReducedQcqpRetractor requires Euclidean scalar/vector Values");
}

double infinityNorm(const Vector& v) {
  if (!v.allFinite()) return std::numeric_limits<double>::infinity();
  return v.size() ? v.lpNorm<Eigen::Infinity>() : 0.0;
}
}  // namespace

ReducedQcqpRetractor::Settings::Settings() {
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

ReducedQcqpRetractor::ReducedQcqpRetractor(
    InequalityModel model,
    const IERetractorParams::shared_ptr& params, const Settings& settings,
    const std::optional<KeyVector>& priorKeys)
    : IERetractor(params), model_(std::move(model)), settings_(settings),
      priorKeys_(priorKeys) {
  if (!params_ || !model_ || !(settings_.feasibilityTolerance > 0) ||
      !std::isfinite(settings_.feasibilityTolerance) ||
      settings_.activeTolerance < settings_.feasibilityTolerance ||
      !std::isfinite(settings_.activeTolerance))
    throw std::invalid_argument("Invalid ReducedQcqpRetractor configuration");
  if (params_->useVaryingSigma && !params_->metricSigmas)
    throw std::invalid_argument("Missing reduced projection metric sigmas");
  if (settings_.al.updatePolicy != AugmentedLagrangianUpdatePolicy::BCL)
    throw std::invalid_argument("Reduced QCQP retraction requires BCL AL updates");
}

ReducedQcqpRetractor::Reduction ReducedQcqpRetractor::buildReduction(
    const IEConstraintManifold& manifold, const Values& target) const {
  const Values& anchor = manifold.values();
  if (infinityNorm(manifold.eConstraints()->violationVector(anchor)) >
      settings_.feasibilityTolerance)
    throw std::invalid_argument(
        "ReducedQcqpRetractor requires an equality-feasible affine anchor");

  const size_t dim = manifold.eBasis()->dim();
  Reduction reduction;
  for (const auto& kv : anchor) {
    if (!euclideanVector(kv.value).allFinite())
      throw std::invalid_argument("Nonfinite ReducedQcqpRetractor anchor");
    Matrix map = dim ? manifold.eBasis()->recoverJacobian(kv.key)
                     : Matrix::Zero(kv.value.dim(), 0);
    if (map.rows() != kv.value.dim() || map.cols() != dim || !map.allFinite())
      throw std::invalid_argument("Invalid affine equality recovery basis");
    reduction.maps.emplace(kv.key, std::move(map));
  }

  // Linearizing the existing prior factory preserves its scalar/varying sigma
  // and optional prior-key semantics. With Euclidean variables this is exact.
  NonlinearFactorGraph priors;
  params_->addPriors(target, priorKeys_ ? *priorKeys_ : target.keys(), priors);
  Matrix metric = Matrix::Zero(dim, dim);
  Vector eta = Vector::Zero(dim);
  const auto linearPriors = priors.linearize(anchor);
  for (const auto& factor : *linearPriors) {
    const auto jacobian = std::dynamic_pointer_cast<JacobianFactor>(factor);
    const auto [A, b] = jacobian->jacobian();
    const Matrix reducedA = A * reduction.maps.at(jacobian->keys().front());
    metric.noalias() += reducedA.transpose() * reducedA;
    eta.noalias() += reducedA.transpose() * b;
  }
  if (!metric.allFinite() || !eta.allFinite())
    throw std::invalid_argument("Nonfinite reduced projection metric/target");
  if (dim == 0) {
    reduction.target = Vector(0);
    return reduction;
  }

  // B can be far from orthonormal (e.g. contact force coordinates recover many
  // accelerations/wrenches). Use u=R*z, H=R'*R so AL sees an identity Hessian.
  Eigen::LLT<Matrix> cholesky(metric);
  if (cholesky.info() != Eigen::Success)
    throw std::invalid_argument(
        "ReducedQcqpRetractor needs a positive definite projection metric");
  const Matrix inverseR = cholesky.matrixU().solve(Matrix::Identity(dim, dim));
  reduction.target = inverseR.transpose() * eta;
  for (auto& [key, map] : reduction.maps) map = (map * inverseR).eval();
  // A large trial (thousands of metric units in a dynamics graph) otherwise
  // makes LM compare tiny decreases against a very large objective. A common
  // length scale changes only the objective's constant positive multiplier.
  const double lengthScale = std::max(1.0, reduction.target.norm());
  reduction.target /= lengthScale;
  for (auto& [key, map] : reduction.maps) map *= lengthScale;
  if (!reduction.target.allFinite())
    throw std::invalid_argument("Nonfinite whitened projection target");
  for (const auto& [key, map] : reduction.maps)
    if (!map.allFinite())
      throw std::invalid_argument("Nonfinite whitened equality recovery basis");
  return reduction;
}

QcqpProblem ReducedQcqpRetractor::buildProblem(
    const IEConstraintManifold& manifold, const Reduction& reduction,
    const IndexSet& blocking) const {
  const size_t dim = reduction.target.size();
  QcqpProblem problem;
  // Solve for v=u-u_target. The equivalent centered objective 0.5*||v||^2
  // avoids subtracting large nearly equal quadratic terms near the target.
  problem.addCost(QpCost(HessianFactor(kReduced, Matrix::Identity(dim, dim),
                                     Vector::Zero(dim), 0.0)));

  // Each model corresponds to the ORIGINAL inequality index. A blocked face
  // is imposed as an equality; all remaining inequalities are still included.
  for (size_t i = 0; i < manifold.iConstraints()->size(); ++i) {
    const auto& original = manifold.iConstraints()->at(i);
    if (original->dim() != 1)
      throw std::invalid_argument("Reduced QCQP requires scalar inequalities");
    const QuadraticConstraint model = model_(*original);
    if (model.isEquality() || original->keys().size() != 1 ||
        original->keys().front() != model.key())
      throw std::invalid_argument("Invalid original-coordinate QCQP inequality");
    const Matrix& F = reduction.maps.at(model.key());
    const Vector x = euclideanVector(manifold.values().at(model.key())) +
                     F * reduction.target;
    if (model.A().rows() != x.size() || model.a().size() != x.size())
      throw std::invalid_argument("QCQP inequality dimension mismatch");
    if (!model.A().allFinite() || !model.a().allFinite() ||
        !std::isfinite(model.b()))
      throw std::invalid_argument("Nonfinite QCQP inequality coefficients");
    // Only the symmetric part contributes to x'Qx. GTSAM permits asymmetric
    // input matrices, so the affine cross term must use that same part too.
    const Matrix Q = 0.5 * (model.A() + model.A().transpose());
    const double sign = model.sense() == QuadraticConstraint::Sense::LessEqual
                            ? 1.0 : -1.0;
    const Matrix A = sign * F.transpose() * Q * F;
    const Vector a = sign * F.transpose() * (Q * x + model.a());
    const double b = sign *
        (model.b() - x.dot(Q * x) - 2.0 * model.a().dot(x));
    if (!A.allFinite() || !a.allFinite() || !std::isfinite(b))
      throw std::invalid_argument("Nonfinite reduced QCQP coefficients");

    // Positive scaling preserves the feasible set. Normalize in the unit
    // projection metric, so force units do not determine AL's penalty schedule.
    // Original residual tolerances are checked separately after recovery.
    const double scale = std::max({2.0 * a.norm(),
        (2.0 * (a - A * reduction.target)).norm(), std::sqrt(A.norm()), 1e-12});
    if (Q.isZero(0.0)) {
      const JacobianFactor row(kReduced, 2.0 * a.transpose(), Vector1(b));
      problem.addConstraint(blocking.count(i)
          ? LinearConstraint::Equal(row, Vector1(scale))
          : LinearConstraint::LessEqual(row, Vector1(scale)));
    } else {
      problem.addConstraint(QuadraticConstraint(
          kReduced, A, a, b,
          blocking.count(i) ? QuadraticConstraint::Sense::Equal
                            : QuadraticConstraint::Sense::LessEqual, scale));
    }
  }
  return problem;
}

Vector ReducedQcqpRetractor::solveProblem(const QcqpProblem& problem,
                                         const Reduction& reduction,
                                         IERetractionInfo* info) const {
  auto al = std::make_shared<AugmentedLagrangianParams>(settings_.al);
  // A normalized residual must still satisfy the original physical units.
  // For a squared cone this bounds its polynomial residual; the original norm
  // residual is additionally validated after recovering the ambient Values.
  double largestSigma = 1.0;
  for (const auto& constraint : problem.eConstraints())
    largestSigma = std::max(largestSigma, constraint->noiseModel()->sigmas()(0));
  for (const auto& constraint : problem.iConstraints())
    largestSigma = std::max(largestSigma, constraint->noiseModel()->sigmas()(0));
  al->absoluteViolationTolerance = std::min(al->absoluteViolationTolerance,
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
      state.bclOmega = std::max(state.bclOmega, al->absoluteStationarityTolerance);
    std::tie(state, penaltyEq, penaltyIneq) =
        optimizer.iterate(state, penaltyEq, penaltyIneq);
    state.converged = state.innerConverged &&
        state.augmentedLagrangianStationarity <= al->absoluteStationarityTolerance &&
        state.generalizedConstraintViolation <= al->absoluteViolationTolerance;
    if (state.converged || !state.innerConverged) break;
  }
  if (info) info->numLMIterations += state.totalUnconstrainedIterations;
  if (!state.converged || !std::isfinite(state.augmentedLagrangianStationarity) ||
      !std::isfinite(state.generalizedConstraintViolation) ||
      state.augmentedLagrangianStationarity > al->absoluteStationarityTolerance ||
      state.generalizedConstraintViolation > al->absoluteViolationTolerance) {
    std::ostringstream message;
    message << "Reduced QCQP AL failed: iterations=" << state.iteration
            << ", stationarity=" << state.augmentedLagrangianStationarity
            << ", violation=" << state.generalizedConstraintViolation
            << ", inner_converged=" << state.innerConverged
            << ", omega=" << state.bclOmega
            << ", lm_iterations=" << state.totalUnconstrainedIterations;
    throw std::runtime_error(message.str());
  }
  const Vector u = state.values.at<Vector>(kReduced) + reduction.target;
  if (!u.allFinite()) throw std::runtime_error("Nonfinite reduced QCQP result");
  return u;
}

IndexSet ReducedQcqpRetractor::validateResult(
    const IEConstraintManifold& manifold, const Values& values,
    const IndexSet& blocking) const {
  for (const auto& kv : values)
    if (!euclideanVector(kv.value).allFinite())
      throw std::runtime_error("Nonfinite recovered QCQP result");
  if (infinityNorm(manifold.eConstraints()->violationVector(values)) >
      settings_.feasibilityTolerance)
    throw std::runtime_error("Reduced QCQP result violates original equalities");
  IndexSet active = blocking;
  for (size_t i = 0; i < manifold.iConstraints()->size(); ++i) {
    const Vector g = manifold.iConstraints()->at(i)->whitenedExpr(values);
    if (!g.allFinite() || g.size() != 1 ||
        g(0) > settings_.feasibilityTolerance ||
        (blocking.count(i) && std::abs(g(0)) > settings_.feasibilityTolerance)) {
      std::ostringstream message;
      message << "Reduced QCQP result violates original inequality " << i
              << ": " << g.transpose();
      throw std::runtime_error(message.str());
    }
    // AL can approach a face from its interior. Exact g>=0 would lose that face
    // and leave the next CMC tangent cone unconstrained in that direction.
    if (g(0) >= -settings_.activeTolerance) active.insert(i);
  }
  return active;
}

IEConstraintManifold ReducedQcqpRetractor::retract(
    const IEConstraintManifold* manifold, const VectorValues& delta,
    const std::optional<IndexSet>& blockingIndices,
    IERetractionInfo* info) const {
  if (!manifold) throw std::invalid_argument("Null QCQP retraction manifold");
  const IndexSet blocking = blockingIndices.value_or(IndexSet{});
  for (size_t index : blocking)
    if (index >= manifold->iConstraints()->size())
      throw std::invalid_argument("Invalid QCQP retraction blocking index");
  // Form the ambient trial point (addition for the supported Euclidean values).
  // It may violate constraints; the reduced solve below projects this target
  // onto the feasible set, with blocking inequalities enforced as equalities.
  const Values target = manifold->values().retract(delta);
  const Reduction reduction = buildReduction(*manifold, target);
  Values result = manifold->values();
  if (reduction.target.size()) {
    const QcqpProblem problem = buildProblem(*manifold, reduction, blocking);
    const Vector u = solveProblem(problem, reduction, info);
    VectorValues step;
    for (const auto& [key, map] : reduction.maps) step.insert(key, map * u);
    result = manifold->values().retract(step);
  }
  const IndexSet active = validateResult(*manifold, result, blocking);
  return manifold->createWithNewValues(result, active);
}

}  // namespace gtdynamics
