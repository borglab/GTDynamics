/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file IEConstraintManifold.cpp
 * @brief Factor-defined types, immutable conditioning, and bound factor views.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmcopt/BarrierRetractor.h>
#include <gtdynamics/cmcopt/IECMPoint.h>
#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/constrained/internal/ReadOnlyBasis.h>
#include <gtdynamics/cmopt/EConstraintManifold.h>

namespace gtdynamics {
namespace {
/** A fixed chart has no tangent directions, independent of redundant rows.
 * Its template stores only the slot layout; no constraint is dropped or solved.
 */
class FixedEqualityBasis final : public TangentSpaceBasis {
  std::map<Key, size_t> dimensions_;

 public:
  explicit FixedEqualityBasis(const std::map<Key, size_t>& dimensions,
                              bool constructed = false)
      : dimensions_(dimensions) {
    is_constructed_ = constructed;
  }
  shared_ptr createWithNewValues(const Values&) const override {
    return std::make_shared<FixedEqualityBasis>(dimensions_, true);
  }
  shared_ptr createWithAdditionalConstraints(
      const NonlinearEqualityConstraints&, const Values&,
      bool = false) const override {
    return std::make_shared<FixedEqualityBasis>(dimensions_, true);
  }
  VectorValues computeTangentVector(const Vector& xi) const override {
    if (xi.size()) throw std::invalid_argument("A fixed chart has no coordinates");
    VectorValues result;
    for (const auto& [key, dimension] : dimensions_)
      result.insert(key, Vector::Zero(dimension));
    return result;
  }
  Vector computeXi(const VectorValues&) const override { return Vector(0); }
  Matrix recoverJacobian(const Key& key) const override {
    return Matrix::Zero(dimensions_.at(key), 0);
  }
  Vector localCoordinates(const Values&, const Values&) const override {
    return Vector(0);
  }
  size_t dim() const override { return 0; }
  void construct(const Values&) override { is_constructed_ = true; }
  void print(const KeyFormatter& = DefaultKeyFormatter) override {
    std::cout << "Fixed equality chart\n";
  }
};

/** Preserve a constraint's embedded expression and derivative ordering while
 * assigning ambient identities and immutably fixing selected local variables.
 */
class BoundExpression {
 public:
  IEConstraintManifold::Bindings bindings;
  Values fixed;
  KeyVector source;
  KeyVector keys;

  BoundExpression(const KeyVector& slots, const IEConstraintManifold::Bindings& binding,
                  const Values& fixedValues)
      : bindings(binding), fixed(fixedValues), source(slots) {
    for (Key slot : source)
      if (!fixed.exists(slot)) keys.push_back(bindings.at(slot));
  }

  Values local(const Values& ambient) const {
    // Evaluate the original expression using its local slot identities.
    Values result;
    for (Key slot : source)
      result.insert(slot, fixed.exists(slot) ? fixed.at(slot)
                                             : ambient.at(bindings.at(slot)));
    return result;
  }

  void derivatives(const std::vector<Matrix>& full,
                   OptionalMatrixVecType output) const {
    if (!output) return;

    // Fixed slots contribute values but have no columns in the bound Jacobian.
    size_t j = 0;
    for (size_t i = 0; i < source.size(); ++i)
      if (!fixed.exists(source[i])) (*output)[j++] = full[i];
  }
};

class BoundCost final : public NoiseModelFactor {
  NoiseModelFactor::shared_ptr factor_;
  BoundExpression expression_;

 public:
  BoundCost(const NoiseModelFactor::shared_ptr& factor,
            const IEConstraintManifold::Bindings& bindings, const Values& fixed)
      : NoiseModelFactor(factor->noiseModel(),
                         BoundExpression(factor->keys(), bindings, fixed).keys),
        factor_(factor),
        expression_(factor->keys(), bindings, fixed) {}
  Vector unwhitenedError(const Values& values,
                         OptionalMatrixVecType H = nullptr) const override {
    std::vector<Matrix> full(expression_.source.size());
    const Vector result = factor_->unwhitenedError(expression_.local(values),
                                                   H ? &full : nullptr);
    expression_.derivatives(full, H);
    return result;
  }

  bool active(const Values& values) const override {
    return factor_->active(expression_.local(values));
  }

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<BoundCost>(*this);
  }
};

class BoundEquality final : public NonlinearEqualityConstraint {
  NonlinearEqualityConstraint::shared_ptr factor_;
  BoundExpression expression_;

 public:
  BoundEquality(const NonlinearEqualityConstraint::shared_ptr& factor,
                const IEConstraintManifold::Bindings& bindings, const Values& fixed)
      : NonlinearEqualityConstraint(
            factor->noiseModel(),
            BoundExpression(factor->keys(), bindings, fixed).keys),
        factor_(factor),
        expression_(factor->keys(), bindings, fixed) {}
  Vector unwhitenedError(const Values& x,
                         OptionalMatrixVecType H = nullptr) const override {
    std::vector<Matrix> full(expression_.source.size());
    Vector result =
        factor_->unwhitenedError(expression_.local(x), H ? &full : nullptr);
    expression_.derivatives(full, H);
    return result;
  }

  NoiseModelFactor::shared_ptr penaltyFactor(double mu = 1) const override {
    return std::make_shared<BoundCost>(factor_->penaltyFactor(mu),
                                       expression_.bindings, expression_.fixed);
  }

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<BoundEquality>(*this);
  }
};

class BoundInequality final : public NonlinearInequalityConstraint {
  NonlinearInequalityConstraint::shared_ptr factor_;
  BoundExpression expression_;

 public:
  BoundInequality(const NonlinearInequalityConstraint::shared_ptr& factor,
                  const IEConstraintManifold::Bindings& bindings, const Values& fixed)
      : NonlinearInequalityConstraint(
            factor->noiseModel(),
            BoundExpression(factor->keys(), bindings, fixed).keys),
        factor_(factor),
        expression_(factor->keys(), bindings, fixed) {}
  Vector unwhitenedExpr(const Values& x,
                        OptionalMatrixVecType H = nullptr) const override {
    std::vector<Matrix> full(expression_.source.size());
    Vector result =
        factor_->unwhitenedExpr(expression_.local(x), H ? &full : nullptr);
    expression_.derivatives(full, H);
    return result;
  }

  NoiseModelFactor::shared_ptr penaltyFactor(double mu = 1) const override {
    return std::make_shared<BoundCost>(factor_->penaltyFactor(mu),
                                       expression_.bindings, expression_.fixed);
  }

  NoiseModelFactor::shared_ptr penaltyFactorCustom(
      InequalityPenaltyFunction::shared_ptr function,
      double mu = 1) const override {
    return std::make_shared<BoundCost>(
        factor_->penaltyFactorCustom(function, mu), expression_.bindings,
        expression_.fixed);
  }

  NonlinearEqualityConstraint::shared_ptr createEqualityConstraint()
      const override;
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<BoundInequality>(*this);
  }
};

/** Equality of the raw expression, never its clipped inequality residual. */
class BoundBoundaryEquality final : public NonlinearEqualityConstraint {
  std::shared_ptr<const BoundInequality> inequality_;

 public:
  explicit BoundBoundaryEquality(
      const std::shared_ptr<const BoundInequality>& inequality)
      : NonlinearEqualityConstraint(inequality->noiseModel(),
                                    inequality->keys()),
        inequality_(inequality) {}
  Vector unwhitenedError(const Values& values,
                         OptionalMatrixVecType H = nullptr) const override {
    return inequality_->unwhitenedExpr(values, H);
  }

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<BoundBoundaryEquality>(*this);
  }
};

NonlinearEqualityConstraint::shared_ptr
BoundInequality::createEqualityConstraint() const {
  return std::make_shared<BoundBoundaryEquality>(
      std::make_shared<BoundInequality>(*this));
}

/** Bind a numerical basis without reinterpreting local coordinate choices. */
class BoundBasis final : public internal::ReadOnlyBasis {
  IEConstraintManifold::Bindings bindings_, inverse_;

  Values localValues(const Values& ambient) const {
    Values result;
    for (const auto& [slot, key] : bindings_)
      result.insert(slot, ambient.at(key));
    return result;
  }

 public:
  BoundBasis(const std::shared_ptr<const TangentSpaceBasis>& basis,
             const IEConstraintManifold::Bindings& bindings)
      : ReadOnlyBasis(basis), bindings_(bindings) {
    for (const auto& [slot, key] : bindings_) inverse_[key] = slot;
  }

  std::optional<KeyVector> basisKeys() const override {
    auto keys = source_->basisKeys();
    if (keys)
      for (Key &key : *keys) key = bindings_.at(key);
    return keys;
  }

  shared_ptr createWithNewValues(const Values& values) const override {
    return std::make_shared<BoundBasis>(
        constructed(source_->createWithNewValues(localValues(values)),
                    localValues(values)), bindings_);
  }

  shared_ptr createWithAdditionalConstraints(
      const NonlinearEqualityConstraints& constraints, const Values& values,
      bool fromScratch = false) const override {
    NonlinearEqualityConstraints local;
    for (const auto& factor : constraints)
      local.emplace_shared<BoundEquality>(factor, inverse_, Values());
    return std::make_shared<BoundBasis>(
        constructed(source_->createWithAdditionalConstraints(
                        local, localValues(values), fromScratch),
                    localValues(values)), bindings_);
  }

  VectorValues computeTangentVector(const Vector& xi) const override {
    VectorValues result;
    for (const auto& [slot, value] : source_->computeTangentVector(xi))
      result.insert(bindings_.at(slot), value);
    return result;
  }

  Vector computeXi(const VectorValues& ambient) const override {
    VectorValues local;
    for (const auto& [slot, key] : bindings_)
      local.insert(slot, ambient.at(key));
    return source_->computeXi(local);
  }

  Matrix recoverJacobian(const Key& key) const override {
    return source_->recoverJacobian(inverse_.at(key));
  }

  Vector localCoordinates(const Values& a, const Values& b) const override {
    return source_->localCoordinates(localValues(a), localValues(b));
  }

  size_t dim() const override { return source_->dim(); }

};

}  // namespace

IEConstraintManifold::IEConstraintManifold(const NonlinearEqualityConstraints& equalities,
                     const NonlinearInequalityConstraints& inequalities,
                     const Values& prototype)
    : IEConstraintManifold(equalities, inequalities, prototype, Params()) {}

IEConstraintManifold::IEConstraintManifold(const Values& prototype, size_t dimension)
    : dimension_(dimension) {
  for (Key key : prototype.keys()) {
    const Value& value = prototype.at(key);
    slots_.emplace(key, Slot{typeid(value), value.dim()});
  }

  equalities_ = std::make_shared<NonlinearEqualityConstraints>();
  inequalities_ = std::make_shared<NonlinearInequalityConstraints>();
}

IEConstraintManifold::IEConstraintManifold(const NonlinearEqualityConstraints& equalities,
                     const NonlinearInequalityConstraints& inequalities,
                     const Values& prototype, const Params& params)
    : IEConstraintManifold(equalities, inequalities, prototype, params,
                            EqualityLayout::Ordinary) {}

IEConstraintManifold::IEConstraintManifold(
    const NonlinearEqualityConstraints& equalities,
    const NonlinearInequalityConstraints& inequalities,
    const Values& prototype, const Params& params, EqualityLayout layout)
    : IEConstraintManifold(prototype, 0) {
  equalityOnlyLayout_ = layout == EqualityLayout::EqualityOnly;
  if (equalityOnlyLayout_ && !inequalities.empty())
    throw std::invalid_argument("An equality layout cannot contain inequalities");
  if (!equalityOnlyLayout_ && equalities.dim() > prototype.dim())
    throw std::invalid_argument("More equality rows than ambient dimensions");

  // Snapshot policy settings before this definition is shared across points.
  params_ = params;
  if (!params_.equalityBasisCreator)
    throw std::invalid_argument("Missing equality basis policy");
  if (!params_.retractorCreator)
    params_.retractorCreator = std::make_shared<BarrierRetractorCreator>();
  params_.retractorCreator =
      params_.retractorCreator->cloneForKeys(KeySet(prototype.keys()));
  params_.equalityBasisCreator = params_.equalityBasisCreator->clone();

  // Factor definitions stay local; point bindings supply ambient identities.
  equalities_ = std::make_shared<NonlinearEqualityConstraints>(equalities);
  inequalities_ =
      std::make_shared<NonlinearInequalityConstraints>(inequalities);
  dimension_ = prototype.dim() > equalities.dim()
                   ? prototype.dim() - equalities.dim() : 0;
  for (const auto& graphKeys : {equalities.keys(), inequalities.keys()})
    for (Key key : graphKeys)
      if (!slots_.count(key))
        throw std::invalid_argument("Constraint references an undeclared slot");

  prepareEqualities();
}

std::map<Key, size_t> IEConstraintManifold::dimensions() const {
  std::map<Key, size_t> result;
  for (const auto& [key, slot] : slots_) result.emplace(key, slot.dimension);
  return result;
}

void IEConstraintManifold::validate(const Bindings& bindings, const Values& values) const {
  if (bindings.size() != slots_.size() || values.size() != slots_.size())
    throw std::invalid_argument(
        "IECMPoint requires complete bindings and exactly its ambient values");

  // Require one ambient owner for every declared local slot.
  KeySet targets;
  for (const auto& [key, slot] : slots_) {
    auto found = bindings.find(key);
    if (found == bindings.end() || !targets.insert(found->second).second ||
        !values.exists(found->second))
      throw std::invalid_argument("Invalid IECMPoint slot binding");
    const Value& value = values.at(found->second);
    if (slot.type != std::type_index(typeid(value)) ||
        slot.dimension != value.dim())
      throw std::invalid_argument(
          "IECMPoint slot value has a different type or dimension");
  }
}

std::pair<NonlinearEqualityConstraints::shared_ptr,
          NonlinearInequalityConstraints::shared_ptr>
IEConstraintManifold::boundConstraints(const Bindings& bindings) const {
  // Unconditioned identity bindings can use the original factors directly.
  bool identity = fixed_.empty();
  for (const auto& [slot, ambient] : bindings) identity &= slot == ambient;
  if (identity)
    return {std::make_shared<NonlinearEqualityConstraints>(*equalities_),
            std::make_shared<NonlinearInequalityConstraints>(*inequalities_)};

  // Wrap evaluation rather than rekeying embedded expressions.
  auto equalities = std::make_shared<NonlinearEqualityConstraints>();
  auto inequalities = std::make_shared<NonlinearInequalityConstraints>();
  for (const auto& factor : *equalities_) {
    auto bound = std::make_shared<BoundEquality>(factor, bindings, fixed_);
    if (bound->empty()) {
      if (!bound->feasible(Values(), 1e-7))
        throw std::invalid_argument("Conditioning violates a fixed equality");
    } else
      equalities->push_back(bound);
  }

  for (const auto& factor : *inequalities_) {
    auto bound = std::make_shared<BoundInequality>(factor, bindings, fixed_);
    // Retain constant inequality blocks so their original indices never shift.
    if (bound->empty() && !bound->feasible(Values(), 1e-7))
      throw std::invalid_argument("Conditioning violates a fixed inequality");
    inequalities->push_back(bound);
  }

  return {equalities, inequalities};
}

IECMPoint IEConstraintManifold::point(const Bindings& bindings, const Values& values,
                         const std::optional<IndexSet>& active) const {
  validate(bindings, values);
  IECMPoint result;
  result.manifold_ = shared_from_this();
  result.bindings_ = bindings;
  result.values_ = values;

  // Binding-specific factor views and retraction preparation are freshly built.
  std::tie(result.equalities_, result.inequalities_) = boundConstraints(bindings);
  result.active_indices_ = identifyActive(result, active);
  result.tangentSpace_ = linearizeAt(result);
  if (params_.retractorCreator) {
    result.retractionPreparation_ = params_.retractorCreator->prepare(result);
    if (inequalityCount(result) &&
        !std::dynamic_pointer_cast<const IERetractor>(result.retractionPreparation_))
      throw std::invalid_argument("Inequalities require an IE retraction policy");
  }
  return result;
}

IECMPoint IEConstraintManifold::point(const Values& values,
                         const std::optional<IndexSet>& active) const {
  Bindings bindings;
  for (const auto& [key, slot] : slots_) bindings.emplace(key, key);
  return point(bindings, values, active);
}

IEConstraintManifold::shared_ptr IEConstraintManifold::condition(const Values& fixed) const {
  auto result = cloneDefinition();
  // Fixing a selected basis slot changes the chart. Choose a fresh general
  // equality chart rather than retaining an invalid independent-key selector.
  result->params_.equalityBasisCreator =
      std::make_shared<OrthonormalBasisCreator>();
  for (Key key : fixed.keys()) {
    const auto found = result->slots_.find(key);
    const Value& value = fixed.at(key);
    if (found == result->slots_.end() ||
        found->second.type != std::type_index(typeid(value)) ||
        found->second.dimension != value.dim())
      throw std::invalid_argument("Invalid conditioning slot");
    result->fixed_.insert(key, fixed.at(key));
    result->slots_.erase(key);
  }

  // Recompute the free layout after capturing the conditioning values.
  Bindings identity;
  size_t total = 0;
  for (const auto& [key, slot] : result->slots_) {
    identity.emplace(key, key);
    total += slot.dimension;
  }

  const auto constraints = result->boundConstraints(identity);
  if (!result->equalityOnlyLayout_ && constraints.first->dim() > total)
    throw std::invalid_argument("Conditioned equalities have unsupported rank");
  result->dimension_ = total > constraints.first->dim()
                           ? total - constraints.first->dim() : 0;
  KeySet freeSlots;
  for (const auto& [key, slot] : result->slots_) freeSlots.insert(key);
  result->params_.retractorCreator =
      result->params_.retractorCreator->cloneForKeys(freeSlots);
  result->prepareEqualities();
  return result;
}

std::shared_ptr<const CMTangentSpace> IEConstraintManifold::linearize(
    const IECMPoint& point) const {
  if (point.manifold().get() != this)
    throw std::invalid_argument("Point belongs to a different runtime type");
  return linearizeAt(point);
}

std::shared_ptr<IEConstraintManifold> IEConstraintManifold::cloneDefinition() const {
  if (typeid(*this) != typeid(IEConstraintManifold))
    throw std::invalid_argument("Native conditioning belongs to its recovery policy");
  return std::make_shared<IEConstraintManifold>(*this);
}

std::shared_ptr<const CMTangentSpace> IEConstraintManifold::linearize(
    const CMPoint& point) const {
  return linearize(dynamic_cast<const IECMPoint&>(point));
}

Values IEConstraintManifold::retractValues(const CMPoint& point,
                                           const Vector& delta) const {
  validatePointType(point);
  return retract(dynamic_cast<const IECMPoint&>(point), delta).values();
}

std::shared_ptr<const EConstraintManifold> IEConstraintManifold::equalityView() const {
  if (auto equality = std::dynamic_pointer_cast<const EConstraintManifold>(
          shared_from_this())) return equality;
  // Retain local factors, bindings and conditioning, but choose an equality policy.
  return EConstraintManifold::fromEqualityView(*this);
}

void IEConstraintManifold::prepareEqualities() {
  Bindings identity;
  for (const auto& [key, slot] : slots_) identity.emplace(key, key);
  auto preparation = std::make_shared<EqualityPreparation>();
  const auto constraints = boundConstraints(identity).first;
  if (equalityOnlyLayout_ && !dim())
    preparation->structure = std::make_shared<FixedEqualityBasis>(dimensions());
  else
    preparation->structure = params_.equalityBasisCreator->prepare(
        constraints, dimensions());
  if (preparation->structure->isConstructed())
    throw std::invalid_argument("Equality preparation must not construct numerics");
  equalityPreparation_ = std::move(preparation);
}

std::shared_ptr<const TangentSpaceBasis> IEConstraintManifold::localEqualityBasis(
    const IECMPoint& point) const {
  const auto construct = [&]() -> std::shared_ptr<const TangentSpaceBasis> {
    bool identity = true;
    for (const auto& [slot, ambient] : point.bindings()) identity &= slot == ambient;
    Values local;
    if (!identity)
      for (const auto& [slot, ambient] : point.bindings())
        local.insert(slot, point.values().at(ambient));
    const Values& values = identity ? point.values() : local;
    auto basis = equalityPreparation_->structure->createWithNewValues(values);
    if (!basis->isConstructed()) basis->construct(values);
    if (basis->dim() != dim())
      throw RetractionFailure("CMC equality chart changed dimension");
    for (Key key : values.keys()) {
      const Matrix block = basis->recoverJacobian(key);
      if (!block.allFinite() || (dim() && block.cols() != dim()) ||
          (!dim() && block.norm() > 1e-12))
        throw RetractionFailure(
            "Equalities do not define the declared constant-rank chart");
      if (dim()) break;
    }
    return basis;
  };
  if (!params_.affineEqualities) return construct();

  // Only certified constant geometry is published once, after full construction.
  std::call_once(equalityPreparation_->affineOnce, [&] {
    equalityPreparation_->affineBasis = construct();
  });
  return equalityPreparation_->affineBasis;
}

std::shared_ptr<const CMTangentSpace> IEConstraintManifold::linearizeAt(
    const IECMPoint& point) const {
  const auto& values = point.values();
  if (point.manifold().get() != this)
    throw std::invalid_argument("Point belongs to a different runtime type");
  auto basis = localEqualityBasis(point);
  bool identity = true;
  for (const auto& [slot, ambient] : point.bindings()) identity &= slot == ambient;
  if (!identity) basis = std::make_shared<BoundBasis>(basis, point.bindings());

  // Pull each active block into equality coordinates and retain its raw rows.
  LinearIConstraintMap tangent, ambient;
  for (size_t index : point.activeIndices()) {
    const auto& constraint = point.iConstraints()->at(index);
    std::vector<Matrix> jacobians(constraint->size());
    const Vector expression = constraint->unwhitenedExpr(values, &jacobians);
    Matrix rows = Matrix::Zero(expression.size(), dim());
    std::vector<std::pair<Key, Matrix>> terms;
    for (size_t i = 0; i < constraint->size(); ++i) {
      if (dim())
        rows += jacobians[i] * basis->recoverJacobian(constraint->keys()[i]);
      terms.emplace_back(constraint->keys()[i], jacobians[i]);
    }
    const auto noise = noiseModel::Diagonal::Sigmas(constraint->sigmas());
    const Vector zero = Vector::Zero(expression.size());
    tangent.emplace(
        index, std::make_shared<JacobianLinearInequalityConstraint>(
                   std::make_shared<JacobianFactor>(1, rows, zero, noise)));
    ambient.emplace(index,
                    std::make_shared<JacobianLinearInequalityConstraint>(
                        std::make_shared<JacobianFactor>(terms, zero, noise)));
  }

  return std::make_shared<CMTangentSpace>(
      basis, tangent, ambient);
}

IECMPoint IEConstraintManifold::retract(const IECMPoint& point, const Vector& delta,
                           const std::optional<IndexSet>& blocking,
                           IERetractionInfo* info,
                           const VectorValues* metricSigmas) const {
  validatePointType(point);
  return retractAmbient(point, point.toAmbient(delta), blocking, info,
                        metricSigmas);
}

Vector IEConstraintManifold::localCoordinates(const IECMPoint& origin,
                                   const IECMPoint& other) const {
  validatePointType(origin);
  validatePointType(other);
  if (origin.bindings() != other.bindings())
    throw std::invalid_argument("Local coordinates require identical bindings");
  return dim() ? origin.eBasis()->localCoordinates(origin.values(),
                                                   other.values())
               : Vector(0);
}

IndexSet IEConstraintManifold::identifyActive(
    const IECMPoint& point, const std::optional<IndexSet>& supplied) const {
  if (supplied) {
    for (size_t index : *supplied)
      if (index >= point.iConstraints()->size())
        throw std::invalid_argument("Invalid active constraint index");
    return *supplied;
  }

  IndexSet indices;
  for (size_t i = 0; i < point.iConstraints()->size(); ++i)
    if (point.iConstraints()->at(i)->active(point.values())) indices.insert(i);
  return indices;
}

size_t IEConstraintManifold::inequalityCount(const IECMPoint& point) const {
  return point.iConstraints()->size();
}

Vector IEConstraintManifold::inequalityResidual(const IECMPoint& point, size_t index) const {
  return point.iConstraints()->at(index)->unwhitenedExpr(point.values());
}

double IEConstraintManifold::equalityViolation(const IECMPoint& point) const {
  return point.eConstraints()->violationNorm(point.values());
}

double IEConstraintManifold::inequalityViolation(const IECMPoint& point) const {
  return point.iConstraints()->violationNorm(point.values());
}

bool IEConstraintManifold::usesVaryingMetric() const {
  return params_.retractorCreator &&
         params_.retractorCreator->commonParams().useVaryingSigma;
}

IECMPoint IEConstraintManifold::retractAmbient(const IECMPoint& point,
                                  const VectorValues& delta,
                                  const std::optional<IndexSet>& blocking,
                                  IERetractionInfo* info,
                                  const VectorValues* metricSigmas) const {
  validatePointType(point);
  auto workspace = point.retractionWorkspace(metricSigmas);
  if (auto ieWorkspace = std::dynamic_pointer_cast<IERetractor>(workspace))
    return ieWorkspace->retract(&point, delta, blocking, info);
  if (inequalityCount(point) || (blocking && !blocking->empty()))
    throw std::invalid_argument("Inequalities require an IE retraction policy");
  return this->point(point.bindings(), workspace->retractValues(point, delta, info));
}

IECMPoint IEConstraintManifold::moveToBoundary(const IECMPoint& point, const IndexSet& active,
                                  IERetractionInfo* info,
                                  const VectorValues* metricSigmas) const {
  validatePointType(point);
  auto workspace = point.retractionWorkspace(metricSigmas);
  if (auto ieWorkspace = std::dynamic_pointer_cast<IERetractor>(workspace))
    return ieWorkspace->moveToBoundary(&point, active, info);
  if (inequalityCount(point) || !active.empty())
    throw std::invalid_argument("Inequalities require an IE retraction policy");
  return this->point(point.bindings(), workspace->retractValues(
      point, point.values().zeroVectors(), info));
}

void IEConstraintManifold::validatePointType(const CMPoint& point) const {
  if (point.manifold().get() != this)
    throw std::invalid_argument("Point belongs to a different runtime type");
}

}  // namespace gtdynamics
