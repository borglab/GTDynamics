/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file FrictionConeConstraints.cpp
 * @brief Direct friction-cone residuals, Jacobians, and signed boundaries.
 * @author Frank Dellaert
 */

#include <gtdynamics/constraints/FrictionConeConstraints.h>

#include <cmath>
#include <stdexcept>

namespace gtdynamics {
namespace {
using namespace gtsam;

double checkedSigma(double sigma) {
  if (!std::isfinite(sigma) || sigma <= 0)
    throw std::invalid_argument(
        "Friction constraint sigma must be positive and finite");
  return sigma;
}

void checkMu(const Vector2 &mu) {
  if (!mu.allFinite() || (mu.array() <= 0).any())
    throw std::invalid_argument(
        "Friction coefficients must be positive and finite");
}

/** An active face must use g(x), not the clamped inequality violation.
 * Store a typed copy so the boundary outlives its source and evaluates the
 * same direct residual, with the original sigma applied exactly once.
 */
template <class Constraint>
class Boundary : public NonlinearEqualityConstraint {
  Constraint constraint_;

public:
  explicit Boundary(const Constraint &constraint)
      : NonlinearEqualityConstraint(constrainedNoise(constraint.sigmas()),
                                    constraint.keys()),
        constraint_(constraint) {}
  Vector unwhitenedError(const Values &values,
                         OptionalMatrixVecType H = nullptr) const override {
    return constraint_.unwhitenedExpr(values, H);
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<Boundary>(*this);
  }
  // Rekey the owned constraint as well as the outer factor's key list.
  NonlinearFactor::shared_ptr rekey(const KeyVector &keys) const override {
    auto result = std::static_pointer_cast<Boundary>(
        NonlinearEqualityConstraint::rekey(keys));
    result->constraint_ =
        *std::static_pointer_cast<Constraint>(constraint_.rekey(keys));
    return result;
  }
  NonlinearFactor::shared_ptr
  rekey(const std::map<Key, Key> &mapping) const override {
    auto result = std::static_pointer_cast<Boundary>(
        NonlinearEqualityConstraint::rekey(mapping));
    result->constraint_ =
        *std::static_pointer_cast<Constraint>(constraint_.rekey(mapping));
    return result;
  }
};

/** A smooth penalty may be nonzero inside the feasible set; unlike an
 * inequality factor, this cost must remain active there. Chain its derivative
 * through the signed residual before applying the cost's noise model.
 */
template <class Constraint> class CustomPenalty : public NoiseModelFactor {
  Constraint constraint_;
  InequalityPenaltyFunction::shared_ptr function_;

public:
  CustomPenalty(const Constraint &constraint, const SharedNoiseModel &noise,
                InequalityPenaltyFunction::shared_ptr function)
      : NoiseModelFactor(noise, constraint.keys()), constraint_(constraint),
        function_(std::move(function)) {}
  Vector unwhitenedError(const Values &values,
                         OptionalMatrixVecType H = nullptr) const override {
    const double residual = constraint_.unwhitenedExpr(values, H)(0);
    if (!H)
      return Vector1((*function_)(residual));
    Matrix11 derivative;
    const double penalty = (*function_)(residual, derivative);
    for (Matrix &jacobian : *H)
      jacobian *= derivative(0, 0);
    return Vector1(penalty);
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<CustomPenalty>(*this);
  }
  // Rekey the owned constraint as well as the outer factor's key list.
  NonlinearFactor::shared_ptr rekey(const KeyVector &keys) const override {
    auto result =
        std::static_pointer_cast<CustomPenalty>(NoiseModelFactor::rekey(keys));
    result->constraint_ =
        *std::static_pointer_cast<Constraint>(constraint_.rekey(keys));
    return result;
  }
  NonlinearFactor::shared_ptr
  rekey(const std::map<Key, Key> &mapping) const override {
    auto result = std::static_pointer_cast<CustomPenalty>(
        NoiseModelFactor::rekey(mapping));
    result->constraint_ =
        *std::static_pointer_cast<Constraint>(constraint_.rekey(mapping));
    return result;
  }
};

// The redundant normal row in the elliptic factory uses the same direct API.
class NormalForceConstraint : public NonlinearInequalityConstraint {
public:
  NormalForceConstraint(Key key, double sigma)
      : NonlinearInequalityConstraint(
            constrainedNoise(Vector1(checkedSigma(sigma))), KeyVector{key}) {}
  Vector unwhitenedExpr(const Values &values,
                        OptionalMatrixVecType H = nullptr) const override {
    if (H)
      (*H)[0] = (Matrix13() << 0, 0, -1).finished();
    return Vector1(-values.at<Vector3>(keys()[0])(2));
  }
  NonlinearEqualityConstraint::shared_ptr
  createEqualityConstraint() const override {
    return std::make_shared<Boundary<NormalForceConstraint>>(*this);
  }
  NoiseModelFactor::shared_ptr
  penaltyFactorCustom(InequalityPenaltyFunction::shared_ptr function,
                      double mu = 1.0) const override {
    if (!function)
      return penaltyFactor(mu);
    return std::make_shared<CustomPenalty<NormalForceConstraint>>(
        *this, penaltyNoise(mu), std::move(function));
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<NormalForceConstraint>(*this);
  }
};
} // namespace

PyramidalFrictionConeConstraint::PyramidalFrictionConeConstraint(gtsam::Key key,
                                                                 size_t edge,
                                                                 double sigma)
    : NonlinearInequalityConstraint(
          constrainedNoise(gtsam::Vector1(checkedSigma(sigma))),
          gtsam::KeyVector{key}),
      edge_(edge) {
  if (edge >= 4)
    throw std::invalid_argument("Friction pyramid edge must be in [0, 4)");
}

gtsam::Vector PyramidalFrictionConeConstraint::unwhitenedExpr(
    const gtsam::Values &values, gtsam::OptionalMatrixVecType H) const {
  if (H) {
    (*H)[0] = gtsam::Matrix14::Zero();
    (*H)[0](0, edge_) = -1;
  }
  return gtsam::Vector1(-values.at<gtsam::Vector4>(keys()[0])(edge_));
}

gtsam::NonlinearInequalityConstraints
PyramidalFrictionConeConstraint::Create(gtsam::Key key, double sigma) {
  gtsam::NonlinearInequalityConstraints result;
  for (size_t edge = 0; edge < 4; ++edge)
    result.emplace_shared<PyramidalFrictionConeConstraint>(key, edge, sigma);
  return result;
}

gtsam::Matrix34
PyramidalFrictionConeConstraint::ForceMatrix(const gtsam::Vector2 &mu) {
  checkMu(mu);
  gtsam::Matrix34 result;
  result << mu(0), -mu(0), 0, 0, 0, 0, mu(1), -mu(1), 1, 1, 1, 1;
  return result;
}

gtsam::NonlinearEqualityConstraint::shared_ptr
PyramidalFrictionConeConstraint::createEqualityConstraint() const {
  return std::make_shared<Boundary<PyramidalFrictionConeConstraint>>(*this);
}

gtsam::NoiseModelFactor::shared_ptr
PyramidalFrictionConeConstraint::penaltyFactorCustom(
    gtsam::InequalityPenaltyFunction::shared_ptr function, double mu) const {
  if (!function)
    return penaltyFactor(mu);
  return std::make_shared<CustomPenalty<PyramidalFrictionConeConstraint>>(
      *this, penaltyNoise(mu), std::move(function));
}

gtsam::NonlinearFactor::shared_ptr
PyramidalFrictionConeConstraint::clone() const {
  return std::make_shared<PyramidalFrictionConeConstraint>(*this);
}

EllipticFrictionConeConstraint::EllipticFrictionConeConstraint(
    gtsam::Key key, const gtsam::Vector2 &mu, double sigma)
    : NonlinearInequalityConstraint(
          constrainedNoise(gtsam::Vector1(checkedSigma(sigma))),
          gtsam::KeyVector{key}) {
  checkMu(mu);
  inverseMu_ = mu.cwiseInverse();
}

gtsam::Vector EllipticFrictionConeConstraint::unwhitenedExpr(
    const gtsam::Values &values, gtsam::OptionalMatrixVecType H) const {
  const gtsam::Vector3 &f = values.at<gtsam::Vector3>(keys()[0]);
  const gtsam::Vector2 t = f.head<2>().cwiseProduct(inverseMu_);
  const double norm = std::hypot(t(0), t(1));
  if (H) {
    (*H)[0] = (gtsam::Matrix13() << 0, 0, -1).finished();
    if (norm > 0)
      (*H)[0].leftCols(2) = (t.cwiseProduct(inverseMu_) / norm).transpose();
  }
  return gtsam::Vector1(norm - f(2));
}

gtsam::NonlinearInequalityConstraints
EllipticFrictionConeConstraint::Create(gtsam::Key key, const gtsam::Vector2 &mu,
                                       double sigma) {
  gtsam::NonlinearInequalityConstraints result;
  result.emplace_shared<NormalForceConstraint>(key, sigma);
  result.emplace_shared<EllipticFrictionConeConstraint>(key, mu, sigma);
  return result;
}

gtsam::NonlinearEqualityConstraint::shared_ptr
EllipticFrictionConeConstraint::createEqualityConstraint() const {
  return std::make_shared<Boundary<EllipticFrictionConeConstraint>>(*this);
}

gtsam::NoiseModelFactor::shared_ptr
EllipticFrictionConeConstraint::penaltyFactorCustom(
    gtsam::InequalityPenaltyFunction::shared_ptr function, double mu) const {
  if (!function)
    return penaltyFactor(mu);
  return std::make_shared<CustomPenalty<EllipticFrictionConeConstraint>>(
      *this, penaltyNoise(mu), std::move(function));
}

gtsam::NonlinearFactor::shared_ptr
EllipticFrictionConeConstraint::clone() const {
  return std::make_shared<EllipticFrictionConeConstraint>(*this);
}

SignedSquareFrictionConeConstraint::SignedSquareFrictionConeConstraint(
    gtsam::Key key, const gtsam::Vector2 &mu, double sigma)
    : NonlinearInequalityConstraint(
          constrainedNoise(gtsam::Vector1(checkedSigma(sigma))),
          gtsam::KeyVector{key}) {
  checkMu(mu);
  ratioSquared_ = (mu(0) / mu(1)) * (mu(0) / mu(1));
  muSquared_ = mu(0) * mu(0);
}

gtsam::Vector SignedSquareFrictionConeConstraint::unwhitenedExpr(
    const gtsam::Values &values, gtsam::OptionalMatrixVecType H) const {
  const gtsam::Vector3 &f = values.at<gtsam::Vector3>(keys()[0]);
  if (H)
    (*H)[0] = (gtsam::Matrix13() << 2 * f(0), 2 * ratioSquared_ * f(1),
               -2 * muSquared_ * std::abs(f(2)))
                  .finished();
  return gtsam::Vector1(f(0) * f(0) + ratioSquared_ * f(1) * f(1) -
                        muSquared_ * f(2) * std::abs(f(2)));
}

gtsam::NonlinearEqualityConstraint::shared_ptr
SignedSquareFrictionConeConstraint::createEqualityConstraint() const {
  return std::make_shared<Boundary<SignedSquareFrictionConeConstraint>>(*this);
}

gtsam::NoiseModelFactor::shared_ptr
SignedSquareFrictionConeConstraint::penaltyFactorCustom(
    gtsam::InequalityPenaltyFunction::shared_ptr function, double mu) const {
  if (!function)
    return penaltyFactor(mu);
  return std::make_shared<CustomPenalty<SignedSquareFrictionConeConstraint>>(
      *this, penaltyNoise(mu), std::move(function));
}

gtsam::NonlinearFactor::shared_ptr
SignedSquareFrictionConeConstraint::clone() const {
  return std::make_shared<SignedSquareFrictionConeConstraint>(*this);
}

} // namespace gtdynamics
