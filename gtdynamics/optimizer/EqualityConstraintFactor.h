#pragma once

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/// Equality constraint that force g(x) = c
template <typename T>
class EqualityConstraintFactor : public ExpressionFactor<T> {
 protected:
  Vector tolerance_;
  double mu_;
  Vector bias_;
  typedef EqualityConstraintFactor<T> This;

 public:
 // TODO: sqrt(mu)
  EqualityConstraintFactor(
      const Expression<T>& expression, const T& value,
      const Vector& tolerance = Vector::Ones(traits<T>::dimension),
      const double& mu = 1,
      const Vector& bias = Vector::Zero(traits<T>::dimension))
      : ExpressionFactor<T>(noiseModel::Diagonal::Sigmas(tolerance / mu), value,
                            expression),
        tolerance_(tolerance),
        mu_(mu),
        bias_(bias) {}

  void update_mu(const double& mu) {
    mu_ = mu;
    NoiseModelFactor::noiseModel_ =
        noiseModel::Diagonal::Sigmas(tolerance_ / mu_);
  }

  void update_bias(const Vector& bias) {
    bias_ = bias;
  }

  Vector unwhitenedError(
      const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const override {
    Vector original_error = ExpressionFactor<T>::unwhitenedError(x, H);
    return original_error + bias_;
  }

  boost::shared_ptr<GaussianFactor> linearize(const Values& x) const override {
    auto original_linear_factor = ExpressionFactor<T>::linearize(x);

    auto jacobian = original_linear_factor->jacobian();
    Matrix A = jacobian.first;
    Vector b = jacobian.second;
    b -= NoiseModelFactor::noiseModel()->whiten(bias_);

    std::map<Key, Matrix> terms;
    int start_idx = 0;
    for (auto key_it = original_linear_factor->begin();
         key_it != original_linear_factor->end(); key_it++) {
      Key key = *key_it;
      auto dim = original_linear_factor->getDim(key_it);
      auto mat = A.middleCols(start_idx, dim);
      terms[key] = mat;
      start_idx += dim;
    }

    return GaussianFactor::shared_ptr(new JacobianFactor(terms, b));
  }

  /// g(x)
  gtsam::Vector original_error(const Values& x) {
    return ExpressionFactor<T>::unwhitenedError(x);
  }

  /// g(x) scaled by tolerance
  gtsam::Vector tolerance_scaled_error(const Values& x) {
    Vector error = original_error(x);
    auto noise = noiseModel::Diagonal::Sigmas(tolerance_);
    return noise->whiten(error);
  }

  bool feasible(const Values& x) {
    Vector gx = original_error(x);
    for (int i = 0; i < gx.size(); i++) {
      if (abs(gx[i]) > tolerance_[i]) {
        return false;
      }
    }
    return true;
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print relies on Testable traits being defined for T
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    ExpressionFactor<T>::print(s, keyFormatter);
    std::cout << "tolerance: " << tolerance_ << "\n";
    std::cout << "mu: " << mu_ << "\n";
    std::cout << "bias: " << bias_ << "\n";
  }


  // TODO: clone, copy constructor
};

}  // namespace gtsam