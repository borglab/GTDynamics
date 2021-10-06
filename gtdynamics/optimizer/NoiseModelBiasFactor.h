/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  NoiseModelBiasFactor.h
 * @brief Factor that adds bias to NoiseModelFactor, used in Augmented
 * Lagrangian optimizer.
 * @author: Yetong Zhang
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtdynamics {

/** Factor that adds bias to NoiseModelFactor, used in Augmented Lagrangian
 * optimizer. */
class NoiseModelBiasFactor : public gtsam::NoiseModelFactor {
 protected:
  // handy typedefs
  typedef NoiseModelBiasFactor This;
  typedef gtsam::NoiseModelFactor Base;

  gtsam::NoiseModelFactor::shared_ptr original_factor_;
  gtsam::Vector bias_;

 public:
  typedef boost::shared_ptr<This> shared_ptr;

  /** Default constructor. */
  NoiseModelBiasFactor() {}

  /** Destructor. */
  ~NoiseModelBiasFactor() override {}

  /**
   * @brief Constructor from original factor and bias.
   */
  NoiseModelBiasFactor(
      const gtsam::NoiseModelFactor::shared_ptr& original_factor,
      const gtsam::Vector& bias)
      : Base(original_factor->noiseModel(), original_factor->keys()),
        original_factor_(original_factor),
        bias_(bias) {}

 protected:
 public:
  /** Print */
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    std::cout << "bias: " << bias_ << "\n";
  }

  /** First call the unwhitenedError of original factor then add the bias. */
  virtual gtsam::Vector unwhitenedError(
      const gtsam::Values& x, boost::optional<std::vector<gtsam::Matrix>&> H =
                                  boost::none) const override {
    gtsam::Vector original_error = original_factor_->unwhitenedError(x, H);
    return original_error + bias_;
  }

  /** First call the linearize of original factor then add the bias. */
  boost::shared_ptr<gtsam::GaussianFactor> linearize(
      const gtsam::Values& x) const override {
    auto original_linear_factor = original_factor_->linearize(x);

    auto jacobian = original_linear_factor->jacobian();
    gtsam::Matrix A = jacobian.first;
    gtsam::Vector b = jacobian.second;
    b -= noiseModel()->whiten(bias_);

    // TODO(yetong): find a more efficient way of construction.
    std::map<gtsam::Key, gtsam::Matrix> terms;
    int start_idx = 0;
    for (auto key_it = original_linear_factor->begin();
         key_it != original_linear_factor->end(); key_it++) {
      gtsam::Key key = *key_it;
      auto dim = original_linear_factor->getDim(key_it);
      auto mat = A.middleCols(start_idx, dim);
      terms[key] = mat;
      start_idx += dim;
    }

    return gtsam::GaussianFactor::shared_ptr(
        new gtsam::JacobianFactor(terms, b));
  }

  /**
   * Creates a shared_ptr clone of the factor with a new noise model.
   */
  NoiseModelFactor::shared_ptr cloneWithNewNoiseModel(
      const gtsam::SharedNoiseModel newNoise) const {
    auto original_factor_new_noise =
        original_factor_->cloneWithNewNoiseModel(newNoise);
    return gtsam::NoiseModelFactor::shared_ptr(
        new This(original_factor_new_noise, bias_));
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NonlinearFactor", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(noiseModel_);
  }

};  // \class NoiseModelBiasFactor
}  // namespace gtdynamics