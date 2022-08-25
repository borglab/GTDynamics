#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/assign/list_of.hpp>
#include <boost/serialization/base_object.hpp>

namespace gtsam {

/** A factor that substitute certain variables of a base factor with constraint
 * manifold variables.
 * It is used for constraint manifold optimization, since the variables
 * connected with constraint factors will be replaced by constraint manifold
 * variables.
 * Note that the noise model is stored twice (both in base factor and the
 * noisemodel of substitute factor. The noisemodel in the base factor will be
 * ignored. */
class ConstBiasFactor : public NoiseModelFactor {
protected:
  typedef NoiseModelFactor Base;
  typedef ConstBiasFactor This;

  // original factor
  Base::shared_ptr base_factor_;
  Vector bias_;

public:
  typedef boost::shared_ptr<This> shared_ptr;

  /** Default constructor for I/O only */
  ConstBiasFactor() {}

  /** Destructor */
  ~ConstBiasFactor() override {}

  /**
   * Constructor
   * @param base_factor   original factor on X
   * @param bias  the bias term
   */
  ConstBiasFactor(const Base::shared_ptr &base_factor, const Vector &bias)
      : Base(base_factor->noiseModel(), base_factor->keys()),
        base_factor_(base_factor), bias_(bias) {}

  ConstBiasFactor(const Base::shared_ptr &base_factor, const Vector &bias,
                  const SharedNoiseModel &noise_model)
      : Base(noise_model, base_factor->keys()), base_factor_(base_factor),
        bias_(bias) {}

  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   * Override this method to finish implementing an N-way factor.
   * If the optional arguments is specified, it should compute
   * both the function evaluation and its derivative(s) in H.
   */
  virtual Vector unwhitenedError(
      const Values &x,
      boost::optional<std::vector<Matrix> &> H = boost::none) const override {
    // std::cout << "base_error:\n" << base_factor_->unwhitenedError(x) << "\n";
    // std::cout << "bias:\n" << bias_ << "\n";
    // Vector error = base_factor_->unwhitenedError(x, H) - bias_;
    // std::cout << "error:\n" << error << "\n";
    // return error;
    return base_factor_->unwhitenedError(x, H) + bias_;
  }

  /** Return a deep copy of this factor. */
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &boost::serialization::make_nvp(
        "ConstBiasFactor", boost::serialization::base_object<Base>(*this));
    ar &BOOST_SERIALIZATION_NVP(base_factor_);
    ar &BOOST_SERIALIZATION_NVP(bias_);
  }

}; // \class ConstBiasFactor

} // namespace gtsam
