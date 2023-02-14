#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/** A factor that adds a constant bias term to the original factor.
 * This factor is used in augmented Lagrangian optimizer to create biased cost
 * functions.
 * Note that the noise model is stored twice (both in base factor and the
 * noisemodel of substitute factor. The noisemodel in the base factor will be
 * ignored. */
class BiasedFactor : public NoiseModelFactor {
 protected:
  typedef NoiseModelFactor Base;
  typedef BiasedFactor This;

  // original factor
  Base::shared_ptr base_factor_;
  Vector bias_;

 public:
  typedef std::shared_ptr<This> shared_ptr;

  /** Default constructor for I/O only */
  BiasedFactor() {}

  /** Destructor */
  ~BiasedFactor() override {}

  /**
   * Constructor
   * @param base_factor   original factor on X
   * @param bias  the bias term
   */
  BiasedFactor(const Base::shared_ptr &base_factor, const Vector &bias)
      : Base(base_factor->noiseModel(), base_factor->keys()),
        base_factor_(base_factor),
        bias_(bias) {}

  BiasedFactor(const Base::shared_ptr &base_factor, const Vector &bias,
               const SharedNoiseModel &noise_model)
      : Base(noise_model, base_factor->keys()),
        base_factor_(base_factor),
        bias_(bias) {}

  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   * Override this method to finish implementing an N-way factor.
   * If the optional arguments is specified, it should compute
   * both the function evaluation and its derivative(s) in H.
   */
  virtual Vector unwhitenedError(
      const Values &x,
      gtsam::OptionalMatrixVecType H = nullptr) const override {
    return base_factor_->unwhitenedError(x, H) + bias_;
  }

  /** Return a deep copy of this factor. */
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &boost::serialization::make_nvp(
        "BiasedFactor", boost::serialization::base_object<Base>(*this));
    ar &BOOST_SERIALIZATION_NVP(base_factor_);
    ar &BOOST_SERIALIZATION_NVP(bias_);
  }
#endif

};  // \class BiasedFactor

}  // namespace gtsam
