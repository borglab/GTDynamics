
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

class NonlinearBiasFactor : public NoiseModelFactor {
 protected:
  // handy typedefs
  typedef NonlinearBiasFactor This;
  typedef NoiseModelFactor Base;

  NoiseModelFactor::shared_ptr original_factor_;
  Vector bias_;

 public:
  typedef boost::shared_ptr<This> shared_ptr;

  /** Default constructor for I/O only */
  NonlinearBiasFactor() {}

  /** Destructor */
  ~NonlinearBiasFactor() override {}

  /**
   * Constructor
   */
  NonlinearBiasFactor(const NoiseModelFactor::shared_ptr& original_factor,
                      const Vector& bias)
      : Base(original_factor->noiseModel(), original_factor->keys()),
        original_factor_(original_factor),
        bias_(bias) {}

 protected:
 public:
  /** Print */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const
                                            override {
    Base::print(s, keyFormatter);
    std::cout << "bias: " << bias_ << "\n";
  }

  // /** Check if two factors are equal */
  // bool equals(const NonlinearFactor& f, double tol = 1e-9) const override;

  // /** get the dimension of the factor (number of rows on linearization) */
  // size_t dim() const override { return original_factor_->dim(); }

  // /// access to the noise model
  // const SharedNoiseModel& noiseModel() const override {
  //   return original_factor_->noiseModel();
  // }

  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   * Override this method to finish implementing an N-way factor.
   * If the optional arguments is specified, it should compute
   * both the function evaluation and its derivative(s) in H.
   */
  virtual Vector unwhitenedError(
      const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const override {
    Vector original_error = original_factor_->unwhitenedError(x, H);
    return original_error + bias_;
  }

  /**
   * Vector of errors, whitened
   * This is the raw error, i.e., i.e. \f$ (h(x)-z)/\sigma \f$ in case of a
   * Gaussian
   */
  Vector whitenedError(const Values& c) const {
    Vector original_whitened_error = original_factor_->whitenedError(c);
    return original_whitened_error + noiseModel()->whiten(bias_);
  }

  /**
   * Vector of errors, whitened, but unweighted by any loss function
   */
  Vector unweightedWhitenedError(const Values& c) const {
    Vector original_unweighted_whitened_error =
        original_factor_->whitenedError(c);
    return original_unweighted_whitened_error +
           noiseModel()->unweightedWhiten(bias_);
  }

  // /**
  //  * Compute the effective weight of the factor from the noise model.
  //  */
  // double weight(const Values& c) const override {
  //   return original_factor_->weight(c);
  // }

  // /**
  //  * Calculate the error of the factor.
  //  * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of
  //  * Gaussian. In this class, we take the raw prediction error \f$ h(x)-z \f$,
  //  * ask the noise model to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and
  //  * then multiply by 0.5.
  //  */
  // double error(const Values& c) const override {}

  /**
   * Linearize a non-linearFactorN to get a GaussianFactor,
   * \f$ Ax-b \approx h(x+\delta x)-z = h(x) + A \delta x - z \f$
   * Hence \f$ b = z - h(x) = - \mathtt{error\_vector}(x) \f$
   */
  boost::shared_ptr<GaussianFactor> linearize(const Values& x) const override {
    auto original_linear_factor = original_factor_->linearize(x);

    auto jacobian = original_linear_factor->jacobian();
    Matrix A = jacobian.first;
    Vector b = jacobian.second;
    b -= noiseModel()->whiten(bias_);

    std::map<Key, Matrix> terms;
    int start_idx = 0;
    for (auto key_it = original_linear_factor->begin(); key_it != original_linear_factor->end(); key_it++) {
      Key key = *key_it;
      auto dim = original_linear_factor->getDim(key_it);
      auto mat = A.middleCols(start_idx, dim);
      terms[key] = mat;
      start_idx += dim;
    }

    return GaussianFactor::shared_ptr(new JacobianFactor(terms, b));
  }

  /**
   * Creates a shared_ptr clone of the
   * factor with a new noise model
   */
  NoiseModelFactor::shared_ptr cloneWithNewNoiseModel(
      const SharedNoiseModel newNoise) const {
    auto original_factor_new_noise =
        original_factor_->cloneWithNewNoiseModel(newNoise);
    auto new_factor =
        boost::make_shared<This>(original_factor_new_noise, bias_);
    return boost::dynamic_pointer_cast<NoiseModelFactor>(new_factor);
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

};  // \class NoiseModelFactor
}  // namespace gtsam