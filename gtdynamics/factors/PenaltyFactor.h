#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <functional>

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#endif

namespace gtdynamics {

using gtsam::Matrix;
using gtsam::NoiseModelFactor;
using gtsam::OptionalJacobian;
using gtsam::SharedNoiseModel;
using gtsam::Values;
using gtsam::Vector;

inline std::function<double(const double &x, OptionalJacobian<1, 1> H)>
RampFunction(const double offset) {
  auto func = [=](const double &x, OptionalJacobian<1, 1> H = {}) -> double {
    if (x < offset) {
      if (H) {
        H->setConstant(0);
      }
      return 0;
    } else {
      if (H) {
        H->setConstant(1);
      }
      return x - offset;
    }
  };
  return func;
}

/** A factor that implements the penalty function for inequality constraint
 * for g(x)>=0. the penalty function is ramp(-g(x))
 * for g(x)<=0, the penalty function is ramp(g(x))
 * Note that the noise model is stored twice (both in base factor and the
 * noisemodel of substitute factor. The noisemodel in the base factor will be
 * ignored. */
class PenaltyFactor : public NoiseModelFactor {
protected:
  typedef NoiseModelFactor Base;
  typedef PenaltyFactor This;

  // original factor
  Base::shared_ptr base_factor_;
  bool positive_;

public:
  typedef std::shared_ptr<This> shared_ptr;

  /** Default constructor for I/O only */
  PenaltyFactor() {}

  /** Destructor */
  ~PenaltyFactor() override {}

  /**
   * Constructor
   * @param base_factor   original factor on X
   * @param bias  the bias term
   */
  PenaltyFactor(const Base::shared_ptr &base_factor, bool positive = true)
      : Base(base_factor->noiseModel(), base_factor->keys()),
        base_factor_(base_factor), positive_(positive) {}

  PenaltyFactor(const Base::shared_ptr &base_factor,
                const SharedNoiseModel &noise_model, bool positive = true)
      : Base(noise_model, base_factor->keys()), base_factor_(base_factor),
        positive_(positive) {}

  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   * Override this method to finish implementing an N-way factor.
   * If the optional arguments is specified, it should compute
   * both the function evaluation and its derivative(s) in H.
   */
  virtual Vector
  unwhitenedError(const Values &x,
                  gtsam::OptionalMatrixVecType H = nullptr) const override {
    Vector error = base_factor_->unwhitenedError(x, H);
    if (positive_) {
      error = -error;
      if (H) {
        for (Matrix &m : *H) {
          m = -m;
        }
      }
    }
    for (int i = 0; i < error.size(); i++) {
      if (error(i) < 0) {
        error(i) = 0;
        if (H) {
          for (Matrix &m : *H) {
            m.row(i).setZero();
          }
        }
      }
    }

    return error;
  }

  /** Return a deep copy of this factor. */
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** Print function. */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "penalty factor" << std::endl;
    Base::print("", keyFormatter);
  }

private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &boost::serialization::make_nvp(
        "PenaltyFactor", boost::serialization::base_object<Base>(*this));
    ar &BOOST_SERIALIZATION_NVP(base_factor_);
  }
#endif

}; // \class PenaltyFactor

} // namespace gtdynamics
