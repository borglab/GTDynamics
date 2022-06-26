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

class ConstVarFactor : public NoiseModelFactor {
 protected:
  typedef NoiseModelFactor Base;
  typedef ConstVarFactor This;
  Base::shared_ptr base_factor_;
  KeySet fixed_keys_;
  Values fixed_values_;
  std::vector<size_t> base_key_index_;

 public:
  /** Default constructor for I/O only */
  ConstVarFactor() {}

  /** Destructor */
  ~ConstVarFactor() override {}

  /**
   * Constructor
   * @param base_factor   original factor on X
   * @param fixed_keys  keys of fixed variables (may contain more variables
   * apart from ones in the factor)
   */
  ConstVarFactor(const Base::shared_ptr& base_factor, const KeySet& fixed_keys)
      : Base(base_factor->noiseModel(),
             computeNewKeys(base_factor, fixed_keys)),
        base_factor_(base_factor),
        fixed_keys_(computeFixedKeys(base_factor, fixed_keys)),
        base_key_index_(computeBaseKeyIndex(base_factor, fixed_keys)) {
    if (!checkActive()) {
      return;
    }
  }

  /// Set values of fixed variables.
  void setFixedValues(const Values& values);

 protected:
  /** Compute keys for the new factor */
  static KeyVector computeNewKeys(const Base::shared_ptr& base_factor,
                                  const KeySet& fixed_keys);

  static KeySet computeFixedKeys(const Base::shared_ptr& base_factor,
                                 const KeySet& fixed_keys);

  static std::vector<size_t> computeBaseKeyIndex(const Base::shared_ptr& base_factor,
                                 const KeySet& fixed_keys);

 public:
  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   * Override this method to finish implementing an N-way factor.
   * If the optional arguments is specified, it should compute
   * both the function evaluation and its derivative(s) in H.
   */
  virtual Vector unwhitenedError(
      const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const override;

  /** Return a deep copy of this factor. */
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** Check if a variable in the base factor is fixed. */
  inline bool isFixed(const Key& key) const { return fixed_keys_.exists(key); }

  /** Check if the factor is active. Note: if all the variables of the original
   * factor are fully constrained, no updates can be made.*/
  inline bool checkActive() const { return size() > 0; }

};  // \class ConstVarFactor

}  // namespace gtsam
