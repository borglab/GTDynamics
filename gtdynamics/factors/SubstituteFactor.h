/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SubstituteFactor.h
 * @brief Factor that substitute certain variables with its corresponding
 * recover function from the constraint manifold. It is used to represent the
 * equivalent new cost factors on manifold varaibles.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <map>

namespace gtdynamics {

using gtsam::Key;
using gtsam::KeySet;
using gtsam::KeyVector;
using gtsam::NoiseModelFactor;
using gtsam::Values;
using gtsam::Vector;

/** A factor that substitute certain variables of a base factor with constraint
 * manifold variables.
 * It is used for constraint manifold optimization, since the variables
 * connected with constraint factors will be replaced by constraint manifold
 * variables.
 * Note that the noise model is stored twice (both in base factor and the
 * noisemodel of substitute factor. The noisemodel in the base factor will be
 * ignored. */
class SubstituteFactor : public NoiseModelFactor {
 protected:
  typedef NoiseModelFactor Base;
  typedef SubstituteFactor This;

  // original factor
  Base::shared_ptr base_factor_;
  // map from base key to key of corresponding constraint manifold
  std::map<Key, Key> replacement_map_;
  // keys of constraint manifold variables
  KeySet cmanifold_keys_;
  // keys of unconstrianed variables
  KeySet unconstrained_keys_;
  // base variables that are in fully constrained CCCs
  Values fc_values_;
  // map from base_key to index of the key in base factor
  std::map<Key, size_t> base_key_index_;

 public:
  typedef std::shared_ptr<This> shared_ptr;

  /// Default constructor for I/O only.
  SubstituteFactor() {}

  /// Destructor.
  ~SubstituteFactor() override {}

  /**
   * Constructor
   * @param base_factor   original factor on X
   * @param replacement_map map from X to CCC
   * @param fc_manifolds CCC that are fully constrained (dimension 0)
   */
  SubstituteFactor(const Base::shared_ptr& base_factor,
                   const std::map<Key, Key>& replacement_map,
                   const Values& fc_manifolds = Values())
      : Base(base_factor->noiseModel(),
             computeNewKeys(base_factor, replacement_map, fc_manifolds)),
        base_factor_(base_factor),
        replacement_map_(replacement_map) {
    if (!checkActive()) {
      return;
    }
    computeBaseKeyIndex();
    classifyKeys(fc_manifolds);
  }

 protected:
  /// Compute keys for the new factor.
  static KeyVector computeNewKeys(const Base::shared_ptr& base_factor,
                                  const std::map<Key, Key>& replacement_map,
                                  const Values& fc_manifolds);

  /// Construct map from base key to key index in base factor.
  void computeBaseKeyIndex();

  /** Classify the variables as either constarined or unconstrained in the
   * subsitute factor. */
  void classifyKeys(const Values& fc_manifolds);

 public:
  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   * Override this method to finish implementing an N-way factor.
   * If the optional arguments is specified, it should compute
   * both the function evaluation and its derivative(s) in H.
   */
  virtual Vector unwhitenedError(
      const Values& x, gtsam::OptionalMatrixVecType H = nullptr) const override;

  /// Return a deep copy of this factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Check if a variable in the base factor is substituted.
  inline bool isReplaced(const Key& key) const {
    return replacement_map_.find(key) != replacement_map_.end();
  }

  /** Check if the factor is active. Note: if all the variables of the original
   * factor are fully constrained, no updates can be made.*/
  inline bool checkActive() const { return size() > 0; }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /// Serialization function.
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "SubstituteFactor", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(base_factor_);
    ar& BOOST_SERIALIZATION_NVP(replacement_map_);
    ar& BOOST_SERIALIZATION_NVP(cmanifold_keys_);
    ar& BOOST_SERIALIZATION_NVP(unconstrained_keys_);
    ar& BOOST_SERIALIZATION_NVP(fc_values_);
    ar& BOOST_SERIALIZATION_NVP(base_key_index_);
  }
#endif

};  // \class SubstituteFactor

}  // namespace gtdynamics
