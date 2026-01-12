/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  GeneralPriorFactor.h
 *  @author Yetong Zhang
 **/
#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <string>

namespace gtsam {

/**
 * A class for a soft prior on any Value type
 */
class GeneralPriorFactor : public NoiseModelFactor {
 private:
  typedef NoiseModelFactor Base;
  Values prior_; /** The measurement */
  size_t dim_;

 public:
  /// shorthand for a smart pointer to a factor
  typedef typename std::shared_ptr<GeneralPriorFactor> shared_ptr;

  /// Typedef to this class
  typedef GeneralPriorFactor This;

  /** default constructor - only use for serialization */
  GeneralPriorFactor() {}

  ~GeneralPriorFactor() override {}

  /** Constructor */
  GeneralPriorFactor(Key key, const Value &prior,
                     const SharedNoiseModel &model = nullptr)
      : Base(model, KeyVector{key}) {
    prior_.insert(key, prior);
    dim_ = prior.dim();
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** implement functions needed for Testable */

  /** print */
  void print(const std::string &s, const KeyFormatter &keyFormatter =
                                       DefaultKeyFormatter) const override {
    std::cout << s << "GeneralPriorFactor on " << keyFormatter(this->keys()[0])
              << "\n";
    if (this->noiseModel_)
      this->noiseModel_->print("  noise model: ");
    else
      std::cout << "no noise model" << std::endl;
  }

  /** equals */
  bool equals(const NonlinearFactor &expected,
              double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           prior_.equals(e->prior_, tol);
  }

  /** implement functions needed to derive from Factor */

  /** vector of errors */
  Vector unwhitenedError(const Values &x, gtsam::OptionalMatrixVecType H =
                                              nullptr) const override {
    if (H) (*H)[0] = Matrix::Identity(dim_, dim_);
    return -x.at(keys_[0]).localCoordinates_(prior_.at(keys_[0]));
  }

  const Value &prior() const { return prior_.at(keys_[0]); }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
    ar &BOOST_SERIALIZATION_NVP(prior_);
  }
#endif
};

template <typename CONTAINER>
inline void AddGeneralPriors(const Values &values, const CONTAINER &keys,
                             double sigma, NonlinearFactorGraph &graph) {
  for (const Key &key : keys) {
    const Value &value = values.at(key);
    graph.emplace_shared<GeneralPriorFactor>(
        key, value, noiseModel::Isotropic::Sigma(value.dim(), sigma));
  }
}

inline void AddGeneralPriors(const Values &values, double sigma,
                             NonlinearFactorGraph &graph) {
  AddGeneralPriors(values, values.keys(), sigma, graph);
}

}  // namespace gtsam
