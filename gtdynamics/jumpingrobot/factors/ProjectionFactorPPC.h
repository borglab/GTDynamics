/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ProjectionFactorPPC.h
 * @brief Derived from CustomProjectionFactor, but estimates body-camera
 * transform and calibration in addition to body pose and 3D landmark
 * @author Yetong Zhang
 */

#pragma once

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>

namespace gtdynamics {

/**
 * Non-linear factor for a constraint derived from a 2D measurement. This factor
 * estimates the body pose, body-camera transform, 3D landmark, and calibration.
 * @addtogroup SLAM
 */
class CustomProjectionFactor
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Point3,
                                      gtsam::Cal3Bundler> {
 protected:
  gtsam::Point2 measured_;  ///< 2D measurement

  // verbosity handling for Cheirality Exceptions
  bool throwCheirality_;  ///< If true, rethrows Cheirality exceptions (default:
                          ///< false)
  bool verboseCheirality_;  ///< If true, prints text for Cheirality exceptions
                            ///< (default: false)

 public:
  /// shorthand for base class type
  typedef gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Point3,
                                   gtsam::Cal3Bundler>
      Base;

  /// shorthand for this class
  typedef CustomProjectionFactor This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  CustomProjectionFactor()
      : measured_(0.0, 0.0),
        throwCheirality_(false),
        verboseCheirality_(false) {}

  /**
   * Constructor
   * TODO: Mark argument order standard (keys, measurement, parameters)
   * @param measured is the 2 dimensional location of point in image (the
   * measurement)
   * @param model is the standard deviation
   * @param poseKey is the index of the camera
   * @param pointKey is the index of the landmark
   * @param K shared pointer to the constant calibration
   */
  CustomProjectionFactor(const gtsam::Point2& measured,
                         const gtsam::SharedNoiseModel& model,
                         gtsam::Key poseKey, gtsam::Key pointKey,
                         gtsam::Key calibKey)
      : Base(model, poseKey, pointKey, calibKey),
        measured_(measured),
        throwCheirality_(false),
        verboseCheirality_(false) {}

  /**
   * Constructor with exception-handling flags
   * TODO: Mark argument order standard (keys, measurement, parameters)
   * @param measured is the 2 dimensional location of point in image (the
   * measurement)
   * @param model is the standard deviation
   * @param poseKey is the index of the camera
   * @param pointKey is the index of the landmark
   * @param K shared pointer to the constant calibration
   * @param throwCheirality determines whether Cheirality exceptions are
   * rethrown
   * @param verboseCheirality determines whether exceptions are printed for
   * Cheirality
   */
  CustomProjectionFactor(const gtsam::Point2& measured,
                         const gtsam::SharedNoiseModel& model,
                         gtsam::Key poseKey, gtsam::Key pointKey,
                         gtsam::Key calibKey, bool throwCheirality,
                         bool verboseCheirality)
      : Base(model, poseKey, pointKey, calibKey),
        measured_(measured),
        throwCheirality_(throwCheirality),
        verboseCheirality_(verboseCheirality) {}

  /** Virtual destructor */
  virtual ~CustomProjectionFactor() {}

  /// @return a deep copy of this factor
  NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new This(*this)));
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "CustomProjectionFactor, z = ";
    gtsam::traits<gtsam::Point2>::Print(measured_);
    Base::print("", keyFormatter);
  }

  /// equals
  bool equals(const gtsam::NonlinearFactor& p,
              double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) &&
           gtsam::traits<gtsam::Point2>::Equals(this->measured_, e->measured_,
                                                tol);
  }

  /// Evaluate error h(x)-z and optionally derivatives
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose, const gtsam::Point3& point,
      const gtsam::Cal3Bundler& K,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none) const override {
    try {
      gtsam::PinholeCamera<gtsam::Cal3Bundler> camera(pose, K);
      return camera.project(point, H1, H2, H3) - measured_;
    } catch (gtsam::CheiralityException& e) {
      if (H1) *H1 = gtsam::Matrix::Zero(2, 6);
      if (H2) *H2 = gtsam::Matrix::Zero(2, 3);
      if (H3) *H3 = gtsam::Matrix::Zero(2, gtsam::Cal3Bundler::Dim());
      if (verboseCheirality_)
        std::cout << e.what() << ": Landmark "
                  << gtsam::DefaultKeyFormatter(this->key2())
                  << " moved behind camera "
                  << gtsam::DefaultKeyFormatter(this->key1()) << std::endl;
      if (throwCheirality_) throw e;
    }
    return gtsam::Vector::Ones(2) * 2.0 * K.fx();
  }

  /** return the measurement */
  const gtsam::Point2& measured() const { return measured_; }

  /** return verbosity */
  inline bool verboseCheirality() const { return verboseCheirality_; }

  /** return flag for throwing cheirality exceptions */
  inline bool throwCheirality() const { return throwCheirality_; }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(measured_);
    ar& BOOST_SERIALIZATION_NVP(throwCheirality_);
    ar& BOOST_SERIALIZATION_NVP(verboseCheirality_);
  }
};

// /// traits
// struct gtsam::traits<CustomProjectionFactor > :
//     public Testable<CustomProjectionFactor > {
// };

typedef gtsam::PriorFactor<gtsam::Cal3Bundler> PriorFactorCal3Bundler;

}  // namespace gtdynamics