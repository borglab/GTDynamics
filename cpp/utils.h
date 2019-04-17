/**
 * @file  utils.h
 * @brief a few utilities
 * @Author: Frank Dellaert and Mandy Xie
 */

#ifndef UTILS_H
#define UTILS_H

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <string>
#include <cmath>
#include <vector>
#include <boost/optional.hpp>

namespace manipulator {

/** Create unit twist for axis direction
    Keyword argument:
        w -- axis direction
        p -- p some point on axis
    return unit twist
*/
gtsam::Vector6 unit_twist(const gtsam::Vector3 &w, const gtsam::Vector3 &p);

/** convert angle to radians
 */
double radians(double degree);

gtsam::Vector radians(const gtsam::Vector &degree);

/// calculate Phi
inline gtsam::Matrix calcPhi(double tau) {
  return (gtsam::Matrix(3, 3) << 1, tau, 0.5 * tau * tau, 0, 1, tau, 0, 0, 1)
      .finished();
}

/// get Qc covariance matrix from noise model
gtsam::Matrix getQc(const gtsam::SharedNoiseModel Qc_model);

/// calculate Q
inline gtsam::Matrix calcQ(const gtsam::Matrix &Qc, double tau) {
  assert(Qc.rows() == Qc.cols());
  return (gtsam::Matrix(3 * Qc.rows(), 3 * Qc.rows())
              << 1.0 / 20 * pow(tau, 5.0) * Qc,
          1.0 / 8 * pow(tau, 4.0) * Qc, 1.0 / 6 * pow(tau, 3.0) * Qc,
          1.0 / 8 * pow(tau, 4.0) * Qc, 1.0 / 3 * pow(tau, 3.0) * Qc,
          1.0 / 2 * pow(tau, 2.0) * Qc, 1.0 / 6 * pow(tau, 3.0) * Qc,
          1.0 / 2 * pow(tau, 2.0) * Qc, tau * Qc)
      .finished();
}

}  // namespace manipulator
#endif