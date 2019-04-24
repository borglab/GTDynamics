/**
 * @file  utils.h
 * @brief a few utilities
 * @Author: Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <boost/optional.hpp>
#include <cmath>
#include <string>
#include <vector>


namespace manipulator {

/** Create unit twist for axis direction
    Keyword argument:
        w -- axis direction
        p -- p some point on axis
    return unit twist
*/
gtsam::Vector6 unit_twist(const gtsam::Vector3 &w, const gtsam::Vector3 &p);

/* convert angle to radians */
double radians(double degree);

/* convert a vector of angles to radians */
gtsam::Vector radians(const gtsam::Vector &degree);

/** calculate AdjointMap jacobian w.r.t. joint coordinate q
 *  Keyword argument:
      q            -- joint angle
      jMi          -- this COM frame, expressed in next link's COM frame at
                      rest configuration
      screw_axis   -- screw axis expressed in kth link's COM
                   frame
*/
gtsam::Matrix6 AdjointMapJacobianQ(double q, const gtsam::Pose3 &jMi,
                                   const gtsam::Vector6 &screw_axis);

/** calculate Gaussian Process system transition matrix
    Keyword argument:
        tau -- timestep
*/
inline gtsam::Matrix3 calcPhi(double tau) {
  return (gtsam::Matrix(3, 3) << 1, tau, 0.5 * tau * tau,  //
          0, 1, tau,                                       //
          0, 0, 1)
      .finished();
}

// get Qc covariance matrix from noise model
gtsam::Matrix getQc(const gtsam::SharedNoiseModel Qc_model);

/** calculate Gaussian Process covaraince
    Keyword argument:
        Qc -- covariance matrix
        tau -- timestep
*/
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
