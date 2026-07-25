/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  GPUtils.h
 * @brief Gaussian process utilities for the white noise on acceleration prior.
 * @author Karthik Shaji - Adapted from gpmp2 by Xinyan Yan and Jing Dong.
 */

#pragma once

#include <gtdynamics/utils/utils.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace gtdynamics {

/// Reject a non-positive support interval.
inline void checkGPDeltaT(double deltaT) {
  if (deltaT <= 0.0) {
    throw std::invalid_argument("GP: deltaT must be > 0.");
  }
}

/// Reject a non-positive support interval or an interpolation time outside it.
inline void checkGPInterval(double deltaT, double tau) {
  checkGPDeltaT(deltaT);
  if (tau < 0.0 || tau > deltaT) {
    throw std::invalid_argument("GP: tau must be in [0, deltaT].");
  }
}

/*
 * These implement the white noise on acceleration prior of Barfoot14rss, used
 * by GPMP2, whose state stacks a pose and a velocity as [p; v]. The white noise
 * on jerk counterparts, whose state also carries an acceleration, are calcPhi
 * and calcQ in utils.h. The two priors do not share a Qc: here it is the power
 * spectral density of the acceleration, there it is that of the jerk.
 *
 * The Qc covariance itself is extracted by getQc, declared in utils.h.
 */

/**
 * @fn Compute the state transition matrix Phi over a time interval.
 * @param dof degrees of freedom of a single state.
 * @param tau time interval between the two states.
 * @returns the 2n x 2n state transition matrix.
 */
inline gtsam::Matrix calcPhiAccel(size_t dof, double tau) {
  const gtsam::Matrix identity = gtsam::Matrix::Identity(dof, dof);
  const gtsam::Matrix zero = gtsam::Matrix::Zero(dof, dof);
  return (gtsam::Matrix(2 * dof, 2 * dof) <<  //
          identity, tau * identity,           //
          zero, identity)
      .finished();
}

/**
 * @fn Compute the process covariance Q over a time interval.
 * @param Qc n x n power spectral density matrix.
 * @param tau time interval between the two states.
 * @returns the 2n x 2n process covariance matrix.
 */
inline gtsam::Matrix calcQAccel(const gtsam::Matrix &Qc, double tau) {
  assert(Qc.rows() == Qc.cols());
  const auto n = Qc.rows();
  return (gtsam::Matrix(2 * n, 2 * n) <<                                //
          std::pow(tau, 3.0) / 3.0 * Qc, std::pow(tau, 2.0) / 2.0 * Qc,  //
          std::pow(tau, 2.0) / 2.0 * Qc, tau * Qc)
      .finished();
}

/**
 * @fn Compute the inverse of the process covariance Q over a time interval.
 * @param Qc n x n power spectral density matrix.
 * @param tau time interval between the two states.
 * @returns the 2n x 2n inverse process covariance matrix.
 */
inline gtsam::Matrix calcQinvAccel(const gtsam::Matrix &Qc, double tau) {
  assert(Qc.rows() == Qc.cols());
  const auto n = Qc.rows();
  // Qc is a covariance, so solve by Cholesky rather than invert explicitly.
  const gtsam::Matrix QcInv = Qc.llt().solve(gtsam::Matrix::Identity(n, n));
  return (gtsam::Matrix(2 * n, 2 * n) <<          //
          12.0 * std::pow(tau, -3.0) * QcInv,    //
          -6.0 * std::pow(tau, -2.0) * QcInv,    //
          -6.0 * std::pow(tau, -2.0) * QcInv,    //
          4.0 * std::pow(tau, -1.0) * QcInv)
      .finished();
}

/**
 * @fn Scalar block coefficients of the interpolation matrices Lambda and Psi.
 * Qc cancels out of both, leaving cubic Hermite polynomials of tau, so
 * Lambda = lambda kron I and Psi = psi kron I for any Qc.
 *
 * Converting from Hermite (given) to Bezier (target): P0 = p1,
 * P1 = p1 + deltaT/3 v1, P2 = p2 - deltaT/3 v2, P3 = p2.
 *
 * @param deltaT time between the two support states.
 * @param tau time from the first support state to the interpolated state.
 * @param lambda filled with the 2 x 2 coefficients of Lambda.
 * @param psi filled with the 2 x 2 coefficients of Psi.
 */
inline void calcInterpCoefficientsAccel(double deltaT, double tau,
                                        gtsam::Matrix2 *lambda,
                                        gtsam::Matrix2 *psi) {
  checkGPInterval(deltaT, tau);
  const double s = tau / deltaT;
  const double s2 = s * s, s3 = s2 * s;
  (*lambda) << 1.0 - 3.0 * s2 + 2.0 * s3, deltaT * (s - 2.0 * s2 + s3),
      (6.0 * s2 - 6.0 * s) / deltaT, 1.0 - 4.0 * s + 3.0 * s2;
  (*psi) << 3.0 * s2 - 2.0 * s3, deltaT * (s3 - s2),
      (6.0 * s - 6.0 * s2) / deltaT, 3.0 * s2 - 2.0 * s;
}

/**
 * @fn Compute the Lambda matrix used to interpolate at time tau.
 * @param Qc n x n power spectral density matrix.
 * @param deltaT time between the two support states.
 * @param tau time from the first support state to the interpolated state.
 * @returns the 2n x 2n Lambda matrix.
 */
inline gtsam::Matrix calcLambdaAccel(const gtsam::Matrix &Qc, double deltaT,
                                     double tau) {
  assert(Qc.rows() == Qc.cols());
  const size_t dof = static_cast<size_t>(Qc.rows());
  return calcPhiAccel(dof, tau) -
         calcQAccel(Qc, tau) * calcPhiAccel(dof, deltaT - tau).transpose() *
             calcQinvAccel(Qc, deltaT) * calcPhiAccel(dof, deltaT);
}

/**
 * @fn Compute the Psi matrix used to interpolate at time tau.
 * @param Qc n x n power spectral density matrix.
 * @param deltaT time between the two support states.
 * @param tau time from the first support state to the interpolated state.
 * @returns the 2n x 2n Psi matrix.
 */
inline gtsam::Matrix calcPsiAccel(const gtsam::Matrix &Qc, double deltaT,
                                  double tau) {
  assert(Qc.rows() == Qc.cols());
  const size_t dof = static_cast<size_t>(Qc.rows());
  return calcQAccel(Qc, tau) * calcPhiAccel(dof, deltaT - tau).transpose() *
         calcQinvAccel(Qc, deltaT);
}

}  // namespace gtdynamics
