/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  GPutils.h
 * @brief Gaussian process utilities: Qc, Q and Phi matrices.
 * @author Karthik Shaji - Adapted from gpmp2 by Xinyan Yan and Jing Dong.
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>

#include <cassert>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace gtdynamics {

/**
 * @fn Extract the Qc covariance matrix from a Gaussian noise model.
 * @param Qc_model Gaussian noise model whose covariance is Qc.
 * @returns the n x n Qc covariance matrix.
 */
inline gtsam::Matrix getQc(const gtsam::SharedNoiseModel &Qc_model) {
  auto gaussian =
      std::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(Qc_model);
  if (!gaussian) {
    throw std::invalid_argument("getQc: Qc_model must be a Gaussian model.");
  }
  return gaussian->covariance();
}

/**
 * @fn Compute the process covariance Q over a time interval.
 * @param Qc n x n power spectral density matrix.
 * @param tau time interval between the two states.
 * @returns the 2n x 2n process covariance matrix.
 */
inline gtsam::Matrix calcQ(const gtsam::Matrix &Qc, double tau) {
  assert(Qc.rows() == Qc.cols());
  const auto n = Qc.rows();
  return (gtsam::Matrix(2 * n, 2 * n) <<             //
          std::pow(tau, 3.0) / 3.0 * Qc, std::pow(tau, 2.0) / 2.0 * Qc,  //
          std::pow(tau, 2.0) / 2.0 * Qc, tau * Qc)
      .finished();
}

/**
 * @fn Compute the state transition matrix Phi over a time interval.
 * @param dof degrees of freedom of a single state.
 * @param tau time interval between the two states.
 * @returns the 2n x 2n state transition matrix.
 */
inline gtsam::Matrix calcPhi(size_t dof, double tau) {
  const gtsam::Matrix identity = gtsam::Matrix::Identity(dof, dof);
  const gtsam::Matrix zero = gtsam::Matrix::Zero(dof, dof);
  return (gtsam::Matrix(2 * dof, 2 * dof) <<  //
          identity, tau * identity,           //
          zero, identity)
      .finished();
}

/**
 * @fn Compute the inverse of the process covariance Q over a time interval.
 * @param Qc n x n power spectral density matrix.
 * @param tau time interval between the two states.
 * @returns the 2n x 2n inverse process covariance matrix.
 */
inline gtsam::Matrix calcQ_inv(const gtsam::Matrix &Qc, double tau) {
  assert(Qc.rows() == Qc.cols());
  const auto n = Qc.rows();
  const gtsam::Matrix Qc_inv = Qc.inverse();
  return (gtsam::Matrix(2 * n, 2 * n) <<                                    //
          12.0 * std::pow(tau, -3.0) * Qc_inv, -6.0 * std::pow(tau, -2.0) * Qc_inv,  //
          -6.0 * std::pow(tau, -2.0) * Qc_inv, 4.0 * std::pow(tau, -1.0) * Qc_inv)
      .finished();
}

/**
 * @fn Compute the Lambda matrix used to interpolate at time tau.
 * @param Qc n x n power spectral density matrix.
 * @param delta_t time between the two support states.
 * @param tau time from the first support state to the interpolated state.
 * @returns the 2n x 2n Lambda matrix.
 */
inline gtsam::Matrix calcLambda(const gtsam::Matrix &Qc, double delta_t,
                                double tau) {
  assert(Qc.rows() == Qc.cols());
  return calcPhi(Qc.rows(), tau) -
         calcQ(Qc, tau) * calcPhi(Qc.rows(), delta_t - tau).transpose() *
             calcQ_inv(Qc, delta_t) * calcPhi(Qc.rows(), delta_t);
}

/**
 * @fn Compute the Psi matrix used to interpolate at time tau.
 * @param Qc n x n power spectral density matrix.
 * @param delta_t time between the two support states.
 * @param tau time from the first support state to the interpolated state.
 * @returns the 2n x 2n Psi matrix.
 */
inline gtsam::Matrix calcPsi(const gtsam::Matrix &Qc, double delta_t,
                             double tau) {
  assert(Qc.rows() == Qc.cols());
  return calcQ(Qc, tau) * calcPhi(Qc.rows(), delta_t - tau).transpose() *
         calcQ_inv(Qc, delta_t);
}

}  // namespace gtdynamics
