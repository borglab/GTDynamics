/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  utils.h
 * @brief Utility methods.
 * @author Frank Dellaert, Mandy Xie, Alejandro Escontrela
 */

#pragma once

#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <boost/optional.hpp>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

namespace gtdynamics {

/**
 * Create unit twist for axis direction
 *
 * @param w Axis direction
 * @param p Some point on axis
 * @param return unit twist
 */
gtsam::Vector6 unit_twist(const gtsam::Vector3 &w, const gtsam::Vector3 &p);

/// Convert angle to radians
double radians(double degree);

/// Convert a vector of angles to radians
gtsam::Vector radians(const gtsam::Vector &degree);

/**
 * Calculate AdjointMap jacobian w.r.t. joint coordinate q
 * @param q joint angle
 * @param jMi this COM frame, expressed in next link's COM frame at rest
 * @param screw_axis screw axis expressed in kth link's COM frame
 */
gtsam::Matrix6 AdjointMapJacobianQ(double q, const gtsam::Pose3 &jMi,
                                   const gtsam::Vector6 &screw_axis);

/**
 * Calculate Gaussian Process system transition matrix
 * @param tau timestep
 */
inline gtsam::Matrix3 calcPhi(double tau) {
  return (gtsam::Matrix(3, 3) << 1, tau, 0.5 * tau * tau,  //
          0, 1, tau,                                       //
          0, 0, 1)
      .finished();
}

/// get Qc covariance matrix from noise model
gtsam::Matrix getQc(const gtsam::SharedNoiseModel Qc_model);

/**
 * Calculate Gaussian Process covaraince
 * @param Qc covariance matrix
 * @param tau timestep
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

/**
 * Snitial trajectory for q is a straight line
 * @param i the ith time step
 * @param totol_step total time steps
 * @param start_q start value of variable q
 * @param end_q end value of variable q
 */
gtsam::Vector q_trajectory(int i, int total_step,
                           gtsam::Vector &start_q,  // NOLINT
                           gtsam::Vector &end_q);   // NOLINT

/**
 * Calculate center of spheres used to represent this link for collision check.
 * @param length a vector of length for each link
 * @param radius a vector of sphere radius for each link
 * @return sphere centers expressed in COM frame of each link
 */
std::vector<std::vector<gtsam::Point3>> sphereCenters(
    std::vector<double> lengths, std::vector<double> radii);

/**
 * Generate circular path
 *
 * @param numOfWayPoints total number of waypoints in cartesian path
 * @param goalAngle goal angle, assuming start at 0 joint angle
 * @param radius radius of circular
 */
std::vector<gtsam::Pose3> circle(int numOfWayPoints, double goalAngle,
                                 double radius);

/**
 * Generate square path
 *
 * @param numOfWayPoints total number of waypoints in cartesian path
 * @param goalAngle goal angle, assuming start at 0 joint angle
 * @param length length of square
 */
std::vector<gtsam::Pose3> square(int numOfWayPoints, double goalAngle,
                                 double length);

/**
 * Read a variable from a text file, and save to vector of matrix.
 * This is used for SDF.
 */
std::vector<gtsam::Matrix> readFromTxt(std::string mat_dir,
                                       gtsam::Point3 &origin,  // NOLINT
                                       double &cell_size);     // NOLINT

/**
 * Obtain the planar jacobian for the given planar axis.
 *
 * @param planar_axis The planar axis.
 */
gtsam::Matrix36 getPlanarJacobian(const gtsam::Vector3 &planar_axis);

}  // namespace gtdynamics
