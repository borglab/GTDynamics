/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  utils.h
 * @brief Utility methods.
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <sdf/sdf.hh>

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include <boost/optional.hpp>

namespace robot {

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

/** initial trajectory for q is a straight line
 * Keyword argument:
        i           -- the ith time step
        totol_step  -- total time steps
        start_q     -- start value of variable q
        end_q       -- end value of variable q
*/
gtsam::Vector q_trajectory(
    int i, int total_step,
    gtsam::Vector &start_q,  // NOLINT
    gtsam::Vector &end_q);  // NOLINT

/** calculate center of spheres used to represent this link for collision
 *  check
 * Key arguments:
 *  length   -- a vector of length for each link
 *  radius   -- a vector of sphere radius for each link
 * return sphere centers expressed in COM frame of each link
 */
std::vector<std::vector<gtsam::Point3>> sphereCenters(
    std::vector<double> lengths, std::vector<double> radii);

/** generation circular path
 *  Keyword argument:
      numOfWayPoints   -- total number of waypoints in cartesian path
      goalAngle        -- goal angle, assuming start at 0 joint angle
      radius           -- radius of circular
*/
std::vector<gtsam::Pose3> circle(int numOfWayPoints, double goalAngle,
                                 double radius);

/** generation square path
 *  Keyword argument:
      numOfWayPoints   -- total number of waypoints in cartesian path
      goalAngle        -- goal angle, assuming start at 0 joint angle
      length           -- length of square
*/
std::vector<gtsam::Pose3> square(int numOfWayPoints, double goalAngle,
                                 double length);

/** read a variable from a text file, and save to vector of matrix
 *  this is used for sdf
 *
 */
std::vector<gtsam::Matrix> readFromTxt(
    std::string mat_dir,
    gtsam::Point3 &origin,  // NOLINT
    double &cell_size);  // NOLINT
}  // namespace robot

namespace robot {
/** obtain the sdf ElementPtr associated with the robot model.
 * Keyword arguments:
 *    sdf_file_path    -- a string containing the absolute to the sdf file.
 *    model_name       -- name of the robot we care about. Must be specified in
        case a world file is specified.
*/
sdf::Model get_sdf(std::string sdf_file_path, std::string model_name = "");
}  // namespace robot
