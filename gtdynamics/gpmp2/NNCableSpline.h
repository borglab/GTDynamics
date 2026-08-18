/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  NNCableSpline.h
 * @brief Neural-network-predicted cable shape between two robot attachments.
 * @author Karthik Shaji
 */

#pragma once

#include <gtdynamics/dynamics/MLP.h>
#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtdynamics/gpmp2/SignedDistanceField.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <memory>
#include <string>
#include <vector>

namespace gtdynamics {

/**
 * Predicts the shape of a cable strung between two attachment points on a
 * robot, as the chord between the forward-kinematics endpoints plus
 * MLP-predicted interior Chebyshev nodal residuals, and samples world points
 * along it with Jacobians with respect to the stacked joint angle vector q.
 * The residuals are predicted in the reference link's frame, as trained.
 * Immutable after construction.
 */
class GTSAM_EXPORT NNCableSpline {
 private:
  RobotQueryPoints fk_;  ///< the two attachments, then the reference link
  std::shared_ptr<const MLP> mlp_;
  std::vector<size_t> inputIndices_;  ///< entries of q the network reads
  size_t numChebNodes_;               ///< N, endpoints included
  gtsam::Matrix interiorWeights_;     ///< numSamples x (N-2) Chebyshev weights
                                      ///< at s_m = m / (numSamples - 1)

  /// The FK query points, with the reference link checked against null.
  static PointOnLinks checkedPoints(const PointOnLink &attachment0,
                                    const PointOnLink &attachment1,
                                    const LinkSharedPtr &referenceLink);

 public:
  /**
   * Constructor.
   * @param robot the robot model
   * @param baseLinkName the link the kinematic tree is rooted at
   * @param joints the joints spanned by q, in the order q indexes them
   * @param attachment0 first cable attachment, in its link's CoM frame
   * @param attachment1 second cable attachment, in its link's CoM frame
   * @param referenceLink link whose world rotation maps the network's
   *        residuals into the world frame
   * @param mlp network mapping the selected q entries to the interior
   *        Chebyshev nodal residuals, row-major (N-2) x 3
   * @param inputIndices entries of q fed to the network, in network order
   * @param numChebNodes number N of Chebyshev-Lobatto nodes on [0, 1]
   * @param numSamples number of points sampled uniformly along the cable
   * @param wTbase pose of the base link in the world frame
   * @throw std::invalid_argument on a null mlp or reference link, on
   *        inputIndices out of range, duplicated, or not matching the
   *        network input size, on a network output size other than
   *        3 * (numChebNodes - 2), on numChebNodes < 3 or numSamples < 2,
   *        or on FK inputs the RobotQueryPoints constructor rejects
   */
  NNCableSpline(const Robot &robot, const std::string &baseLinkName,
                const std::vector<JointSharedPtr> &joints,
                const PointOnLink &attachment0, const PointOnLink &attachment1,
                const LinkSharedPtr &referenceLink,
                const std::shared_ptr<const MLP> &mlp,
                const std::vector<size_t> &inputIndices, size_t numChebNodes,
                size_t numSamples, const gtsam::Pose3 &wTbase = gtsam::Pose3());

  /// Return the number of joints spanned by q.
  size_t dof() const { return fk_.dof(); }

  /// Return the number of samples along the cable.
  size_t numSamples() const { return interiorWeights_.rows(); }

  /// Return the number N of Chebyshev-Lobatto nodes, endpoints included.
  size_t numChebNodes() const { return numChebNodes_; }

  /// Return the entries of q fed to the network.
  const std::vector<size_t> &inputIndices() const { return inputIndices_; }

  /**
   * World positions of the cable samples.
   * @param q stacked joint angles
   * @param wPts filled with the world position of each sample
   * @param ptJacobians if non-null, filled with a 3 x dof matrix per sample
   */
  void samplePoints(const gtsam::Vector &q, std::vector<gtsam::Point3> *wPts,
                    std::vector<gtsam::Matrix> *ptJacobians = nullptr) const;

  /// World positions of the cable samples, one per column (3 x numSamples).
  gtsam::Matrix worldPoints(const gtsam::Vector &q) const;
};  // \class NNCableSpline

/// Hinge loss of every cable sample at configuration q, using
/// epsilon + radii(m) as each sample's standoff. If Hq is non-null, it is
/// filled with the numSamples x dof Jacobian with respect to q.
GTSAM_EXPORT gtsam::Vector nnCableSDFError(const gtsam::Vector &q,
                                           const NNCableSpline &cable,
                                           const SignedDistanceField &sdf,
                                           double epsilon,
                                           const gtsam::Vector &radii,
                                           gtsam::Matrix *Hq = nullptr);

}  // namespace gtdynamics
