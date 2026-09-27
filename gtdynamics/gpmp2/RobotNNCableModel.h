/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/** @file RobotNNCableModel.h @brief Robot adapter for NNCableSpline. */

#pragma once

#include <gtdynamics/gpmp2/NNCableSpline.h>
#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtdynamics/gpmp2/SignedDistanceField.h>

#include <memory>
#include <string>
#include <vector>

namespace gtdynamics {

/** Maps a robot configuration to the primitives consumed by NNCableSpline. */
class GTSAM_EXPORT RobotNNCableModel {
 private:
  RobotQueryPoints fk_;  ///< two attachments, then the reference link
  std::shared_ptr<const NNCableSpline> spline_;
  std::vector<size_t> inputIndices_;

  static PointOnLinks checkedPoints(const PointOnLink &attachment0,
                                    const PointOnLink &attachment1,
                                    const LinkSharedPtr &referenceLink);

 public:
  RobotNNCableModel(
      const Robot &robot, const std::string &baseLinkName,
      const std::vector<JointSharedPtr> &joints,
      const PointOnLink &attachment0, const PointOnLink &attachment1,
      const LinkSharedPtr &referenceLink,
      const std::shared_ptr<const NNCableSpline> &spline,
      const std::vector<size_t> &inputIndices,
      const gtsam::Pose3 &wTbase = gtsam::Pose3());

  size_t dof() const { return fk_.dof(); }
  size_t numSamples() const { return spline_->numSamples(); }
  const std::vector<size_t> &inputIndices() const { return inputIndices_; }

  void samplePoints(const gtsam::Vector &q,
                    std::vector<gtsam::Point3> *wPts,
                    std::vector<gtsam::Matrix> *ptJacobians = nullptr) const;

  gtsam::Matrix worldPoints(const gtsam::Vector &q) const;
};  // \class RobotNNCableModel

GTSAM_EXPORT gtsam::Vector nnCableSDFError(
    const gtsam::Vector &q, const RobotNNCableModel &cable,
    const SignedDistanceField &sdf, double epsilon, const gtsam::Vector &radii,
    gtsam::Matrix *Hq = nullptr);

}  // namespace gtdynamics
