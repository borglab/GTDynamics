/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/** @file RobotNNCableModel.cpp @brief Robot adapter for NNCableSpline. */

#include <gtdynamics/gpmp2/RobotNNCableModel.h>
#include <gtdynamics/gpmp2/ObstacleCost.h>

#include <set>
#include <stdexcept>

namespace gtdynamics {

/* ************************************************************************* */
PointOnLinks RobotNNCableModel::checkedPoints(
    const PointOnLink &attachment0, const PointOnLink &attachment1,
    const LinkSharedPtr &referenceLink) {
  if (!referenceLink) {
    throw std::invalid_argument(
        "RobotNNCableModel: referenceLink must not be null.");
  }
  return {attachment0, attachment1,
          PointOnLink(referenceLink, gtsam::Point3(0.0, 0.0, 0.0))};
}

/* ************************************************************************* */
RobotNNCableModel::RobotNNCableModel(
    const Robot &robot, const std::string &baseLinkName,
    const std::vector<JointSharedPtr> &joints, const PointOnLink &attachment0,
    const PointOnLink &attachment1, const LinkSharedPtr &referenceLink,
    const std::shared_ptr<const NNCableSpline> &spline,
    const std::vector<size_t> &inputIndices, const gtsam::Pose3 &wTbase)
    : fk_(robot, baseLinkName, joints,
          checkedPoints(attachment0, attachment1, referenceLink), wTbase),
      spline_(spline),
      inputIndices_(inputIndices) {
  if (!spline_) {
    throw std::invalid_argument("RobotNNCableModel: spline must not be null.");
  }
  if (inputIndices_.size() != spline_->inputDim()) {
    throw std::invalid_argument(
        "RobotNNCableModel: inputIndices must have one entry per spline input.");
  }
  std::set<size_t> unique;
  for (size_t index : inputIndices_) {
    if (index >= dof()) {
      throw std::invalid_argument(
          "RobotNNCableModel: an input index is out of range of q.");
    }
    if (!unique.insert(index).second) {
      throw std::invalid_argument(
          "RobotNNCableModel: input indices must be distinct.");
    }
  }
}

/* ************************************************************************* */
void RobotNNCableModel::samplePoints(
    const gtsam::Vector &q, std::vector<gtsam::Point3> *wPts,
    std::vector<gtsam::Matrix> *ptJacobians) const {
  const bool computeJacobians = ptJacobians != nullptr;
  std::vector<gtsam::Pose3> wTls;
  std::vector<gtsam::Matrix> poseJacobians;
  fk_.queryPoses(q, &wTls, computeJacobians ? &poseJacobians : nullptr);

  gtsam::Point3 wP[2];
  gtsam::Matrix JwP[2];
  for (int i = 0; i < 2; ++i) {
    if (computeJacobians) {
      gtsam::Matrix36 Hpose;
      wP[i] = wTls[i].transformFrom(fk_.points()[i].point, Hpose);
      JwP[i] = Hpose * poseJacobians[i];
    } else {
      wP[i] = wTls[i].transformFrom(fk_.points()[i].point);
    }
  }

  gtsam::Vector input(inputIndices_.size());
  for (size_t j = 0; j < inputIndices_.size(); ++j) {
    input(j) = q(inputIndices_[j]);
  }
  std::vector<gtsam::Matrix> primitiveJacobians;
  spline_->samplePoints(wP[0], wP[1], wTls[2].rotation(), input, wPts,
                        computeJacobians ? &primitiveJacobians : nullptr);

  if (computeJacobians) {
    ptJacobians->resize(numSamples());
    const gtsam::Matrix JwRr = poseJacobians[2].topRows(3);
    for (size_t m = 0; m < numSamples(); ++m) {
      const gtsam::Matrix &H = primitiveJacobians[m];
      gtsam::Matrix J = H.middleCols<3>(0) * JwP[0] +
                        H.middleCols<3>(3) * JwP[1] +
                        H.middleCols<3>(6) * JwRr;
      for (size_t j = 0; j < inputIndices_.size(); ++j) {
        J.col(inputIndices_[j]) += H.col(9 + j);
      }
      (*ptJacobians)[m] = J;
    }
  }
}

/* ************************************************************************* */
gtsam::Matrix RobotNNCableModel::worldPoints(const gtsam::Vector &q) const {
  std::vector<gtsam::Point3> wPts;
  samplePoints(q, &wPts);
  return internal::pointsToMatrix(wPts);
}

/* ************************************************************************* */
gtsam::Vector nnCableSDFError(
    const gtsam::Vector &q, const RobotNNCableModel &cable,
    const SignedDistanceField &sdf, double epsilon, const gtsam::Vector &radii,
    gtsam::Matrix *Hq) {
  std::vector<gtsam::Point3> wPts;
  std::vector<gtsam::Matrix> ptJacobians;
  cable.samplePoints(q, &wPts, Hq ? &ptJacobians : nullptr);
  return internal::hingeLossOverPoints(wPts, ptJacobians, sdf, epsilon, radii,
                                       Hq);
}

}  // namespace gtdynamics
