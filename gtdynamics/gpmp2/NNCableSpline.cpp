/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  NNCableSpline.cpp
 * @brief Neural-network-predicted cable shape between two robot attachments.
 * @author Karthik Shaji
 */

#include <gtdynamics/gpmp2/NNCableSpline.h>
#include <gtdynamics/gpmp2/ObstacleCost.h>
#include <gtsam/basis/Chebyshev2.h>

#include <set>
#include <stdexcept>
#include <vector>

namespace gtdynamics {

/* ************************************************************************* */
PointOnLinks NNCableSpline::checkedPoints(const PointOnLink &attachment0,
                                          const PointOnLink &attachment1,
                                          const LinkSharedPtr &referenceLink) {
  // Attachment links are checked by the RobotQueryPoints constructor.
  if (!referenceLink) {
    throw std::invalid_argument(
        "NNCableSpline: referenceLink must not be null.");
  }
  return {attachment0, attachment1,
          PointOnLink(referenceLink, gtsam::Point3(0.0, 0.0, 0.0))};
}

/* ************************************************************************* */
NNCableSpline::NNCableSpline(
    const Robot &robot, const std::string &baseLinkName,
    const std::vector<JointSharedPtr> &joints, const PointOnLink &attachment0,
    const PointOnLink &attachment1, const LinkSharedPtr &referenceLink,
    const std::shared_ptr<const MLP> &mlp,
    const std::vector<size_t> &inputIndices, size_t numChebNodes,
    size_t numSamples, const gtsam::Pose3 &wTbase)
    : fk_(robot, baseLinkName, joints,
          checkedPoints(attachment0, attachment1, referenceLink), wTbase),
      mlp_(mlp),
      inputIndices_(inputIndices),
      numChebNodes_(numChebNodes) {
  if (!mlp_) {
    throw std::invalid_argument("NNCableSpline: mlp must not be null.");
  }
  if (numChebNodes_ < 3) {
    throw std::invalid_argument("NNCableSpline: numChebNodes must be >= 3.");
  }
  if (numSamples < 2) {
    throw std::invalid_argument("NNCableSpline: numSamples must be >= 2.");
  }
  if (inputIndices_.size() != mlp_->inputDim()) {
    throw std::invalid_argument(
        "NNCableSpline: inputIndices must have one entry per network input.");
  }
  std::set<size_t> unique;
  for (size_t index : inputIndices_) {
    if (index >= dof()) {
      throw std::invalid_argument(
          "NNCableSpline: an input index is out of range of q.");
    }
    if (!unique.insert(index).second) {
      throw std::invalid_argument(
          "NNCableSpline: input indices must be distinct.");
    }
  }
  if (mlp_->outputDim() != 3 * (numChebNodes_ - 2)) {
    throw std::invalid_argument(
        "NNCableSpline: the network output size must be "
        "3 * (numChebNodes - 2) interior nodal residuals.");
  }

  // The evaluation is linear in the nodal residuals, so these barycentric
  // weights are also the exact spline Jacobian; the endpoint columns multiply
  // the identically-zero endpoint residuals and are dropped.
  interiorWeights_ = gtsam::Matrix(numSamples, numChebNodes_ - 2);
  for (size_t m = 0; m < numSamples; ++m) {
    const double s = static_cast<double>(m) / (numSamples - 1);
    const gtsam::Matrix weights =
        gtsam::Chebyshev2::CalculateWeights(numChebNodes_, s, 0.0, 1.0);
    interiorWeights_.row(m) = weights.row(0).segment(1, numChebNodes_ - 2);
  }
}

/* ************************************************************************* */
void NNCableSpline::samplePoints(const gtsam::Vector &q,
                                 std::vector<gtsam::Point3> *wPts,
                                 std::vector<gtsam::Matrix> *ptJacobians) const {
  const bool computeJacobians = (ptJacobians != nullptr);

  std::vector<gtsam::Pose3> wTls;
  std::vector<gtsam::Matrix> poseJacobians;
  fk_.queryPoses(q, &wTls, computeJacobians ? &poseJacobians : nullptr);

  // Attachment world points, chained through the link pose Jacobians.
  gtsam::Point3 p[2];
  gtsam::Matrix Jp[2];
  for (int i = 0; i < 2; ++i) {
    if (computeJacobians) {
      gtsam::Matrix36 Hpose;
      p[i] = wTls[i].transformFrom(fk_.points()[i].point, Hpose);
      Jp[i] = Hpose * poseJacobians[i];
    } else {
      p[i] = wTls[i].transformFrom(fk_.points()[i].point);
    }
  }
  const gtsam::Matrix3 R = wTls[2].rotation().matrix();

  // Network residuals from the selected q entries.
  gtsam::Vector x(inputIndices_.size());
  for (size_t j = 0; j < inputIndices_.size(); ++j) x(j) = q(inputIndices_[j]);
  gtsam::Matrix Jmlp;
  const gtsam::Vector y = mlp_->forward(x, computeJacobians ? &Jmlp : nullptr);
  const size_t nrInterior = numChebNodes_ - 2;
  // Row-major reshape: interior node j's residual is y.segment(3j, 3).
  const Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 3,
                                       Eigen::RowMajor>>
      interiorResiduals(y.data(), nrInterior, 3);

  gtsam::Matrix Jw;
  if (computeJacobians) {
    Jw = poseJacobians[2].topRows(3);  // body-frame angular Jacobian
  }

  wPts->resize(numSamples());
  if (computeJacobians) ptJacobians->resize(numSamples());
  for (size_t m = 0; m < numSamples(); ++m) {
    const double s = static_cast<double>(m) / (numSamples() - 1);
    const gtsam::Vector3 v =
        interiorResiduals.transpose() * interiorWeights_.row(m).transpose();
    (*wPts)[m] = (1.0 - s) * p[0] + s * p[1] + R * v;
    if (computeJacobians) {
      gtsam::Matrix J = (1.0 - s) * Jp[0] + s * Jp[1];
      // The rotation moves as R * Exp(w), so d(R v)/dq = -R [v]x Jw.
      J.noalias() -= R * gtsam::skewSymmetric(v) * Jw;

      gtsam::Matrix weightedJmlp =
          gtsam::Matrix::Zero(3, inputIndices_.size());
      for (size_t j = 0; j < nrInterior; ++j) {
        weightedJmlp.noalias() +=
            interiorWeights_(m, j) * Jmlp.middleRows(3 * j, 3);
      }

      const gtsam::Matrix rotatedJmlp = R * weightedJmlp;
      for (size_t j = 0; j < inputIndices_.size(); ++j) {
        J.col(inputIndices_[j]) += rotatedJmlp.col(j);
      }

      (*ptJacobians)[m] = J;
    }
  }
}

/* ************************************************************************* */
gtsam::Matrix NNCableSpline::worldPoints(const gtsam::Vector &q) const {
  std::vector<gtsam::Point3> wPts;
  samplePoints(q, &wPts);
  return internal::pointsToMatrix(wPts);
}

/* ************************************************************************* */
gtsam::Vector nnCableSDFError(const gtsam::Vector &q,
                              const NNCableSpline &cable,
                              const SignedDistanceField &sdf, double epsilon,
                              const gtsam::Vector &radii, gtsam::Matrix *Hq) {
  std::vector<gtsam::Point3> wPts;
  std::vector<gtsam::Matrix> ptJacobians;
  cable.samplePoints(q, &wPts, Hq ? &ptJacobians : nullptr);
  return internal::hingeLossOverPoints(wPts, ptJacobians, sdf, epsilon, radii,
                                       Hq);
}

}  // namespace gtdynamics
