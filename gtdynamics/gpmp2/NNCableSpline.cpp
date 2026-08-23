/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  NNCableSpline.cpp
 * @brief Robot-independent neural-network-predicted cable shape.
 * @author Karthik Shaji
 */

#include <gtdynamics/gpmp2/NNCableSpline.h>
#include <gtdynamics/gpmp2/ObstacleCost.h>
#include <gtsam/basis/Chebyshev2.h>

#include <stdexcept>
#include <vector>

namespace gtdynamics {

/* ************************************************************************* */
NNCableSpline::NNCableSpline(const std::shared_ptr<const MLP> &mlp,
                             size_t numChebNodes, size_t numSamples)
    : mlp_(mlp), numChebNodes_(numChebNodes) {
  if (!mlp_) {
    throw std::invalid_argument("NNCableSpline: mlp must not be null.");
  }
  if (numChebNodes_ < 3) {
    throw std::invalid_argument("NNCableSpline: numChebNodes must be >= 3.");
  }
  if (numSamples < 2) {
    throw std::invalid_argument("NNCableSpline: numSamples must be >= 2.");
  }
  if (mlp_->outputDim() != 3 * (numChebNodes_ - 2)) {
    throw std::invalid_argument(
        "NNCableSpline: the network output size must be "
        "3 * (numChebNodes - 2) interior nodal residuals.");
  }

  interiorWeights_ = gtsam::Matrix(numSamples, numChebNodes_ - 2);
  for (size_t m = 0; m < numSamples; ++m) {
    const double s = static_cast<double>(m) / (numSamples - 1);
    const gtsam::Matrix weights =
        gtsam::Chebyshev2::CalculateWeights(numChebNodes_, s, 0.0, 1.0);
    interiorWeights_.row(m) = weights.row(0).segment(1, numChebNodes_ - 2);
  }
}

/* ************************************************************************* */
void NNCableSpline::samplePoints(
    const gtsam::Point3 &p0, const gtsam::Point3 &p1,
    const gtsam::Rot3 &wRreference, const gtsam::Vector &input,
    std::vector<gtsam::Point3> *wPts,
    std::vector<gtsam::Matrix> *primitiveJacobians) const {
  if (static_cast<size_t>(input.size()) != inputDim()) {
    throw std::invalid_argument("NNCableSpline: input has the wrong size.");
  }
  const bool computeJacobians = primitiveJacobians != nullptr;
  gtsam::Matrix Jmlp;
  const gtsam::Vector y =
      mlp_->forward(input, computeJacobians ? &Jmlp : nullptr);
  const size_t nrInterior = numChebNodes_ - 2;
  const Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 3,
                                       Eigen::RowMajor>>
      interiorResiduals(y.data(), nrInterior, 3);
  const gtsam::Matrix3 R = wRreference.matrix();

  wPts->resize(numSamples());
  if (computeJacobians) primitiveJacobians->resize(numSamples());
  for (size_t m = 0; m < numSamples(); ++m) {
    const double s = static_cast<double>(m) / (numSamples() - 1);
    const gtsam::Vector3 v =
        interiorResiduals.transpose() * interiorWeights_.row(m).transpose();
    (*wPts)[m] = (1.0 - s) * p0 + s * p1 + R * v;
    if (computeJacobians) {
      gtsam::Matrix &H = (*primitiveJacobians)[m];
      H = gtsam::Matrix::Zero(3, 9 + inputDim());
      H.middleCols<3>(0) = (1.0 - s) * gtsam::Matrix3::Identity();
      H.middleCols<3>(3) = s * gtsam::Matrix3::Identity();
      H.middleCols<3>(6) = -R * gtsam::skewSymmetric(v);

      gtsam::Matrix weightedJmlp = gtsam::Matrix::Zero(3, inputDim());
      for (size_t j = 0; j < nrInterior; ++j) {
        weightedJmlp.noalias() +=
            interiorWeights_(m, j) * Jmlp.middleRows(3 * j, 3);
      }
      H.rightCols(inputDim()).noalias() = R * weightedJmlp;
    }
  }
}

/* ************************************************************************* */
gtsam::Matrix NNCableSpline::worldPoints(
    const gtsam::Point3 &p0, const gtsam::Point3 &p1,
    const gtsam::Rot3 &wRreference, const gtsam::Vector &input) const {
  std::vector<gtsam::Point3> wPts;
  samplePoints(p0, p1, wRreference, input, &wPts);
  return internal::pointsToMatrix(wPts);
}

}  // namespace gtdynamics
