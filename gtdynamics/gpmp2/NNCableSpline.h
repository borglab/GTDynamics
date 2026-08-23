/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  NNCableSpline.h
 * @brief Robot-independent neural-network-predicted cable shape.
 * @author Karthik Shaji
 */

#pragma once

#include <gtdynamics/dynamics/MLP.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>

#include <memory>
#include <vector>

namespace gtdynamics {

/** Robot-independent cable chord plus MLP-predicted Chebyshev residuals. */
class GTSAM_EXPORT NNCableSpline {
 private:
  std::shared_ptr<const MLP> mlp_;
  size_t numChebNodes_;
  gtsam::Matrix interiorWeights_;

 public:
  NNCableSpline(const std::shared_ptr<const MLP> &mlp, size_t numChebNodes,
                size_t numSamples);

  size_t numSamples() const { return interiorWeights_.rows(); }
  size_t numChebNodes() const { return numChebNodes_; }
  size_t inputDim() const { return mlp_->inputDim(); }

  /**
   * Evaluate world cable samples. Each optional primitive Jacobian has columns
   * [p0(3), p1(3), reference rotation(3), input(inputDim)]. The rotation uses
   * a right/body-frame perturbation.
   */
  void samplePoints(
      const gtsam::Point3 &p0, const gtsam::Point3 &p1,
      const gtsam::Rot3 &wRreference, const gtsam::Vector &input,
      std::vector<gtsam::Point3> *wPts,
      std::vector<gtsam::Matrix> *primitiveJacobians = nullptr) const;

  gtsam::Matrix worldPoints(const gtsam::Point3 &p0,
                            const gtsam::Point3 &p1,
                            const gtsam::Rot3 &wRreference,
                            const gtsam::Vector &input) const;
};  // \class NNCableSpline

}  // namespace gtdynamics
