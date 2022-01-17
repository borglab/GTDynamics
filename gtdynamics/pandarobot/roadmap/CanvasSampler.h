/**
 * @file  CanvasSampler.h
 * @brief  CanvasSampler
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#pragma once

#include <gtsam/geometry/Pose3.h>

#include <vector>

std::vector<std::vector<size_t>> uniformRelationships(size_t numABsamples, size_t numACsamples, size_t kernel_size = 1);

namespace gtdynamics {
class CanvasSampler {
 private:
  gtsam::Point3 A_, B_, C_;
  gtsam::Rot3 bRc_;

 public:
  CanvasSampler(const gtsam::Point3& A, const gtsam::Point3& B,
                const gtsam::Point3& C);
  std::vector<gtsam::Pose3> uniformSample(const size_t numABsamples,
                                          const size_t numACsamples);
  std::vector<gtsam::Pose3> randomSample(const size_t numsamples);
  // randomRelationships? sort of kNN
};
}  // namespace gtdynamics