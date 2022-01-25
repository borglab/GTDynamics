/**
 * @file  CanvasSampler.cpp
 * @brief  CanvasSampler
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#include "CanvasSampler.h"

#include <gtsam/geometry/Pose3.h>

#include <vector>

namespace gtdynamics {
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Vector2;
CanvasSampler::CanvasSampler(const Point3& A, const Point3& B, const Point3& C)
    : A_{A}, B_{B}, C_{C} {
  Point3 AB_unit = (B - A).normalized();
  Point3 AC_unit = (C - A).normalized();
  Point3 normal = AB_unit.cross(AC_unit);
  bRc_ = Rot3(AB_unit, AC_unit, normal);
}

/**
 * Contiguous poses on the vector are contiguous in the AC axis (except edge
 * cases) Interpreting the AB direction as horizontal (rows) and the AC as 
 * vertical (columns), poses are stored in a column-major basis.
 */
std::vector<Pose3> CanvasSampler::uniformSample(const size_t numABsamples,
                                                const size_t numACsamples) {
  Point3 AB = B_ - A_;
  Point3 AC = C_ - A_;

  std::vector<Pose3> samples(numABsamples * numACsamples);
  for (size_t i = 1, idx = 0; i <= numABsamples; i++) {
    for (size_t j = 1; j <= numACsamples; j++) {
      Point3 btsample =
          A_ + (AB * i) / (numABsamples + 1) + (AC * j) / (numACsamples + 1);
      samples[idx++] = Pose3(bRc_, btsample);
    }
  }
  return samples;
}
std::vector<Pose3> CanvasSampler::randomSample(const size_t numsamples) {
  Point3 AB = (B_ - A_).normalized();
  Point3 AC = (C_ - A_).normalized();

  std::vector<Pose3> samples(numsamples);
  for (size_t i = 0; i < numsamples; i++) {
    Vector2 random = Vector2::Random();
    Point3 btsample = A_ + (AB * (random(0) + 1)) + (AC * (random(1) + 1)) / 2;
    samples[i] = Pose3(bRc_, btsample);
  }
  return samples;
}

std::vector<std::vector<size_t>> CanvasSampler::uniformPoseLocality(
    size_t numABsamples, size_t numACsamples, size_t kernel_size) {
  std::vector<std::vector<size_t>> pose_locality(numABsamples * numACsamples);
  for (size_t i = 0; i < numABsamples; ++i) {
    for (size_t j = 0; j < numACsamples; ++j) {
      size_t min_i = (i > kernel_size ? i : kernel_size) - kernel_size;
      size_t max_i = (i + kernel_size + 1 < numABsamples ? i + kernel_size + 1
                                                         : numABsamples);
      for (size_t slide_i = min_i; slide_i < max_i; ++slide_i) {
        size_t min_j = (j > kernel_size ? j : kernel_size) - kernel_size;
        size_t max_j = (j + kernel_size + 1 < numACsamples ? j + kernel_size + 1
                                                           : numACsamples);
        for (size_t slide_j = min_j; slide_j < max_j; ++slide_j) {
          pose_locality[numACsamples * i + j].push_back(numACsamples * slide_i +
                                                        slide_j);
        }
      }
    }
  }
  return pose_locality;
}
}  // namespace gtdynamics