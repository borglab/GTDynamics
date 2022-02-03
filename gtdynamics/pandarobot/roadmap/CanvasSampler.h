/**
 * @file  CanvasSampler.h
 * @brief  CanvasSampler
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#pragma once

#include <gtsam/geometry/Pose3.h>

#include <vector>

// Class used when sampling poses embedded in Canvas
namespace gtdynamics {
class CanvasSampler {
 private:
  gtsam::Point3 A_, B_, C_;  // Canvas corners
  gtsam::Rot3 bRc_;  // Canvas orientation, see constructor for value details

 public:
  /**
   * @brief Construct a new Canvas Sampler object.
   * Canvas orientation constrained by x_c = normalize(B_-A_), y_c =
   * normalize(C_-A_)
   *
   * @param A := Upper left corner  (or equivalent)
   * @param B := Upper right corner (or equivalent)
   * @param C := Lower left corner (or equivalent)
   */
  CanvasSampler(const gtsam::Point3& A, const gtsam::Point3& B,
                const gtsam::Point3& C);

  /**
   * @brief Sample the Canvas Pose Space uniformly as a rectangular planar
   * lattice. All the poses will have the same orientation equal to the canvas
   * one (bRc_, value initialized in constructor).
   *
   * There are numABsamples different values on the AB direction, and
   * numACsamples on the AC one. The edges are not in the samples.
   *
   * @param numABsamples := number of equidistant samples on AB direction
   * @param numACsamples := number of equidistant samples on AC direction
   * @return std::vector<gtsam::Pose3> := vector of sampled poses
   */
  std::vector<gtsam::Pose3> uniformSample(const size_t numABsamples,
                                          const size_t numACsamples);

  /**
   * @brief Returns for every pose (specified by its index) what other poses are
   * considered to be local to it (i.e. close enough to it).
   *
   * On a general case, given a pose, its locality is defined as cube centered
   * on the pose and that encompasses 2*kernel_size+1 poses in total in each
   * direction counting the specified pose (kernel_size on the right, same on
   * the left, up and down). Total of (2*kernel_size+1)^2
   *
   * This can then be used when creating a graph from these poses: only add an
   * edge to the local set poses and not the global one. More importantly, this
   * can be used when a (quasi-)differentiable transformation is applied to the
   * canvas space (in our case, an inverse kinematics one). When trying to find
   * the close points in this space, one can start from the local pose set
   * instead of naively comparing it with the global set of points.
   *
   * @param numABsamples := parameter used in uniformSample, number of
   * equidistant samples on AB direction
   * @param numACsamples := parameter used in uniformSample, number of
   * equidistant samples on AC direction
   * @param kernel_size := controles the size of the cube, as explained above
   * @return std::vector<std::vector<size_t>> := a vector that gives for each
   * pose a vector of local poses, the poses represented as indices
   */
  static std::vector<std::vector<size_t>> uniformPoseLocality(
      size_t numABsamples, size_t numACsamples, size_t kernel_size = 1);

  /**
   * @brief Sample the canvas space randomly. All the poses will have the same
   * orientation equal to the canvas one (bRc_, value initialized in
   * constructor).
   *
   * The pose position is what is sampled, and it is computed as such:
   *
   * p = A_ + x * AB_vector + y * AC_vector where x and y are random numbers
   * between (0,1)
   *
   * @param numsamples := number of samples to be made
   * @return std::vector<gtsam::Pose3> := vector of poses sampled
   */
  std::vector<gtsam::Pose3> randomSample(const size_t numsamples);
  // todo: not urgent and might not be used, randomRelationships? sort of kNN
};
}  // namespace gtdynamics