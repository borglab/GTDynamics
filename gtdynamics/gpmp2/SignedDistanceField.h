/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SignedDistanceField.h
 * @brief Signed distance field with trilinear interpolation.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong and Mustafa Mukadam.
 */

#pragma once

#include <gtdynamics/gpmp2/SDFException.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

#include <memory>
#include <string>
#include <vector>

namespace gtdynamics {

/**
 * Signed distance field sampled on a uniform grid, queried by trilinear
 * interpolation. The field is expressed in some frame s: to use it as a world
 * obstacle field let s be the world frame, and to use it as the collision
 * geometry of a rigid link let s be that link's CoM frame.
 *
 * The data is a vector of matrices, one per z layer, where each matrix is
 * indexed as data[z](row, col) with row spanning y and col spanning x. A numpy
 * array laid out as sdf[nx, ny, nz] therefore transposes into layer z as
 * sdf[:, :, z].T -- passing sdf[:, :, z] directly swaps the x and y axes, which
 * leaves the distances plausible but silently transposes the gradient. The
 * constructor taking positions and distances is immune to this, since every
 * value carries the position it was sampled at.
 */
class SignedDistanceField {
 public:
  /// Fractional grid index of a point, in (row, col, z) = (y, x, z) order.
  struct FloatIndex {
    double row, col, z;
    FloatIndex(double row, double col, double z) : row(row), col(col), z(z) {}
  };

  using shared_ptr = std::shared_ptr<SignedDistanceField>;

 private:
  using This = SignedDistanceField;

  gtsam::Point3 origin_;
  size_t fieldRows_, fieldCols_, fieldZ_;
  double cellSize_;
  std::vector<gtsam::Matrix> data_;

 public:
  /// Default constructor, only for serialization.
  SignedDistanceField() : fieldRows_(0), fieldCols_(0), fieldZ_(0),
                          cellSize_(0.0) {}

  /**
   * Constructor with all data.
   * @param origin the (x, y, z) position of cell (0, 0, 0), in frame s
   * @param cellSize the side length of a grid cell
   * @param data one matrix per z layer, each indexed as (row = y, col = x)
   */
  SignedDistanceField(const gtsam::Point3 &origin, double cellSize,
                      const std::vector<gtsam::Matrix> &data);

  /// Constructor with no data, to be filled in layer by layer through
  /// initFieldData; every layer reads as zero distance until it is set.
  SignedDistanceField(const gtsam::Point3 &origin, double cellSize,
                      size_t fieldRows, size_t fieldCols, size_t fieldZ);

  /**
   * Constructor from sampled positions and their signed distances. The origin,
   * cell size and grid extent are inferred from the positions, which must be
   * exactly the nodes of a uniform grid of equal spacing on all three axes.
   * Anything else throws rather than yield a warped field. Each distance is
   * placed by its own position, so the columns may arrive in any order and the
   * axis convention of the caller's array cannot transpose the field.
   *
   * @param positions 3 x N matrix of node positions, in frame s
   * @param distances N signed distances, one per column of positions
   * @param tol tolerance for matching a position onto a grid node, in metres
   */
  SignedDistanceField(const gtsam::Matrix &positions,
                      const gtsam::Vector &distances, double tol = 1e-6);

  ~SignedDistanceField() {}

  /// Insert one z layer of the field, indexed as (row = y, col = x).
  void initFieldData(size_t zIndex, const gtsam::Matrix &fieldLayer);

  /// Return the signed distance at a point expressed in frame s.
  double getSignedDistance(const gtsam::Point3 &point) const {
    return signedDistance(convertPoint3toCell(point));
  }

  /// Return the signed distance at a point, and its gradient in metric units.
  double getSignedDistance(const gtsam::Point3 &point, gtsam::Vector3 &g) const;

  /// Convert a point in frame s to a fractional grid index.
  FloatIndex convertPoint3toCell(const gtsam::Point3 &point) const;

  /// Convert a fractional grid index to a point in frame s.
  gtsam::Point3 convertCelltoPoint3(const FloatIndex &cell) const {
    return origin_ + gtsam::Point3(cell.col * cellSize_, cell.row * cellSize_,
                                   cell.z * cellSize_);
  }

  /// Trilinear interpolation of the signed distance at a fractional index.
  double signedDistance(const FloatIndex &idx) const;

  /// Gradient of the trilinear interpolation, with respect to the index.
  /// Not differentiable exactly at a grid point.
  gtsam::Vector3 gradient(const FloatIndex &idx) const;

  /// Raw access to one grid cell.
  double signedDistance(size_t r, size_t c, size_t z) const {
    return data_[z](r, c);
  }

  const gtsam::Point3 &origin() const { return origin_; }
  size_t xCount() const { return fieldCols_; }
  size_t yCount() const { return fieldRows_; }
  size_t zCount() const { return fieldZ_; }
  double cellSize() const { return cellSize_; }
  const std::vector<gtsam::Matrix> &rawData() const { return data_; }

  /// Equality up to a tolerance.
  bool equals(const This &expected, double tol = 1e-9) const;

  /// Print contents.
  void print(const std::string &s = "") const;
};  // \class SignedDistanceField

}  // namespace gtdynamics

/// traits
namespace gtsam {
template <>
struct traits<gtdynamics::SignedDistanceField>
    : public Testable<gtdynamics::SignedDistanceField> {};
}  // namespace gtsam
