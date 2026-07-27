/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SignedDistanceField.cpp
 * @brief Signed distance field with trilinear interpolation.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong and Mustafa Mukadam.
 */

#include <gtdynamics/gpmp2/SignedDistanceField.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace gtdynamics {

/* ************************************************************************* */
SignedDistanceField::SignedDistanceField(const gtsam::Point3 &origin,
                                         double cellSize,
                                         const std::vector<gtsam::Matrix> &data)
    : origin_(origin),
      fieldRows_(0),
      fieldCols_(0),
      fieldZ_(0),
      cellSize_(cellSize) {
  if (cellSize <= 0.0) {
    throw std::invalid_argument("SignedDistanceField: cellSize must be > 0.");
  }
  if (data.empty()) {
    throw std::invalid_argument(
        "SignedDistanceField: data must contain at least one z layer.");
  }
  fieldRows_ = static_cast<size_t>(data[0].rows());
  fieldCols_ = static_cast<size_t>(data[0].cols());
  fieldZ_ = data.size();
  if (fieldRows_ == 0 || fieldCols_ == 0) {
    throw std::invalid_argument(
        "SignedDistanceField: z layers must not be empty.");
  }
  for (size_t z = 0; z < fieldZ_; ++z) {
    if (static_cast<size_t>(data[z].rows()) != fieldRows_ ||
        static_cast<size_t>(data[z].cols()) != fieldCols_) {
      throw std::invalid_argument(
          "SignedDistanceField: all z layers must have identical dimensions.");
    }
  }
  data_ = data;
}

/* ************************************************************************* */
SignedDistanceField::SignedDistanceField(const gtsam::Point3 &origin,
                                         double cellSize, size_t fieldRows,
                                         size_t fieldCols, size_t fieldZ)
    : origin_(origin),
      fieldRows_(fieldRows),
      fieldCols_(fieldCols),
      fieldZ_(fieldZ),
      cellSize_(cellSize) {
  if (cellSize <= 0.0) {
    throw std::invalid_argument("SignedDistanceField: cellSize must be > 0.");
  }
  if (fieldRows == 0 || fieldCols == 0 || fieldZ == 0) {
    throw std::invalid_argument(
        "SignedDistanceField: field dimensions must be >= 1.");
  }
  // Zero-filled so a query before every initFieldData call is still defined.
  data_ = std::vector<gtsam::Matrix>(
      fieldZ_, gtsam::Matrix::Zero(fieldRows_, fieldCols_));
}

/* ************************************************************************* */
SignedDistanceField::SignedDistanceField(const gtsam::Point3 &origin,
                                         double cellSize, size_t fieldRows,
                                         size_t fieldCols, size_t fieldZ,
                                         const gtsam::Vector &values)
    : SignedDistanceField(origin, cellSize, fieldRows, fieldCols, fieldZ) {
  if (static_cast<size_t>(values.size()) !=
      fieldRows_ * fieldCols_ * fieldZ_) {
    throw std::invalid_argument(
        "SignedDistanceField: values must hold one distance per grid node.");
  }
  // Each contiguous run of cols values is one row: a row major layer map.
  using RowMajorMatrix =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  const size_t layerSize = fieldRows_ * fieldCols_;
  for (size_t z = 0; z < fieldZ_; ++z) {
    data_[z] = Eigen::Map<const RowMajorMatrix>(values.data() + z * layerSize,
                                                fieldRows_, fieldCols_);
  }
}

/* ************************************************************************* */
SignedDistanceField::SignedDistanceField(const gtsam::Matrix &positions,
                                         const gtsam::Vector &distances,
                                         double tol) {
  if (positions.rows() != 3) {
    throw std::invalid_argument(
        "SignedDistanceField: positions must be a 3 x N matrix.");
  }
  if (positions.cols() != distances.size()) {
    throw std::invalid_argument(
        "SignedDistanceField: positions and distances must agree in count.");
  }

  // Recover each axis's nodes, then its uniform spacing.
  std::vector<double> spacing(3);
  std::vector<double> low(3);
  std::vector<size_t> count(3);
  for (int axis = 0; axis < 3; ++axis) {
    std::vector<double> values;
    values.reserve(positions.cols());
    for (Eigen::Index n = 0; n < positions.cols(); ++n) {
      values.push_back(positions(axis, n));
    }
    std::sort(values.begin(), values.end());
    std::vector<double> nodes;
    for (double value : values) {
      if (nodes.empty() || value - nodes.back() > tol) nodes.push_back(value);
    }
    if (nodes.size() < 2) {
      throw std::invalid_argument(
          "SignedDistanceField: each axis needs at least two distinct "
          "positions.");
    }
    low[axis] = nodes.front();
    count[axis] = nodes.size();
    spacing[axis] = (nodes.back() - nodes.front()) / (nodes.size() - 1);
    for (size_t i = 0; i < nodes.size(); ++i) {
      if (std::fabs(nodes[i] - (low[axis] + i * spacing[axis])) > tol) {
        throw std::invalid_argument(
            "SignedDistanceField: positions are not uniformly spaced.");
      }
    }
  }
  if (std::fabs(spacing[0] - spacing[1]) > tol ||
      std::fabs(spacing[0] - spacing[2]) > tol) {
    throw std::invalid_argument(
        "SignedDistanceField: spacing must be equal on all three axes.");
  }
  if (static_cast<size_t>(positions.cols()) !=
      count[0] * count[1] * count[2]) {
    throw std::invalid_argument(
        "SignedDistanceField: positions do not fill the grid exactly once.");
  }

  origin_ = gtsam::Point3(low[0], low[1], low[2]);
  cellSize_ = spacing[0];
  fieldCols_ = count[0];
  fieldRows_ = count[1];
  fieldZ_ = count[2];
  data_ = std::vector<gtsam::Matrix>(
      fieldZ_, gtsam::Matrix::Zero(fieldRows_, fieldCols_));

  // Scatter each distance to the node its own position names.
  std::vector<bool> filled(positions.cols(), false);
  for (Eigen::Index n = 0; n < positions.cols(); ++n) {
    size_t idx[3];
    for (int axis = 0; axis < 3; ++axis) {
      const double fractional = (positions(axis, n) - low[axis]) / cellSize_;
      const double rounded = std::round(fractional);
      if (std::fabs(fractional - rounded) * cellSize_ > tol ||
          rounded < 0.0 || rounded >= static_cast<double>(count[axis])) {
        throw std::invalid_argument(
            "SignedDistanceField: a position does not lie on a grid node.");
      }
      idx[axis] = static_cast<size_t>(rounded);
    }
    const size_t flat =
        (idx[2] * fieldRows_ + idx[1]) * fieldCols_ + idx[0];
    if (filled[flat]) {
      throw std::invalid_argument(
          "SignedDistanceField: two positions land on the same grid node.");
    }
    filled[flat] = true;
    data_[idx[2]](idx[1], idx[0]) = distances(n);
  }
}

/* ************************************************************************* */
void SignedDistanceField::initFieldData(size_t zIndex,
                                        const gtsam::Matrix &fieldLayer) {
  if (zIndex >= fieldZ_) {
    throw std::out_of_range(
        "SignedDistanceField::initFieldData: zIndex out of range.");
  }
  if (static_cast<size_t>(fieldLayer.rows()) != fieldRows_ ||
      static_cast<size_t>(fieldLayer.cols()) != fieldCols_) {
    throw std::invalid_argument(
        "SignedDistanceField::initFieldData: fieldLayer dimensions must match "
        "the field.");
  }
  data_[zIndex] = fieldLayer;
}

/* ************************************************************************* */
double SignedDistanceField::getSignedDistance(const gtsam::Point3 &point,
                                              gtsam::Vector3 &g) const {
  const FloatIndex pidx = convertPoint3toCell(point);
  const gtsam::Vector3 gridIndex = gradient(pidx);
  // The gradient comes back in (row, col, z) order, so swap to (x, y, z).
  g = gtsam::Vector3(gridIndex(1), gridIndex(0), gridIndex(2)) / cellSize_;
  return signedDistance(pidx);
}

/* ************************************************************************* */
SignedDistanceField::FloatIndex SignedDistanceField::convertPoint3toCell(
    const gtsam::Point3 &point) const {
  // Only a default constructed field can be in this state.
  if (data_.empty()) {
    throw std::runtime_error(
        "SignedDistanceField: an uninitialized field cannot be queried.");
  }
  if (point.x() < origin_.x() ||
      point.x() > (origin_.x() + (fieldCols_ - 1.0) * cellSize_) ||
      point.y() < origin_.y() ||
      point.y() > (origin_.y() + (fieldRows_ - 1.0) * cellSize_) ||
      point.z() < origin_.z() ||
      point.z() > (origin_.z() + (fieldZ_ - 1.0) * cellSize_)) {
    throw SDFQueryOutOfRange();
  }
  return FloatIndex((point.y() - origin_.y()) / cellSize_,
                    (point.x() - origin_.x()) / cellSize_,
                    (point.z() - origin_.z()) / cellSize_);
}

/* ************************************************************************* */
double SignedDistanceField::signedDistance(const FloatIndex &idx) const {
  const double lr = std::floor(idx.row), lc = std::floor(idx.col),
               lz = std::floor(idx.z);
  const double hr = lr + 1.0, hc = lc + 1.0, hz = lz + 1.0;
  const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc),
               lzi = static_cast<size_t>(lz);
  // Clamp so a query exactly on the far face does not read past the grid.
  const size_t hri = std::min(lri + 1, fieldRows_ - 1),
               hci = std::min(lci + 1, fieldCols_ - 1),
               hzi = std::min(lzi + 1, fieldZ_ - 1);
  return (hr - idx.row) * (hc - idx.col) * (hz - idx.z) *
             signedDistance(lri, lci, lzi) +
         (idx.row - lr) * (hc - idx.col) * (hz - idx.z) *
             signedDistance(hri, lci, lzi) +
         (hr - idx.row) * (idx.col - lc) * (hz - idx.z) *
             signedDistance(lri, hci, lzi) +
         (idx.row - lr) * (idx.col - lc) * (hz - idx.z) *
             signedDistance(hri, hci, lzi) +
         (hr - idx.row) * (hc - idx.col) * (idx.z - lz) *
             signedDistance(lri, lci, hzi) +
         (idx.row - lr) * (hc - idx.col) * (idx.z - lz) *
             signedDistance(hri, lci, hzi) +
         (hr - idx.row) * (idx.col - lc) * (idx.z - lz) *
             signedDistance(lri, hci, hzi) +
         (idx.row - lr) * (idx.col - lc) * (idx.z - lz) *
             signedDistance(hri, hci, hzi);
}

/* ************************************************************************* */
gtsam::Vector3 SignedDistanceField::gradient(const FloatIndex &idx) const {
  const double lr = std::floor(idx.row), lc = std::floor(idx.col),
               lz = std::floor(idx.z);
  const double hr = lr + 1.0, hc = lc + 1.0, hz = lz + 1.0;
  const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc),
               lzi = static_cast<size_t>(lz);
  // Clamp so a query exactly on the far face does not read past the grid.
  const size_t hri = std::min(lri + 1, fieldRows_ - 1),
               hci = std::min(lci + 1, fieldCols_ - 1),
               hzi = std::min(lzi + 1, fieldZ_ - 1);
  return gtsam::Vector3(
      (hc - idx.col) * (hz - idx.z) *
              (signedDistance(hri, lci, lzi) - signedDistance(lri, lci, lzi)) +
          (idx.col - lc) * (hz - idx.z) *
              (signedDistance(hri, hci, lzi) - signedDistance(lri, hci, lzi)) +
          (hc - idx.col) * (idx.z - lz) *
              (signedDistance(hri, lci, hzi) - signedDistance(lri, lci, hzi)) +
          (idx.col - lc) * (idx.z - lz) *
              (signedDistance(hri, hci, hzi) - signedDistance(lri, hci, hzi)),

      (hr - idx.row) * (hz - idx.z) *
              (signedDistance(lri, hci, lzi) - signedDistance(lri, lci, lzi)) +
          (idx.row - lr) * (hz - idx.z) *
              (signedDistance(hri, hci, lzi) - signedDistance(hri, lci, lzi)) +
          (hr - idx.row) * (idx.z - lz) *
              (signedDistance(lri, hci, hzi) - signedDistance(lri, lci, hzi)) +
          (idx.row - lr) * (idx.z - lz) *
              (signedDistance(hri, hci, hzi) - signedDistance(hri, lci, hzi)),

      (hr - idx.row) * (hc - idx.col) *
              (signedDistance(lri, lci, hzi) - signedDistance(lri, lci, lzi)) +
          (idx.row - lr) * (hc - idx.col) *
              (signedDistance(hri, lci, hzi) - signedDistance(hri, lci, lzi)) +
          (hr - idx.row) * (idx.col - lc) *
              (signedDistance(lri, hci, hzi) - signedDistance(lri, hci, lzi)) +
          (idx.row - lr) * (idx.col - lc) *
              (signedDistance(hri, hci, hzi) - signedDistance(hri, hci, lzi)));
}

/* ************************************************************************* */
bool SignedDistanceField::equals(const This &expected, double tol) const {
  if (fieldRows_ != expected.fieldRows_ ||
      fieldCols_ != expected.fieldCols_ || fieldZ_ != expected.fieldZ_ ||
      std::fabs(cellSize_ - expected.cellSize_) > tol ||
      !gtsam::traits<gtsam::Point3>::Equals(origin_, expected.origin_, tol)) {
    return false;
  }
  for (size_t z = 0; z < fieldZ_; ++z) {
    if (!gtsam::equal_with_abs_tol(data_[z], expected.data_[z], tol)) {
      return false;
    }
  }
  return true;
}

/* ************************************************************************* */
void SignedDistanceField::print(const std::string &s) const {
  std::cout << s;
  std::cout << "field origin:     " << origin_.transpose() << std::endl;
  std::cout << "field resolution: " << cellSize_ << std::endl;
  std::cout << "field size:       " << fieldCols_ << " x " << fieldRows_
            << " x " << fieldZ_ << std::endl;
}

}  // namespace gtdynamics
