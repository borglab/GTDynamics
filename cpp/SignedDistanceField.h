/**
 * @file  SignedDistanceField.h
 * @brief signed distance filed used for collision check
 * @Origin: GPMP2
 */
#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace manipulator {

/**
 * Signed distance field use vector<Matrix> as data type
 * Matrix represent the X (col) & Y (row) dimension, and vector is used to Z
 */
class SignedDistanceField {
 public:
  // index and float_index is <row, col, z>
  typedef boost::tuple<size_t, size_t, size_t> index;
  typedef boost::tuple<double, double, double> float_index;
  typedef boost::shared_ptr<SignedDistanceField> shared_ptr;

 private:
  gtsam::Point3 origin_;
  // geometry setting of signed distance field
  size_t field_rows_, field_cols_, field_z_;
  double cell_size_;
  // sdf data
  std::vector<gtsam::Matrix> data_;

 public:
  // default constructor
  SignedDistanceField() {}

  // constructor with data
  SignedDistanceField(const gtsam::Point3& origin, double cell_size,
                      const std::vector<gtsam::Matrix>& data)
      : origin_(origin),
        field_rows_(data[0].rows()),
        field_cols_(data[0].cols()),
        field_z_(data.size()),
        cell_size_(cell_size),
        data_(data) {}

  // destructor
  ~SignedDistanceField() {}

  /** give a point, search for signed distance field and (optional) gradient
   * Keyword arguments:
        point -- query position
   * return signed distance
 */
  inline double getSignedDistance(const gtsam::Point3& point) const {
    const float_index point_index = convertPoint3toCell(point);
    return signedDistance(point_index);
  }

  /** give a point, search for signed distance field and (optional) gradient
   * Keyword arguments:
      point -- query position
      g     -- returned gradient reference
   * return signed distance
  */
  inline double getSignedDistance(const gtsam::Point3& point,
                                  gtsam::Vector3& g) const {
    const float_index point_index = convertPoint3toCell(point);
    const gtsam::Vector3 gradient_index = gradient(point_index);
    // convert gradient of index to gradient of metric unit
    g = gtsam::Vector3(gradient_index(1), gradient_index(0),
                       gradient_index(2)) /
        cell_size_;
    return signedDistance(point_index);
  }

  // convert point3 to cell coordinate
  inline float_index convertPoint3toCell(const gtsam::Point3& point) const {
    // check point range
    if (point.x() < origin_.x() ||
        point.x() > (origin_.x() + (field_cols_ - 1.0) * cell_size_) ||
        point.y() < origin_.y() ||
        point.y() > (origin_.y() + (field_rows_ - 1.0) * cell_size_) ||
        point.z() < origin_.z() ||
        point.z() > (origin_.z() + (field_z_ - 1.0) * cell_size_)) {
      throw std::runtime_error("Querying SDF out of range");
    }

    const double col = (point.x() - origin_.x()) / cell_size_;
    const double row = (point.y() - origin_.y()) / cell_size_;
    const double z = (point.z() - origin_.z()) / cell_size_;
    return boost::make_tuple(row, col, z);
  }

  // convert cell coordinate to point3
  inline gtsam::Point3 convertCelltoPoint3(const float_index& cell) const {
    return origin_ + gtsam::Point3(cell.get<1>() * cell_size_,
                                   cell.get<0>() * cell_size_,
                                   cell.get<2>() * cell_size_);
  }

  // tri-linear interpolation
  inline double signedDistance(const float_index& index) const {
    const double lr = floor(index.get<0>()), lc = floor(index.get<1>()),
                 lz = floor(index.get<2>());
    const double hr = lr + 1.0, hc = lc + 1.0, hz = lz + 1.0;
    const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc),
                 lzi = static_cast<size_t>(lz), hri = static_cast<size_t>(hr),
                 hci = static_cast<size_t>(hc), hzi = static_cast<size_t>(hz);
    return (hr - index.get<0>()) * (hc - index.get<1>()) *
               (hz - index.get<2>()) * signedDistance(lri, lci, lzi) +
           (index.get<0>() - lr) * (hc - index.get<1>()) *
               (hz - index.get<2>()) * signedDistance(hri, lci, lzi) +
           (hr - index.get<0>()) * (index.get<1>() - lc) *
               (hz - index.get<2>()) * signedDistance(lri, hci, lzi) +
           (index.get<0>() - lr) * (index.get<1>() - lc) *
               (hz - index.get<2>()) * signedDistance(hri, hci, lzi) +
           (hr - index.get<0>()) * (hc - index.get<1>()) *
               (index.get<2>() - lz) * signedDistance(lri, lci, hzi) +
           (index.get<0>() - lr) * (hc - index.get<1>()) *
               (index.get<2>() - lz) * signedDistance(hri, lci, hzi) +
           (hr - index.get<0>()) * (index.get<1>() - lc) *
               (index.get<2>() - lz) * signedDistance(lri, hci, hzi) +
           (index.get<0>() - lr) * (index.get<1>() - lc) *
               (index.get<2>() - lz) * signedDistance(hri, hci, hzi);
  }

  // access
  inline double signedDistance(size_t r, size_t c, size_t z) const {
    return data_[z](r, c);
  }

  /** gradient operator for tri-linear interpolation
   *  gradient regrads to float_index
   * not differentiable at index point
  */
  inline gtsam::Vector3 gradient(const float_index& index) const {
    const double lr = floor(index.get<0>()), lc = floor(index.get<1>()),
                 lz = floor(index.get<2>());
    const double hr = lr + 1.0, hc = lc + 1.0, hz = lz + 1.0;
    const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc),
                 lzi = static_cast<size_t>(lz), hri = static_cast<size_t>(hr),
                 hci = static_cast<size_t>(hc), hzi = static_cast<size_t>(hz);
    return gtsam::Vector3(
        (hc - index.get<1>()) * (hz - index.get<2>()) *
                (signedDistance(hri, lci, lzi) -
                 signedDistance(lri, lci, lzi)) +
            (index.get<1>() - lc) * (hz - index.get<2>()) *
                (signedDistance(hri, hci, lzi) -
                 signedDistance(lri, hci, lzi)) +
            (hc - index.get<1>()) * (index.get<2>() - lz) *
                (signedDistance(hri, lci, hzi) -
                 signedDistance(lri, lci, hzi)) +
            (index.get<1>() - lc) * (index.get<2>() - lz) *
                (signedDistance(hri, hci, hzi) - signedDistance(lri, hci, hzi)),

        (hr - index.get<0>()) * (hz - index.get<2>()) *
                (signedDistance(lri, hci, lzi) -
                 signedDistance(lri, lci, lzi)) +
            (index.get<0>() - lr) * (hz - index.get<2>()) *
                (signedDistance(hri, hci, lzi) -
                 signedDistance(hri, lci, lzi)) +
            (hr - index.get<0>()) * (index.get<2>() - lz) *
                (signedDistance(lri, hci, hzi) -
                 signedDistance(lri, lci, hzi)) +
            (index.get<0>() - lr) * (index.get<2>() - lz) *
                (signedDistance(hri, hci, hzi) - signedDistance(hri, lci, hzi)),

        (hr - index.get<0>()) * (hc - index.get<1>()) *
                (signedDistance(lri, lci, hzi) -
                 signedDistance(lri, lci, lzi)) +
            (index.get<0>() - lr) * (hc - index.get<1>()) *
                (signedDistance(hri, lci, hzi) -
                 signedDistance(hri, lci, lzi)) +
            (hr - index.get<0>()) * (index.get<1>() - lc) *
                (signedDistance(lri, hci, hzi) -
                 signedDistance(lri, hci, lzi)) +
            (index.get<0>() - lr) * (index.get<1>() - lc) *
                (signedDistance(hri, hci, hzi) -
                 signedDistance(hri, hci, lzi)));
  }

  // return origin_
  const gtsam::Point3& origin() const { return origin_; }
  // return field data column size
  size_t xCount() const { return field_cols_; }
  // return field data row size
  size_t yCount() const { return field_rows_; }
  // return field data z size
  size_t zCount() const { return field_z_; }
  // return cell size
  double cellSize() const { return cell_size_; }
  // return field raw data
  const std::vector<gtsam::Matrix>& rawData() const { return data_; }

  /// print
  void print(const std::string& str = "") const {
    std::cout << str;
    std::cout << "field origin:     ";
    origin_.print();
    std::cout << "field resolution: " << cell_size_ << std::endl;
    std::cout << "field size:       " << field_z_ << " x " << field_cols_ 
              << " y " << field_rows_ << std::endl;
  }

  /// save to file
  void saveSDF(const std::string filename);

  /// load from file
  void loadSDF(const std::string filename);

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /* version */) {
    ar& BOOST_SERIALIZATION_NVP(origin_);
    ar& BOOST_SERIALIZATION_NVP(field_rows_);
    ar& BOOST_SERIALIZATION_NVP(field_cols_);
    ar& BOOST_SERIALIZATION_NVP(field_z_);
    ar& BOOST_SERIALIZATION_NVP(cell_size_);
    ar& BOOST_SERIALIZATION_NVP(data_);
  }
};
}  // namespace manipulator
