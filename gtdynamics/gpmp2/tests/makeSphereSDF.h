/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  makeSphereSDF.h
 * @brief Sample the exact signed distance to a sphere onto a grid, for tests.
 * @author Karthik Shaji
 */

#pragma once

#include <gtdynamics/gpmp2/SignedDistanceField.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>

#include <vector>

namespace gtdynamics {

/// Sample the exact signed distance to a sphere onto a grid. The layer for z
/// index k is indexed as (row = y, col = x), matching SignedDistanceField.
inline SignedDistanceField makeSphereSDF(const gtsam::Point3 &center,
                                         double radius,
                                         const gtsam::Point3 &origin,
                                         double cell, size_t nx, size_t ny,
                                         size_t nz) {
  std::vector<gtsam::Matrix> data(nz);
  for (size_t k = 0; k < nz; ++k) {
    gtsam::Matrix layer(ny, nx);
    for (size_t i = 0; i < ny; ++i) {
      for (size_t j = 0; j < nx; ++j) {
        const gtsam::Point3 p =
            origin + gtsam::Point3(j * cell, i * cell, k * cell);
        layer(i, j) = (p - center).norm() - radius;
      }
    }
    data[k] = layer;
  }
  return SignedDistanceField(origin, cell, data);
}

}  // namespace gtdynamics
