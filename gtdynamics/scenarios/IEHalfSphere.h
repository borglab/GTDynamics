/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IEHalfSphere.h
 * @brief Manifold with boundary representing upper half sphere
 * @author: Yetong Zhang
 */

#pragma once
#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IERetractor.h>
#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtdynamics/utils/utils.h>

namespace gtsam {

inline Key PointKey(const int k) { return Symbol('p', k); }

/** Manifold with boundary representing the upper half sphere defined by
 * 1) sqrt(x^2 + y^2 + z^2) - r = 0
 * 2) z >= 0
 */
class IEHalfSphere {
public:
  double r;                // radius
  double sphere_tol = 1.0; // sphere constraint tolerance
  double z_tol = 1.0;      // positive z constraint tolerance

  /// Constructor.
  IEHalfSphere(const double _r = 1.0) : r(_r) {}

  /// Equality constraints defining the manifold.
  gtdynamics::EqualityConstraints eConstraints(const int k) const {
    gtdynamics::EqualityConstraints constraints;
    gtsam::Expression<Point3> point_expr(PointKey(k));
    Double_ norm_expr(&norm3, point_expr);
    Double_ sphere_expr = Double_(r) - norm_expr;
    constraints.emplace_shared<gtdynamics::DoubleExpressionEquality>(
        sphere_expr, sphere_tol);
    return constraints;
  }

  /// Inequality constraints defining the manifold.
  gtdynamics::InequalityConstraints iConstraints(const int k) const {
    gtdynamics::InequalityConstraints constraints;
    gtsam::Expression<Point3> point_expr(PointKey(k));
    gtsam::Expression<double> z_expr(&point3_z, point_expr);
    constraints.emplace_shared<gtdynamics::DoubleExpressionInequality>(z_expr,
                                                                       z_tol);
    return constraints;
  }

  /// Inequality constraints defining the dome manifold.
  gtdynamics::InequalityConstraints iDomeConstraints(const int k) const {
    gtdynamics::InequalityConstraints constraints;
    gtsam::Expression<Point3> point_expr(PointKey(k));
    Double_ norm_expr(norm3, point_expr);
    Double_ sphere_expr = Double_(r) - norm_expr;
    constraints.emplace_shared<gtdynamics::DoubleExpressionInequality>(
        sphere_expr, sphere_tol, "sphere");
    Double_ z_expr(&point3_z, point_expr);
    constraints.emplace_shared<gtdynamics::DoubleExpressionInequality>(
        z_expr, z_tol, "positive_z");
    return constraints;
  }

  static void ExportValues(const Values &values, const size_t num_steps,
                           const std::string &file_path) {
    std::ofstream file;
    file.open(file_path);
    for (int k = 0; k <= num_steps; k++) {
      Key point_key = gtsam::Symbol('p', k);
      Point3 point = values.at<Point3>(point_key);
      file << point.x() << " " << point.y() << " " << point.z() << "\n";
    }
    file.close();
  }

  static void ExportVector(const VectorValues &values, const size_t num_steps,
                           const std::string &file_path) {
    std::ofstream file;
    file.open(file_path);
    for (int k = 0; k <= num_steps; k++) {
      Key point_key = gtsam::Symbol('p', k);
      Vector tv = values.at(point_key);
      file << tv(0) << " " << tv(1) << " " << tv(2) << "\n";
    }
    file << "\n";
    file.close();
  }

  static void PrintValues(const Values &values, const size_t num_steps) {
    std::cout << std::setw(10) << "x" << std::setw(10) << "y" << std::setw(10)
              << "z"
              << "\n";
    for (size_t k = 0; k <= num_steps; k++) {
      auto p = values.at<Point3>(PointKey(k));
      std::cout << std::setprecision(3) << std::setw(10) << p.x()
                << std::setw(10) << p.y() << std::setw(10) << p.z() << "\n";
    }
  }

  static void PrintDelta(const VectorValues &values, const size_t num_steps) {
    std::cout << std::setw(10) << "x" << std::setw(10) << "y" << std::setw(10)
              << "z"
              << "\n";
    for (size_t k = 0; k <= num_steps; k++) {
      auto p = values.at(PointKey(k));
      std::cout << std::setw(10) << p(0) << std::setw(10) << p(1)
                << std::setw(10) << p(2) << "\n";
    }
  }
};

/** Manually defined retractor for half sphere manifold. */
class HalfSphereRetractor : public IERetractor {
  double r_;

public:
  HalfSphereRetractor(const IEHalfSphere &half_sphere)
      : IERetractor(), r_(half_sphere.r) {}

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractInfo* retract_info = nullptr) const override {
    Key key = manifold->values().keys().front();
    Point3 p = manifold->values().at<Point3>(key);
    Vector3 v = delta.at(key);
    Point3 new_p = p + v;

    if (blocking_indices && blocking_indices->size() > 0) {
      new_p.z() = 0;
    }
    new_p.z() = std::max(0.0, new_p.z());
    new_p = normalize(new_p) * r_;

    Values new_values;
    new_values.insert(key, new_p);
    return manifold->createWithNewValues(new_values);
  }
};


/** Manually defined retractor for half sphere manifold. */
class SphereRetractor : public IERetractor {
  double r_;

public:
  SphereRetractor(const IEHalfSphere &half_sphere)
      : IERetractor(), r_(half_sphere.r) {}

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractInfo* retract_info = nullptr) const override {
    Key key = manifold->values().keys().front();
    Point3 p = manifold->values().at<Point3>(key);
    Vector3 v = delta.at(key);
    Point3 new_p = p + v;

    new_p = normalize(new_p) * r_;
    Values new_values;
    new_values.insert(key, new_p);
    return manifold->createWithNewValues(new_values);
  }
};


/** Manually defined retractor for half sphere manifold. */
class DomeRetractor : public IERetractor {
  double r_;

public:
  DomeRetractor(const IEHalfSphere &half_sphere)
      : IERetractor(), r_(half_sphere.r) {}

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractInfo* retract_info = nullptr) const override {

    Key key = manifold->values().keys().front();
    Point3 p = manifold->values().at<Point3>(key);
    Vector3 v = delta.at(key);
    Point3 new_p = p + v;

    new_p.z() = std::max(0.0, new_p.z());
    if (new_p.norm() > r_) {
      new_p = r_ / new_p.norm() * new_p;
    }

    Values new_values;
    new_values.insert(key, new_p);
    return manifold->createWithNewValues(new_values);
  }

  IEConstraintManifold
  moveToBoundary(const IEConstraintManifold *manifold,
                 const IndexSet &blocking_indices,
                 IERetractInfo* retract_info = nullptr) const override {
    Key key = manifold->values().keys().front();
    Point3 p = manifold->values().at<Point3>(key);

    // std::cout << "old p: \n" << p << "\n";

    bool is_sphere = false;
    bool is_positive_z = false;

    IndexSet active_indices = blocking_indices;
    for (const auto &constraint_idx : manifold->activeIndices()) {
      active_indices.insert(constraint_idx);
    }

    for (const auto &constraint_idx : active_indices) {
      const auto &constraint = manifold->iConstraints()->at(constraint_idx);
      if (constraint->name() == "sphere") {
        is_sphere = true;
      }
      if (constraint->name() == "positive_z") {
        is_positive_z = true;
      }
    }

    if (is_positive_z) {
      p.z() = 0;
    }

    if (is_sphere) {
      p = r_ / p.norm() * p;
    }

    // std::cout << "new p: \n" << p << "\n";

    Values new_values;
    new_values.insert(key, p);
    return manifold->createWithNewValues(new_values);
  }
};

} // namespace gtsam
