/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file IEHalfSphere.h
 * @brief Manifold with boundary representing the upper half-sphere.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmcopt/IERetractor.h>
#include <gtdynamics/utils/utils.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/constrained/NonlinearInequalityConstraint.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/expressions.h>

#include <fstream>
#include <iomanip>
#include <iostream>

namespace gtdynamics {
using namespace gtsam;

/** Key for the point variable at timestep k. */
inline Key PointKey(const int k) { return Symbol('p', k); }

/**
 * Manifold with boundary representing the upper half-sphere:
 * 1) sqrt(x^2 + y^2 + z^2) - r = 0.
 * 2) z >= 0.
 */
class IEHalfSphere {
 public:
  /// Radius of the sphere.
  double radius_;

  /// Sphere constraint tolerance.
  double sphere_tol_ = 1.0;

  /// Positive z constraint tolerance.
  double z_tol_ = 1.0;

  /** Constructor. */
  IEHalfSphere(const double radius = 1.0) : radius_(radius) {}

  /** Return equality constraints defining the sphere manifold. */
  NonlinearEqualityConstraints eConstraints(const int k) const {
    NonlinearEqualityConstraints constraints;
    Expression<Point3> point_expr(PointKey(k));
    Double_ norm_expr(&norm3, point_expr);
    Double_ sphere_expr = Double_(radius_) - norm_expr;
    constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
        sphere_expr, 0.0, Vector1(sphere_tol_));
    return constraints;
  }

  /** Return inequality constraints defining the upper half-space. */
  NonlinearInequalityConstraints iConstraints(const int k) const {
    NonlinearInequalityConstraints constraints;
    Expression<Point3> point_expr(PointKey(k));
    Expression<double> z_expr(&point3_z, point_expr);
    constraints.push_back(
        ScalarExpressionInequalityConstraint::GeqZero(z_expr, z_tol_));
    return constraints;
  }

  /** Return inequality constraints defining the solid upper dome. */
  NonlinearInequalityConstraints iDomeConstraints(const int k) const {
    NonlinearInequalityConstraints constraints;
    Expression<Point3> point_expr(PointKey(k));
    Double_ norm_expr(norm3, point_expr);
    Double_ sphere_expr = Double_(radius_) - norm_expr;
    constraints.push_back(ScalarExpressionInequalityConstraint::GeqZero(
        sphere_expr, sphere_tol_));
    Double_ z_expr(&point3_z, point_expr);
    constraints.push_back(
        ScalarExpressionInequalityConstraint::GeqZero(z_expr, z_tol_));
    return constraints;
  }

  /** Project a point onto the feasible upper half-sphere. */
  Point3 project(const Point3 &pt) const {
    Point3 projected_pt = pt;
    if (projected_pt.z() < 0) {
      projected_pt.z() = 0;
    }
    double pt_norm = projected_pt.norm();
    projected_pt = projected_pt * (radius_ / pt_norm);
    return projected_pt;
  }

  /** Export trajectory values to a plain text file. */
  static void ExportValues(const Values &values, const size_t num_steps,
                           const std::string &file_path) {
    std::ofstream file;
    file.open(file_path);
    for (size_t k = 0; k <= num_steps; k++) {
      Key point_key = Symbol('p', k);
      Point3 point = values.at<Point3>(point_key);
      file << point.x() << " " << point.y() << " " << point.z() << "\n";
    }
    file.close();
  }

  /** Export tangent-vector values to a plain text file. */
  static void ExportVector(const VectorValues &values, const size_t num_steps,
                           const std::string &file_path) {
    std::ofstream file;
    file.open(file_path);
    for (size_t k = 0; k <= num_steps; k++) {
      Key point_key = Symbol('p', k);
      Vector tv = values.at(point_key);
      file << tv(0) << " " << tv(1) << " " << tv(2) << "\n";
    }
    file << "\n";
    file.close();
  }

  /** Print trajectory values in a compact table. */
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

  /** Print tangent-vector values in a compact table. */
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

}  // namespace gtdynamics
