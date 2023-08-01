/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IEHalfSphere.h
 * @brief Manifold with boundary/corners formulated by only inequality
 * constraints
 * @author: Yetong Zhang
 */

#pragma once
#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtdynamics/manifold/IneqConstraintManifold.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/geometry/Point3.h>

namespace gtsam {

inline Key PointKey(const int k) { return Symbol('p', k); }

class IEHalfSphere {
public:
  double r;
  double sphere_tol = 1.0;
  double z_tol = 1.0;

  IEHalfSphere(const double _r = 1.0) : r(_r) {}

  gtdynamics::EqualityConstraints eConstraints(const int k) const {
    gtdynamics::EqualityConstraints constraints;
    gtsam::Expression<Point3> point_expr(PointKey(k));
    Double_ norm_expr(&norm3, point_expr);
    Double_ sphere_expr = norm_expr - Double_(r);
    constraints.emplace_shared<gtdynamics::DoubleExpressionEquality>(sphere_expr, sphere_tol);
    return constraints;
  }

  gtdynamics::InequalityConstraints iConstraints(const int k) const {
    gtdynamics::InequalityConstraints constraints;
    gtsam::Expression<Point3> point_expr(PointKey(k));
    gtsam::Expression<double> z_expr(&point3_z, point_expr);
    constraints.emplace_shared<gtdynamics::DoubleExpressionInequality>(z_expr, 1.0);
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
    std::cout << std::setw(10) << "x" << std::setw(10)  << "y" << std::setw(10) << "z" << "\n";
    for (size_t k=0; k<=num_steps; k++) {
      auto p = values.at<Point3>(PointKey(k));
      std::cout << std::setprecision(3)<< std::setw(10) << p.x() << std::setw(10) << p.y() << std::setw(10) << p.z() << "\n";
    }
  }

  static void PrintDelta(const VectorValues &values, const size_t num_steps) {
    std::cout << std::setw(10) << "x" << std::setw(10) << "y" << std::setw(10) << "z" << "\n";
    for (size_t k=0; k<=num_steps; k++) {
      auto p = values.at(PointKey(k));
      std::cout <<std::setw(10) << p(0) << std::setw(10) << p(1) << std::setw(10) << p(2) << "\n";
    }
  }
};

} // namespace gtsam
