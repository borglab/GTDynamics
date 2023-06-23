/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IneqConstraintManifold.h
 * @brief Manifold with boundary/corners formulated by only inequality
 * constraints
 * @author: Yetong Zhang
 */

#pragma once

// #include "manifold/TspaceBasis.h"
#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/expressions.h>

namespace gtsam {

/** Manifold representing values satisfying inequality constraints, e.g.,
 * {X : h(X)>=0}. */
class IneqConstraintManifold {
public:
  typedef std::shared_ptr<IneqConstraintManifold> shared_ptr;

protected:
  gtdynamics::InequalityConstraints i_constraints_;
  gtsam::Values values_; // values of variables in CCC
  size_t dim_;           // dimension of constraint manifold
  IndexSet active_indices_;

public:
  IneqConstraintManifold(const gtdynamics::InequalityConstraints &i_constraints,
                         const gtsam::Values &values,
                         const IndexSet &active_indices)
      : i_constraints_(i_constraints), values_(values), dim_(values_.dim()),
        active_indices_(active_indices) {}

  virtual ~IneqConstraintManifold() {}

  const gtdynamics::InequalityConstraints &constraints() const {
    return i_constraints_;
  }

  const Values &values() const { return values_; }

  /// Set of inequality constraints that are currently active.
  const IndexSet &activeIndices() const { return active_indices_; }

  /// Dimension of the constraint manifold.
  inline size_t dim() const { return dim_; }

  /// Set of blocking constraints when going in the direction g, return set of
  /// blocking constraint indices, and the projected vector
  virtual std::pair<IndexSet, VectorValues>
  identifyBlockingConstraints(const VectorValues &g) const;

  /** retract that forces the set of tight constraints as equality constraints.
   * Will also enforce satisfying other inequality constraints. */
  virtual IneqConstraintManifold::shared_ptr
  retract(const gtsam::VectorValues &delta,
          const IndexSet &tight_indices) const;

  /// Retract using barrier
  IneqConstraintManifold::shared_ptr
  retractBarrier(const gtsam::VectorValues &delta,
                 const IndexSet &tight_indices) const;

  /// Retract using barrier
  IneqConstraintManifold::shared_ptr
  retractLineSearch(const gtsam::VectorValues &delta,
                    const IndexSet &tight_indices) const;
};

double point3_z(const Point3& p, OptionalJacobian<1, 3> H = {});


class DomeManifold : public IneqConstraintManifold {
protected:
  gtsam::Key key_;
  double radius_;

public:
  DomeManifold(const gtsam::Key key, const double radius,
               const gtsam::Point3 &point)
      : IneqConstraintManifold(ConstructConstraints(key, radius),
                               ConstructValues(key, point),
                               IdentifyActiveIndices(radius, point)),
        key_(key), radius_(radius) {}

protected:
  static gtsam::Values ConstructValues(const gtsam::Key key,
                                       const gtsam::Point3 &point) {
    Values values;
    values.insert(key, point);
    return values;
  }

  static gtdynamics::InequalityConstraints
  ConstructConstraints(const gtsam::Key key, const double radius) {
    gtdynamics::InequalityConstraints constraints;

    double tolerance = 1.0;
    gtsam::Expression<Point3> point_expr(key);
    gtsam::Double_ norm_expr(norm3, point_expr);
    gtsam::Double_ sphere_expr = gtsam::Double_(radius) - norm_expr;
    constraints.emplace_shared<gtdynamics::DoubleExpressionInequality>(
        sphere_expr, tolerance);

    gtsam::Double_ z_expr(point3_z, point_expr);
    constraints.emplace_shared<gtdynamics::DoubleExpressionInequality>(
        z_expr, tolerance);
    return constraints;
  }

  static IndexSet IdentifyActiveIndices(const double radius,
                                        const gtsam::Point3 &point) {
    IndexSet active_indices;
    if (radius - point.norm() < 1e-8) {
      active_indices.insert(0);
    }
    if (point.z() < 1e-8) {
      active_indices.insert(1);
    }
    return active_indices;
  }

 public:
  virtual std::pair<IndexSet, VectorValues>
  identifyBlockingConstraints(const VectorValues &g) const override{
    IndexSet blocking_indcies;
    gtsam::Vector3 vec = g.at(key_);

    if (active_indices_.find(0) != active_indices_.end()) {
      gtsam::Vector3 normal_vec = 1/radius_ * values_.at<Point3>(key_);
      if (dot(vec, normal_vec) > 0) {
        vec = vec - dot(vec, normal_vec) * normal_vec;
        blocking_indcies.insert(0);
      }
    }

    if (active_indices_.find(1) != active_indices_.end()) {
      double g_z = vec(2);
      if ((g_z) < 0) {
        vec(2) = 0;
        blocking_indcies.insert(1);
      }
    }
    VectorValues new_g;
    new_g.insert(key_, vec);
    return std::make_pair(blocking_indcies, new_g);
  }

  virtual IneqConstraintManifold::shared_ptr
  retract(const gtsam::VectorValues &delta,
          const IndexSet &tight_indices) const override {
    Point3 old_point = values_.at<Point3>(key_);
    Vector3 delta_vec = delta.at(key_);
    Point3 point = old_point + delta_vec;

    // IndexSet active_indices = tight_indices;
    if (point.z() < 0) {
      // active_indices.insert(1);
      point.z() = 0;
    }
    if (point.norm() > radius_) {
      // active_indices.insert(0);
      point = radius_ / point.norm() * point;
    }

    return std::make_shared<DomeManifold>(key_, radius_, point);
  }
};

Values
CollectManifoldValues(const std::vector<IneqConstraintManifold::shared_ptr> &manifolds);



} // namespace gtsam
