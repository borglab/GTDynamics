/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  CMCostFactor.h
 * @brief Pull an ambient cost back to constrained point coordinates.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <map>

namespace gtdynamics {

using gtsam::Key;
using gtsam::KeyVector;
using gtsam::NoiseModelFactor;
using gtsam::Values;
using gtsam::Vector;

/** Evaluate an original ambient cost through point-local slot connections.
 * The original factor supplies unwhitened residuals; this factor applies its
 * shared noise model once. Numerical recovery always uses the current point.
 */
class CMCostFactor : public NoiseModelFactor {
 public:
  enum class PointType { Inequality, Equality };

  /// Temporary ownership information used only while constructing factors.
  struct SlotOwner {
    Key pointKey;
    Key slot;
  };
  using OwnershipIndex = std::map<Key, SlotOwner>;

 protected:
  typedef NoiseModelFactor Base;
  typedef CMCostFactor This;

 private:
  struct Argument {
    size_t originalIndex = 0;
    Key slot = 0;

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
      ar& BOOST_SERIALIZATION_NVP(originalIndex);
      ar& BOOST_SERIALIZATION_NVP(slot);
    }
#endif
  };

  // Each connection corresponds to the same entry in this factor's keys().
  struct Connection {
    bool point = false;
    size_t originalIndex = 0;  // Used only for a free-variable connection.
    std::vector<Argument> arguments;

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
      ar& BOOST_SERIALIZATION_NVP(point);
      ar& BOOST_SERIALIZATION_NVP(originalIndex);
      ar& BOOST_SERIALIZATION_NVP(arguments);
    }
#endif
  };

  struct Layout {
    KeyVector keys;
    std::vector<Connection> connections;
    Values fixedAmbientValues;
  };

  PointType pointType_ = PointType::Inequality;
  Base::shared_ptr originalFactor_;
  std::vector<Connection> connections_;
  Values fixedAmbientValues_;

  static Layout buildLayout(const Base::shared_ptr& originalFactor,
                            const OwnershipIndex& ownership,
                            const Values& fixedPoints, PointType pointType);
  CMCostFactor(const Base::shared_ptr& originalFactor, PointType pointType,
               Layout layout);

 public:
  typedef std::shared_ptr<This> shared_ptr;

  /// Default constructor for I/O only.
  CMCostFactor() {}

  /// Compile argument-to-slot connections, then discard the ownership index.
  CMCostFactor(const Base::shared_ptr& originalFactor,
               const OwnershipIndex& ownership,
               const Values& fixedPoints = Values(),
               PointType pointType = PointType::Inequality);

  Vector unwhitenedError(
      const Values& x, gtsam::OptionalMatrixVecType H = nullptr) const override;

  /// Copy connections and fixed values while sharing the original cost.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<This>(*this);
  }

  /// Fully fixed costs have no optimizable variables.
  inline bool checkActive() const { return size() > 0; }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "CMCostFactor", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(pointType_);
    ar& BOOST_SERIALIZATION_NVP(originalFactor_);
    ar& BOOST_SERIALIZATION_NVP(connections_);
    ar& BOOST_SERIALIZATION_NVP(fixedAmbientValues_);
  }
#endif

};  // \class CMCostFactor

/// Build point costs using the bindings of all points, including fixed ones.
gtsam::NonlinearFactorGraph pointCostGraph(
    const gtsam::NonlinearFactorGraph& graph, const Values& points,
    CMCostFactor::PointType pointType);

}  // namespace gtdynamics
