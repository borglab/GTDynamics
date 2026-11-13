/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  CMCostFactor.cpp
 * @brief Slot connections for costs on constrained points.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/constrained/CMCostFactor.h>
#include <gtdynamics/cmopt/ECMPoint.h>

namespace gtdynamics {
namespace {

// GTSAM values must be extracted using their exact concrete point category.
const CMPoint& costPoint(const Values& values, Key key,
                         CMCostFactor::PointType pointType) {
  if (pointType == CMCostFactor::PointType::Equality)
    return values.at(key).cast<ECMPoint>();
  return values.at(key).cast<IECMPoint>();
}

void validateSlot(const CMPoint& point, Key slot, Key ambientKey) {
  const auto found = point.bindings().find(slot);
  if (found == point.bindings().end() || found->second != ambientKey)
    throw std::invalid_argument("Cost argument does not match point slot binding");
}

}  // namespace

/* ************************************************************************* */
CMCostFactor::Layout CMCostFactor::buildLayout(
    const Base::shared_ptr& originalFactor, const OwnershipIndex& ownership,
    const Values& fixedPoints, PointType pointType) {
  if (!originalFactor)
    throw std::invalid_argument("A point cost requires an original factor");

  Layout layout;
  std::map<Key, size_t> pointConnections;
  for (size_t index = 0; index < originalFactor->keys().size(); ++index) {
    const Key ambientKey = originalFactor->keys()[index];
    const auto found = ownership.find(ambientKey);
    if (found == ownership.end()) {
      layout.keys.push_back(ambientKey);
      Connection connection;
      connection.originalIndex = index;
      layout.connections.push_back(std::move(connection));
      continue;
    }

    const auto& owner = found->second;
    if (fixedPoints.exists(owner.pointKey)) {
      const CMPoint& point = costPoint(fixedPoints, owner.pointKey, pointType);
      if (point.dim())
        throw std::invalid_argument("A fixed point cost argument must have dimension zero");
      validateSlot(point, owner.slot, ambientKey);
      layout.fixedAmbientValues.insert(ambientKey, point.recoverSlot(owner.slot));
      continue;
    }

    // Several original arguments may contribute to the same point Jacobian.
    auto entry = pointConnections.emplace(owner.pointKey, layout.keys.size());
    if (entry.second) {
      layout.keys.push_back(owner.pointKey);
      Connection connection;
      connection.point = true;
      layout.connections.push_back(std::move(connection));
    }
    layout.connections.at(entry.first->second).arguments.push_back(
        {index, owner.slot});
  }

  if (gtsam::KeySet(layout.keys.begin(), layout.keys.end()).size() != layout.keys.size())
    throw std::invalid_argument("Point cost keys collide with ambient variable keys");
  return layout;
}

/* ************************************************************************* */
CMCostFactor::CMCostFactor(const Base::shared_ptr& originalFactor,
                           const OwnershipIndex& ownership,
                           const Values& fixedPoints, PointType pointType)
    : CMCostFactor(originalFactor, pointType,
                   buildLayout(originalFactor, ownership, fixedPoints, pointType)) {}

CMCostFactor::CMCostFactor(const Base::shared_ptr& originalFactor,
                           PointType pointType, Layout layout)
    : Base(originalFactor->noiseModel(), layout.keys),
      pointType_(pointType),
      originalFactor_(originalFactor),
      connections_(std::move(layout.connections)),
      fixedAmbientValues_(std::move(layout.fixedAmbientValues)) {}

/* ************************************************************************* */
Vector CMCostFactor::unwhitenedError(
    const Values& x, gtsam::OptionalMatrixVecType H) const {
  // Reconstruct the original factor's ambient arguments from x, evaluate its
  // residual, and convert any requested Jacobians to point/free coordinates.
  // x uses this factor's keys (point keys and free-variable keys), whereas
  // originalFactor_ expects ambient keys. connections_[index] describes how
  // the variable at keys()[index] supplies the original factor's arguments.
  // For example, with original keys [v17, q17, u] and point c0 bindings
  // {slot0 -> q17, slot1 -> v17}, this factor has keys [c0, u]:
  //   connection 0: original argument 0 -> slot1, argument 1 -> slot0;
  //   connection 1: free variable supplying original argument 2.

  // Start the ambient argument Values with constants captured from fixed points.
  // These arguments need neither a connection to x nor an output Jacobian block.
  Values ambient = fixedAmbientValues_;

  // Prepare temporary storage for recovery derivatives when H is requested.
  // Index it by [connection][argument within that connection]. Each matrix
  // maps point tangent increments to the corresponding ambient slot's tangent.
  std::vector<std::vector<gtsam::Matrix>> recoveryJacobians;
  if (H) recoveryJacobians.resize(connections_.size());

  // Fill the remaining ambient arguments from each connected optimization
  // variable: copy a free variable, or recover the slots supplied by a point.
  for (size_t index = 0; index < connections_.size(); ++index) {
    const auto& connection = connections_[index];
    if (!connection.point) {
      // Copy this free variable directly from x into the original argument Values.
      // Its key is unchanged, but its position in the two factors can differ.
      const Key ambientKey = originalFactor_->keys()[connection.originalIndex];
      ambient.insert(ambientKey, x.at(keys()[index]));
      continue;
    }

    // Fetch the current point and reserve one recovery derivative per argument
    // it supplies. A single point may supply several original arguments.
    const CMPoint& point = costPoint(x, keys()[index], pointType_);
    if (H) recoveryJacobians[index].resize(connection.arguments.size());

    // Recover each connected slot under the ambient key expected by the cost.
    for (size_t argumentIndex = 0; argumentIndex < connection.arguments.size();
         ++argumentIndex) {
      const auto& argument = connection.arguments[argumentIndex];
      // Locate the destination key and verify that this point's slot still binds
      // to it. originalIndex selects the cost argument; slot selects local state.
      const Key ambientKey = originalFactor_->keys()[argument.originalIndex];
      validateSlot(point, argument.slot, ambientKey);
      if (H) {
        // Insert the recovered value and save its derivative with respect to
        // the point tangent for the later chain-rule multiplication.
        ambient.insert(ambientKey, point.recoverSlot(
            argument.slot, recoveryJacobians[index][argumentIndex]));
      } else {
        // Insert just the value when the caller requested only the residual.
        ambient.insert(ambientKey, point.recoverSlot(argument.slot));
      }
    }
  }

  // Evaluate and return the residual immediately when no derivatives are needed.
  if (!H) return originalFactor_->unwhitenedError(ambient);

  // Evaluate the original cost once to obtain its residual and all ambient
  // Jacobians. originalH[a] is d(residual)/d(argument a), in original key order.
  std::vector<gtsam::Matrix> originalH(originalFactor_->keys().size());
  const Vector error = originalFactor_->unwhitenedError(ambient, originalH);

  // Assemble one output Jacobian per optimization variable, in this factor's
  // key order: pass through free-variable blocks and combine point arguments.
  H->resize(connections_.size());
  for (size_t index = 0; index < connections_.size(); ++index) {
    const auto& connection = connections_[index];
    if (!connection.point) {
      // Copy the original derivative for this free variable without a coordinate
      // change, selecting it by its original argument position.
      (*H)[index] = originalH.at(connection.originalIndex);
      continue;
    }

    // Build this point's Jacobian by multiplying each original argument block
    // by its saved recovery derivative and summing the contributions. Initialize
    // with the first product, then add the rest:
    // d(residual)/d(point tangent) = sum_a originalH[a] * recoveryH[a].
    for (size_t argumentIndex = 0; argumentIndex < connection.arguments.size();
         ++argumentIndex) {
      const auto& argument = connection.arguments[argumentIndex];
      const auto contribution = originalH.at(argument.originalIndex) *
                                recoveryJacobians[index][argumentIndex];
      if (!argumentIndex)
        (*H)[index] = contribution;
      else
        (*H)[index] += contribution;
    }
  }
  return error;
}

/* ************************************************************************* */
gtsam::NonlinearFactorGraph pointCostGraph(
    const gtsam::NonlinearFactorGraph& graph, const Values& points,
    CMCostFactor::PointType pointType) {
  CMCostFactor::OwnershipIndex ownership;
  Values fixedPoints;
  for (Key pointKey : points.keys()) {
    const CMPoint& point = costPoint(points, pointKey, pointType);
    if (!point.manifold())
      throw std::invalid_argument("A cost graph requires initialized points");
    for (const auto& [slot, ambientKey] : point.bindings()) {
      if (points.exists(ambientKey) ||
          !ownership.emplace(ambientKey, CMCostFactor::SlotOwner{pointKey, slot}).second)
        throw std::invalid_argument("Point bindings have colliding ambient ownership");
    }
    if (!point.dim()) fixedPoints.insert(pointKey, points.at(pointKey));
  }

  gtsam::NonlinearFactorGraph result;
  for (const auto& factor : graph) {
    if (!factor) continue;
    bool usesPoint = false;
    for (Key key : factor->keys()) {
      if (points.exists(key))
        throw std::invalid_argument("Point cost keys collide with ambient variable keys");
      usesPoint |= ownership.count(key) != 0;
    }
    if (!usesPoint) {
      result.push_back(factor);
      continue;
    }
    const auto noise = std::dynamic_pointer_cast<NoiseModelFactor>(factor);
    if (!noise)
      throw std::invalid_argument("Point costs require noise-model costs");
    auto transformed = std::make_shared<CMCostFactor>(
        noise, ownership, fixedPoints, pointType);
    // Fully fixed costs remain in the ambient objective, outside Gaussian variables.
    if (transformed->checkActive()) result.push_back(transformed);
  }
  return result;
}

}  // namespace gtdynamics
