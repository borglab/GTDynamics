/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file IERetractor.cpp
 * @brief IERetractor implementation.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmcopt/IERetractor.h>
#include <gtdynamics/cmcopt/IECMPoint.h>

#include <stdexcept>

namespace gtdynamics {
using namespace gtsam;

IERetractor::shared_ptr IERetractor::bind(const IECMPoint &point) const {
  return std::static_pointer_cast<IERetractor>(CMRetractor::bindPoint(point));
}

CMRetractor::shared_ptr IERetractor::bindPoint(const CMPoint &point) const {
  const auto *iePoint = dynamic_cast<const IECMPoint *>(&point);
  if (!iePoint) throw std::invalid_argument("IE retraction requires an IECMPoint");
  return bind(*iePoint);
}

Values IERetractor::retractValues(const CMPoint &point,
                                 const VectorValues &delta,
                                 CMRetractionInfo *info) const {
  const auto *iePoint = dynamic_cast<const IECMPoint *>(&point);
  if (!iePoint) throw std::invalid_argument("IE retraction requires an IECMPoint");
  return retract(iePoint, delta, {}, info).values();
}

CMRetractor::shared_ptr IERetractorCreator::prepare(const CMPoint &point) const {
  const auto *iePoint = dynamic_cast<const IECMPoint *>(&point);
  if (!iePoint) throw std::invalid_argument("IE retraction requires an IECMPoint");
  return create(*iePoint);
}

IERetractor::shared_ptr UniversalIERetractorCreator::create(
    const IECMPoint &point) const {
  return retractor_->bind(point);
}

IECMPoint
IERetractor::moveToBoundary(const IECMPoint *point,
                            const IndexSet &blocking_indices,
                            IERetractionInfo *retract_info) const {
  VectorValues delta = point->values().zeroVectors();
  return retract(point, delta, blocking_indices, retract_info);
}


}  // namespace gtdynamics
