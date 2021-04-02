/**
 * @file  PriorFactor.h
 * @brief Copy of PriorFactor from gtsam, so that I can more easily wrap
 * additional template types
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#pragma once

#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/base/Testable.h>

#include <string>

namespace gtdynamics {

/**
 * A class for a soft prior on any Value type
 */
template <typename T>
class PriorFactor : public gtsam::PriorFactor<T> {
 public:
  using gtsam::PriorFactor<T>::PriorFactor;

  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    gtsam::PriorFactor<T>::print(s, keyFormatter);
  }
};

}  // namespace gtdynamics
