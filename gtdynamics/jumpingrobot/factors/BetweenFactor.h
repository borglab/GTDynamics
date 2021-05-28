/**
 * @file  BetweenFactor.h
 * @brief Copy of BetweenFactor from gtsam, so that I can more easily wrap
 * additional template types
 * @author Yetong Zhang
 */

#pragma once

#include <gtdynamics/utils/DynamicsSymbol.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/base/Testable.h>

#include <string>

namespace gtdynamics {

/**
 * A class for a between factor of any Value type
 */
template <typename T>
class BetweenFactor : public gtsam::BetweenFactor<T> {
 public:
  using gtsam::BetweenFactor<T>::BetweenFactor;

  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 GTDKeyFormatter) const override {
    gtsam::BetweenFactor<T>::print(s, keyFormatter);
  }
};

}  // namespace gtdynamics
