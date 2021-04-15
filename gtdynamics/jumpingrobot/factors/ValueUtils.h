/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ClippingActuatorFactor.h
 * @brief Factors related to pneumatic actuator.
 * @Author: Yetong Zhang
 */

#pragma once

#include <gtsam/nonlinear/Values.h>

// #include <boost/optional.hpp>
// #include <iostream>
// #include <string>

namespace gtdynamics {

gtsam::Values ExtractValues(const gtsam::Values& values, const gtsam::KeyVector& keys) {
    gtsam::Values extracted_values;
    for (gtsam::Key key: keys) {
        extracted_values.insert(key, values.at(key));
    }
    return extracted_values;
}

gtsam::Values ExtractValues(const gtsam::Values& values, const gtsam::KeySet& keys) {
    gtsam::Values extracted_values;
    for (gtsam::Key key: keys) {
        extracted_values.insert(key, values.at(key));
    }
    return extracted_values;
}

gtsam::KeyVector KeySetToKeyVector(const gtsam::KeySet& keys) {
    return gtsam::KeyVector(keys.begin(), keys.end());
}

}