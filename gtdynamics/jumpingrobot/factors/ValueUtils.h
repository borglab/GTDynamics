/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ValueUtils.h
 * @brief Some value related utils which are hard to implement in Python.
 * @Author: Yetong Zhang
 */

#pragma once

#include <gtsam/nonlinear/Values.h>

namespace gtdynamics {

/* Extract values with specified keys, (note: we need this because values.at() is not wrapped in Python) */
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

/* Turn a KeySet to KeyVector */
gtsam::KeyVector KeySetToKeyVector(const gtsam::KeySet& keys) {
    return gtsam::KeyVector(keys.begin(), keys.end());
}

}