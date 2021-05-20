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

gtsam::NonlinearFactorGraph RekeyNonlinearGraph(const gtsam::NonlinearFactorGraph& graph, 
                                                const gtsam::KeyVector& replaced_keys, 
                                                const gtsam::KeyVector& replacement_keys) {
    std::map<gtsam::Key, gtsam::Key> rekey_mapping;
    for (size_t i=0; i<replaced_keys.size(); i++) {
        rekey_mapping.insert(std::make_pair(replaced_keys[i], replacement_keys[i]));
    }
    return graph.rekey(rekey_mapping);
}

gtsam::Values RekeyValues(const gtsam::Values& values, const int offset, 
                          const gtsam::KeyVector& skip_keys) {
    gtsam::KeySet skip_keyset(skip_keys.begin(), skip_keys.end());
    gtsam::Values new_values;
    for (gtsam::Key key: values.keys()) {
        if (skip_keyset.exists(key)) {
            new_values.insert(key, values.at(key));
        }
        else {
            new_values.insert(key + offset, values.at(key));
        }
    }
    return new_values;
}

}