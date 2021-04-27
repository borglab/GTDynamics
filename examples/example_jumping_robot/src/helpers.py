"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  helpers.py
 * @brief Helper functions.
 * @author Yetong Zhang
"""

import gtsam
import numpy as np

def mergeValues(values: gtsam.Values, values_add: gtsam.Values, overwrite=False):
    """ insert values, skip duplicate keys

    Args:
        values (gtsam.Values): values to insert into
        values_add (gtsam.Values): values to insert
        overwrite (bool): overwrite values if duplicate keys
    """
    new_values = gtsam.Values(values_add)
    for key in values_add.keys():
        if values.exists(key):
            if overwrite:
                values.erase(key)
            else:
                new_values.erase(key)
    values.insert(new_values)


# def interpolate_pressures(times: np.array) -> np.array:
#     [0, 0.1, 0.2]
#     k x 5 [[100, 10, 10, 10, 10], [], ]


#     [k, 5, 2, 2]
