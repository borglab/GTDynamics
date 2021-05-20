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
import gtdynamics as gtd

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


def rekeyGraph(graph, offset, skip_keys=[]):
    """ Change the keys of a graph with some offset. """

    replaced_keys = []
    replacement_keys = []
    for key in graph.keys():
        if not key in skip_keys:
            replaced_keys.append(key)
            replacement_keys.append(key + offset)
    return gtd.RekeyNonlinearGraph(graph, replaced_keys, replacement_keys)


def rekeyValues(values, offset, skip_keys=[]):
    """ Change the keys of values with some offset. """
    return gtd.RekeyValues(values, offset, skip_keys)
    