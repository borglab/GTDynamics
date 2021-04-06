"""
@file   helpers.py
@brief  helper functions
@author Yetong Zhang
"""

import gtsam

def mergeValues(values: gtsam.Values, values_add: gtsam.Values):
    """ insert values, skip duplicate keys

    Args:
        values (gtsam.Values): values to insert into
        values_add (gtsam.Values): values to insert
    """
    new_values = gtsam.Values(values_add)
    for key in values_add.keys():
        if values.exists(key):
            new_values.erase(key)
    values.insert(new_values)