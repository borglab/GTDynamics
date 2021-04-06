import gtsam

def mergeValues(values, values_add):
    new_values = gtsam.Values(values_add)
    for key in values_add.keys():
        if values.exists(key):
            new_values.erase(key)
    values.insert(new_values)