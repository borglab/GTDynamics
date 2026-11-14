"""Declare a ROS parameter and read it back as the type of its default.

Launch substitutions arrive YAML-parsed, so a rail sign of "-1" becomes an
integer and "0.1" a double; declaring with dynamic typing and casting here
stops a numeric launch argument from failing a strictly typed declaration.
"""

from rcl_interfaces.msg import ParameterDescriptor


def declare(node, name, default):
    node.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
    value = node.get_parameter(name).value
    if isinstance(default, bool):
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)
    if isinstance(default, float):
        return float(value)
    if isinstance(default, int):
        return int(value)
    if isinstance(default, str):
        return '' if value is None else str(value)
    return value
