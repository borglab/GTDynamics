// Please refer to: https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
// These are required to save one copy operation on Python calls
py::bind_map<gtdynamics::ContactPoints>(m_, "ContactPoints");
