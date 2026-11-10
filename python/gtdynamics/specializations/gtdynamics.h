// Please refer to: https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
// These are required to save one copy operation on Python calls
py::bind_vector<gtdynamics::PointOnLinks>(m_, "PointOnLinks");
py::bind_vector<gtdynamics::SelfCollisionPairs>(m_, "SelfCollisionPairs");
py::bind_map<gtdynamics::ContactPointGoals>(m_, "ContactPointGoals");

// Extend GTSAM's existing value container with the concrete GTDynamics points.
// Register the equality overload first: ECMPoint is also an IECMPoint in Python.
auto constrainedPointValues = py::reinterpret_borrow<
    py::class_<gtsam::Values, std::shared_ptr<gtsam::Values>>>(
    py::module_::import("gtsam").attr("Values"));
constrainedPointValues
    .def("insert", [](gtsam::Values& values, gtsam::Key key,
                      const gtdynamics::ECMPoint& point) {
      values.insert(key, point);
    }, py::arg("key"), py::arg("value"))
    .def("insert", [](gtsam::Values& values, gtsam::Key key,
                      const gtdynamics::IECMPoint& point) {
      values.insert(key, point);
    }, py::arg("key"), py::arg("value"))
    .def("atECMPoint", [](const gtsam::Values& values, gtsam::Key key) {
      return values.at<gtdynamics::ECMPoint>(key);
    }, py::arg("key"))
    .def("atIECMPoint", [](const gtsam::Values& values, gtsam::Key key) {
      return values.at<gtdynamics::IECMPoint>(key);
    }, py::arg("key"));
