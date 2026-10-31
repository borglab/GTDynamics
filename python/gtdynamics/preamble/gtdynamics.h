// Ensure these STL aliases are treated as opaque container types so the
// bind_vector/bind_map specializations behave predictably in Python.
PYBIND11_MAKE_OPAQUE(gtdynamics::PointOnLinks);
PYBIND11_MAKE_OPAQUE(gtdynamics::SelfCollisionPairs);
PYBIND11_MAKE_OPAQUE(gtdynamics::ContactPointGoals);

// Expose the existing nested parameter type without inventing a wrapper class.
namespace gtdynamics {
using ConstraintManifoldParams = ConstraintManifold::Params;
}
