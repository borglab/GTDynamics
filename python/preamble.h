namespace pybind11 { namespace detail {
    template <typename T>
    struct type_caster<boost::optional<T>> : optional_caster<boost::optional<T>> {};
}}

/// Needed to ensure there is reference passing
/// https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html#making-opaque-types
// PYBIND11_MAKE_OPAQUE(std::vector<gtdynamics::JointSharedPtr>);
// PYBIND11_MAKE_OPAQUE(std::vector<gtdynamics::LinkSharedPtr>);
// PYBIND11_MAKE_OPAQUE(std::map<std::string, gtdynamics::JointSharedPtr>);
// PYBIND11_MAKE_OPAQUE(std::map<std::string, gtdynamics::LinkSharedPtr>);
