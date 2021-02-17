namespace pybind11 { namespace detail {
    template <typename T>
    struct type_caster<boost::optional<T>> : optional_caster<boost::optional<T>> {};
}}

PYBIND11_MAKE_OPAQUE(std::vector<gtdynamics::JointSharedPtr>);
PYBIND11_MAKE_OPAQUE(std::vector<gtdynamics::LinkSharedPtr>);
