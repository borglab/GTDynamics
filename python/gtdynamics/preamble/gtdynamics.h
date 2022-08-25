// Support for binding boost::optional types in C++11.
// https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
namespace pybind11 { namespace detail {
    template <typename T>
    struct type_caster<boost::optional<T>> : optional_caster<boost::optional<T>> {};
}}
