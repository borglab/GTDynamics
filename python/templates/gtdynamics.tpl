// Minimal template file for wrapping C++ code.

{include_boost}

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <boost/optional.hpp>

#include "gtsam/base/serialization.h"
#include "gtsam/config.h"
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.

{includes}
#include <boost/serialization/export.hpp>

{boost_class_export}

{holder_type}

// Preamble for STL classes
#include "python/preamble.h"

using namespace std;

namespace py = pybind11;

PYBIND11_MODULE({module_name}, m_) {{
    m_.doc() = "pybind11 wrapper of {module_name}";

    {wrapped_namespace}

// Specializations for STL classes
#include "python/specializations.h"

}}
