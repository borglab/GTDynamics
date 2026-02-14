// Minimal template file for wrapping C++ code.

#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/functional.h>
#include <pybind11/iostream.h>

#include "gtsam/config.h"
#include "gtsam/base/serialization.h"
#include "gtsam/base/utilities.h"  // for RedirectCout.

{includes}

{boost_class_export}

// Preamble for STL classes
#include "python/gtdynamics/preamble/{module_name}.h"

using namespace std;

namespace py = pybind11;

{submodules}

{module_def} {{
    m_.doc() = "pybind11 wrapper of {module_name}";

{submodules_init}

    {wrapped_namespace}
// Specializations for STL classes
#include "python/gtdynamics/specializations/{module_name}.h"

}}
