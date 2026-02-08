# Find Python - use Development.Module (doesn't require full libpython,
# which may not be available in manylinux containers).
find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)

# Set the old-style variable for compatibility with gtwrap/pybind11 scripts.
set(PYTHON_EXECUTABLE "${Python_EXECUTABLE}" CACHE PATH "Python executable" FORCE)

# Pre-set WRAP_PYTHON_VERSION so that gtwrap_get_python_version() (called
# internally when gtwrapConfig.cmake is loaded) takes the "specific version"
# branch and does not fall back to its own find_package(Python) call which
# requests the full Development component and may fail in CI containers.
set(WRAP_PYTHON_VERSION
    "${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}.${Python_VERSION_PATCH}"
    CACHE STRING "The version of Python to build the wrappers against." FORCE)

# Find gtwrap (shipped with GTSAM).
find_package(gtwrap REQUIRED)

message(STATUS "GTDynamics building with Python: ${Python_EXECUTABLE}")
message(STATUS "Python Version Found: ${Python_VERSION}")

set(PYTHON_REQUIREMENTS_PATH "requirements.txt")
set(PYTHON_TESTS "${PROJECT_NAME}.tests")

set(GTDYNAMICS_PYTHON_PATH ${PROJECT_SOURCE_DIR}/python/)

file(TO_NATIVE_PATH "${PROJECT_BINARY_DIR}/python" GTD_PYTHON_BINARY_DIR)

configure_file(${GTDYNAMICS_PYTHON_PATH}/templates/pyproject.toml.in
               ${GTD_PYTHON_BINARY_DIR}/pyproject.toml)
configure_file(${GTDYNAMICS_PYTHON_PATH}/templates/${PROJECT_NAME}.tpl
               ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl)
file(COPY ${GTDYNAMICS_PYTHON_PATH}/${PROJECT_NAME}
     DESTINATION ${GTD_PYTHON_BINARY_DIR})

set(interface_headers
    ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}.i
    ${ADDITIONAL_INTERFACE_FILES}
)

pybind_wrap(
  ${PROJECT_NAME}_py # target
  "${interface_headers}" # interface header file
  "${PROJECT_NAME}.cpp" # the generated cpp
  "${PROJECT_NAME}" # module_name
  "${PROJECT_NAME}" # top namespace in the cpp file
  "${ignore}" # ignore classes
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl
  ${PROJECT_NAME} # libs
  "${PROJECT_NAME}" # dependencies
  ${GTDYNAMICS_ENABLE_BOOST_SERIALIZATION} # use_boost_serialization
)

set_target_properties(
  ${PROJECT_NAME}_py
  PROPERTIES OUTPUT_NAME "${PROJECT_NAME}"
             LIBRARY_OUTPUT_DIRECTORY "${GTD_PYTHON_BINARY_DIR}/${PROJECT_NAME}"
             DEBUG_POSTFIX "" # Otherwise you will have a wrong name
             RELWITHDEBINFO_POSTFIX "" # Otherwise you will have a wrong name
)

add_custom_target(
  python-install
  COMMAND ${PYTHON_EXECUTABLE} -m pip install .
  DEPENDS ${PROJECT_NAME}_py
  WORKING_DIRECTORY ${GTD_PYTHON_BINARY_DIR})

if(UNIX)
  set(GTD_PATH_SEP ":")
else()
  set(GTD_PATH_SEP ";")
endif()

# Unit tests
set(python_unit_test_suites)
macro(PYTHON_UNIT_TEST_SUITE suiteName directory)
  add_custom_target(
    python-test.${suiteName}
    COMMAND
      ${CMAKE_COMMAND} -E env # add package to python path so no need to install
      "PYTHONPATH=${GTD_PYTHON_BINARY_DIR}${GTD_PATH_SEP}$ENV{PYTHONPATH}"
      ${PYTHON_EXECUTABLE} -m unittest discover -v -s .
    DEPENDS ${PROJECT_NAME}_py
    WORKING_DIRECTORY ${directory})
  list(APPEND python_unit_test_suites python-test.${suiteName})
endmacro()

PYTHON_UNIT_TEST_SUITE(base ${PROJECT_SOURCE_DIR}/python/tests)

if(GTDYNAMICS_BUILD_CABLE_ROBOT)
  PYTHON_UNIT_TEST_SUITE(cablerobot ${PROJECT_SOURCE_DIR}/gtdynamics/cablerobot/src)
endif()
if(GTDYNAMICS_BUILD_JUMPING_ROBOT)
  PYTHON_UNIT_TEST_SUITE(jumpingrobot ${PROJECT_SOURCE_DIR}/gtdynamics/jumpingrobot/src)
endif()

# Custom make command to run all Python tests
add_custom_target(
  python-test
  DEPENDS ${PROJECT_NAME}_py ${python_unit_test_suites})
