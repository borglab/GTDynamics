find_package(gtwrap)

set(WRAP_PYTHON_VERSION
    "Default"
    CACHE STRING "The Python version to use for wrapping")
gtwrap_get_python_version(${WRAP_PYTHON_VERSION})
message(STATUS "Wrapper Python Version: ${WRAP_PYTHON_VERSION}")

set(PYTHON_REQUIREMENTS_PATH "requirements.txt")
set(PYTHON_TESTS "${PROJECT_NAME}.tests")

set(GTDYNAMICS_PYTHON_PATH ${PROJECT_SOURCE_DIR}/python/)

configure_file(${GTDYNAMICS_PYTHON_PATH}/templates/setup.py.in
               ${PROJECT_BINARY_DIR}/python/setup.py)
configure_file(${GTDYNAMICS_PYTHON_PATH}/templates/__init__.py.in
               ${PROJECT_BINARY_DIR}/python/${PROJECT_NAME}/__init__.py)
configure_file(${GTDYNAMICS_PYTHON_PATH}/requirements.txt
               ${PROJECT_BINARY_DIR}/python/requirements.txt COPYONLY)
configure_file(${GTDYNAMICS_PYTHON_PATH}/templates/${PROJECT_NAME}.tpl
               ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl)

pybind_wrap(
  ${PROJECT_NAME}_py # target
  ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}.i # interface header file
  "${PROJECT_NAME}.cpp" # the generated cpp
  "${PROJECT_NAME}" # module_name
  "${PROJECT_NAME}" # top namespace in the cpp file
  "${ignore}" # ignore classes
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl
  ${PROJECT_NAME} # libs
  "${PROJECT_NAME}" # dependencies
  ON # use boost
)

set_target_properties(
  ${PROJECT_NAME}_py
  PROPERTIES OUTPUT_NAME "${PROJECT_NAME}"
             LIBRARY_OUTPUT_DIRECTORY
             "${PROJECT_BINARY_DIR}/python/${PROJECT_NAME}"
             DEBUG_POSTFIX "" # Otherwise you will have a wrong name
             RELWITHDEBINFO_POSTFIX "" # Otherwise you will have a wrong name
)

add_custom_target(
  python-install
  COMMAND ${PYTHON_EXECUTABLE} ${PROJECT_BINARY_DIR}/python/setup.py install
  DEPENDS ${PROJECT_NAME}_py
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/python)

# Custom make command to run all Python tests
add_custom_target(
  python-test
  COMMAND ${PYTHON_EXECUTABLE} -m unittest discover
  DEPENDS ${PROJECT_NAME}_py
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/python/tests)
