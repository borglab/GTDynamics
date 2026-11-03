find_package(Boost 1.75 CONFIG REQUIRED)
add_executable(g1_full_dynamics full_dynamics.cpp)
target_link_libraries(g1_full_dynamics PRIVATE gtdynamics)
target_include_directories(g1_full_dynamics PRIVATE ${Boost_INCLUDE_DIRS})
target_compile_definitions(g1_full_dynamics PRIVATE
  GTDYNAMICS_BENCHMARK_BUILD_TYPE="$<CONFIG>"
  GTDYNAMICS_BENCHMARK_COMPILER="${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_options(g1_full_dynamics PRIVATE "LINKER:--disable-new-dtags")
endif()
add_test(NAME g1_full_dynamics_factors COMMAND g1_full_dynamics --self-test)

if(GTDYNAMICS_BUILD_PYTHON)
  find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
  pybind11_add_module(_g1_kinodynamics MODULE kinodynamics_native.cpp)
  target_link_libraries(_g1_kinodynamics PRIVATE gtdynamics)
  set_property(TARGET _g1_kinodynamics PROPERTY NO_SYSTEM_FROM_IMPORTED ON)
  target_include_directories(_g1_kinodynamics BEFORE PRIVATE
    ${pybind11_INCLUDE_DIR} ${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR})
  set_target_properties(_g1_kinodynamics PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}" DEBUG_POSTFIX "")
  add_dependencies(_g1_kinodynamics gtdynamics_py)
  if(WIN32)
    set(g1_path_sep "\;")
  else()
    set(g1_path_sep ":")
  endif()
  add_test(NAME g1_kinodynamics_python
    COMMAND ${CMAKE_COMMAND} -E env
      "PYTHONPATH=$<TARGET_FILE_DIR:_g1_kinodynamics>${g1_path_sep}${PROJECT_BINARY_DIR}/python${g1_path_sep}$ENV{PYTHONPATH}"
      ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_kinodynamics.py)
endif()

gtsamAddTestsGlob(g1_kinodynamics "testKinodynamicGauge.cpp" "" "gtdynamics")
