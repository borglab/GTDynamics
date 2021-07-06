cmake_minimum_required(VERSION 3.0)
enable_testing()
project(gtdynamics
        LANGUAGES CXX C
        VERSION 1.0.0
        DESCRIPTION "Full kinodynamics constraints for arbitrary robot configurations with factor graphs."
        HOMEPAGE_URL "https://github.com/borglab/GTDynamics")

add_compile_options(-faligned-new)

# Enforce c++11 standards
add_compile_options(-std=c++11) # CMake 3.1 and earlier
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ##############################################################################
# Find Dependencies #
# ##############################################################################

# Find GTSAM components.
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBS gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# For unit tests and scripts.
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost is automatically added when we add GTSAM above.
# This ensures both GTSAM and GTDynamics have the same Boost version.
include_directories(${Boost_INCLUDE_DIR})

if(NOT (${Boost_VERSION} LESS 105600))
  message(
    "Ignoring Boost restriction on optional lvalue assignment from rvalues")
  add_definitions(-DBOOST_OPTIONAL_ALLOW_BINDING_TO_RVALUES
                  -DBOOST_OPTIONAL_CONFIG_ALLOW_BINDING_TO_RVALUES)
endif()

# For Python wrapper.
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# For parsing urdf/sdf files.
# Exact version required so that all tests pass.
find_package(sdformat8 8.9.1 EXACT REQUIRED)

# ##############################################################################
# Dynamics library
# ##############################################################################
option(GTDYNAMICS_BUILD_CABLE_ROBOT "Build Cable Robot" OFF)
option(GTDYNAMICS_BUILD_JUMPING_ROBOT "Build Jumping Robot" OFF)

add_subdirectory(gtdynamics)

option(GTDYNAMICS_BUILD_PYTHON "Build Python wrapper" ON)
message(STATUS "Build Python Wrapper: ${GTDYNAMICS_BUILD_PYTHON}")

if(GTDYNAMICS_BUILD_PYTHON)
  add_subdirectory(python)
endif()

# Process subdirectories.
add_subdirectory(tests)

option(GTDYNAMICS_BUILD_SCRIPTS "Build all scripts" OFF)
if(GTDYNAMICS_BUILD_SCRIPTS)
  add_subdirectory(scripts)
endif()

option(GTDYNAMICS_BUILD_EXAMPLES "Build all examples" OFF)
if(GTDYNAMICS_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

message(STATUS "===============================================================")
message(STATUS "================  Configuration Options  ======================")
message(STATUS "Project                                     : ${PROJECT_NAME}")
message(STATUS "CMAKE_CXX_COMPILER_ID type                  : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMAKE_CXX_COMPILER_VERSION                  : ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "GTSAM Version                               : ${GTSAM_VERSION}")
message(STATUS "Boost Version                               : ${Boost_VERSION}")
message(STATUS "Sdformat Version                            : ${sdformat8_VERSION}")
message(STATUS "Build Python                                : ${GTDYNAMICS_BUILD_PYTHON}")
if(GTDYNAMICS_BUILD_PYTHON)
  message(STATUS "Python Version                              : ${WRAP_PYTHON_VERSION}")
endif()
message(STATUS "Build Scripts                               : ${GTDYNAMICS_BUILD_SCRIPTS}")
message(STATUS "Build Examples                              : ${GTDYNAMICS_BUILD_EXAMPLES}")
message(STATUS "===============================================================")
