cmake_minimum_required(VERSION 3.0)
enable_testing()
project(gtdynamics
        LANGUAGES CXX C
        VERSION 1.0.0
        DESCRIPTION "Full kinodynamics constraints for arbitrary robot configurations with factor graphs."
        HOMEPAGE_URL "https://github.com/borglab/GTDynamics")

add_compile_options(-faligned-new)
set (CMAKE_CXX_STANDARD 11)

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

# Boost - same requirement as gtsam.
find_package(Boost 1.58 REQUIRED)
find_package(
  Boost
  COMPONENTS filesystem
             system
             thread
             program_options
             serialization
             regex
             timer
  REQUIRED)

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
find_package(sdformat8 REQUIRED)

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
message(STATUS "===============================================================")
