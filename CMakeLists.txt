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

# For cython wrapper.
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# For parsing urdf/sdf files.
find_package(sdformat8 REQUIRED)

# ##############################################################################
# Dynamics library
# ##############################################################################

# Paths to SDF & URDF files.
add_definitions(-DURDF_PATH="${PROJECT_SOURCE_DIR}/urdfs")
add_definitions(-DSDF_PATH="${PROJECT_SOURCE_DIR}/sdfs")

# All subdirectories that contain source code relevant to this library.
set(SOURCE_SUBDIRS dynamics universal_robot factors utils)

# Option to include cable robot stuff
option(GTDYNAMICS_BUILD_CABLE_ROBOT "Build Cable Robot" OFF)
if(GTDYNAMICS_BUILD_CABLE_ROBOT)
  list(APPEND SOURCE_SUBDIRS cablerobot)
endif()

# Add this directory as include.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Include sdformat directories.
include_directories(${SDFormat_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS})

# Glob all source/header files in SOURCE_SUBDIRS.
file(GLOB sources "*.cpp")
file(GLOB headers "*.h")

foreach(SOURCE_SUBDIR ${SOURCE_SUBDIRS})
  file(GLOB GLOB_RESULT
       ${CMAKE_CURRENT_SOURCE_DIR}/gtdynamics/${SOURCE_SUBDIR}/*.cpp)
  list(APPEND sources ${GLOB_RESULT})
  file(GLOB GLOB_RESULT
       ${CMAKE_CURRENT_SOURCE_DIR}/gtdynamics/${SOURCE_SUBDIR}/*.h)
  list(APPEND headers ${GLOB_RESULT})
  install(FILES ${GLOB_RESULT} DESTINATION include/gtdynamics/${SOURCE_SUBDIR})
  add_subdirectory(gtdynamics/${SOURCE_SUBDIR}) # Process subdirectory.
endforeach()

add_library(gtdynamics SHARED ${sources} ${headers})

target_link_libraries(gtdynamics ${GTSAM_LIBS} ${SDFormat_LIBRARIES})
target_include_directories(gtdynamics PUBLIC ${SDFormat_INCLUDE_DIRS})

foreach(SOURCE_SUBDIR ${SOURCE_SUBDIRS})
  target_include_directories(
    gtdynamics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_SUBDIR})
endforeach()

# Install library and headers.
install(
  TARGETS gtdynamics
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
set_target_properties(gtdynamics PROPERTIES LINKER_LANGUAGE CXX)

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
