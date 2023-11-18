cmake_minimum_required(VERSION 3.0)
enable_testing()
project(gtdynamics
        LANGUAGES CXX C
        VERSION 1.0.0
        DESCRIPTION "Full kinodynamics constraints for arbitrary robot configurations with factor graphs.")

add_compile_options(-faligned-new)

# Enforce c++17 standards
add_compile_options(-std=c++17) # CMake 3.1 and earlier
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

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

# # Boost is automatically added when we add GTSAM above.
# # This ensures both GTSAM and GTDynamics have the same Boost version.
# include_directories(${Boost_INCLUDE_DIR})

if(NOT (${Boost_VERSION} LESS 105600))
  message(
    "Ignoring Boost restriction on optional lvalue assignment from rvalues")
  add_definitions(-DBOOST_OPTIONAL_ALLOW_BINDING_TO_RVALUES
                  -DBOOST_OPTIONAL_CONFIG_ALLOW_BINDING_TO_RVALUES)
endif()

# For Python wrapper.
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# For parsing urdf/sdf files.
set(SDFormat_VERSION 12)
find_package(sdformat${SDFormat_VERSION} REQUIRED)

# ##############################################################################
# Dynamics library
# ##############################################################################
option(GTDYNAMICS_WITH_TBB                       "Use Intel Threaded Building Blocks (TBB) if available" OFF)
option(GTDYNAMICS_BUILD_CABLE_ROBOT "Build Cable Robot" OFF)
option(GTDYNAMICS_BUILD_JUMPING_ROBOT "Build Jumping Robot" OFF)
option(GTDYNAMICS_BUILD_PANDA_ROBOT "Build Panda Robot" OFF)

# Enable or disable serialization with GTDYNAMICSENABLE_BOOST_SERIALIZATION
option(GTDYNAMICS_ENABLE_BOOST_SERIALIZATION "Enable Boost serialization" ON)
if(GTDYNAMICS_ENABLE_BOOST_SERIALIZATION)
  add_compile_definitions(GTSAM_ENABLE_BOOST_SERIALIZATION)
  add_compile_definitions(GTDYNAMICS_ENABLE_BOOST_SERIALIZATION)
endif()

include(cmake/HandleTBB.cmake)              # TBB

add_subdirectory(gtdynamics)

option(GTDYNAMICS_BUILD_PYTHON "Build Python wrapper" ON)
message(STATUS "Build Python Wrapper: ${GTDYNAMICS_BUILD_PYTHON}")

if(GTDYNAMICS_BUILD_PYTHON)
  add_subdirectory(python)
endif()

# Process subdirectories.
add_subdirectory(tests)

option(GTDYNAMICS_BUILD_SCRIPTS "Build all scripts" ON)
if(GTDYNAMICS_BUILD_SCRIPTS)
  add_subdirectory(scripts)
endif()

option(GTDYNAMICS_BUILD_EXAMPLES "Build all examples" ON)
if(GTDYNAMICS_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

message(STATUS "===============================================================")
message(STATUS "================  Configuration Options  ======================")
message(STATUS "Project                                     : ${PROJECT_NAME}")
message(STATUS "Build Type                                  : ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_CXX_COMPILER_ID type                  : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMAKE_CXX_COMPILER_VERSION                  : ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "GTSAM Version                               : ${GTSAM_VERSION}")
message(STATUS "Boost Version                               : ${Boost_VERSION}")
message(STATUS "SDFormat Version                            : ${sdformat${SDFormat_VERSION}_VERSION}")

if(TBB_FOUND)
  message(STATUS "Use Intel TBB                               : Yes (${TBB_VERSION})")
else()
message(STATUS "Use Intel TBB                               : NO")
endif(TBB_FOUND)

message(STATUS "Build Python                                : ${GTDYNAMICS_BUILD_PYTHON}")
if(GTDYNAMICS_BUILD_PYTHON)
  message(STATUS "Python Version                              : ${WRAP_PYTHON_VERSION}")
endif()
message(STATUS "Build march=native                          : ${GTSAM_BUILD_WITH_MARCH_NATIVE}")
message(STATUS "Build Scripts                               : ${GTDYNAMICS_BUILD_SCRIPTS}")
message(STATUS "Build Examples                              : ${GTDYNAMICS_BUILD_EXAMPLES}")
message(STATUS "Build Robots")
message(STATUS "  Cable Robot                               : ${GTDYNAMICS_BUILD_CABLE_ROBOT}")
message(STATUS "  Jumping Robot                             : ${GTDYNAMICS_BUILD_JUMPING_ROBOT}")
message(STATUS "  Panda Robot                               : ${GTDYNAMICS_BUILD_PANDA_ROBOT}")
message(STATUS "===============================================================")

# Create the export .cmake file.
# We should use `install(EXPORT) instead of export()` so it exports the install file.
install(EXPORT "${PROJECT_NAME}-exports"
  FILE gtdynamics-exports.cmake
  DESTINATION lib/cmake/gtdynamics
)

include(CMakePackageConfigHelpers)

# Set the variables to be used for the cmake config file.
if(WIN32 AND NOT CYGWIN)
  set(INSTALL_CMAKE_DIR CMake/${PROJECT_NAME})
else()
  set(INSTALL_CMAKE_DIR lib/cmake/${PROJECT_NAME})
endif()

# Configure the config file that is includes the exports
configure_package_config_file(
  ${CMAKE_SOURCE_DIR}/Config.cmake.in
  "${CMAKE_BINARY_DIR}/gtdynamicsConfig.cmake"
  INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}"
  INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})

# Generate the config file.
write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/gtdynamicsConfigVersion.cmake"
  VERSION ${gtdynamics_VERSION}
  COMPATIBILITY SameMajorVersion
)

# Install the config files.
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/gtdynamicsConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/gtdynamicsConfigVersion.cmake
  DESTINATION lib/cmake/gtdynamics)

# Include CPack *after* all flags. Needed for packaging the project.
include(CPack)
