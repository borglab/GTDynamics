cmake_minimum_required(VERSION 2.6)
enable_testing()
project(dynamics CXX C)

#####################################
# Find dependent libraries
#####################################

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBS gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost - same requirement as gtsam
find_package(Boost 1.43 REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)
find_package(Boost COMPONENTS regex REQUIRED)
find_package(Boost COMPONENTS timer REQUIRED)

include_directories(${Boost_INCLUDE_DIR})

if(NOT (${Boost_VERSION} LESS 105600))
	message("Ignoring Boost restriction on optional lvalue assignment from rvalues")
	add_definitions(-DBOOST_OPTIONAL_ALLOW_BINDING_TO_RVALUES -DBOOST_OPTIONAL_CONFIG_ALLOW_BINDING_TO_RVALUES)
endif()

# CppUnitLite for tests
#find_package(CppUnitLite REQUIRED)

#####################################
# Process subdirs
#####################################

add_subdirectory(cpp)

# for cython wrapper
find_package(GTSAM REQUIRED) # Make sure gtsam's install folder is in your PATH
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Wrap
include(GtsamCythonWrap)
include_directories(${GTSAM_EIGENCY_INSTALL_PATH})
wrap_and_install_library_cython("dynamics.h"
                                "from gtsam.gtsam cimport *" # extra import of gtsam/gtsam.pxd Cython header
                                "your_install_path"
                                "libraries_to_link_with_the_cython_module"
                                "dependencies_which_need_to_be_built_before_the_wrapper"
                                )
#Optional: install_cython_scripts and install_cython_files. See GtsamCythonWrap.cmake.