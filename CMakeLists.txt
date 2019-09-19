cmake_minimum_required(VERSION 2.6)
enable_testing()
project(dynamics CXX C)

#add_compile_options(
#	-faligned-new
#    )

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