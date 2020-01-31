cmake_minimum_required(VERSION 3.0)
enable_testing()
project(dynamics CXX C)

add_compile_options(
	-faligned-new
)

#####################################
# Find dependent libraries
#####################################

# Find GTSAM components.
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBS gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)
include(GtsamMatlabWrap)

# For unit tests and scripts.
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost - same requirement as gtsam.
find_package(Boost 1.43 REQUIRED)
find_package(Boost COMPONENTS filesystem system thread program_options serialization regex timer REQUIRED)

include_directories(${Boost_INCLUDE_DIR})

if(NOT (${Boost_VERSION} LESS 105600))
	message("Ignoring Boost restriction on optional lvalue assignment from rvalues")
	add_definitions(-DBOOST_OPTIONAL_ALLOW_BINDING_TO_RVALUES -DBOOST_OPTIONAL_CONFIG_ALLOW_BINDING_TO_RVALUES)
endif()

# For cython wrapper.
find_package(GTSAM REQUIRED) # Make sure gtsam's install folder is in your PATH
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# For parsing urdf/sdf files.
find_package(sdformat8 EXACT REQUIRED)

#####################################
# Dynamics library
#####################################

# Paths to SDF & URDF files.
add_definitions( -DURDF_PATH="${PROJECT_SOURCE_DIR}/urdfs" )
add_definitions( -DSDF_PATH="${PROJECT_SOURCE_DIR}/sdfs" )

# All subdirectories that contain source code relevant to this library.
set(SOURCE_SUBDIRS kinodynamics universal_robot factors utils)

# Create dynamics library and link source and header files.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Include sdformat directories.
include_directories(${SDFormat_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS})

# Glob all source/header files in SOURCE_SUBDIRS.
file(GLOB sources "*.cpp" "*.h")

foreach(SOURCE_SUBDIR ${SOURCE_SUBDIRS})
    file(GLOB GLOB_RESULT
         ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_SUBDIR}/*.cpp
         ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_SUBDIR}/*.h)
	list(APPEND sources ${GLOB_RESULT})
	add_subdirectory(${SOURCE_SUBDIR}) # Process subdirectory.
endforeach()

add_library(dynamics SHARED ${sources})

target_link_libraries(dynamics ${GTSAM_LIBS} ${SDFormat_LIBRARIES})
target_include_directories(dynamics PUBLIC ${SDFormat_INCLUDE_DIRS})

foreach(SOURCE_SUBDIR ${SOURCE_SUBDIRS})
    target_include_directories(dynamics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_SUBDIR})
endforeach()

# Install library.
install(TARGETS dynamics LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
set_target_properties(dynamics PROPERTIES LINKER_LANGUAGE CXX)

# Process subdirectories.
add_subdirectory(tests)
add_subdirectory(scripts)