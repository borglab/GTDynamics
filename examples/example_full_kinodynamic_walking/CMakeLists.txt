cmake_minimum_required(VERSION 3.0)
project(gtdynamics_demo C CXX)

## Find Dependencies

# For parsing urdf/sdf files.
find_package(sdformat8 REQUIRED)

# GTSAM.
find_package(GTSAM REQUIRED)

# Find GTDynamics.
find_library(GTDYNAMICS NAMES gtdynamics)

## Build Executable

add_executable(exec main2.cpp)
target_link_libraries(exec PUBLIC ${GTDYNAMICS} gtsam ${SDFormat_LIBRARIES})
target_include_directories(exec PUBLIC ${GTSAM_INCLUDE_DIR} ${SDFormat_INCLUDE_DIRS} ${CMAKE_PREFIX_PATH}/include)