cmake_minimum_required(VERSION 3.0)
project(gtdynamics_demo C CXX)

## Find Dependencies

# For parsing urdf/sdf files.
find_package(sdformat8 REQUIRED)

# GTSAM.
find_package(GTSAM REQUIRED)

# Find GTDynamics.
find_library(GTDYNAMICS NAMES gtdynamics)

## Build Executables

# Walk forward one meter.
add_executable(exec_walk_forward main_walk_forward.cpp)
target_link_libraries(exec_walk_forward PUBLIC ${GTDYNAMICS} gtsam ${SDFormat_LIBRARIES})
target_include_directories(exec_walk_forward PUBLIC ${GTSAM_INCLUDE_DIR} ${SDFormat_INCLUDE_DIRS} ${CMAKE_PREFIX_PATH}/include)

# Rotate robot 90 degrees.
add_executable(exec_rotate main_rotate.cpp)
target_link_libraries(exec_rotate PUBLIC ${GTDYNAMICS} gtsam ${SDFormat_LIBRARIES})
target_include_directories(exec_rotate PUBLIC ${GTSAM_INCLUDE_DIR} ${SDFormat_INCLUDE_DIRS} ${CMAKE_PREFIX_PATH}/include)
