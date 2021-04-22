cmake_minimum_required(VERSION 3.0)
project(example_spider_walking C CXX)

## Build Executables

# Walk forward one meter.
add_executable(exec_walk_forward main.cpp)
target_link_libraries(exec_walk_forward PUBLIC gtdynamics)
target_include_directories(exec_walk_forward PUBLIC $${CMAKE_PREFIX_PATH}/include)

# Rotate robot 90 degrees.
#add_executable(exec_rotate main_rotate.cpp)
#target_link_libraries(exec_rotate PUBLIC ${GTDYNAMICS} gtsam ${SDFormat_LIBRARIES})
#target_include_directories(exec_rotate PUBLIC ${GTSAM_INCLUDE_DIR} ${SDFormat_INCLUDE_DIRS} ${CMAKE_PREFIX_PATH}/include)
