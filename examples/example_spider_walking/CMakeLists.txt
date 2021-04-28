cmake_minimum_required(VERSION 3.0)
project(example_spider_walking C CXX)

# Build Executables

# Walk forward one meter.
set(WALK_FORWARD ${PROJECT_NAME}_forward)
add_executable(${WALK_FORWARD} main.cpp)
target_link_libraries(${WALK_FORWARD} PUBLIC gtdynamics)
target_include_directories(${WALK_FORWARD} PUBLIC ${CMAKE_PREFIX_PATH}/include)

add_custom_target(
  ${WALK_FORWARD}.run
  COMMAND ./${WALK_FORWARD}
  DEPENDS ${WALK_FORWARD}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/examples/${PROJECT_NAME})

# Rotate robot 90 degrees.
set(ROTATE ${PROJECT_NAME}_rotate)
add_executable(${ROTATE} main_rotate.cpp)
target_link_libraries(${ROTATE} PUBLIC gtdynamics)
target_include_directories(${ROTATE} PUBLIC ${CMAKE_PREFIX_PATH}/include)

add_custom_target(
  ${ROTATE}.run
  COMMAND ./${ROTATE}
  DEPENDS ${ROTATE}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/examples/${PROJECT_NAME})
