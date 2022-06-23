cmake_minimum_required(VERSION 3.0)
project(example_a1_walking C CXX)

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
