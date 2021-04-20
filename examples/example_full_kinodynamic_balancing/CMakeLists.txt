cmake_minimum_required(VERSION 3.0)
project(example_full_kinodynamic_balancing C CXX)

# Find Dependencies

# For parsing urdf/sdf files.
find_package(sdformat8 REQUIRED)

# GTSAM.
find_package(GTSAM REQUIRED)

# Find GTDynamics.
find_library(GTDYNAMICS NAMES gtdynamics)
find_path(GTDYNAMICS_INCLUDE_DIR NAMES gtdynamics)

# Build Executable
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${GTDYNAMICS} gtsam
                                             ${SDFormat_LIBRARIES})
target_include_directories(
  ${PROJECT_NAME} PUBLIC ${GTDYNAMICS_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR}
                         ${SDFormat_INCLUDE_DIRS} ${CMAKE_PREFIX_PATH}/include)

add_custom_target(
  ${PROJECT_NAME}.run
  COMMAND ./${PROJECT_NAME}
  DEPENDS ${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/examples/${PROJECT_NAME})
