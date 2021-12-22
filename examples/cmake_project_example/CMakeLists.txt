cmake_minimum_required(VERSION 3.0)
project(example C CXX)

# gtsam
find_package(GTSAM REQUIRED)

# gtdynamics
find_path(gtdynamics_include gtdynamics)
find_library(gtdynamics_lib gtdynamics)
# these next 3 lines are temporary until we create a proper cmake module for gtdynamics.
set(SDFormat_VERSION 10)
find_package(sdformat${SDFormat_VERSION} REQUIRED)
list(APPEND gtdynamics_lib ${SDFormat_LIBRARIES})

# Build Executable
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${gtdynamics_lib} gtsam)
target_include_directories(${PROJECT_NAME} PUBLIC ${gtdynamics_include})

# Convenience cmake command to build and run in one command with `make -j8 example.run`.
add_custom_target(
  ${PROJECT_NAME}.run
  COMMAND ./${PROJECT_NAME}
  DEPENDS ${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
