cmake_minimum_required(VERSION 3.0)
project(example_quadruped_mp C CXX)

# Build Executable
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC gtdynamics)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_PREFIX_PATH}/include)

add_custom_target(
  ${PROJECT_NAME}.run
  COMMAND ./${PROJECT_NAME}
  DEPENDS ${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/examples/${PROJECT_NAME})
