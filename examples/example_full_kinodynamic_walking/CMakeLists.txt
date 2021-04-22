cmake_minimum_required(VERSION 3.0)
project(example_full_kinodynamic_walking C CXX)

# Build Executables

# Walk forward one meter.
add_executable(main_walk_forward main_walk_forward.cpp)
target_link_libraries(main_walk_forward PUBLIC gtdynamics)
target_include_directories(main_walk_forward
                           PUBLIC ${CMAKE_PREFIX_PATH}/include)

add_custom_target(
  main_walk_forward.run
  COMMAND ./main_walk_forward
  DEPENDS main_walk_forward
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/examples/${PROJECT_NAME})

# Rotate robot 90 degrees.
add_executable(main_rotate main_rotate.cpp)
target_link_libraries(main_rotate PUBLIC gtdynamics)
target_include_directories(main_rotate PUBLIC ${CMAKE_PREFIX_PATH}/include)

add_custom_target(
    main_rotate.run
  COMMAND ./main_rotate
  DEPENDS main_rotate
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/examples/${PROJECT_NAME})
