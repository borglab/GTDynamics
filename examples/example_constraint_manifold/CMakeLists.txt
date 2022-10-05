cmake_minimum_required(VERSION 3.0)
project(example_constraint_manifold C CXX)

# Build Executable


# Connecte Poses.
set(CONNECTED_POSES ${PROJECT_NAME}_connected_poses)
add_executable(${CONNECTED_POSES} main_connected_poses.cpp)
target_link_libraries(${CONNECTED_POSES} PUBLIC gtdynamics)
target_include_directories(${CONNECTED_POSES} PUBLIC ${CMAKE_PREFIX_PATH}/include)

add_custom_target(
  ${CONNECTED_POSES}.run
  COMMAND ./${CONNECTED_POSES}
  DEPENDS ${CONNECTED_POSES}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/examples/${PROJECT_NAME})


# Range Constraint.
set(RANGE_CONSTRAINT ${PROJECT_NAME}_range_constraint)
add_executable(${RANGE_CONSTRAINT} main_range_constraint.cpp)
target_link_libraries(${RANGE_CONSTRAINT} PUBLIC gtdynamics)
target_include_directories(${RANGE_CONSTRAINT} PUBLIC ${CMAKE_PREFIX_PATH}/include)

add_custom_target(
  ${RANGE_CONSTRAINT}.run
  COMMAND ./${RANGE_CONSTRAINT}
  DEPENDS ${RANGE_CONSTRAINT}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/examples/${PROJECT_NAME})


# Arm Planning.
set(ARM ${PROJECT_NAME}_arm)
add_executable(${ARM} main_arm_kinematic_planning.cpp)
target_link_libraries(${ARM} PUBLIC gtdynamics)
target_include_directories(${ARM} PUBLIC ${CMAKE_PREFIX_PATH}/include)

add_custom_target(
  ${ARM}.run
  COMMAND ./${ARM}
  DEPENDS ${ARM}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/examples/${PROJECT_NAME})

# Arm Planning.
set(CABLE_ROBOT ${PROJECT_NAME}_cable_robot)
add_executable(${CABLE_ROBOT} main_cablerobot.cpp)
target_link_libraries(${CABLE_ROBOT} PUBLIC gtdynamics)
target_include_directories(${CABLE_ROBOT} PUBLIC ${CMAKE_PREFIX_PATH}/include)

add_custom_target(
  ${CABLE_ROBOT}.run
  COMMAND ./${CABLE_ROBOT}
  DEPENDS ${CABLE_ROBOT}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/examples/${PROJECT_NAME})

# Cart pole planning.
set(CART_POLE ${PROJECT_NAME}_cart_pole)
add_executable(${CART_POLE} main_cartpole.cpp CartPoleUtils.h CartPoleUtils.cpp)
target_link_libraries(${CART_POLE} PUBLIC gtdynamics)
target_include_directories(${CART_POLE} PUBLIC ${CMAKE_PREFIX_PATH}/include)

add_custom_target(
  ${CART_POLE}.run
  COMMAND ./${CART_POLE}
  DEPENDS ${CART_POLE}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/examples/${PROJECT_NAME})


# Quadruped planning.
set(QUADRUPED_MP ${PROJECT_NAME}_quadruped_mp)
add_executable(${QUADRUPED_MP} main_quadruped.cpp QuadrupedUtils.h QuadrupedUtils.cpp)
target_link_libraries(${QUADRUPED_MP} PUBLIC gtdynamics)
target_include_directories(${QUADRUPED_MP} PUBLIC ${CMAKE_PREFIX_PATH}/include)

add_custom_target(
  ${QUADRUPED_MP}.run
  COMMAND ./${QUADRUPED_MP}
  DEPENDS ${QUADRUPED_MP}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/examples/${PROJECT_NAME})

