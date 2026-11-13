cmake_minimum_required(VERSION 3.9...3.29)
project(example_scate_ur10e C CXX)

# SCATE on the UR10e: the offline plan (scate_ur10e_init), the closed loop
# with an obstacle discovered online (scate_ur10e_online), reactive SCATE with
# VICON sensing and moving obstacles (scate_ur10e_reactive), and the study of
# the base-disturbance cost (scate_ur10e_base_wrench). Only the UR10e's
# robot file and collision spheres live here; the planner, online loop and
# scenario are the header-only SCATE code in example_scate_ur5.
set(SCATE_UR5_DIR ${CMAKE_SOURCE_DIR}/examples/example_scate_ur5)
foreach(program scate_ur10e_init scate_ur10e_online scate_ur10e_reactive
                scate_ur10e_base_wrench)
  add_executable(${program} ${program}.cpp)
  target_link_libraries(${program} PUBLIC gtdynamics)
  target_include_directories(${program} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${SCATE_UR5_DIR})

  add_custom_target(
    ${program}.run
    COMMAND ./${program}
    DEPENDS ${program}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endforeach()

# Plots and a standalone 3D replay page of the replay.json written by
# scate_ur10e_online. The viewer and plot script are shared by the SCATE apps.
set(SCATE_APPS_DIR ${CMAKE_SOURCE_DIR}/apps/scate)
add_custom_target(
  scate_ur10e_online.plot
  COMMAND python3 ${SCATE_APPS_DIR}/shared/scripts/plot_scate.py replay.json --app ur10e
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Plotting replay.json from scate_ur10e_online"
  VERBATIM)

# Plots and a standalone 3D replay page of the reactive run (moving obstacles),
# in reactive_plots/.
add_custom_target(
  scate_ur10e_reactive.plot
  COMMAND python3 ${SCATE_APPS_DIR}/shared/scripts/plot_scate.py reactive_replay.json --app ur10e
          --out reactive_plots
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Plotting reactive_replay.json from scate_ur10e_reactive"
  VERBATIM)

# Copy this build's replay.json into the app as its hosted recording.
add_custom_target(
  scate_ur10e_online.export
  COMMAND ${CMAKE_COMMAND} -E copy replay.json ${SCATE_APPS_DIR}/ur10e/public/replay.json
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Copying replay.json to apps/scate/ur10e/public"
  VERBATIM)

# Copy the base-disturbance comparison (cost off, sigma 3, 0.3, 0.1) written by
# scate_ur10e_base_wrench into the SCATE UR10e Base app as its recording.
add_custom_target(
  scate_ur10e_base_wrench.export
  COMMAND ${CMAKE_COMMAND} -E copy base_wrench_replay.json
          ${SCATE_APPS_DIR}/ur10e-reactive/public/replay.json
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Copying base_wrench_replay.json to apps/scate/ur10e-reactive/public"
  VERBATIM)

# The live simulation's native solver (live_scenario.h). The SCATE UR10e Base
# app's dev server runs `scate_live solve`; scate_live.describe writes the robot
# descriptions (link frames, screw axes, spheres, default experiment) that the
# app uses to drag the arms.
add_executable(scate_live scate_live.cpp)
target_link_libraries(scate_live PUBLIC gtdynamics)
target_include_directories(scate_live PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${SCATE_UR5_DIR})
add_custom_target(
  scate_live.describe
  COMMAND ./scate_live describe ur10e ${SCATE_APPS_DIR}/ur10e-reactive/public/robots/ur10e.json
  COMMAND ./scate_live describe ur5 ${SCATE_APPS_DIR}/ur10e-reactive/public/robots/ur5.json
  DEPENDS scate_live
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Writing the live app's robot descriptions to apps/scate/ur10e-reactive/public/robots"
  VERBATIM)
