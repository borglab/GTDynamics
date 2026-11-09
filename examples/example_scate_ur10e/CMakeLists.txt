cmake_minimum_required(VERSION 3.9...3.29)
project(example_scate_ur10e C CXX)

# SCATE on the UR10e: the offline plan (scate_ur10e_init) and the closed loop
# with an obstacle discovered online (scate_ur10e_online). Only the UR10e's
# robot file and collision spheres live here; the planner, online loop and
# scenario are the header-only SCATE code in example_scate_ur5.
set(SCATE_UR5_DIR ${CMAKE_SOURCE_DIR}/examples/example_scate_ur5)
foreach(program scate_ur10e_init scate_ur10e_online scate_ur10e_reactive)
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
