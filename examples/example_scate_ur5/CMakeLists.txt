cmake_minimum_required(VERSION 3.9...3.29)
project(example_scate_ur5 C CXX)

# SCATE on the UR5: the offline plan (scate_init) and the closed loop with an
# obstacle discovered online (scate_online). The SCATE headers live in this
# folder; tests/ includes them from here as well.
foreach(program scate_init scate_online)
  add_executable(${program} ${program}.cpp)
  target_link_libraries(${program} PUBLIC gtdynamics)
  target_include_directories(${program} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  add_custom_target(
    ${program}.run
    COMMAND ./${program}
    DEPENDS ${program}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endforeach()

# Plots and a standalone 3D replay page of the replay.json written by
# scate_online. The viewer and plot script are shared by the SCATE apps.
set(SCATE_APPS_DIR ${CMAKE_SOURCE_DIR}/apps/scate)
add_custom_target(
  scate_online.plot
  COMMAND python3 ${SCATE_APPS_DIR}/shared/scripts/plot_scate.py replay.json --app ur5
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Plotting replay.json from scate_online"
  VERBATIM)

# Copy this build's replay.json into the app as its hosted recording.
add_custom_target(
  scate_online.export
  COMMAND ${CMAKE_COMMAND} -E copy replay.json ${SCATE_APPS_DIR}/ur5/public/replay.json
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Copying replay.json to apps/scate/ur5/public"
  VERBATIM)
