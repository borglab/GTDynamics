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

# Plots of the replay.json written by scate_online.
add_custom_target(
  scate_online.plot
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/plot_scate.py replay.json
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Plotting replay.json from scate_online"
  VERBATIM)
