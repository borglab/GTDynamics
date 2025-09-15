# CMake file to make compiling all the examples easier.

#find_package(GTSAM REQUIRED)

# Variable to hold all the example directories
set(EXAMPLE_SUBDIRS
    scripts
    example_a1_walking
    example_cart_pole_trajectory_optimization
    example_forward_dynamics
    example_full_kinodynamic_balancing
    example_full_kinodynamic_walking
    example_inverted_pendulum_trajectory_optimization
    # example_jumping_robot  # Python based example
    example_quadruped_mp
    example_spider_walking
    example_constraint_manifold
  )

# Add each example subdirectory for compilation
foreach(EXAMPLE ${EXAMPLE_SUBDIRS})
  add_subdirectory(${EXAMPLE})
endforeach()

# --- Process Python Animation Scripts ---
# This automatically finds 'animate.py' scripts and creates .animate targets.
message(STATUS "Searching for python animation scripts...")
foreach(EXAMPLE_NAME ${EXAMPLE_SUBDIRS})
  set(ANIMATION_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/${EXAMPLE_NAME}/animate.py")

  if(EXISTS ${ANIMATION_SCRIPT})
    message(STATUS "  -- Found animation script for ${EXAMPLE_NAME}, creating '.animate' target")
    add_custom_target(${EXAMPLE_NAME}.animate
      COMMAND
      python3 ${ANIMATION_SCRIPT}
      WORKING_DIRECTORY
      "${CMAKE_CURRENT_SOURCE_DIR}/${EXAMPLE_NAME}"
      COMMENT
      "Running animation for ${EXAMPLE_NAME}..."
      VERBATIM
    )
  endif()
endforeach()