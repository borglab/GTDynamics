# CMake file to make compiling all the examples easier.

# Variable to hold all the example directories
set(EXAMPLE_SUBDIRS
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
