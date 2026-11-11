# The benchmark is opt-in at runtime, but available in an examples build.
add_executable(admm_vs_proxqp admm_vs_proxqp.cpp)
target_link_libraries(admm_vs_proxqp PRIVATE gtdynamics)
