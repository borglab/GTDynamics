find_package(Boost 1.75 CONFIG REQUIRED)
add_executable(g1_full_dynamics full_dynamics.cpp)
target_link_libraries(g1_full_dynamics PRIVATE gtdynamics)
target_include_directories(g1_full_dynamics PRIVATE ${Boost_INCLUDE_DIRS})
target_compile_definitions(g1_full_dynamics PRIVATE
  GTDYNAMICS_BENCHMARK_BUILD_TYPE="$<CONFIG>"
  GTDYNAMICS_BENCHMARK_COMPILER="${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_options(g1_full_dynamics PRIVATE "LINKER:--disable-new-dtags")
endif()
add_test(NAME g1_full_dynamics_factors COMMAND g1_full_dynamics --self-test)
