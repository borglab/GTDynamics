cmake_minimum_required(VERSION 3.16)

# Also usable on its own with installed GTDynamics and GTSAM.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(motion_planning_webdemo_backend LANGUAGES CXX C)
  set(CMAKE_CXX_STANDARD 17)
  include(CMakeFindDependencyMacro)
  find_package(GTSAM REQUIRED)
  find_package(gtdynamics REQUIRED)
  enable_testing()
endif()

# Boost.JSON is built once in optimizer.cpp, so no extra JSON library is linked.
find_package(Boost 1.75 CONFIG REQUIRED)
add_executable(motion_planning_optimizer optimizer.cpp)
target_link_libraries(motion_planning_optimizer PRIVATE gtdynamics)
target_include_directories(motion_planning_optimizer PRIVATE ${Boost_INCLUDE_DIRS})
target_compile_definitions(motion_planning_optimizer PRIVATE
  MOTION_PLANNING_PANDA_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../../../models/urdfs/panda/panda.urdf")

add_executable(motion_planning_factor_test test_factors.cpp)
target_link_libraries(motion_planning_factor_test PRIVATE gtdynamics)
# An installed GTSAM can have transitive shared dependencies without its own
# RUNPATH. For these local executables, retain CMake's runtime directories as
# inherited RPATH on Linux so their dependencies resolve without shell setup.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_options(motion_planning_optimizer PRIVATE "LINKER:--disable-new-dtags")
  target_link_options(motion_planning_factor_test PRIVATE "LINKER:--disable-new-dtags")
endif()
add_test(NAME motion_planning_factor_jacobians COMMAND motion_planning_factor_test)
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
  add_test(NAME motion_planning_native_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_optimizer.py
            $<TARGET_FILE:motion_planning_optimizer>)
endif()
