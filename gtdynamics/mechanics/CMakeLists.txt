# This directory currently contributes source files via top-level globbing.

if(GTSAM_BUILD_TESTS AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
  add_subdirectory(tests)
endif()
