include_directories(${CMAKE_SOURCE_DIR}/tests)

gtsamAddTestsGlob(factors "test*.cpp" "" "gtdynamics")
