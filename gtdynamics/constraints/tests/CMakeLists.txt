include_directories(${CMAKE_SOURCE_DIR}/tests)

gtsamAddTestsGlob(constraints "test*.cpp" "" "gtdynamics")
