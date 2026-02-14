include_directories(${CMAKE_SOURCE_DIR}/tests)

gtsamAddTestsGlob(tests "test*.cpp" "" "gtdynamics")
