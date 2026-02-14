include_directories(${CMAKE_SOURCE_DIR}/tests)

gtsamAddTestsGlob(mechanics "test*.cpp" "" "gtdynamics")
