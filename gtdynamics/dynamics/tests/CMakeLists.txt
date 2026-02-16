include_directories(${CMAKE_SOURCE_DIR}/tests)

gtsamAddTestsGlob(dynamics "test*.cpp" "" "gtdynamics")
