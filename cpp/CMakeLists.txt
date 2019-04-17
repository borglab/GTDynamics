cmake_minimum_required(VERSION 2.6)
enable_testing()

# Library sources
file(GLOB sources "*.cpp" "*.h")
add_library(dynamics SHARED ${sources})
target_link_libraries(dynamics ${GTSAM_LIBS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Install library
install(TARGETS dynamics LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
set_target_properties(dynamics PROPERTIES LINKER_LANGUAGE CXX)

# Process subdirs
add_subdirectory(tests)
