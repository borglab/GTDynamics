enable_testing()
include(GtsamMatlabWrap)

# All subdirectories that contain source code relevant to this library.
set(SOURCE_SUBDIRS trajectory_optimization universal_robot factors utils)

# Create dynamics library and link source and header files.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB sources "*.cpp" "*.h")

foreach(SOURCE_SUBDIR ${SOURCE_SUBDIRS})
    file(GLOB GLOB_RESULT
         ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_SUBDIR}/*.cpp
         ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_SUBDIR}/*.h)
    list(APPEND sources ${GLOB_RESULT})
endforeach()

add_library(dynamics SHARED ${sources})

target_link_libraries(dynamics ${GTSAM_LIBS} urdfdom_model urdfdom_world)

foreach(SOURCE_SUBDIR ${SOURCE_SUBDIRS})
    target_include_directories(dynamics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_SUBDIR})
endforeach()

# Install library.
install(TARGETS dynamics LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
set_target_properties(dynamics PROPERTIES LINKER_LANGUAGE CXX)

# Process subdirectories.
add_subdirectory(tests)
add_subdirectory(scripts)
add_subdirectory(factors)
add_subdirectory(trajectory_optimization)