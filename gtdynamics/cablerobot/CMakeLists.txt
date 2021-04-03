message("THIS IS A DEBUG MESSAGE")

# All subdirectories that contain source code relevant to this library.
# set(CABLE_SOURCE_SUBDIRS factors robot)
set(CABLE_SOURCE_SUBDIRS factors utils)

# Add this directory as include.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Glob all source/header files in CABLE_SOURCE_SUBDIRS.
file(GLOB cable_sources "*.cpp")
file(GLOB cable_headers "*.h")

foreach(SOURCE_SUBDIR ${CABLE_SOURCE_SUBDIRS})
  file(GLOB GLOB_RESULT
       ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_SUBDIR}/*.cpp)
  list(APPEND cable_sources ${GLOB_RESULT})
  file(GLOB GLOB_RESULT
       ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_SUBDIR}/*.h)
  list(APPEND cable_headers ${GLOB_RESULT})
  install(FILES ${GLOB_RESULT} DESTINATION include/${SOURCE_SUBDIR})
  add_subdirectory(${SOURCE_SUBDIR}) # Process subdirectory.
endforeach()

list(APPEND headers ${cable_headers})
list(APPEND sources ${cable_sources})

add_subdirectory(tests)
