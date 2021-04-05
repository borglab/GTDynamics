# All subdirectories that contain source code relevant to this library.
set(SOURCE_SUBDIRS dynamics factors universal_robot utils)

# Include cable robot stuff
if(GTDYNAMICS_BUILD_CABLE_ROBOT)
  list(APPEND SOURCE_SUBDIRS cablerobot)
  list(APPEND SOURCE_SUBDIRS cablerobot/factors)
endif()

# Add this directory as include.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Include sdformat directories.
include_directories(${SDFormat_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS})

## Glob all source/header files in SOURCE_SUBDIRS.
file(GLOB headers "*.h")
file(GLOB sources "*.cpp")

foreach(SOURCE_SUBDIR ${SOURCE_SUBDIRS})
  file(GLOB GLOB_RESULT
       ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_SUBDIR}/*.cpp)
  list(APPEND sources ${GLOB_RESULT})
  file(GLOB GLOB_RESULT
       ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_SUBDIR}/*.h)
  list(APPEND headers ${GLOB_RESULT})
  install(FILES ${GLOB_RESULT} DESTINATION include/${SOURCE_SUBDIR})

  # Process subdirectory.
  message(STATUS "Building ${SOURCE_SUBDIR}")
  add_subdirectory(${SOURCE_SUBDIR})
endforeach()

## Generate and install config file
configure_file(config.h.in config.h)
list(APPEND sources "${PROJECT_BINARY_DIR}/${PROJECT_NAME}/config.h")
install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}/config.h" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

## Generate library target
# Needs to be a SHARED library so that the wrapper can compile correctly
add_library(gtdynamics SHARED ${sources} ${headers})

set_target_properties(gtdynamics PROPERTIES LINKER_LANGUAGE CXX)

## Link all dependencies
target_link_libraries(gtdynamics ${GTSAM_LIBS} ${SDFormat_LIBRARIES})


## Include headers needed

# Add includes for source directories 'BEFORE' any other include
# paths so that the compiler uses GTDynamics headers in our source directory instead
# of any previously installed GTDynamics headers.
target_include_directories(gtdynamics BEFORE PUBLIC
  # main gtdynamics includes:
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/>
  # config.h
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
)

target_include_directories(gtdynamics PUBLIC ${SDFormat_INCLUDE_DIRS})


## Install library and headers.
install(
  TARGETS gtdynamics
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
