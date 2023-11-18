# All subdirectories that contain source code relevant to this library.
set(SOURCE_SUBDIRS universal_robot utils factors optimizer kinematics statics dynamics manifold imanifold scenarios)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include cable robot stuff
if(GTDYNAMICS_BUILD_CABLE_ROBOT)
  add_subdirectory(cablerobot)
  set(ADDITIONAL_INTERFACE_FILES ${ADDITIONAL_INTERFACE_FILES} PARENT_SCOPE)
endif()

# Include jumping robot stuff
if(GTDYNAMICS_BUILD_JUMPING_ROBOT)
  add_subdirectory(jumpingrobot)
  set(ADDITIONAL_INTERFACE_FILES ${ADDITIONAL_INTERFACE_FILES} PARENT_SCOPE)
endif()

# Include panda robot stuff
if(GTDYNAMICS_BUILD_PANDA_ROBOT)
  add_subdirectory(pandarobot)
  set(ADDITIONAL_INTERFACE_FILES ${ADDITIONAL_INTERFACE_FILES} PARENT_SCOPE)
endif()

# Add this directory as include.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Include sdformat directories.
include_directories(${SDFormat_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS})

## Glob all source/header files in SOURCE_SUBDIRS.
file(GLOB headers "*.h")
file(GLOB sources "*.cpp")

include(GNUInstallDirs)
foreach(SOURCE_SUBDIR ${SOURCE_SUBDIRS})
  file(GLOB GLOB_RESULT
       ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_SUBDIR}/*.cpp)
  list(APPEND sources ${GLOB_RESULT})
  file(GLOB GLOB_RESULT
       ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_SUBDIR}/*.h)
  list(APPEND headers ${GLOB_RESULT})
  # Install header file to INSTALL_INCUDEDIR/gtdynamics/subdirectory.
  install(FILES ${GLOB_RESULT} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/${SOURCE_SUBDIR})

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

find_package(SuiteSparse_config REQUIRED)
find_package(AMD REQUIRED)
find_package(COLAMD REQUIRED)
find_package(CHOLMOD REQUIRED)
find_package(CAMD REQUIRED)
find_package(CCOLAMD REQUIRED)
find_package(SPQR REQUIRED)

set_target_properties(gtdynamics PROPERTIES LINKER_LANGUAGE CXX)

## Link all dependencies
# list(APPEND SUITESPARSE_LIBRARIES ${AMD_LIBRARY})
# list(APPEND SUITESPARSE_LIBRARIES ${CAMD_LIBRARY})
# list(APPEND SUITESPARSE_LIBRARIES ${COLAMD_LIBRARY})
# list(APPEND SUITESPARSE_LIBRARIES ${CCOLAMD_LIBRARY})
# list(APPEND SUITESPARSE_LIBRARIES ${CHOLMOD_LIBRARY})
# list(APPEND SUITESPARSE_LIBRARIES ${SUITESPARSE_CONFIG_LIBRARY})
# list(APPEND SUITESPARSE_LIBRARIES ${SPQR_LIBRARY})
# target_link_libraries (gtdynamics PUBLIC ${SUITESPARSE_LIBRARIES})

target_link_libraries (gtdynamics PUBLIC SuiteSparse::SPQR SuiteSparse::CHOLMOD)
target_link_libraries(gtdynamics PUBLIC ${GTSAM_LIBS}) 
target_link_libraries(gtdynamics PUBLIC ${SDFormat_LIBRARIES})
target_link_libraries(gtdynamics PUBLIC ${GTDYNAMICS_ADDITIONAL_LIBRARIES})


## Include headers needed

# Add TBB headers
if(GTDYNAMICS_WITH_TBB)
  target_include_directories(gtdynamics PUBLIC ${TBB_INCLUDE_DIRS})
endif()

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
# target_include_directories(gtdynamics PUBLIC ${SuiteSparse_INCLUDE_DIRS})

# message(STATUS "SPQR_INCLUDE_DIRS: ${SPQR_INCLUDE_DIRS}")
# message(STATUS "SUITESPARSEQR_INCLUDE_DIRS: ${SUITESPARSEQR_INCLUDE_DIRS}")
# message(STATUS "SPQR_LIBRARY_DIRS: ${SPQR_LIBRARY_DIRS}")
# message(STATUS "SUITESPARSEQR_LIBRARY_DIRS: ${SUITESPARSEQR_LIBRARY_DIRS}")

## Install library and headers.
install(
  TARGETS gtdynamics
  EXPORT "${PROJECT_NAME}-exports"
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
