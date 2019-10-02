cmake_minimum_required(VERSION 3.0)
enable_testing()

include(GtsamMatlabWrap)

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

# Wrap
# include(GtsamCythonWrap)
# include_directories(${GTSAM_EIGENCY_INSTALL_PATH})
# wrap_and_install_library("dynamics.h"
#                                 "from gtsam.gtsam cimport *" # extra import of gtsam/gtsam.pxd Cython header
#                                 ""
#                                 ""
#                                 ""
#                                 )
# #Optional: install_cython_scripts and install_cython_files. See GtsamCythonWrap.cmake.
wrap_and_install_library("dynamics.h" "dynamics" "" "")