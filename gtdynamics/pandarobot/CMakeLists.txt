# add cablerobot subfolders to gtdynamics' SOURCE_SUBDIRS list
list(APPEND SOURCE_SUBDIRS pandarobot/ikfast)
set(SOURCE_SUBDIRS ${SOURCE_SUBDIRS} PARENT_SCOPE)

# add wrapper interface file
# list(APPEND ADDITIONAL_INTERFACE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/pandarobot.i)
# set(ADDITIONAL_INTERFACE_FILES ${ADDITIONAL_INTERFACE_FILES} PARENT_SCOPE)

add_subdirectory(tests)
