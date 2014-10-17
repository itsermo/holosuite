
#set(CMAKE_AUTOMOC ON)

# Glob all sources file inside directory ${CMAKE_CURRENT_SOURCE_DIR}
file(GLOB_RECURSE HOLOSUITE_SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/*.h
	${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/*.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
	${CMAKE_CURRENT_SOURCE_DIR}/*.C
	${CMAKE_CURRENT_SOURCE_DIR}/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/*.cg
	${CMAKE_CURRENT_SOURCE_DIR}/*.cl
)

foreach(f ${HOLOSUITE_SRC_FILES})
    # Get the path of the file relative to ${DIRECTORY},
    # then alter it (not compulsory)
    file(RELATIVE_PATH SRCGR ${CMAKE_CURRENT_SOURCE_DIR} ${f})
    #set(SRCGR "${SRCGR}")

    # Extract the folder, ie remove the filename part
    string(REGEX REPLACE "(.*)(/[^/]*)$" "\\1" SRCGR ${SRCGR})

    # Source_group expects \\ (double antislash), not / (slash)
    string(REPLACE / \\ SRCGR ${SRCGR})
    source_group("${SRCGR}" FILES ${f})
endforeach()

SET (holosuite_BIN ${PROJECT_NAME})

#ADD_LIBRARY(holosuite-lib ${LIB_SRC_FILES})
ADD_EXECUTABLE(${holosuite_BIN} ${HOLOSUITE_SRC_FILES})

#qt5_use_modules(${project_BIN} Widgets)

TARGET_LINK_LIBRARIES(${holosuite_BIN} holosuite-lib ${LIBLOG4CXX_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${OPENNI2_LIBRARY} ${FFMPEG_LIBRARIES} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})

IF(UNIX)
	TARGET_LINK_LIBRARIES(${holosuite_BIN} ${GLUT_glut_LIBRARY})
ENDIF()

IF(WITH_AUDIO)
	TARGET_LINK_LIBRARIES(${holosuite_BIN} ${OPUS_LIBRARIES} ${PORTAUDIO_LIBRARIES})
ENDIF()

IF(WITH_DSCP2)
	TARGET_LINK_LIBRARIES(${holosuite_BIN} ${CG_LIBRARY} ${CG_GL_LIBRARY})
	IF(UNIX)
		TARGET_LINK_LIBRARIES(${holosuite_BIN} ${X11_LIBRARIES})
	ENDIF()
ENDIF()

IF(WITH_ZSPACE_API)
	TARGET_LINK_LIBRARIES(${holosuite_BIN} ${ZSPACE_API_LIBRARY})
ENDIF()

SET_TARGET_PROPERTIES(${holosuite_BIN} PROPERTIES VERSION "${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}" OUTPUT_NAME ${holosuite_BIN} CLEAN_DIRECT_OUTPUT 1)

# file(GLOB SHADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/holorender/shaders/*.cg
					   # ${CMAKE_CURRENT_SOURCE_DIR}/holorender/shaders/*.cl
# )

# # Copy shader files
# add_custom_target(copy-shader-files)
# get_target_property(MyTargetLocation holosuite-lib LOCATION)
# get_filename_component(MyTargetDir ${MyTargetLocation} PATH)

# foreach(SHADER_FILE ${SHADER_FILES})
  # add_custom_command(TARGET copy-shader-files PRE_BUILD
                     # COMMAND ${CMAKE_COMMAND} -E
                         # copy_if_different ${SHADER_FILE} ${MyTargetDir})
  # IF(WIN32)
    # add_custom_command(TARGET copy-shader-files PRE_BUILD
                     # COMMAND ${CMAKE_COMMAND} -E
                         # copy_if_different ${SHADER_FILE} ${CMAKE_BINARY_DIR}/src)
  # ENDIF()
# endforeach()
# add_dependencies(holosuite-lib copy-shader-files)

INSTALL(TARGETS ${holosuite_BIN} DESTINATION bin)
