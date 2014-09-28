
#set(CMAKE_AUTOMOC ON)

set(HOLOSUITE_LIB_UNITY_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Glob all sources file inside directory ${CMAKE_CURRENT_SOURCE_DIR}
file(GLOB_RECURSE UNITY_LIB_SRC_FILES
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

foreach(f ${UNITY_LIB_SRC_FILES})
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

#SET (holosuite_BIN ${PROJECT_NAME})

ADD_LIBRARY(holosuite-lib-unity SHARED ${UNITY_LIB_SRC_FILES})

TARGET_INCLUDE_DIRECTORIES (holosuite-lib-unity PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

TARGET_LINK_LIBRARIES(holosuite-lib-unity holosuite-lib ${LIBLOG4CXX_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${OPENNI2_LIBRARY} ${FFMPEG_LIBRARIES})

IF(WITH_AUDIO)
	TARGET_LINK_LIBRARIES(holosuite-lib-unity ${OPUS_LIBRARIES} ${PORTAUDIO_LIBRARIES})
ENDIF()

IF(WITH_DSCP2)
	TARGET_LINK_LIBRARIES(holosuite-lib-unity ${CG_LIBRARY} ${CG_GL_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
	IF(UNIX)
		TARGET_LINK_LIBRARIES(holosuite-lib-unity ${X11_LIBRARIES} ${GLUT_glut_LIBRARY})
	ENDIF()
ENDIF()

SET_TARGET_PROPERTIES(holosuite-lib-unity PROPERTIES VERSION "${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}" OUTPUT_NAME holosuite-unity CLEAN_DIRECT_OUTPUT 1)