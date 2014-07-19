
#set(CMAKE_AUTOMOC ON)

# Glob all sources file inside directory ${CMAKE_CURRENT_SOURCE_DIR}
file(GLOB_RECURSE SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/*.h
	${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/*.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
	${CMAKE_CURRENT_SOURCE_DIR}/*.C
	${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)

foreach(f ${SRC_FILES})
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

ADD_EXECUTABLE(${holosuite_BIN} ${SRC_FILES})

#qt5_use_modules(${project_BIN} Widgets)

TARGET_LINK_LIBRARIES(${holosuite_BIN} ${holosuite_LIBS} ${LIBLOG4CXX_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${OPENNI2_LIBRARY} ${FFMPEG_LIBRARIES} ${OPUS_LIBRARIES} ${PORTAUDIO_LIBRARIES})
SET_TARGET_PROPERTIES(${holosuite_BIN} PROPERTIES VERSION "${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}" OUTPUT_NAME ${holosuite_BIN} CLEAN_DIRECT_OUTPUT 1)

INSTALL(TARGETS ${holosuite_BIN} DESTINATION bin)
