# <Copyright Holder>. Copyright (C) <Copyright Year(s)>. <License>.

#
# Project Properties
#
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.8)
PROJECT (holosuite)
SET (APPLICATION_NAME "Holosuite")
SET (APPLICATION_CODENAME "${PROJECT_NAME}")
SET (APPLICATION_COPYRIGHT_YEARS "2014")
SET (APPLICATION_VERSION_MAJOR 1)
SET (APPLICATION_VERSION_MINOR 0)
SET (APPLICATION_VERSION_PATCH 0)
SET (APPLICATION_VERSION_TYPE SNAPSHOT)
SET (APPLICATION_VERSION_STRING "${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}.${APPLICATION_VERSION_PATCH}-${APPLICATION_VERSION_TYPE}")
SET (APPLICATION_VENDOR_ID "edu.mit.media.obmg")
SET (APPLICATION_VENDOR_NAME "Massachussetts Institute of Technology - Object-Based Media Group")
SET (APPLICATION_VENDOR_URL "obm.media.mit.edu")
SET (APPLICATION_ID "${APPLICATION_VENDOR_ID}.${PROJECT_NAME}")

macro(get_WIN32_WINNT version)
    if (WIN32 AND CMAKE_SYSTEM_VERSION)
        set(ver ${CMAKE_SYSTEM_VERSION})
        string(REPLACE "." "" ver ${ver})
        string(REGEX REPLACE "([0-9])" "0\\1" ver ${ver})

        set(${version} "0x${ver}")
    endif()
endmacro()

#Need to define windows version macro for boost sockets
if(WIN32)
	get_WIN32_WINNT(ver)
	add_definitions(-D_WIN32_WINNT=${ver})
endif()

#using c++ 11 standard for GCC, requires GCC 4.7 or greater to compile
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	list(APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	list(APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

# Get rid of CMP0020 warning
#
if(POLICY CMP0020)
cmake_policy(SET CMP0020 NEW) # remove if CMake >= 2.8.11 required
endif()

OPTION(WITH_AUDIO "Enables audio compression and sampling for video conferencing" OFF)
OPTION(WITH_STATIC_LOG4CXX "Defines LOG4CXX_STATIC for use with static LOG4CXX libraries" ${WIN32})
OPTION(BOOST_DISABLE_AUTOLINK "Defines BOOST_ALL_NO_LIB macro to turn off boost auto linking" ${WIN32})
OPTION(WITH_DSCP2 "Enables compiling DSCP holographic render algorithm for MK II" OFF)
OPTION(WITH_DSCP4 "Enables compiling DSCP holographic render algorithm for MK IV" ON)
OPTION(WITH_ZSPACE_API "Enables ZSpace API for OpenGL renderer" OFF)
OPTION(WITH_LEAPSDK "Enables Leap SDK for user input" OFF)
OPTION(WITH_STATIC_FREEGLUT "Enables FREEGLUT_STATIC preprocessor macro" ${WIN32})
#OPTION(WITH_ODE "Enables ODE Physics Engine support" OFF)
OPTION(WITH_ASSIMP "Enables 3D object asset import" OFF)
OPTION(WITH_TRACE_LOG "Enables trace log for FPS counting, could slow things down" OFF)
OPTION(WITH_UDT "Uses UDT instead of TCP for networking. Requires UDT library." OFF)
OPTION(WITH_STATIC_UDT "Uses static library of UDT" ${WIN32})
OPTION(WITH_STATIC_GLEW "Uses GLEW static library linking" ${WIN32})
OPTION(BUILD_HOLOSUITE_LIB_STATIC "Builds the library as static" ${WIN32})

IF(MSVC)
	OPTION(WITH_VS_FOLDERS "Organizes project source code into folders for Visual Studio" ON)
ENDIF(MSVC)

IF(MSVC)
	IF(WITH_VS_FOLDERS)
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)
	set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")
	ENDIF(WITH_VS_FOLDERS)
ENDIF(MSVC)

#
# Debugging Options
#
SET (CMAKE_VERBOSE_MAKEFILE 0) # Use 1 for debugging, 0 for release

#
# Project Output Paths
#
SET (MAINFOLDER ${PROJECT_SOURCE_DIR})
SET (EXECUTABLE_OUTPUT_PATH "${MAINFOLDER}/bin")
SET (LIBRARY_OUTPUT_PATH "${MAINFOLDER}/lib")
SET (SHADER_OUTPUT_PATH "${MAINFOLDER}/shaders")
SET (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})


#
# Project Search Paths
#
LIST (APPEND CMAKE_PREFIX_PATH "${MAINFOLDER}")
LIST (APPEND CMAKE_PREFIX_PATH "${MAINFOLDER}/tools")
LIST (APPEND CMAKE_PREFIX_PATH "${MAINFOLDER}/thirdparty")
SET (CMAKE_MODULE_PATH "${MAINFOLDER}/tools/share/cmake")
#INCLUDE_DIRECTORIES("${MAINFOLDER}/include")

#
# Locate Project Prerequisites 
#
FIND_PACKAGE (PCL 1.7.1 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})

SET (Boost_ADDITIONAL_VERSIONS "1.55" "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0" "1.47" "1.47.0" "1.48" "1.48.0" "1.49" "1.49.0" "1.50" "1.50.0")
FIND_PACKAGE (Boost 1.54 COMPONENTS "date_time" "filesystem" "graph" "iostreams" "program_options" "regex" "serialization" "signals" "system" "thread" "wserialization" "program_options" REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
IF(BOOST_DISABLE_AUTOLINK)
	ADD_DEFINITIONS(-DBOOST_ALL_NO_LIB)
ENDIF(BOOST_DISABLE_AUTOLINK)

FIND_PACKAGE (Log4Cxx REQUIRED)
INCLUDE_DIRECTORIES(${LIBLOG4CXX_INCLUDE_DIRS})
#GET_FILENAME_COMPONENT(LOG4CXX_LIB_DIR ${LOG4CXX_LIBRARY} PATH)
#LINK_DIRECTORIES(${LOG4CXX_LIB_DIR})
IF(WITH_STATIC_LOG4CXX)
	ADD_DEFINITIONS(-DLOG4CXX_STATIC)
ENDIF(WITH_STATIC_LOG4CXX)

IF(WITH_STATIC_FREEGLUT)
	ADD_DEFINITIONS(-DFREEGLUT_STATIC)
ENDIF()

FIND_PACKAGE (OpenNI2 REQUIRED)
INCLUDE_DIRECTORIES(${OPENNI2_INCLUDE_DIRS})
#GET_FILENAME_COMPONENT(OPENNI2_LIBRARY_DIR ${OPENNI2_LIBRARY} PATH)
LINK_DIRECTORIES(${OPENNI2_LIBRARY_DIR})

FIND_PACKAGE (OpenCV 2.4.8 REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIB_DIR})

FIND_PACKAGE(VTK REQUIRED)
INCLUDE_DIRECTORIES(${VTK_INCLUDE_DIRS})
LINK_DIRECTORIES(${VTK_LIBRARY_DIRS})

FIND_PACKAGE(FFMPEG REQUIRED)
INCLUDE_DIRECTORIES(${FFMPEG_INCLUDE_DIR})

FIND_PACKAGE(OpenGL REQUIRED)
INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})

FIND_PACKAGE(GLUT REQUIRED)
INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR})

FIND_PACKAGE(assimp REQUIRED)
INCLUDE_DIRECTORIES(${assimp_INCLUDE_DIRS})

FIND_PACKAGE(GLEW REQUIRED)
INCLUDE_DIRECTORIES(${GLEW_INCLUDE_DIR})

# Optional packages
FIND_PACKAGE(Opus QUIET)
FIND_PACKAGE(Portaudio 19 QUIET)
FIND_PACKAGE(Cg QUIET)
FIND_PACKAGE(ZSpace QUIET)
FIND_PACKAGE(LeapSDK QUIET)
FIND_PACKAGE(GLM QUIET)
FIND_PACKAGE(SDL2 QUIET)
FIND_PACKAGE(CUDA QUIET)
FIND_PACKAGE(OpenCL QUIET)
FIND_PACKAGE(X11 QUIET)
FIND_PACKAGE(PNG QUIET)
FIND_PACKAGE(DSCP4 QUIET)
FIND_PACKAGE(UDT QUIET)

IF(UNIX)
	FIND_PACKAGE(X11)
ENDIF()

IF(OPUS_FOUND AND PORTAUDIO_FOUND)
	SET(WITH_AUDIO YES)
ENDIF()

IF((OPENGL_FOUND AND GLUT_FOUND AND CG_FOUND) OR (UNIX AND X11_FOUND AND OPENGL_FOUND AND GLUT_FOUND AND CG_FOUND))
	SET(WITH_DSCP2 YES)
ENDIF()

IF(DSCP4_FOUND AND ((OPENGL_FOUND AND GLUT_FOUND) OR (UNIX AND X11_FOUND AND OPENGL_FOUND AND GLUT_FOUND)))
	SET(WITH_DSCP4 YES)
ENDIF()

IF(ZSPACE_FOUND)
	SET(WITH_ZSPACE_API YES)
ENDIF()

IF(LEAP_FOUND)
	SET(WITH_LEAPSDK YES)
ENDIF()

IF(assimp_FOUND)
	SET(WITH_ASSIMP YES)
ENDIF()

# enable trace logs for profiling and timing
IF(WITH_TRACE_LOG)
	ADD_DEFINITIONS(-DTRACE_LOG_ENABLED)
ENDIF(WITH_TRACE_LOG)

#
# Configure Files
#
FILE (GLOB_RECURSE CONFIGINPUTS1 include/*.in.h.cmake)
FILE (GLOB_RECURSE CONFIGINPUTS2 include/*.h.in.cmake)
FILE (GLOB_RECURSE CONFIGINPUTS3 src/*.in.h.cmake)
FILE (GLOB_RECURSE CONFIGINPUTS4 src/*.h.in.cmake)
FILE (GLOB_RECURSE CONFIGINPUTS5 test/*.in.h.cmake)
FILE (GLOB_RECURSE CONFIGINPUTS6 test/*.h.in.cmake)
LIST (APPEND CONFIGINPUTS "${CONFIGINPUTS1}")
LIST (APPEND CONFIGINPUTS "${CONFIGINPUTS2}")
LIST (APPEND CONFIGINPUTS "${CONFIGINPUTS3}")
LIST (APPEND CONFIGINPUTS "${CONFIGINPUTS4}")
LIST (APPEND CONFIGINPUTS "${CONFIGINPUTS5}")
LIST (APPEND CONFIGINPUTS "${CONFIGINPUTS6}")
FOREACH (CONFIGINPUT ${CONFIGINPUTS})
    STRING (REPLACE ".in.h.cmake" ".h" CONFIGOUTPUT1 "${CONFIGINPUT}")
    STRING (REPLACE ".h.in.cmake" ".h" CONFIGOUTPUT2 "${CONFIGOUTPUT1}")
    CONFIGURE_FILE ("${CONFIGINPUT}" "${CONFIGOUTPUT2}")
ENDFOREACH (CONFIGINPUT ${CONFIGINPUTS})

# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

#
# Add Build Targets
#
ADD_SUBDIRECTORY(src)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

#
# Add Install Targets
#
IF (EXISTS "${MAINFOLDER}/include/${PROJECT_NAME}" AND IS_DIRECTORY "${MAINFOLDER}/include/${PROJECT_NAME}")
    INSTALL(DIRECTORY "${MAINFOLDER}/include/${PROJECT_NAME}" DESTINATION "include")
ENDIF (EXISTS "${MAINFOLDER}/include/${PROJECT_NAME}" AND IS_DIRECTORY "${MAINFOLDER}/include/${PROJECT_NAME}")

#
# Add Documentation Targets
#
#INCLUDE (DocumentationTargets)

message(STATUS "")
message(STATUS "holosuite Build Configuration Status")
message(STATUS "====================================")
message(STATUS "")
message(STATUS "Audio")
message(STATUS "  Building audio subsystem:            ${WITH_AUDIO}")
message(STATUS "    Found Opus audio codec:            ${OPUS_FOUND}")
message(STATUS "    Found Portaudio library:           ${PORTAUDIO_FOUND}")

message(STATUS "")
message(STATUS "Rendering")
message(STATUS "  ZSpace 3D display support:           ${WITH_ZSPACE_API}")
message(STATUS "  DSCP2 support for MK II holovideo:   ${WITH_DSCP2}")
message(STATUS "    Found OpenGL:                      ${OPENGL_FOUND}")
message(STATUS "    Found GLUT:                        ${GLUT_FOUND}")
message(STATUS "    Found CG:                          ${CG_FOUND}")
if(UNIX)
  message(STATUS "    Found X11:                         ${X11_FOUND}")
endif()
message(STATUS "  DSCP4 support for MK IV holovideo:   ${WITH_DSCP4}")
message(STATUS "    Found OpenGL:                      ${OPENGL_FOUND}")
message(STATUS "    Found GLUT:                        ${GLUT_FOUND}")
message(STATUS "    Found CUDA:                        ${CUDA_FOUND}")
message(STATUS "    Found OpenCL:                      ${OPENCL_FOUND}")
message(STATUS "    Found PNG:                         ${PNG_FOUND}")
if(UNIX)
  message(STATUS "    Found X11:                         ${X11_FOUND}")
endif()
message(STATUS "  ASSIMP importer for 3D model files:  ${WITH_ASSIMP}")
message(STATUS "")

message(STATUS "Networking")
message(STATUS "  Use UDT instead of TCP (broken):     ${WITH_UDT}")
message(STATUS "")

message(STATUS "Input Device")
message(STATUS "  LeapSDK support:                     ${WITH_LEAPSDK}")
message(STATUS "")

message(STATUS "Build Options")
message(STATUS "  Build static lib:                    ${BUILD_HOLOSUITE_LIB_STATIC}")
message(STATUS "  Build type:                          ${CMAKE_BUILD_TYPE}")
message(STATUS "  Tracelog (FPS, performance logging): ${WITH_TRACE_LOG}")
message(STATUS "")

message(STATUS "Now just type:")
message(STATUS "")
message(STATUS "  make")
message(STATUS "  sudo make install")
message(STATUS "")
