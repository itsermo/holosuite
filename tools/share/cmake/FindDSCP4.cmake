# - Find dscp4 library
# Find the native libdscp4 headers and libraries.
#
#  DSCP4_INCLUDE_DIRS   - where to find dscp4.h, dscp4_defs.h
#  DSCP4_LIBRARIES      - List of libraries when using libdscp4
#  DSCP4_FOUND          - True if DSCP4 is found.

find_package(PNG QUIET)
find_package(OpenGL QUIET)
find_package(GLEW QUIET)
find_package(GLM QUIET)
find_package(SDL2 QUIET)
find_package(CUDA QUIET)
find_package(OpenCL QUIET)
find_package(Log4Cxx QUIET)

# Look for the dscp4.h header file.
find_path( DSCP4_INCLUDE_DIR
  NAMES dscp4.h
  PATHS /usr/include /usr/local/include
  PATH_SUFFIXES dscp4
  DOC "DSCP4 include directory" )
mark_as_advanced( DSCP4_INCLUDE_DIR )

# Look for the DSCP4 library.
find_library( DSCP4_LIBRARY
  NAMES libdscp4.so
  PATH_SUFFIXES lib64 lib libs64 libs libs/Win32 libs/Win64
  PATHS /usr /usr/local
  DOC "Path to DSCP4 library" )
mark_as_advanced( DSCP4_LIBRARY )

find_library(DSCP4_LIBRARY_DEBUG
  NAMES libdscp4d
  PATH_SUFFIXES lib64 lib libs64 libs libs/Win32 libs/Win64
  PATHS /usr /usr/local
  DOC "Path to DSCP4 library" )
mark_as_advanced( DSCP4_LIBRARY_DEBUG )

# handle the QUIETLY and REQUIRED arguments and set OPUSFILE_FOUND to TRUE if 
# all listed variables are TRUE
include( ${CMAKE_ROOT}/Modules/FindPackageHandleStandardArgs.cmake )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( DSCP4 DEFAULT_MSG DSCP4_LIBRARY DSCP4_INCLUDE_DIR )

list(APPEND DSCP4_LIBRARIES ${LIBLOG4CXX_LIBRARIES} ${PNG_LIBRARIES} ${ZLIB_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_gl_LIBRARY} ${GLEW_LIBRARY} ${SDL2_LIBRARY} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${OPENCL_LIBRARY} ${OPENCL_LIBRARIES} )

if(UNIX)
	list (APPEND DSCP4_LIBRARIES ${X11_LIBRARIES})
endif()

if(DSCP4_LIBRARY_DEBUG)
	LIST (APPEND DSCP4_LIBS optimized ${DSCP4_LIBRARY} debug ${DSCP4_LIBRARY_DEBUG})
	LIST (APPEND DSCP4_LIBRARIES ${DSCP4_LIBS})
else()
	LIST(APPEND DSCP4_LIBRARIES ${DSCP4_LIBRARY})
endif()

set(DSCP4_INCLUDE_DIRS ${DSCP4_INCLUDE_DIR})
