# - Find ZSpace library
# Just finds the zspace api library
#
# ZSPACE_API_LIBRARY - The lib file
# ZSPACE_API_INCLUDE_DIR - The include dir
# ZSPACE_API_FOUND - True if library is found, false if not

# Look for the opusfile header file.
find_path( ZSPACE_API_INCLUDE_DIR
  NAMES Inc/zSpace.h
  DOC "ZSpace API include directory" )
mark_as_advanced( ZSPACE_API_INCLUDE_DIR )

find_library( ZSPACE_API_LIBRARY
  NAMES zSpaceApi zSpaceApi64
  DOC "Path to ZSpace API library" )
mark_as_advanced( ZSPACE_API_LIBRARY )

include( ${CMAKE_ROOT}/Modules/FindPackageHandleStandardArgs.cmake )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( ZSpace DEFAULT_MSG ZSPACE_API_LIBRARY ZSPACE_API_INCLUDE_DIR )