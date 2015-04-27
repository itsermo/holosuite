# - Find dscp4 library
# Find the native libdscp4 headers and libraries.
#
#  DSCP4_INCLUDE_DIRS   - where to find dscp4.h, dscp4_defs.h
#  DSCP4_LIBRARIES      - List of libraries when using libdscp4
#  DSCP4_FOUND          - True if DSCP4 is found.


# Look for the dscp4.h header file.
find_path( DSCP4_INCLUDE_DIR
  NAMES dscp4.h
  DOC "DSCP4 include directory" )
mark_as_advanced( DSCP4_INCLUDE_DIR )

# Look for the DSCP4 library.
find_library( DSCP4_LIBRARY
  NAMES libdscp4
  DOC "Path to DSCP4 library" )
mark_as_advanced( DSCP4_LIBRARY )

find_library( DSCP4_LIBRARY_DEBUG
  NAMES libdscp4d
  DOC "Path to DSCP4 library" )
mark_as_advanced( DSCP4_LIBRARY_DEBUG )

# handle the QUIETLY and REQUIRED arguments and set OPUSFILE_FOUND to TRUE if 
# all listed variables are TRUE
include( ${CMAKE_ROOT}/Modules/FindPackageHandleStandardArgs.cmake )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( DSCP4 DEFAULT_MSG DSCP4_LIBRARY DSCP4_INCLUDE_DIR )

if(DSCP4_LIBRARY_DEBUG)
	LIST (APPEND DSCP4_LIBRARIES optimized ${DSCP4_LIBRARY} debug ${DSCP4_LIBRARY_DEBUG})
else()
	set(DSCP4_LIBRARIES ${DSCP4_LIBRARY} )
endif()

set( DSCP4_INCLUDE_DIRS ${DSCP4_INCLUDE_DIR} )
