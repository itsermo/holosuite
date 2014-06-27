# # Michael Aaron Safyan (michaelsafyan@gmail.com). Copyright (C) 2009. Simplified BSD License.
IF(!WIN32)
INCLUDE (FindPackageHandleStandardArgs)
FIND_PACKAGE(PkgConfig ${Log4Cxx_FIND_REQUIRED} ${Log4Cxx_FIND_QUIETLY})
IF (PKG_CONFIG_FOUND)
   SET(PKG_CONFIG_PATH_ENV_VAR $ENV{PKG_CONFIG_PATH})
   IF (NOT PKG_CONFIG_PATH_ENV_VAR)
         IF (Log4Cxx_FIND_REQUIRED)
         MESSAGE (FATAL_ERROR "Environment variable PKG_CONFIG_PATH not set. Setting this variable is required in order for pkg-config to locate installed software packages.")
        ENDIF (Log4Cxx_FIND_REQUIRED)
     ENDIF (NOT PKG_CONFIG_PATH_ENV_VAR)
     PKG_CHECK_MODULES (Log4Cxx liblog4cxx)
     IF (Log4Cxx_FOUND)
         SET(Log4Cxx_LIBRARY ${Log4Cxx_LIBRARIES})
         SET(Log4Cxx_INCLUDE_DIR ${Log4Cxx_INCLUDEDIR})
         SET(Log4Cxx_LIBRARY_DIR ${Log4Cxx_LIBRARY_DIRS})
         IF (NOT Log4Cxx_FIND_QUIETLY)
             MESSAGE(STATUS "    includedir: ${Log4Cxx_INCLUDE_DIR}")
             MESSAGE(STATUS "    librarydir: ${Log4Cxx_LIBRARY_DIR}")
         ENDIF (NOT Log4Cxx_FIND_QUIETLY)
     ENDIF(Log4Cxx_FOUND)
ENDIF (PKG_CONFIG_FOUND)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Log4Cxx DEFAULT_MSG Log4Cxx_LIBRARY Log4Cxx_INCLUDE_DIR)
ELSE()

# Locate the log4cxx logging library
# This module defines
# LOG4CXX_LIBRARY
# LOG4CXX_FOUND, if false, do not try to link to Ariba
# LOG4CXX_INCLUDE_DIR, where to find the headers
#
# Created by Mathias Gottschlag. This was influenced by the FindOpenAL.cmake
# module by Eric Wing.

#=============================================================================
# Copyright 2005-2009 Kitware, Inc.
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)

# If you use this, you shall include log4cxx files like
# #include <log4cxx/log4cxx.h>

FIND_PATH(LOG4CXX_INCLUDE_DIR log4cxx/log4cxx.h
  PATH_SUFFIXES include
  PATHS
  ~/Library/Frameworks
  /Library/Frameworks
  /usr/local
  /usr
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt
)

FIND_LIBRARY(LOG4CXX_LIBRARY_DEBUG
  NAMES log4cxxd
  PATH_SUFFIXES lib64 lib libs64 libs libs/Win32 libs/Win64
  PATHS
  ~/Library/Frameworks
  /Library/Frameworks
  /usr/local
  /usr
  /sw
  /opt/local
  /opt/csw
  /opt
)

FIND_LIBRARY(LOG4CXX_LIBRARY
  NAMES log4cxx
  PATH_SUFFIXES lib64 lib libs64 libs libs/Win32 libs/Win64
  PATHS
  ~/Library/Frameworks
  /Library/Frameworks
  /usr/local
  /usr
  /sw
  /opt/local
  /opt/csw
  /opt
)


# handle the QUIETLY and REQUIRED arguments and set LOG4CXX_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Log4cxx DEFAULT_MSG LOG4CXX_LIBRARY LOG4CXX_INCLUDE_DIR)


#SET(LOG4CXX_LIBRARY_DEBUG ${LOG4CXX_LIBRARY})


LIST (APPEND LOG4CXX_LIBRARIES optimized ${LOG4CXX_LIBRARY} debug ${LOG4CXX_LIBRARY_DEBUG})

MARK_AS_ADVANCED(LOG4CXX_LIBRARY_DEBUG LOG4CXX_LIBRARY LOG4CXX_INCLUDE_DIR)

ENDIF()
