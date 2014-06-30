FIND_PACKAGE (UnitTestPlusPlus REQUIRED)

INCLUDE_DIRECTORIES(${UnitTestPlusPlus_INCLUDE_DIRS})
LINK_DIRECTORIES(${UnitTestPlusPlus_LIBRARY_DIRS})

FILE (GLOB_RECURSE test_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
SET (test_LIBS ${Log4Cxx_LIBRARIES} ${UnitTestPlusPlus_LIBRARIES} ${holosuite_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OPENNI2_LIBRARY} ${FFMPEG_LIBRARIES})
SET (test_BIN ${PROJECT_NAME}-unittests)

ADD_EXECUTABLE(${test_BIN} ${test_SRCS})
TARGET_LINK_LIBRARIES(${test_BIN} ${test_LIBS})

ADD_CUSTOM_TARGET(check ALL "${MAINFOLDER}/bin/${test_BIN}" DEPENDS ${test_BIN} COMMENT "Executing unit tests..." VERBATIM SOURCES ${test_SRCS})
ADD_CUSTOM_TARGET(test "${MAINFOLDER}/bin/${test_BIN}" DEPENDS ${test_BIN} COMMENT "Executing unit tests..." VERBATIM SOURCES ${test_SRCS})
