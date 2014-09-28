add_subdirectory(holosuite-lib)
add_subdirectory(holosuite)

IF(WITH_UNITY_PLUGIN)
add_subdirectory(holosuite-lib-unity)
ENDIF()