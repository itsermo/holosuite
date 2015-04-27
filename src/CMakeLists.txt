ADD_SUBDIRECTORY(holosuite-lib)
ADD_SUBDIRECTORY(holosuite)

IF(WITH_UNITY_PLUGIN)
	ADD_SUBDIRECTORY(holosuite-lib-unity)
	ADD_SUBDIRECTORY(holosuite-lib-unity-test)
ENDIF()