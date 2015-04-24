IF(WITH_DSCP4)
	option(WITH_CUDA "Enables CUDA compute building" ${CUDA_FOUND})
	option(WITH_LIBLOG4CXX "Uses Log4Cxx library for logging status and error messages" ${LIBLOG4CXX_FOUND})
	option(WITH_OPENCL "Enables OpenCL compute building" ${OPENCL_FOUND})
	option(WITH_PNG "Enables screenshot functionality (saving render buffer to PNG)" ${PNG_FOUND})
	option(BUILD_DSCP4_STATIC "Builds DSCP4 as a static lib" ${WIN32})
	
	ADD_SUBDIRECTORY(libdscp4)
ENDIF()

ADD_SUBDIRECTORY(holosuite-lib)
ADD_SUBDIRECTORY(holosuite)

IF(WITH_UNITY_PLUGIN)
	ADD_SUBDIRECTORY(holosuite-lib-unity)
	ADD_SUBDIRECTORY(holosuite-lib-unity-test)
ENDIF()