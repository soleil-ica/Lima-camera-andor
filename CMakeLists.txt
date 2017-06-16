# CMakeLists.txt for Andor camera

IF(UNIX)
  SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--add-needed")
ENDIF()


SET(andor_srcs src/AndorCamera.cpp  src/AndorInterface.cpp
	 src/AndorDetInfoCtrlObj.cpp src/AndorSyncCtrlObj.cpp
	 src/AndorBinCtrlObj.cpp src/AndorShutterCtrlObj.cpp
	 src/AndorRoiCtrlObj.cpp)
	 
FILE(STRINGS "VERSION" andor_vers)

ADD_LIBRARY(limaandor SHARED ${andor_srcs})

#INCLUDES
TARGET_INCLUDE_DIRECTORIES(limaandor PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
IF(WIN32)
	TARGET_INCLUDE_DIRECTORIES(limaandor PUBLIC "C:/Program files/Andor SDK")
	TARGET_INCLUDE_DIRECTORIES(limaandor PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/sdk/msvc/include")
ELSE()
	TARGET_INCLUDE_DIRECTORIES(limaandor PUBLIC "${COMMON_PATH}/include")
ENDIF()


IF(WIN32)
	TARGET_COMPILE_DEFINITIONS(limaandor PRIVATE LIBANDOR_EXPORTS)
ENDIF()

#LIBRARY
IF(WIN32)
	FIND_LIBRARY(ANDOR_LIB atmcd64m "C:/Program files/Andor SDK")
	IF (("${ANDOR_LIB}" STREQUAL "ANDOR_LIB-NOTFOUND"))
	  MESSAGE(FATAL_ERROR "andor : libatmcd32d not found, you can search it in cmake-gui or try to install it if it isn't here.")
	ENDIF()
ELSE()
	FIND_LIBRARY(ANDOR_LIB andor)
	IF (("${ANDOR_LIB}" STREQUAL "ANDOR_LIB-NOTFOUND"))
	  MESSAGE(FATAL_ERROR "andor : libandor not found, you can search it in cmake-gui or try to install it if it isn't here.")
	ENDIF()
ENDIF()

TARGET_LINK_LIBRARIES(limaandor limacore)
TARGET_LINK_LIBRARIES(limaandor ${ANDOR_LIB})

SET_TARGET_PROPERTIES(limaandor PROPERTIES VERSION ${andor_vers} SOVERSION "1")
IF(WIN32)
	SET_TARGET_PROPERTIES(limaandor PROPERTIES PREFIX "lib")
	INSTALL(TARGETS limaandor DESTINATION lib)
ELSE()
	INSTALL(TARGETS limaandor LIBRARY DESTINATION lib)
ENDIF()

IF(COMPILE_SIP)
    SET(NAME "andor")
	include(CameraRunSIP)
    INSTALL(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Andor")
ENDIF()
