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
    SET(INCLUDES)
    FILE(GLOB sipfiles RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/sip" "${CMAKE_CURRENT_SOURCE_DIR}/sip/*.sip")
    FOREACH(sipfile ${sipfiles})
        SET(INCLUDES 
        "${INCLUDES} 
        %Include ${sipfile}")
    ENDFOREACH()

    CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/sip/limamodules.sip.in sip/limaandor.sip)
    SET(SIP_CONCAT_PARTS 1)
    SET(SIP_INCLUDES ${SIP_INCLUDES}
        "${CMAKE_SOURCE_DIR}/third-party/Processlib/sip"
        "${CMAKE_BINARY_DIR}/sip/core"
        "${CMAKE_SOURCE_DIR}/third-party/Processlib/tasks/sip"
        "${CMAKE_SOURCE_DIR}/common/sip"
        "${CMAKE_SOURCE_DIR}/hardware/sip"
        "${CMAKE_SOURCE_DIR}/control/sip"
        "${CMAKE_SOURCE_DIR}/control/software_operation/sip"
        "${CMAKE_CURRENT_SOURCE_DIR}/sip")
    set(SIP_DISABLE_FEATURES WITH_CONFIG)
    set(SIP_CHECK_EXC ON)
    ADD_SIP_PYTHON_MODULE(limaandor ${CMAKE_CURRENT_BINARY_DIR}/sip/limaandor.sip)
    TARGET_INCLUDE_DIRECTORIES(python_module_limaandor PRIVATE
                               ${PYTHON_INCLUDE_DIRS}
                               "${CMAKE_SOURCE_DIR}/sip"
                               "${CMAKE_SOURCE_DIR}/sip/core")
    TARGET_LINK_LIBRARIES(python_module_limaandor limaandor)

    INSTALL(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Andor")
ENDIF()
