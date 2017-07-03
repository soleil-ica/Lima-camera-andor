###########################################################################
# This file is part of LImA, a Library for Image Acquisition
#
#  Copyright (C) : 2009-2017
#  European Synchrotron Radiation Facility
#  CS40220 38043 Grenoble Cedex 9 
#  FRANCE
# 
#  Contact: lima@esrf.fr
# 
#  This is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or
#  (at your option) any later version.
# 
#  This software is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
set(NAME "andor")

if(UNIX)
  set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--add-needed")
endif()


set(${NAME}_srcs src/AndorCamera.cpp  src/AndorInterface.cpp
	 src/AndorDetInfoCtrlObj.cpp src/AndorSyncCtrlObj.cpp
	 src/AndorBinCtrlObj.cpp src/AndorShutterCtrlObj.cpp
	 src/AndorRoiCtrlObj.cpp)

add_library(lima${NAME} SHARED ${${NAME}_srcs})

#INCLUDES
target_include_directories(lima${NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
if(WIN32)
	target_include_directories(lima${NAME} PUBLIC "C:/Program files/Andor SDK")
	target_include_directories(lima${NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/sdk/msvc/include")
endif()


if(WIN32)
	target_compile_definitions(lima${NAME} PRIVATE LIBANDOR_EXPORTS)
endif()

#LIBRARY
if(WIN32)
	find_library(ANDOR_ATMCD64M_LIB atmcd64m "C:/Program files/Andor SDK")
	if (("${ANDOR_ATMCD64M_LIB}" STREQUAL "ANDOR_ATMCD64M_LIB-NOTFOUND"))
	  message(FATAL_ERROR "${NAME} : libatmcd32d not found, you can search it in cmake-gui or try to install it if it isn't here.")
	endif()
else()
	find_library(ANDOR_LIB andor)
	if (("${ANDOR_LIB}" STREQUAL "ANDOR_LIB-NOTFOUND"))
	  message(FATAL_ERROR "${NAME} : libandor not found, you can search it in cmake-gui or try to install it if it isn't here.")
	endif()
endif()

target_link_libraries(lima${NAME} limacore)
target_link_libraries(lima${NAME} ${ANDOR_LIB})

include(CameraVERSION)
if(WIN32)
	set_target_properties(lima${NAME} PROPERTIES PREFIX "lib")
	install(TARGETS lima${NAME} DESTINATION lib)
else()
	install(TARGETS lima${NAME} LIBRARY DESTINATION lib)
endif()

if(LIMA_ENABLE_PYTHON)
	limatools_run_sip_for_camera(${NAME})
    install(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Andor")
endif()
