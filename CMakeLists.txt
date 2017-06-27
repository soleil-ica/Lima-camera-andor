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

if(UNIX)
  set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--add-needed")
endif()


set(andor_srcs src/AndorCamera.cpp  src/AndorInterface.cpp
	 src/AndorDetInfoCtrlObj.cpp src/AndorSyncCtrlObj.cpp
	 src/AndorBinCtrlObj.cpp src/AndorShutterCtrlObj.cpp
	 src/AndorRoiCtrlObj.cpp)
	 
file(STRINGS "VERSION" andor_vers)

add_library(limaandor SHARED ${andor_srcs})

#INCLUDES
target_include_directories(limaandor PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
if(WIN32)
	target_include_directories(limaandor PUBLIC "C:/Program files/Andor SDK")
	target_include_directories(limaandor PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/sdk/msvc/include")
endif()


if(WIN32)
	target_compile_definitions(limaandor PRIVATE LIBANDOR_EXPORTS)
endif()

#LIBRARY
if(WIN32)
	find_library(ANDOR_LIB atmcd64m "C:/Program files/Andor SDK")
	if (("${ANDOR_LIB}" STREQUAL "ANDOR_LIB-NOTFOUND"))
	  message(FATAL_ERROR "andor : libatmcd32d not found, you can search it in cmake-gui or try to install it if it isn't here.")
	endif()
else()
	find_library(ANDOR_LIB andor)
	if (("${ANDOR_LIB}" STREQUAL "ANDOR_LIB-NOTFOUND"))
	  message(FATAL_ERROR "andor : libandor not found, you can search it in cmake-gui or try to install it if it isn't here.")
	endif()
endif()

target_link_libraries(limaandor limacore)
target_link_libraries(limaandor ${ANDOR_LIB})

set_target_properties(limaandor PROPERTIES VERSION ${andor_vers} SOVERSION "1")
if(WIN32)
	set_target_properties(limaandor PROPERTIES PREFIX "lib")
	install(TARGETS limaandor DESTINATION lib)
else()
	install(TARGETS limaandor LIBRARY DESTINATION lib)
endif()

if(COMPILE_SIP)
    set(NAME "andor")
	include(CameraRunSIP)
    install(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Andor")
endif()
