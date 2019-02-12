set(ANDOR_INCLUDE_DIRS)
set(ANDOR_LIBRARIES)
set(ANDOR_DEFINITIONS)

if(WIN32)
  # Missing find_path here
  find_library(ANDOR_LIBRARIES atmcd64m "c:/program files/andor sdk")
  find_path(ANDOR_INCLUDE_DIRS "atmcd32d.h" "c:/program files/andor sdk")
else()
  find_path(ANDOR_INCLUDE_DIRS "atmcdLXd.h")
  find_library(ANDOR_LIBRARIES andor)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Andor DEFAULT_MSG
  ANDOR_LIBRARIES
  ANDOR_INCLUDE_DIRS
)
