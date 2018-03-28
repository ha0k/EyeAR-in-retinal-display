if(CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT APPLE)
  set(bit_dest "64")
else()
  set(bit_dest "")
endif()
set(ROS_INSTALL_DIR CACHE FILE "")
set(ROS_LIB_CPP CACHE FILE "")
set(ROS_LIB_SERIALIZATION CACHE FILE	"")
set(ROS_LIB_CONSOLE CACHE FILE	"")
set(ROS_LIB_TIME CACHE PATH	"")
macro(ROS_find_api_library name version)
  find_library(${name}_LIBRARY
    NAMES ${name}.${version} ${name}
    PATHS "${ROS_INSTALL_DIR}/lib${bit_dest}"
    NO_DEFAULT_PATH
    )
  find_library(${name}_LIBRARY
    NAMES ${name}.${version} ${name}
    )
  if(WIN32)
    find_file(${name}_DLL
      NAMES ${name}.${version}.dll
      PATHS "${ROS_INSTALL_DIR}/bin${bit_dest}"
      NO_DEFAULT_PATH
      )
    find_file(${name}_DLL
      NAMES ${name}.${version}.dll
      )
  endif()
endmacro()


# Include
find_path(ROS_INCLUDE
  NAMES base64.h
  PATHS "${ROS_INSTALL_DIR}/include"
  NO_DEFAULT_PATH
  )
find_path(ROS_INCLUDE
  NAMES base64.h
  )
SET(ROS_LIBRARIES ${ROS_LIBRARY})
SET(ROS_INCLUDE_DIRS ${ROS_INCLUDE})
# Check to make sure we found what we were looking for
function(ROS_report_error error_message required)
  if(ROS_FIND_REQUIRED AND required)
    message(FATAL_ERROR "${error_message}")
  else()
    if(NOT ROS_FIND_QUIETLY)
      message(STATUS "${error_message}")
    endif(NOT ROS_FIND_QUIETLY)
  endif()
endfunction()

if(NOT ROS_LIBRARY)
  ROS_report_error("ROS library not found.  Please locate before proceeding." TRUE)
endif()
if(NOT ROS_INCLUDE)
  ROS_report_error("ROS headers (ROS.h and friends) not found.  Please locate before proceeding." TRUE)
endif()

# Macro for setting up dummy targets
function(ROS_add_imported_library name lib_location dll_lib dependent_libs)
  set(CMAKE_IMPORT_FILE_VERSION 1)

  # Create imported target
  add_library(${name} SHARED IMPORTED)

  # Import target "ROS" for configuration "Debug"
  if(WIN32)
    set_target_properties(${name} PROPERTIES
      IMPORTED_IMPLIB "${lib_location}"
      #IMPORTED_LINK_INTERFACE_LIBRARIES "glu32;opengl32"
      IMPORTED_LOCATION "${dll_lib}"
      IMPORTED_LINK_INTERFACE_LIBRARIES "${dependent_libs}"
      )
  elseif(UNIX)
    set_target_properties(${name} PROPERTIES
      #IMPORTED_LINK_INTERFACE_LIBRARIES "glu32;opengl32"
      IMPORTED_LOCATION "${lib_location}"
      # We don't have versioned filenames for now, and it may not even matter.
      #IMPORTED_SONAME "${ROS_soname}"
      IMPORTED_LINK_INTERFACE_LIBRARIES "${dependent_libs}"
      )
  else()
    # Unknown system, but at least try and provide the minimum required
    # information.
    set_target_properties(${name} PROPERTIES
      IMPORTED_LOCATION "${lib_location}"
      IMPORTED_LINK_INTERFACE_LIBRARIES "${dependent_libs}"
      )
  endif()

  # Commands beyond this point should not need to know the version.
  set(CMAKE_IMPORT_FILE_VERSION)
endfunction()

# Sets up a dummy target
#ROS_add_imported_library(ROS "${ROS_LIBRARY}" "${ROS_DLL}" "${OPENGL_LIBRARIES}")

macro(ROS_check_same_path libA libB)
  if(_ROS_path_to_${libA})
    if(NOT _ROS_path_to_${libA} STREQUAL _ROS_path_to_${libB})
      # ${libA} and ${libB} are in different paths.  Make sure there isn't a ${libA} next
      # to the ${libB}.
      get_filename_component(_ROS_name_of_${libA} "${${libA}_LIBRARY}" NAME)
      if(EXISTS "${_ROS_path_to_${libB}}/${_ROS_name_of_${libA}}")
        message(WARNING " ${libA} library found next to ${libB} library that is not being used.  Due to the way we are using rpath, the copy of ${libA} next to ${libB} will be used during loading instead of the one you intended.  Consider putting the libraries in the same directory or moving ${_ROS_path_to_${libB}}/${_ROS_name_of_${libA} out of the way.")
      endif()
    endif()
    set( _${libA}_rpath "-Wl,-rpath,${_ROS_path_to_${libA}}" )
  endif()
endmacro()

# Since libROS.1.dylib is built with an install name of @rpath, we need to
# compile our samples with the rpath set to where ROS exists.
if(APPLE)
  get_filename_component(_ROS_path_to_ROS "${ROS_LIBRARY}" PATH)
  if(_ROS_path_to_ROS)
    set( _ROS_rpath "-Wl,-rpath,${_ROS_path_to_ROS}" )
  endif()
  set( ROS_rpath ${_ROS_rpath} ${_ROSu_rpath} ${_ROS_prime_rpath} )
  list(REMOVE_DUPLICATES ROS_rpath)
endif()
