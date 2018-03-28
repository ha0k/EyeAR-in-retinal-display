if(CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT APPLE)
  set(bit_dest "64")
else()
  set(bit_dest "")
endif()
set(ROSDEPS_INSTALL_DIR ${ROSDEPS_DIR})
message(STATUS "${ROSDEPS_INSTALL_DIR} IS WHERE THE LIBRARY IS")
macro(ROSDEPS_find_api_library name version)
  find_library(${name}_LIBRARY
    NAMES ${name}.${version} ${name}
    PATHS "${ROSDEPS_INSTALL_DIR}/lib${bit_dest}"
    NO_DEFAULT_PATH
    )
  find_library(${name}_LIBRARY
    NAMES ${name}.${version} ${name}
    )
  if(WIN32)
    find_file(${name}_DLL
      NAMES ${name}.${version}.dll
      PATHS "${ROSDEPS_INSTALL_DIR}/bin${bit_dest}"
      NO_DEFAULT_PATH
      )
    find_file(${name}_DLL
      NAMES ${name}.${version}.dll
      )
  endif()
endmacro()

# Include
find_path(ROSDEPS_INCLUDE
  NAMES bzlib.h
  PATHS "${ROSDEPS_INSTALL_DIR}/include"
  NO_DEFAULT_PATH
  )
find_path(ROSDEPS_INCLUDE
  NAMES bzlib.h
  )
SET(ROSDEPS_LIBRARIES ${ROSDEPS_DIR}lib)
message(STATUS "${ROSDEPS_LIBRARIES} IS WHERE LIB DIRECTORIES ARE")
SET(ROSDEPS_INCLUDE_DIRS ${ROSDEPS_INCLUDE})
# Check to make sure we found what we were looking for
function(ROSDEPS_report_error error_message required)
  if(ROSDEPS_FIND_REQUIRED AND required)
    message(FATAL_ERROR "${error_message}")
  else()
    if(NOT ROSDEPS_FIND_QUIETLY)
      message(STATUS "${error_message}")
    endif(NOT ROSDEPS_FIND_QUIETLY)
  endif()
endfunction()


if(NOT ROSDEPS_INCLUDE)
  ROSDEPS_report_error("ROSDEPS headers (ROSDEPS.h and friends) not found.  Please locate before proceeding." TRUE)
endif()

# Macro for setting up dummy targets
function(ROSDEPS_add_imported_library name lib_location dll_lib dependent_libs)
  set(CMAKE_IMPORT_FILE_VERSION 1)

  # Create imported target
  add_library(${name} SHARED IMPORTED)

  # Import target "ROSDEPS" for configuration "Debug"
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
      #IMPORTED_SONAME "${ROSDEPS_soname}"
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
#ROSDEPS_add_imported_library(ROSDEPS "${ROSDEPS_LIBRARY}" "${ROSDEPS_DLL}" "${OPENGL_LIBRARIES}")

macro(ROSDEPS_check_same_path libA libB)
  if(_ROSDEPS_path_to_${libA})
    if(NOT _ROSDEPS_path_to_${libA} STREQUAL _ROSDEPS_path_to_${libB})
      # ${libA} and ${libB} are in different paths.  Make sure there isn't a ${libA} next
      # to the ${libB}.
      get_filename_component(_ROSDEPS_name_of_${libA} "${${libA}_LIBRARY}" NAME)
      if(EXISTS "${_ROSDEPS_path_to_${libB}}/${_ROSDEPS_name_of_${libA}}")
        message(WARNING " ${libA} library found next to ${libB} library that is not being used.  Due to the way we are using rpath, the copy of ${libA} next to ${libB} will be used during loading instead of the one you intended.  Consider putting the libraries in the same directory or moving ${_ROSDEPS_path_to_${libB}}/${_ROSDEPS_name_of_${libA} out of the way.")
      endif()
    endif()
    set( _${libA}_rpath "-Wl,-rpath,${_ROSDEPS_path_to_${libA}}" )
  endif()
endmacro()

# Since libROSDEPS.1.dylib is built with an install name of @rpath, we need to
# compile our samples with the rpath set to where ROSDEPS exists.
if(APPLE)
  get_filename_component(_ROSDEPS_path_to_ROSDEPS "${ROSDEPS_LIBRARY}" PATH)
  if(_ROSDEPS_path_to_ROSDEPS)
    set( _ROSDEPS_rpath "-Wl,-rpath,${_ROSDEPS_path_to_ROSDEPS}" )
  endif()

  set( ROSDEPS_rpath ${_ROSDEPS_rpath} ${_ROSDEPSu_rpath} ${_ROSDEPS_prime_rpath} )
  list(REMOVE_DUPLICATES ROSDEPS_rpath)
endif()
