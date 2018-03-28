#
# OptiX-Renderer
#

# This sets up the name of our project.  For our purposes the main thing this controls is
# the name of the VS solution file.
project(Physically-Based-AR)

# This enforces a particular version of CMake that we require to process the script files
# properly.
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
# As of CMake 2.6 policies were introduced in order to provide a mechanism for
# adding backwards compatibility one feature at a time.  We will just specify
# that all policies will use version 2.6 symantics.
cmake_policy(VERSION 2.6)

# Add paths to our CMake code to the module path, so they can be found automatically by
# CMake.
set(CMAKE_MODULE_PATH
  "${CMAKE_SOURCE_DIR}/CMake"
  ${CMAKE_MODULE_PATH}
  )

# Set the default build to Release.  Note this doesn't do anything for the VS
# default build target which defaults to Debug when you first start it.
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# Tells CMake to build all the libraries as shared libraries by default.  This can be
# overrided by individual libraries later.
set(BUILD_SHARED_LIBS ON)
##########
# Process our custom setup scripts here.

# Include all CMake Macros.
include(Macros)
# Determine information about the compiler
include (CompilerInfo)
# Check for specific machine/compiler options.
include (ConfigCompilerFlags)

# Turn off the warning that NVCC issues when generating PTX from our CUDA samples.  This
# is a custom extension to the FindCUDA code distributed by CMake.
OPTION(CUDA_REMOVE_GLOBAL_MEMORY_SPACE_WARNING "Suppress the \"Advisory: Cannot tell what pointer points to, assuming global memory space\" warning nvcc makes." ON)

# For Xcode 5, gcc is actually clang, so we have to tell CUDA to treat the compiler as
# clang, so that it doesn't mistake it for something else.
if(USING_CLANG_C)
  set(CUDA_HOST_COMPILER "clang" CACHE FILEPATH "Host side compiler used by NVCC")
endif()

# Find at least a 2.3 version of CUDA.
find_package(CUDA 2.3 REQUIRED)

# Present the CUDA_64_BIT_DEVICE_CODE on the default set of options.
mark_as_advanced(CLEAR CUDA_64_BIT_DEVICE_CODE)

# Add some useful default arguments to the nvcc flags.  This is an example of how we use
# PASSED_FIRST_CONFIGURE.  Once you have configured, this variable is TRUE and following
# block of code will not be executed leaving you free to edit the values as much as you
# wish from the GUI or from ccmake.
if(NOT PASSED_FIRST_CONFIGURE)
  set(flag "--use_fast_math")
  list(FIND CUDA_NVCC_FLAGS ${flag} index)
  if(index EQUAL -1)
    list(APPEND CUDA_NVCC_FLAGS ${flag})
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} CACHE LIST "Semi-colon delimit multiple arguments." FORCE)
  endif()

  if (CUDA_VERSION VERSION_LESS "3.0")
    set(flag "--keep")
    list(FIND CUDA_NVCC_FLAGS ${flag} index)
    if(index EQUAL -1)
      list(APPEND CUDA_NVCC_FLAGS ${flag})
      set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} CACHE LIST "Semi-colon delimit multiple arguments." FORCE)
    endif()
  endif()

  if( APPLE )
    # Undef'ing __BLOCKS__ for OSX builds.  This is due to a name clash between OSX 10.6
    # C headers and CUDA headers
    set(flag "-U__BLOCKS__")
    list(FIND CUDA_NVCC_FLAGS ${flag} index)
    if(index EQUAL -1)
      list(APPEND CUDA_NVCC_FLAGS ${flag})
      set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} CACHE LIST "Semi-colon delimit multiple arguments." FORCE)
    endif()
  endif()
endif(NOT PASSED_FIRST_CONFIGURE)

# This passes a preprocessor definition to cl.exe when processing CUDA code.
if(USING_WINDOWS_CL)
  list(APPEND CUDA_NVCC_FLAGS --compiler-options /D_USE_MATH_DEFINES)
endif()

# Put all the runtime stuff in the same directory.  By default, CMake puts each targets'
# output into their own directory.  We want all the targets to be put in the same
# directory, and we can do this by setting these variables.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Create a flag for mac which will allow apps to add the local cuda toolkit
# install path to the app's rpath.
if( APPLE )
  set( CUDA_TOOLKIT_RPATH_FLAG "-Wl,-rpath,${CUDA_TOOLKIT_ROOT_DIR}/lib" )
endif()

# Locate the NVRT distribution.  Search the SDK first, then look in the system.
# Search for the OptiX libraries and include files.
find_package(OptiX REQUIRED)

# Add the path to the OptiX headers to our include paths.
include_directories(
  "${OptiX_INCLUDE}"
  )

# Force the perforce plugin off for the SDK.
set(USE_PERFORCE OFF CACHE BOOL "This is forced off." FORCE)

##################################################################
# SUtil compilation

set(SAMPLES_PTX_DIR "${CMAKE_BINARY_DIR}/lib/ptx" CACHE PATH "Path to where the samples look for the PTX code.")
set(SAMPLES_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Path to the samples directory.")

set(CUDA_GENERATED_OUTPUT_DIR ${SAMPLES_PTX_DIR})

if (WIN32)
  string(REPLACE "/" "\\\\" SAMPLES_PTX_DIR ${SAMPLES_PTX_DIR})
else (WIN32)
  if ( USING_GNU_C AND NOT APPLE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DM_PI=3.14159265358979323846" )
  endif()
endif (WIN32)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/sampleConfig.h.in ${CMAKE_CURRENT_BINARY_DIR}/sampleConfig.h @ONLY)

# Path to sutil.h that all the samples need
if(WIN32)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/sutil
		     ${CMAKE_CURRENT_SOURCE_DIR}/scenemanagment
		     ${CMAKE_SOURCE_DIR}/include
                     ${OpenCV_INCLUDE_DIRS}/
                     ${OptiX_INCLUDE}/optixu
                     ${CMAKE_CURRENT_BINARY_DIR}
                     ${CUDA_INCLUDE_DIRS} )
else()
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/sutil
		     ${CMAKE_CURRENT_SOURCE_DIR}/scenemanagment
                     ${OptiX_INCLUDE}/optixu
		     ${CMAKE_SOURCE_DIR}/include
                     ${OpenCV_INCLUDE_DIRS}/
                     ${CMAKE_CURRENT_BINARY_DIR}
                     ${CUDA_INCLUDE_DIRS} )
endif()
include(FindSUtilGLUT)

set(SAMPLES_CUDA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cuda)

#########################################################
# OPTIX_add_sample_executable
#
# Convience function for adding samples to the code.  You can copy the contents of this
# funtion into your individual project if you wish to customize the behavior.  Note that
# in CMake, functions have their own scope, whereas macros use the scope of the caller.

function(OPTIX_add_sample_executable target_name)

  # These calls will group PTX and CUDA files into their own directories in the Visual
  # Studio projects.
  source_group("PTX Files"  REGULAR_EXPRESSION ".+\\.ptx$")
  source_group("CUDA Files" REGULAR_EXPRESSION ".+\\.cu$")

  # Separate the sources from the CMake and CUDA options fed to the macro.  This code
  # comes from the CUDA_COMPILE_PTX macro found in FindCUDA.cmake.  We are copying the
  # code here, so that we can use our own name for the target.  target_name is used in the
  # creation of the output file names, and we want this to be unique for each target in
  # the SDK.
  CUDA_GET_SOURCES_AND_OPTIONS(source_files cmake_options options ${ARGN})

  # Create the rules to build the PTX from the CUDA files.
  CUDA_WRAP_SRCS( ${target_name} PTX generated_files ${source_files} ${cmake_options}
    OPTIONS ${options} )

  # Here is where we create the rule to make the executable.  We define a target name and
  # list all the source files used to create the target.  In addition we also pass along
  # the cmake_options parsed out of the arguments.
  add_executable(${target_name}
    ${source_files}
    ${generated_files}
    ${cmake_options}
    )

  # Most of the samples link against the sutil library and the optix library.  Here is the
  # rule that specifies this linkage.
if(WIN32)
    find_package( OpenCV REQUIRED )
	find_package(BOOST REQUIRED)
  link_directories(${BOOST_LIBRARIES})

  target_link_libraries( ${target_name}
    scenemanagment
    sutil
    optix
	${OpenCV_LIBS}
    ${BOOST_LIBRARIES}	
    ${optix_rpath}
    )
else()
  target_link_libraries( ${target_name}
    scenemanagment
    sutil
    optix
    ${optix_rpath}
    )
endif()
  if(USING_GNU_CXX)
    target_link_libraries( ${target_name} m ) # Explicitly link against math library (C samples don't do that by default)
  endif()
endfunction()

#########################################################
#  List of samples found in subdirectories.
#
# If you wish to start your own sample, you can copy one of the sample's directories.
# Just make sure you rename all the occurances of the sample's name in the C code as well
# and the CMakeLists.txt file.
# Our sutil library.  The rules to build it are found in the subdirectory.
add_subdirectory(sutil)
if(WIN32)
add_subdirectory(EyeAR)
endif()
add_subdirectory(scenemanagment)

# nvcommon library used by isgShadows.  If you don't compile isgShadows, you can comment
# this out.
add_subdirectory(nvcommon)

# This copies out dlls into the build directories, so that users no longer need to copy
# them over in order to run the samples.  This depends on the cook sample being compiled.
# If you remove this sample from the list of compiled samples, then you should change
# "cook" found below to the name of one of your other samples.
if(WIN32)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT APPLE)
    set(bit_dest "64")
  else()
    set(bit_dest "")
  endif()
  foreach(config ${CMAKE_CONFIGURATION_TYPES})
    get_target_property(loc cook ${config}_LOCATION)
    if(loc)
      # A little helper function
      function(copy_dll lib)
        get_filename_component(path ${loc} PATH)
        get_filename_component(name ${lib} NAME)
        #message("${CMAKE_COMMAND} -E copy_if_different ${lib} ${path}/${name}")
        execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${lib} ${path}/${name})
      endfunction()
      
      # Copy the binary directory into the build
      file(GLOB dlls "${OptiX_INSTALL_DIR}/bin${bit_dest}/*.dll")
      foreach(file ${dlls})
        copy_dll("${file}")
      endforeach()
    else()
      message(WARNING "Unable to find location to copy DLLs into the build")
    endif()
  endforeach()
endif(WIN32)

#################################################################

# Now that everything is done, indicate that we have finished configuring at least once.
# We use this variable to set certain defaults only on the first pass, so that we don't
# continually set them over and over again.
set(PASSED_FIRST_CONFIGURE ON CACHE INTERNAL "Already Configured once?")

