
#	SceneManagement cmake
#   EDIT MEEEEE!!!!!
#

set(sources
	scenemanagment.cpp
	scenemanagment.h
  )

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
# For commonStructs.h
include_directories(${SAMPLES_CUDA_DIR})



include_directories( ${Boost_Include_DIRS} )
FIND_PACKAGE(Boost REQUIRED)
    IF (Boost_FOUND)
        INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
        ADD_DEFINITIONS( "-DHAS_BOOST" )
	ENDIF()
SET(HAVE_CV 0)
	find_package(OpenCV REQUIRED)
	IF(OPENCV_FOUND)
		add_definitions(-DHAVE_CV)
		include_directories( ${OPENCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include
				     ${OPENCV_DIR}/build/include/
                                     )
		SET(HAVE_CV 1)
	ENDIF(OPENCV_FOUND)

# Make the library.
add_library(scenemanagment STATIC ${sources})

# Use gcc rather than g++ to link if we are linking statically against libgcc_s and libstdc++
if(USING_GNU_C OR USING_GNU_CXX)
  if(GCC_LIBSTDCPP_HACK)
    set_target_properties(scenemanagment PROPERTIES LINKER_LANGUAGE "C")
    target_link_libraries(scenemanagment LINK_PRIVATE ${STATIC_LIBSTDCPP} )
  endif()
endif()

# Note that if the GLUT_LIBRARIES and OPENGL_LIBRARIES haven't been looked for,
# these variable will be empty.
target_link_libraries(scenemanagment
  optix
  optixu
  ${OPENCV_LIBRARIES}
  ${Boost_LIBRARIES}
  )
if(WIN32)
  target_link_libraries(scenemanagment winmm.lib
)
endif()
add_perforce_to_target( scenemanagment )

# Copy the free glut dlls as part of the sutil build process
if(WIN32)
  if(CMAKE_GENERATOR MATCHES "Visual Studio")
    set( build_configuration "$(ConfigurationName)" )
  else()
    set( build_configuration "${CMAKE_BUILD_TYPE}")
  endif()

  add_custom_command(TARGET sutil
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    -D build_configuration:STRING=${build_configuration}
    -D "output_directory:PATH=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_CFG_INTDIR}"
    -P "${CMAKE_SOURCE_DIR}/CMake/CopyDLL.cmake"
    )
endif()

if(RELEASE_INSTALL_BINARY_SAMPLES AND NOT RELEASE_STATIC_BUILD)
  # If performing a release install, we want to use rpath for our install name.
  # The executables' rpaths will then be set to @executable_path so we can invoke
  # the samples from an arbitrary location and it will still find this library.
  set_target_properties(scenemanagment PROPERTIES
    INSTALL_NAME_DIR "@rpath"
    BUILD_WITH_INSTALL_RPATH ON
    )
  install(TARGETS scenemanagment
    RUNTIME DESTINATION ${SDK_BINARY_INSTALL_DIR}
    LIBRARY DESTINATION ${SDK_BINARY_INSTALL_DIR}
    )
endif()

# Make the list of sources available to the parent directory for installation needs.
set(scenemanagment "${sources}" PARENT_SCOPE)
