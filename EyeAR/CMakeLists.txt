
#	EYEAR COMPONENT -- RENDERER
#  Copyright (c) 2008 - 2009 NVIDIA Corporation.  All rights reserved.
#
#  NVIDIA Corporation and its licensors retain all intellectual property and proprietary
#  rights in and to this software, related documentation and any modifications thereto.
#  Any use, reproduction, disclosure or distribution of this software and related
#  documentation without an express license agreement from NVIDIA Corporation is strictly
#  prohibited.
#
#  TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW, THIS SOFTWARE IS PROVIDED *AS IS*
#  AND NVIDIA AND ITS SUPPLIERS DISCLAIM ALL WARRANTIES, EITHER EXPRESS OR IMPLIED,
#  INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
#  PARTICULAR PURPOSE.  IN NO EVENT SHALL NVIDIA OR ITS SUPPLIERS BE LIABLE FOR ANY
#  SPECIAL, INCIDENTAL, INDIRECT, OR CONSEQUENTIAL DAMAGES WHATSOEVER (INCLUDING, WITHOUT
#  LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION, LOSS OF
#  BUSINESS INFORMATION, OR ANY OTHER PECUNIARY LOSS) ARISING OUT OF THE USE OF OR
#  INABILITY TO USE THIS SOFTWARE, EVEN IF NVIDIA HAS BEEN ADVISED OF THE POSSIBILITY OF
#  SUCH DAMAGES
#

if(GLUT_FOUND AND OPENGL_FOUND)
  include_directories(${GLUT_INCLUDE_DIR})
  add_definitions(-DGLUT_FOUND -DGLUT_NO_LIB_PRAGMA)

  # See top level CMakeLists.txt file for documentation of OPTIX_add_sample_executable.
  OPTIX_add_sample_executable( EyeAR
    main.cpp
    #
    # camera
    #
    dof_camera.cu
    #
    # intersection
    #
    parallelogram.cu
    screen.cu
    pixel.cu
	screen.h
    sphere.cu
    sphere_texcoord.cu
    #
    # shading
    #
    clearcoat.cu
    constantbg.cu
    distributed_phong.cu
    distributed_phong.h
    distributed_brdf.cu
    distributed_brdf.h
    #
    # other
    #
    commonStructs.h
    helpers.h
    radical_inverse_jitter.h
    radical_inverse_jitter.cpp
    sharpview.h
    sharpview.cpp
    eyearscene.h
    eyearscene.cpp
    )


else()
  # GLUT or OpenGL not found
  message("Disabling cook, which requires glut and opengl.")
endif()

SET(HAVE_CV 0)
	find_package(OpenCV REQUIRED)
	IF(OPENCV_FOUND)
		add_definitions(-DHAVE_CV)
		include_directories( ${OPENCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include
				     ${OPENCV_DIR}/build/include/)
		if(Win32)			
		elseif()
			include_libraries(${OPENCV_LIBRARIES})
		endif()
		SET(HAVE_CV 1)
	ENDIF(OPENCV_FOUND)
find_package(CUDA REQUIRED)
include_directories(
${OpenCV_DIR}/include/opencv
${OpenCV_DIR}/include
)
set(OpenCV_LIBS 
"opencv_gpu;opencv_contrib;opencv_legacy;opencv_objdetect;opencv_calib3d;opencv_features2d;opencv_video;opencv_highgui;opencv_ml;opencv_imgproc;opencv_flann;opencv_core")
link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib/x64 )	

