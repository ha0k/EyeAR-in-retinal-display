#ifdef __APPLE__
#  include <OpenGL/gl.h>
#else
#  include <GL/glew.h>
#  include <GL/gl.h>
#  include <GL/glut.h>
#endif
#include <GLUTDisplay.h>
#include "eyearscene.h"

#include <windows.h>

/*
 * Copyright (c) 2008 - 2009 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and proprietary
 * rights in and to this software, related documentation and any modifications thereto.
 * Any use, reproduction, disclosure or distribution of this software and related
 * documentation without an express license agreement from NVIDIA Corporation is strictly
 * prohibited.
 *
 * TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW, THIS SOFTWARE IS PROVIDED *AS IS*
 * AND NVIDIA AND ITS SUPPLIERS DISCLAIM ALL WARRANTIES, EITHER EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE.  IN NO EVENT SHALL NVIDIA OR ITS SUPPLIERS BE LIABLE FOR ANY
 * SPECIAL, INCIDENTAL, INDIRECT, OR CONSEQUENTIAL DAMAGES WHATSOEVER (INCLUDING, WITHOUT
 * LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION, LOSS OF
 * BUSINESS INFORMATION, OR ANY OTHER PECUNIARY LOSS) ARISING OUT OF THE USE OF OR
 * INABILITY TO USE THIS SOFTWARE, EVEN IF NVIDIA HAS BEEN ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGES
 */

//-----------------------------------------------------------------------------
//
//  cook.cpp -- Cook style distributed ray tracing example, showing the '1984'
//              billiard balls scene as well as an alternate scene to demonstrate
//              depth of field
//
//  options:  -t   | --texture-path  <path_to_textures>  Specify path to textures
//            -mb  | --motion-blur                       Choose 1984 Cook scene
//            -dof | --depth-of-field                    DoF test (default)
//
//-----------------------------------------------------------------------------
#define BRDF_SAMPLING_RES_THETA_H       90
#define BRDF_SAMPLING_RES_THETA_D       90
#define BRDF_SAMPLING_RES_PHI_D         360

//-----------------------------------------------------------------------------
// 
// Main 
//
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
EyeARScene* mainScene;

void printUsageAndExit( const std::string& argv0, bool doExit = true )
{
  std::cerr
    << "Usage  : " << argv0 << " [options]\n"
    << "App options:\n"
    << "  -h  | --help                               Print this usage message\n"
    << "  -t  | --texture-path <path>                Specify path to textures\n"
    << "  -mb | --motion-blur                        1984 Cook motion blur scene\n"
    << " -dof | --depth-of-field                     Depth of field test (default)\n"
    << "  -f3                                        Use float3 output buffer (default is float4)\n"
    << std::endl;
  GLUTDisplay::printUsage();
  if ( doExit ) exit(1);
}



//various callbacks

//#define LOADER
//#define NEWINTERFACE
#if defined(LOADER) || defined(NEWINTERFACE)
#include "ExperimentImageLoader.hpp"
ImageLoader loader;

#endif


int main( int argc, char** argv )
{

#if defined(LOADER) || defined(NEWINTERFACE)
  srand(1984);
    loader.runProgram();
#else
  GLUTDisplay::init( argc, argv );
  srand(1984);

  // Process command line options
  std::string texture_path;
  bool use_float3_output_buffer = true;

  if( !GLUTDisplay::isBenchmark() ) printUsageAndExit( argv[0], false );

  if( texture_path.empty() ) {
    texture_path = std::string( sutilSamplesDir() ) + "/cook/data";
  }
  
  try{

  	mainScene = new EyeARScene( texture_path, use_float3_output_buffer );
    GLUTDisplay::run( "EyeAR", mainScene, GLUTDisplay::CDProgressive );
  }catch(optix::Exception &e){
  }
#endif

  std::cout << "Shutdown Complete" << std::endl;
#ifdef LOADER
  loader.stopProgram();
#endif
  return 0;
}
