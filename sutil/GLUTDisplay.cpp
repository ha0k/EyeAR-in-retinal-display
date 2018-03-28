
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


#if defined(__APPLE__)
#  include <GLUT/glut.h>
#  define GL_FRAMEBUFFER_SRGB_EXT           0x8DB9
#  define GL_FRAMEBUFFER_SRGB_CAPABLE_EXT   0x8DBA
#else
#  include <GL/glew.h>
#  if defined(_WIN32)
#    include <GL/wglew.h>
#  endif
#  include <GL/glut.h>
#endif

#include <GLUTDisplay.h>
#include <Mouse.h>
#include <DeviceMemoryLogger.h>

#include <optixu/optixu_math_stream_namespace.h>

#include <algorithm>
#include <cassert>
#include <iostream>
#include <cstdlib>
#include <cstdio> //sprintf
#include <sstream>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <optixu/optixu_math_stream_namespace.h>
#include <glm/gtc/type_ptr.hpp>
#include <opencv2\core\core.hpp>
// #define NVTX_ENABLE enables the nvToolsExt stuff from Nsight in NsightHelper.h
//#define NVTX_ENABLE

#include <NsightHelper.h>
using namespace optix;
double fov;
//-----------------------------------------------------------------------------
// 
// GLUTDisplay class implementation 
//-----------------------------------------------------------------------------
int frameThisSecond = 0;
Mouse*         GLUTDisplay::m_mouse                = 0;
PinholeCamera* GLUTDisplay::m_camera               = 0;
SampleScene*   GLUTDisplay::m_scene                = 0;

AccelDescriptor GLUTDisplay::m_accel_desc;

double         GLUTDisplay::m_last_frame_time      = 0.0;
unsigned int   GLUTDisplay::m_last_frame_count     = 0;
unsigned int   GLUTDisplay::m_frame_count          = 0;

bool           GLUTDisplay::m_display_fps          = true;
double         GLUTDisplay::m_fps_update_threshold = 0.5;
char           GLUTDisplay::m_fps_text[32];
float3         GLUTDisplay::m_text_color           = make_float3( 0.95f );
float3         GLUTDisplay::m_text_shadow_color    = make_float3( 0.10f );

bool           GLUTDisplay::m_print_mem_usage      = false;

GLUTDisplay::contDraw_E GLUTDisplay::m_app_continuous_mode = CDNone;
GLUTDisplay::contDraw_E GLUTDisplay::m_cur_continuous_mode = CDNone;

bool           GLUTDisplay::m_display_frames       = true;
bool           GLUTDisplay::m_save_frames_to_file  = false;
std::string    GLUTDisplay::m_save_frames_basename = "";

std::string    GLUTDisplay::m_camera_pose          = "";

int            GLUTDisplay::m_initial_window_width = -1;
int            GLUTDisplay::m_initial_window_height= -1;

int            GLUTDisplay::m_old_window_height    = -1;
int            GLUTDisplay::m_old_window_width     = -1;
int            GLUTDisplay::m_old_window_x         = -1;
int            GLUTDisplay::m_old_window_y         = -1;
int            GLUTDisplay::m_old_window_x_offset  = -1;
int            GLUTDisplay::m_old_window_y_offset  = -1;

unsigned int   GLUTDisplay::m_texId                = 0;
bool           GLUTDisplay::m_sRGB_supported       = false;
bool           GLUTDisplay::m_use_sRGB             = false;

bool           GLUTDisplay::m_initialized          = false;
bool           GLUTDisplay::m_requires_display     = false;
bool           GLUTDisplay::m_benchmark_no_display = false;
unsigned int   GLUTDisplay::m_warmup_frames        = 50u;
unsigned int   GLUTDisplay::m_timed_frames         = 100u;
double         GLUTDisplay::m_warmup_start         = 0;
double         GLUTDisplay::m_warmup_time          = 10.0;
double         GLUTDisplay::m_benchmark_time       = 10.0;
unsigned int   GLUTDisplay::m_benchmark_frame_start= 0;
double         GLUTDisplay::m_benchmark_frame_time = 0;
std::string    GLUTDisplay::m_title                = "";

double         GLUTDisplay::m_progressive_timeout  = -1.;
double         GLUTDisplay::m_start_time           = 0.0;

int            GLUTDisplay::m_num_devices          = 0;

bool           GLUTDisplay::m_enable_cpu_rendering = false;

bool           GLUTDisplay::m_use_PBO              = true;

bool           GLUTDisplay::m_vca_requested = false;
VCAOptions     GLUTDisplay::m_vca_options;

GLUTDisplay::cameraMode_E GLUTDisplay::m_camera_mode = CMApplication;
  static	std::string    m_var_file;
  static	std::string    m_scene_file;
int            GLUTDisplay::m_special_key = -1;
double         GLUTDisplay::m_special_key_prev_camera_update_time = -1.0;
double         GLUTDisplay::m_special_key_camera_speed = 1.0;

float fx_ = 0.999861;
float fy_ = 1.25575;
float cx_ = -0.5+0.497208;
float cy_ = -0.5+0.435728;
float s = 0.00950071;
float fovX = 0;
float fovY = 0;
glm::mat4 ModelViewMatrix;
struct SmoothSequence
{
	std::vector<cv::Point2f> values;
	long int visit;
	const int n;
	SmoothSequence();
	SmoothSequence(int n);
	inline double smooth(double value, double t);
};

SmoothSequence::SmoothSequence():
	values(200,cv::Point2f(0.0,0.0)),visit(0),n(200){

}
SmoothSequence::SmoothSequence(int n)
	:
	values(n, cv::Point2f(0.0, 0.0)),
	visit(0), n(n)
{

}

double SmoothSequence::smooth(double value, double t)
{
	values[visit % n].x = t;
	values[visit % n].y = value;
	visit++;
	if(values.size() > n){
		values.clear();
		
	}
	cv::Vec4f lineparam;
	cv::fitLine(values, lineparam, CV_DIST_L12, 0, 0.01, 0.01);
	float & vx = lineparam(0);
	float & vy = lineparam(1);
	float & x0 = lineparam(2);
	float & y0 = lineparam(3);
	double est = y0 + (t - x0)/vx*vy;

	return est;
}
void GLUTDisplay::setCameraData(double xPosition,double yPosition, double zPosition,double xRotation,double yRotation,double zRotation, double wRotation){


	fovX = glm::degrees(2*glm::atan(0.5f/fx_));
	fovY = glm::degrees(2*glm::atan(0.5f/fy_));


	std::cout << "Field Of View X:" << fovX << std::endl;
	std::cout << "Field Of View Y:" << fovY << std::endl;

	glm::vec3 cameraPosition(-xPosition,zPosition,yPosition);
	glm::vec3 pos(0,0,0);
	
	glm::mat4 matrix(1.0); //identity matrix
	glm::quat rotation(wRotation,xRotation,yRotation,zRotation); // rotation from PTAM
	glm::mat4 ptamCameraMatrix = glm::mat4_cast(rotation); // PTAM CAMERA MATRIX
	float orientationmatrix[16] = { 1,0,0,0,
									0,0,-1,0,
									0,-1,0,0,
									0,0,0,1
								};
	glm::mat4 orientMat = glm::translate(ptamCameraMatrix,pos);
	ptamCameraMatrix = glm::make_mat4(orientationmatrix) * orientMat;
	glm::vec4 uPoint(0,1,0,1);
	glm::vec4 fPoint(0,0,-0.5f,1);
	glm::vec4 cPoint(0,0,0,1);
	
	glm::vec3 upPoint = glm::vec3((ptamCameraMatrix*uPoint));// + cameraPosition;
	glm::vec3 focusPoint = glm::vec3((ptamCameraMatrix*fPoint)) + cameraPosition;
	glm::vec3 camPoint = glm::vec3((ptamCameraMatrix*cPoint)) + cameraPosition;

	float* mvm = glm::value_ptr(ptamCameraMatrix);
	mvm[3] = cameraPosition.x; mvm[7] = cameraPosition.y; mvm[11] = cameraPosition.z;

	ptamCameraMatrix = glm::make_mat4(mvm);

	glm::vec3 fp = glm::vec3(focusPoint.x,focusPoint.y,focusPoint.z);// + pos;
	glm::vec3 up = glm::vec3(upPoint.x,upPoint.y,upPoint.z);// + pos;
	m_camera->setParameters(make_float3(camPoint.x,camPoint.y,camPoint.z),
                         make_float3(fp[0],fp[1],fp[2]),
						 make_float3(up.x,up.y,up.z),
                         fovX, 
                         fovY,
						 PinholeCamera::KeepNone);
//	m_scene->signalCameraChanged();
	m_camera->setup();
}
void GLUTDisplay::setCameraData(float fovx,float fovy){
  std::cout << "Setting Camera Data Parameters" << std::endl;
	m_camera->setParameters( make_float3( 0.0f, 0.0f, 0.0f ), // eye
                    make_float3( 0.0, 0.0f,  -1.0f ), // lookat
                    make_float3( 0.0f, 1.0f,  0.0f ), // up
            fovx, 
            fovy,
			PinholeCamera::KeepNone);
	m_scene->signalCameraChanged();
	m_camera->setup();
}
void GLUTDisplay::setCameraData(double xPos,double yPos, double zPos){
  std::cout << "Setting camera from 3 parameters" << std::endl;
	glm::mat4 matrix(1.0);
	glm::vec3 pos(xPos,yPos,zPos);

	glm::mat4 mat = glm::translate(matrix,pos);// * m::mat4_cast(rotation);

	glm::vec4 focusPoint(0,-1,0,1);	
	glm::vec4 upPoint(0,0,1,1);
	
	focusPoint = mat*focusPoint;
	upPoint = mat*upPoint;
	
	glm::vec3 fp = glm::vec3(focusPoint.x,focusPoint.y,focusPoint.z);// + pos;
	glm::vec3 up = glm::vec3(upPoint.x,upPoint.y,upPoint.z);// + pos;

	m_camera->setParameters(make_float3(xPos,yPos,zPos),
                         make_float3(fp[0],fp[1],fp[2]),
						 make_float3(0,0,1),
                         fov, 
                         fov,
						 PinholeCamera::KeepNone);
	m_scene->signalCameraChanged();
	m_camera->setup();
}

void GLUTDisplay::refreshScreen(){
	m_scene->signalCameraChanged();
	m_camera->setup();
	glutPostRedisplay();
}


std::string GLUTDisplay::getVarFileName() {
  return m_var_file;
}

std::string GLUTDisplay::getSceneFileName() {
  return m_scene_file;
}

inline void removeArg( int& i, int& argc, char** argv ) 
{
  char* disappearing_arg = argv[i];
  for(int j = i; j < argc-1; ++j) {
    argv[j] = argv[j+1];
  }
  argv[argc-1] = disappearing_arg;
  --argc;
  --i;
}

void GLUTDisplay::printUsage( bool doQuit )
{
  std::cerr
    << "Standard options:\n"
    << "  -d  | --dim=<width>x<height>               Set image dimensions, e.g. --dim=300x300 or -d=300x300\n"
    << "  -D  | --num-devices=<num_devices>          Set desired number of GPUs\n"
    << " -CPU | --enable-cpu                         Enable CPU execution of OptiX programs.  The number of threads can be set with --num-devices.\n"
    << "  -p  | --pose=\"[<eye>][<lookat>][<up>]vfov\" Camera pose, e.g. --pose=\"[0,0,-1][0,0,0][0,1,0]45.0\"\n"
    << "  -s  | --save-frames[=<file_basename>]      Save each frame to frame_XXXX.ppm or file_basename_XXXX.ppm\n"
    << "  -N  | --no-display                         Don't display the image to the GLUT window\n"
    << "  -M  | --mem-usage                          Print memory usage after every frame\n"
    << "  -b  | --benchmark[=<w>x<t>]                Render and display 'w' warmup and 't' timing frames, then exit\n"
    << "  -bb | --timed-benchmark=<w>x<t>            Render and display 'w' warmup and 't' timing seconds, then exit\n";

  if(!GLUTDisplay::m_requires_display)
  {
    std::cerr
      << "  -B  | --benchmark-no-display=<w>x<t>       Render 'w' warmup and 't' timing frames, then exit\n"
      << "  -BB | --timed-benchmark-no-display=<w>x<t> Render 'w' warmup and 't' timing seconds, then exit\n";
  }

  std::cerr
    // << "  -c  | --cache                              Acceleration structure disk caching\n" // This arg is parsed here, but advertised in the sample because they don't all support it.
    << "        --build <name>                       Acceleration structure builder (Default: " << m_accel_desc.builder << ")\n"
    << "        --trav <name>                        Acceleration structure traverser (Default: " << m_accel_desc.traverser << ")\n"
    << "        --refine <n>                         Acceleration structure refinement passes (Default: " << m_accel_desc.refine << ")\n"
    << "        --refit <n>                          Acceleration structure refitting (Default: " << m_accel_desc.refit << ")\n"
    << "        --nopbo                              Output to OptiX buffer instead of OpenGL Pixel Buffer Object\n";


  std::cerr
    << "        --game-cam                           Enable game-style camera: move with arrow keys, turn/look with mouse\n"
    << "        --game-cam-speed                     Scale speed when moving game camera\n";
  
  std::cerr << std::endl;

  std::cerr
    << "Standard mouse interaction:\n"
    << "  left mouse           Camera Rotate/Orbit (when interactive)\n"
    << "  middle mouse         Camera Pan/Truck (when interactive)\n"
    << "  right mouse          Camera Dolly (when interactive)\n"
    << "  right mouse + shift  Camera FOV (when interactive)\n"
    << std::endl;

  std::cerr
    << "Game-mode interaction (\"--game-cam\" and \"--game-cam-speed\" flags):\n"
    << "  left mouse           Camera Turn (Yaw) and Look (Pitch)\n"
    << "  up/down arrow        Camera Move Forward/Back\n"
    << "  left/right arrow     Camera Strafe\n"
    << "  page up/down         Camera Move Vertical\n"
    << std::endl;

  std::cerr
    << "Standard keystrokes:\n"
    << "  q Quit\n"
    << "  f Toggle full screen\n"
    << "  r Toggle continuous mode (progressive refinement, animation, or benchmark)\n"
    << "  R Set progressive refinement to never timeout and toggle continuous mode\n"
    << "  b Start/stop a benchmark\n"
    << "  d Toggle frame rate display\n"
    << "  s Save a frame to 'out.ppm'\n"
    << "  m Toggle memory usage printing\n"
    << "  c Print camera pose\n"
    << std::endl;

  if(doQuit) quit(1);
}

// Not all samples support VCA options, so print this on demand.
void GLUTDisplay::printVCAOptions( )
{
  std::cerr
    << "VCA Options:\n"
    << "        --vca-url                            VCA cluster manager WebSockets URL.\n"
    << "                                             URL format is wss://example.com:443, with wss:// and :443 required.\n"
    << "                                             Pass an empty url to enable VCA on the local machine.\n"
    << "\n"
    << "        --vca-user                           Username for the account on the VCA cluster. Not needed when running locally.\n"
    << "        --vca-password                       Password for the account on the VCA cluster. Not needed when running locally.\n"
    << "        --vca-nodes                          Number of nodes to reserve on the VCA cluster.\n"
    << "        --vca-config                         Index of the VCA configuration to use (run once to list configurations).\n";

  std::cerr << std::endl;
}

void GLUTDisplay::init( int& argc, char** argv )
{
  m_var_file = "./cfg/var.cfg";
  m_scene_file = argv[1];
  std::cout << m_scene_file << " is the scene file we load" << std::endl;
  std::cout << m_var_file << " is the variable file" << std::endl;
  for(int i = 0; i < argc, i++;){
	  std::cout << "Argument: " << argv[i] << std::endl;
  }
  m_initialized = true;
  // Check for incompatible flags
  if (m_vca_requested) {
    m_use_PBO = false;
    if (m_benchmark_no_display) {
      std::cerr << "ERROR: --vca-* and --benchmark-no-display flags are not compatible" << std::endl;
      exit(2);
    }
    if (m_save_frames_to_file) {
      std::cerr << "ERRR: --vca-* and --save-frames flags are not compatible" << std::endl;
      exit(2);
    }
  }

  if (!m_benchmark_no_display)
  {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

  }
}

namespace {
  Mouse::Mode make_MouseMode( GLUTDisplay::cameraMode_E mode )
  {
    if ( mode == GLUTDisplay::CMApplication ) return Mouse::Application;
    if ( mode == GLUTDisplay::CMGame ) return Mouse::Game;
    assert(0 && "Unknown mouse mode");
    return Mouse::Application; // satisfy compiler
  }
};
int Winid;
void GLUTDisplay::run( const std::string& title, SampleScene* scene, contDraw_E continuous_mode )
{
  if ( !m_initialized ) {
    std::cerr << "ERROR - GLUTDisplay::run() called before GLUTDisplay::init()" << std::endl;
    exit(2);
  }
  m_scene = scene;
  m_title = title;
  m_scene->enableCPURendering(m_enable_cpu_rendering);
  if (m_vca_requested && !m_scene->getVCAEnabled()) {
    std::cerr << "WARNING: VCA mode requested in GLUTDisplay but ignored by scene" << std::endl;
  }
  m_scene->setNumDevices( 1 );
  m_scene->setAccelDescriptor( m_accel_desc );
  m_scene->setUsePBOBuffer( m_use_PBO );

  if( m_benchmark_no_display ) {
    runBenchmarkNoDisplay();
    quit(0);
  }

  if( m_print_mem_usage ) {
    DeviceMemoryLogger::logDeviceDescription(m_scene->getContext(), std::cerr);
    DeviceMemoryLogger::logCurrentMemoryUsage(m_scene->getContext(), std::cerr, "Initial memory available: " );
    std::cerr << std::endl;
  }

  // Initialize GLUT and GLEW first. Now initScene can use OpenGL and GLEW.
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  if( m_initial_window_width > 0 && m_initial_window_height > 0)
    glutInitWindowSize( m_initial_window_width, m_initial_window_height );
  else
    glutInitWindowSize( 128, 128 );
  glutInitWindowPosition(0,0);
  Winid = glutCreateWindow( m_title.c_str() );
  glutHideWindow();
#if !defined(__APPLE__)
  glewInit();
  if (glewIsSupported( "GL_EXT_texture_sRGB GL_EXT_framebuffer_sRGB")) {
    m_sRGB_supported = true;
  }
#else
  m_sRGB_supported = true;
#endif
#if defined(_WIN32)
  // Turn off vertical sync
  wglSwapIntervalEXT(0);
#endif

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  // If m_app_continuous_mode was already set to CDBenchmark* on the command line then preserve it.
  setContinuousMode( m_app_continuous_mode == CDNone ? continuous_mode : m_app_continuous_mode );

  int buffer_width;
  int buffer_height;
  try {
    // Set up scene
    SampleScene::InitialCameraData camera_data;
    m_scene->initScene( camera_data );

    if( m_initial_window_width > 0 && m_initial_window_height > 0)
      m_scene->resize( m_initial_window_width, m_initial_window_height );

    if ( !m_camera_pose.empty() )
      camera_data = SampleScene::InitialCameraData( m_camera_pose );
      m_camera = new PinholeCamera( camera_data.eye,
                                   camera_data.lookat,
                                   camera_data.up,
                                   camera_data.hfov,
                                   camera_data.vfov,
                                   PinholeCamera::KeepNone );
    Buffer buffer = m_scene->getOutputBuffer();
    RTsize buffer_width_rts, buffer_height_rts;
    buffer->getSize( buffer_width_rts, buffer_height_rts );
    buffer_width  = static_cast<int>(buffer_width_rts);
    buffer_height = static_cast<int>(buffer_height_rts);
    m_mouse = new Mouse( m_camera, buffer_width, buffer_height, make_MouseMode( m_camera_mode ) );
  } catch( Exception& e ){
    sutilReportError( e.getErrorString().c_str() );
    exit(2);
  }

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, 1, 0, 1, -1, 1 );
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  system("clear");
  std::cout << buffer_width << "," << buffer_height;
  glViewport(0, 0, buffer_width, buffer_height);
  glutInitWindowPosition(-50,0);
  glutShowWindow();

  // reshape window to the correct window resize
  glutReshapeWindow( buffer_width, buffer_height);

  glutPositionWindow(-4, -5 ); //correcting the offset with OpenCV

  // Set callbacks
  glutKeyboardFunc(keyPressed);
  if (m_camera_mode == CMGame) {
    // To reduce lag when using keys to control the camera,
    // ignore key repeats and handle key pressed/up events ourselves
    glutIgnoreKeyRepeat(1);
    glutSpecialFunc(specialKeyPressed);
    glutSpecialUpFunc(specialKeyUp);
  }
//  glutIdleFunc(idle);
  glutDisplayFunc(display);
//  glutMouseFunc(mouseButton);
//  glutMotionFunc(mouseMotion);
  glutReshapeFunc(resize);

  // Initialize timer
  sutilCurrentTime( &m_last_frame_time );
  m_frame_count = 0;
  m_last_frame_count = 0;
  m_start_time = m_last_frame_time;
  if( m_cur_continuous_mode == CDBenchmarkTimed ) {
    m_warmup_start = m_last_frame_time;
    m_warmup_frames = 0;
    m_timed_frames = 0;
  }
  m_benchmark_frame_start = 0;

  //Calculate window position offset
  m_old_window_x_offset = glutGet(GLUT_INIT_WINDOW_X) - glutGet(GLUT_WINDOW_X);
  m_old_window_y_offset = glutGet(GLUT_INIT_WINDOW_Y) - glutGet(GLUT_WINDOW_Y);
  
  // Enter main loop
  glutMainLoop();
}

void GLUTDisplay::setCamera(SampleScene::InitialCameraData& camera_data)
{
  m_camera->setParameters(camera_data.eye,
                         camera_data.lookat,
                         camera_data.up,
                         camera_data.hfov, 
                         camera_data.vfov,
                         PinholeCamera::KeepVertical );
}

// This is an internal function that does the actual work.
void GLUTDisplay::setCurContinuousMode(contDraw_E continuous_mode)
{
  m_cur_continuous_mode = continuous_mode;

  sutilCurrentTime( &m_start_time );
  //glutIdleFunc( m_cur_continuous_mode!=CDNone ? idle : 0 );
}

// This is an API function for restaring the progressive timeout timer. 
void GLUTDisplay::restartProgressiveTimer()
{
  // Unless the user has overridden it, progressive implies a finite continuous drawing timeout.
  if(m_app_continuous_mode == CDProgressive && m_progressive_timeout < 0.0 && !m_vca_requested) {
    m_progressive_timeout = 10.0;
  }
}

// This is an API function for the app to specify its desired mode.
void GLUTDisplay::setContinuousMode(contDraw_E continuous_mode)
{
  m_app_continuous_mode = continuous_mode;

  // Unless the user has overridden it, progressive implies a finite continuous drawing timeout.
  restartProgressiveTimer();

  setCurContinuousMode(m_app_continuous_mode);
}

void GLUTDisplay::postRedisplay()
{
//  glutPostRedisplay();
}

void GLUTDisplay::drawText( const std::string& text, float x, float y, void* font )
{
  // Save state
  glPushAttrib( GL_CURRENT_BIT | GL_ENABLE_BIT );

  glDisable( GL_TEXTURE_2D );
  glDisable( GL_LIGHTING );
  glDisable( GL_DEPTH_TEST);

  glColor3fv( &( m_text_shadow_color.x) ); // drop shadow
  // Shift shadow one pixel to the lower right.
  glWindowPos2f(x + 1.0f, y - 1.0f);
  for( std::string::const_iterator it = text.begin(); it != text.end(); ++it )
    glutBitmapCharacter( font, *it );

  glColor3fv( &( m_text_color.x) );        // main text
  glWindowPos2f(x, y);
  for( std::string::const_iterator it = text.begin(); it != text.end(); ++it )
    glutBitmapCharacter( font, *it );

  // Restore state
  glPopAttrib();
}

void GLUTDisplay::runBenchmarkNoDisplay( )
{
  // Set up scene
  SampleScene::InitialCameraData initial_camera_data;
  m_scene->setUsePBOBuffer( false );
  m_scene->initScene( initial_camera_data );


  if( m_initial_window_width > 0 && m_initial_window_height > 0)
    m_scene->resize( m_initial_window_width, m_initial_window_height );

  if ( !m_camera_pose.empty() )
    initial_camera_data = SampleScene::InitialCameraData( m_camera_pose );

  // Initialize camera according to scene params
  m_camera = new PinholeCamera( initial_camera_data.eye,
                               initial_camera_data.lookat,
                               initial_camera_data.up,
                               initial_camera_data.hfov, // hfov is ignored when using keep vertical
                               initial_camera_data.vfov,
                               PinholeCamera::KeepNone );
  m_mouse = new Mouse( m_camera, m_initial_window_width, m_initial_window_height );
  m_mouse->handleResize( m_initial_window_width, m_initial_window_height );

  float3 eye, U, V, W;
  m_camera->getEyeUVW( eye, U, V, W );
  SampleScene::RayGenCameraData camera_data( eye, U, V, W );


  // Initial compilation
  double compilation_time = 0.0f;
  {
    double start_time, finish_time;
    sutilCurrentTime( &start_time );
    m_scene->getContext()->compile();
    sutilCurrentTime( &finish_time );
    compilation_time = finish_time - start_time;
  }

  // Accel build
  double accel_build_time = 0.0f;
  {
    double start_time, finish_time;
    sutilCurrentTime( &start_time );
    m_scene->getContext()->launch( 0, 0 );
    sutilCurrentTime( &finish_time );
    accel_build_time = finish_time - start_time;
  }
  printf( "PREPROCESS: %s | compile %g sec | accelbuild %g sec\n",
          m_title.c_str(),
          compilation_time,
          accel_build_time );
  fflush(stdout);

  
  // Warmup frames
  if ( m_cur_continuous_mode == CDBenchmarkTimed ) {
    // Count elapsed time
    double start_time, finish_time;
    sutilCurrentTime( &start_time );
    m_warmup_frames = 0;
    do
    {
      m_scene->trace( camera_data );
      sutilCurrentTime( &finish_time );
      m_warmup_frames++;
    } while( finish_time-start_time < m_warmup_time);
  } else {
    // Count frames
    for( unsigned int i = 0; i < m_warmup_frames; ++i ) {
      m_scene->trace( camera_data );
    }
  }

  // Timed frames
  double start_time, finish_time;
  sutilCurrentTime( &start_time );
  if ( m_cur_continuous_mode == CDBenchmarkTimed ) {
    // Count elapsed time
    m_timed_frames = 0;
    do
    {
      m_scene->trace( camera_data );
      sutilCurrentTime( &finish_time );
      m_timed_frames++;
    } while( finish_time-start_time < m_benchmark_time );
  } else
  {
    // Count frames
    for( unsigned int i = 0; i < m_timed_frames; ++i ) {
      m_scene->trace( camera_data );
    }
    sutilCurrentTime( &finish_time );
  }

  // Save image if necessary
  if( m_save_frames_to_file ) {
    std::string filename = m_save_frames_basename.empty() ?  m_title + ".ppm" : m_save_frames_basename+ ".ppm"; 
    Buffer buffer = m_scene->getOutputBuffer();
    sutilDisplayFilePPM( filename.c_str(), buffer->get() );
  }

  double total_time = finish_time-start_time;
  sutilPrintBenchmark( m_title.c_str(), total_time, m_warmup_frames, m_timed_frames);
}

void GLUTDisplay::keyPressed(unsigned char key, int x, int y)
{
  try {
    if( m_scene->keyPressed(key, x, y) ) {
      //glutPostRedisplay();
      return;
    }
  } catch( Exception& e ){
    sutilReportError( e.getErrorString().c_str() );
    exit(2);
  }

  switch (key) {
  case 27: // esc
  case 'q':
    quit();
  case 'f':
    if ( m_old_window_width == -1) { // We are in non-fullscreen mode
      m_old_window_width  = glutGet(GLUT_WINDOW_WIDTH);
      m_old_window_height = glutGet(GLUT_WINDOW_HEIGHT);
      m_old_window_x      = glutGet(GLUT_WINDOW_X) + m_old_window_x_offset;
      m_old_window_y      = glutGet(GLUT_WINDOW_Y) + m_old_window_y_offset;
      glutFullScreen();
    } else { // We are in fullscreen mode
      glutPositionWindow( m_old_window_x, m_old_window_y );
      glutReshapeWindow( m_old_window_width, m_old_window_height );
      m_old_window_width = m_old_window_height = -1;
    }
    //glutPostRedisplay();
    break;

  case 'R':
    setProgressiveDrawingTimeout(0.0);
    // Fall through

  case 'r':
    if(m_app_continuous_mode == CDProgressive) {
      if(m_cur_continuous_mode == CDProgressive) {
        setCurContinuousMode(CDNone);
      } else if(m_cur_continuous_mode == CDNone) {
        setCurContinuousMode(CDProgressive);
      }
      break;
    }
    if(m_app_continuous_mode == CDAnimated) {
      if(m_cur_continuous_mode == CDAnimated) {
        setCurContinuousMode(CDNone);
      } else if(m_cur_continuous_mode == CDNone) {
        setCurContinuousMode(CDAnimated);
      }
      break;
    }
    // Fall through to benchmark mode if the app hasn't specified a kind of continuous to do

  case 'b':
    if(m_cur_continuous_mode == CDBenchmarkTimed) {
      // Turn off the benchmark and print the results
      double current_time;
      sutilCurrentTime(&current_time);
      double total_time = current_time-m_benchmark_frame_time;
      sutilPrintBenchmark(m_title.c_str(), total_time, m_warmup_frames, m_timed_frames);
      setCurContinuousMode(m_app_continuous_mode);
    } else {
      // Turn on the benchmark and set continuous rendering
      std::cerr << "Benchmark started. Press 'b' again to end.\n";
      setCurContinuousMode(CDBenchmarkTimed);
      m_benchmark_time = 1e37f; // Do a timed benchmark, but forever.
      m_benchmark_frame_start = m_frame_count;

      double current_time;
      sutilCurrentTime(&current_time);
      m_warmup_start = current_time;
      m_benchmark_frame_time = current_time;
      m_warmup_frames = 0;
      m_warmup_time = 0;
      m_timed_frames = 0;
    }
    break;

  case 'd':
    m_display_fps = !m_display_fps;
    break;

  case 'P':
    printf("Pixel pos:%d,%d\n", x, y );
    break;

  case 's':
    sutilDisplayFilePPM( "out.ppm", m_scene->getOutputBuffer()->get() );
    break;

  case 'm':
    m_print_mem_usage =  !m_print_mem_usage;
    //glutPostRedisplay();
    break;

  case 'c':
    float3 eye, lookat, up;
    float hfov, vfov;

    m_camera->getEyeLookUpFOV(eye, lookat, up, hfov, vfov);
    std::cerr << '"' << eye << lookat << up << vfov << '"' << std::endl;
    break;

    case '[':
      m_scene->incrementCPUThreads(-1);
      break;
      
    case ']':
      m_scene->incrementCPUThreads(1);
      break;



  default:
    return;
  }
}


void GLUTDisplay::specialKeyPressed(int key, int x, int y)
{
  // Reset progressive counter
  sutilCurrentTime( &m_start_time );

  m_special_key = key;
  sutilCurrentTime( &m_special_key_prev_camera_update_time );
  moveCameraWithKey();
  //glutPostRedisplay();
  // Repeat action in idle() until key is released

}


void GLUTDisplay::specialKeyUp(int key, int x, int y)
{
  m_special_key = -1;
  m_special_key_prev_camera_update_time = -1;
  //glutIdleFunc( m_cur_continuous_mode!=CDNone ? idle : 0 );
}


void GLUTDisplay::mouseButton(int button, int state, int x, int y)
{
  sutilCurrentTime( &m_start_time );
  m_mouse->handleMouseFunc( button, state, x, y, glutGetModifiers() );
  if ( state != GLUT_UP )
    m_scene->signalCameraChanged();
  //glutPostRedisplay();
}


void GLUTDisplay::mouseMotion(int x, int y)
{
  sutilCurrentTime( &m_start_time );
  m_mouse->handleMoveFunc( x, y );
  m_scene->signalCameraChanged();
  if (m_app_continuous_mode == CDProgressive) {
    setCurContinuousMode(CDProgressive);
  }
  //glutPostRedisplay();
}


void GLUTDisplay::resize(int width, int height)
{
  // disallow size 0
  width  = max(1, width);
  height = max(1, height);

  sutilCurrentTime( &m_start_time );
  m_scene->signalCameraChanged();
  m_mouse->handleResize( width, height );

  try {
    m_scene->resize(width, height);
  } catch( Exception& e ){
    sutilReportError( e.getErrorString().c_str() );
    exit(2);
  }

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, 1, 0, 1, -1, 1);
  glViewport(0, 0, width, height);
  if (m_app_continuous_mode == CDProgressive) {
    setCurContinuousMode(CDProgressive);
  }
  //glutPostRedisplay();
}


void GLUTDisplay::moveCameraWithKey()
{
  assert(m_special_key >= 0);

  double current_time = -1;
  sutilCurrentTime( &current_time );
  const double delta_time = std::max(0.0, current_time - m_special_key_prev_camera_update_time);

  // Make speed depend on length of look vector, which is usually tied to scene scale.
  const float3 lookvec = m_camera->getLookVector();
  const float translation = float(m_special_key_camera_speed * length(lookvec) * delta_time);
  switch (m_special_key) {

    case GLUT_KEY_UP:
      m_camera->translate(make_float3(0.0f, 0.0f, translation));
      break;

    case GLUT_KEY_DOWN:
      m_camera->translate(make_float3(0.0f, 0.0f, -translation));
      break;

    case GLUT_KEY_RIGHT:
      m_camera->translate(make_float2(translation, 0.0f));
      break;

    case GLUT_KEY_LEFT:
      m_camera->translate(make_float2(-translation, 0.0f));
      break;

    case GLUT_KEY_PAGE_UP:
      m_camera->translate(make_float3(0.0f, translation, 0.0f));
      break;

    case GLUT_KEY_PAGE_DOWN:
      m_camera->translate(make_float3(0.0f, -translation, 0.0f));
      break;

    default:
      return;
  }

  m_special_key_prev_camera_update_time = current_time;
  m_scene->signalCameraChanged();

}

void GLUTDisplay::idle()
{
  if (m_special_key >= 0) {
    moveCameraWithKey();

    // Reset progressive counter
    sutilCurrentTime( &m_start_time );

  }

//  glutPostRedisplay();
}

void GLUTDisplay::displayFrame()
{
  GLboolean sRGB = GL_FALSE;
  if (m_use_sRGB && m_sRGB_supported) {
    glGetBooleanv( GL_FRAMEBUFFER_SRGB_CAPABLE_EXT, &sRGB );
    if (sRGB) {
      glEnable(GL_FRAMEBUFFER_SRGB_EXT);
    }
  }

  // Draw the resulting image
 
  const int buffer_width = m_scene->getImageWidth();
  const int buffer_height = m_scene->getImageHeight();
  RTformat buffer_format = RT_FORMAT_UNKNOWN;
  RTsize elementSize = 0;
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable( GL_BLEND );
  glClearColor(0.0,0.0,0.0,0.0);
  glClear(GL_COLOR_BUFFER_BIT);
  m_scene->updateTexture();
  // Check for stream buffer first, for VCA
  bool is_stream_buffer = false;
  Buffer buffer = m_scene->getProgressiveStreamBuffer();
  if (buffer) {

    // From Programming Guide:
    // Stream buffers must be of RT_FORMAT_UNSIGNED_BYTE4 format
    buffer_format = RT_FORMAT_UNSIGNED_BYTE4;
    elementSize = 4;
    is_stream_buffer = true;

  } else {

    buffer = m_scene->getOutputBuffer(); 
    buffer_format = buffer->getFormat();
    elementSize = buffer->getElementSize();

    if( m_save_frames_to_file ) {
      static char fname[128];
      std::string basename = m_save_frames_basename.empty() ? "frame" : m_save_frames_basename;
      sprintf(fname, "%s_%05d.ppm", basename.c_str(), m_frame_count);
      sutilDisplayFilePPM( fname, buffer->get() );
    }
  }

  if (!(buffer && buffer_width > 0 && buffer_height > 0)) {
    fprintf(stderr, "Scene did not return a valid buffer to display.\n");
    exit(2);
  }

  unsigned int pboId = 0;
  if( m_scene->usesPBOBuffer() )
    pboId = buffer->getGLBOId();

  if (pboId)
  {
    if (!m_texId)
    {
      glGenTextures( 1, &m_texId );
      glBindTexture( GL_TEXTURE_2D, m_texId);

      // Change these to GL_LINEAR for super- or sub-sampling
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

      // GL_CLAMP_TO_EDGE for linear filtering, not relevant for nearest.
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

      glBindTexture( GL_TEXTURE_2D, 0);
    }

    glBindTexture( GL_TEXTURE_2D, m_texId );

    // send PBO to texture
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pboId);

    RTsize elementSize = buffer->getElementSize();
    if      ((elementSize % 8) == 0) glPixelStorei(GL_UNPACK_ALIGNMENT, 8);
    else if ((elementSize % 4) == 0) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    else if ((elementSize % 2) == 0) glPixelStorei(GL_UNPACK_ALIGNMENT, 2);
    else                             glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    {
      nvtx::ScopedRange r( "glTexImage" );
      if(buffer_format == RT_FORMAT_UNSIGNED_BYTE4) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, buffer_width, buffer_height, 0, GL_BGRA, GL_UNSIGNED_BYTE, 0);
      } else if(buffer_format == RT_FORMAT_FLOAT4) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F_ARB, buffer_width, buffer_height, 0, GL_RGBA, GL_FLOAT, 0);
      } else if(buffer_format == RT_FORMAT_FLOAT3) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F_ARB, buffer_width, buffer_height, 0, GL_RGB, GL_FLOAT, 0);
      } else if(buffer_format == RT_FORMAT_FLOAT) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE32F_ARB, buffer_width, buffer_height, 0, GL_LUMINANCE, GL_FLOAT, 0);
      } else {
        assert(0 && "Unknown buffer format");
      }
    }
    glBindBuffer( GL_PIXEL_UNPACK_BUFFER, 0 );

    glEnable(GL_TEXTURE_2D);

    // Initialize offsets to pixel center sampling.

    float u = 0.5f/buffer_width;
    float v = 0.5f/buffer_height;
	glBegin(GL_QUADS);
    glTexCoord2f(u, v);
    glVertex2f(0.0f, 0.0f);
    glTexCoord2f(1.0f, v);
    glVertex2f(1.0f, 0.0f);
    glTexCoord2f(1.0f - u, 1.0f - v);
    glVertex2f(1.0f, 1.0f);
    glTexCoord2f(u, 1.0f - v);
    glVertex2f(0.0f, 1.0f);
    glEnd();
    glDisable(GL_TEXTURE_2D);
	m_scene->getFramePixels();
    glBegin(GL_QUADS);
    glTexCoord2f(u, v);
    glVertex2f(0.0f, 0.0f);
    glTexCoord2f(1.0f, v);
    glVertex2f(1.0f, 0.0f);
    glTexCoord2f(1.0f - u, 1.0f - v);
    glVertex2f(1.0f, 1.0f);
    glTexCoord2f(u, 1.0f - v);
    glVertex2f(0.0f, 1.0f);
    glEnd();
    glDisable(GL_TEXTURE_2D);
  } else
   {
    GLvoid* imageData = buffer->map();
    assert( imageData );

    GLenum gl_data_type = GL_FALSE;
    GLenum gl_format = GL_FALSE;

    switch (buffer_format) {
          case RT_FORMAT_UNSIGNED_BYTE4:
            gl_data_type = GL_UNSIGNED_BYTE;
            gl_format    = is_stream_buffer ? GL_RGBA : GL_BGRA;
            break;

          case RT_FORMAT_FLOAT:
            gl_data_type = GL_FLOAT;
            gl_format    = GL_LUMINANCE;
            break;

          case RT_FORMAT_FLOAT3:
            gl_data_type = GL_FLOAT;
            gl_format    = GL_RGB;
            break;

          case RT_FORMAT_FLOAT4:
            gl_data_type = GL_FLOAT;
            gl_format    = GL_RGBA;
            break;

          default:
            fprintf(stderr, "Unrecognized buffer data type or format.\n");
            exit(2);
            break;
    }

    int align = 1;
    if      ((elementSize % 8) == 0) align = 8; 
    else if ((elementSize % 4) == 0) align = 4;
    else if ((elementSize % 2) == 0) align = 2;
    glPixelStorei(GL_UNPACK_ALIGNMENT, align);

    NVTX_RangePushA("glDrawPixels");
    glDrawPixels( static_cast<GLsizei>( buffer_width ), static_cast<GLsizei>( buffer_height ),
      gl_format, gl_data_type, imageData);
    NVTX_RangePop();

    buffer->unmap();
  }
  if (m_use_sRGB && m_sRGB_supported && sRGB) {
    glDisable(GL_FRAMEBUFFER_SRGB_EXT);
  }
}
double LinearInterpolate(
   double y1,double y2,
   double mu)
{
   return(y1*(1-mu)+y2*mu);
}
double CosineInterpolate(
   double y1,double y2,
   double mu)
{
   double mu2;

   mu2 = (1-cos(mu*3.14195))/2;
   return(y1*(1-mu2)+y2*mu2);
}

double Aperture= 0;
double Focus = 0;
double VarAt = 5.0;
bool isNew;
double timer;
int count = 0;

SmoothSequence blah;
SmoothSequence blah2;
float timeDifference = 0;
float f = 0;
float frameDifference;
int oldTimeSinceStart = 0;

//static const unsigned WIDTH  = 2560u;//1024u;//2048u;//1024u;
//static const unsigned HEIGHT = 1600u;//768u;//768u*2u;//768u; //TEST
void GLUTDisplay::display()
{
//  glViewport(0, 0, WIDTH, HEIGHT);
//  std::cout << count++ << std::endl;
  if( m_cur_continuous_mode == CDProgressive && m_progressive_timeout > 0.0 ) {
    // If doing progressive refinement, see if we're done
    double current_time;
    sutilCurrentTime( &current_time );
    if( current_time - m_start_time > m_progressive_timeout ) {
      setCurContinuousMode( CDNone );
      return;
    }
  }

  bool display_requested = true;

  try {			
		  if(m_scene->isNewMeasurement){
			  m_scene->isNewMeasurement = false;
			  Aperture = m_scene->curSceneApertureSize;
			m_scene->glutAperture = Aperture;
			timeDifference = 0;
		  }
		 int timeSinceStart = glutGet(GLUT_ELAPSED_TIME);
         float deltaTime = (timeSinceStart - oldTimeSinceStart) * 0.001;
		 oldTimeSinceStart = timeSinceStart;

		timeDifference += 1 * deltaTime;
		if(timeDifference > 1)timeDifference = 1;
		if(m_scene->useEnhancedAperture)
		f = LinearInterpolate(f,m_scene->curSceneFocusDepth, timeDifference/4);//blah.smooth(fx,current_time2);
		else
      f = m_scene->curSceneFocusDepth;//HACK!
		if(f < 0.05f)f=0.05f;if(f > 1000.0)f=1.0; //HACK another fixed focus distance!!
		m_scene->signalCameraChanged();

		m_scene->glutFocus = f;

		float3 eye, U, V, W;
		m_camera->getEyeUVW( eye, U, V, W );
		SampleScene::RayGenCameraData camera_data( eye, U, V, W );
		{nvtx::ScopedRange r( "trace" );
			optix::Context temp = m_scene->getContext();
			float3 nn, up;
			float nnf;
			m_camera->getEyeLookUpFOV(nn, nn, up, nnf, nnf);
			temp["upVec"]->setFloat(up);
				m_scene->trace( camera_data );
		}

		  
		double current_time;
		frameThisSecond++;
		sutilCurrentTime( &current_time );
		frameDifference = current_time;
		double dt = current_time - m_last_frame_time;
		if( dt > 1){
			//std::cout << "tick update" << std::endl;
			m_last_frame_time = current_time;
			m_last_frame_count = m_frame_count;
			float amount =   (float)m_scene->RayFrameTotal;			
			m_scene->RayFrameTotal = 0;
			frameThisSecond = 0;
//			std::cout << "RaysPerSecond: " << amount  <<std::endl;
		}
		// Always count rendered frames
		++m_frame_count;

		// Not strictly necessary


		glClearColor(0.0, 1.0, 0.0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT);
		if( m_display_frames ) {
			nvtx::ScopedRange r( "displayFrame" );
			displayFrame();
		}
		sutilCurrentTime( &current_time );
		frameDifference = current_time;
	} catch( Exception& e ){
		sutilReportError( e.getErrorString().c_str() );
		exit(2);
	}


  // Do not draw text on 1st frame -- issue on linux causes problems with 
  // glDrawPixels call if drawText glutBitmapCharacter is called on first frame.
  if ( m_display_fps && m_cur_continuous_mode != CDNone && m_frame_count > 1 ) {
    // Output fps 
    double current_time;
    sutilCurrentTime( &current_time );
    double dt = current_time - m_last_frame_time;
    if( dt > m_fps_update_threshold ) {
      sprintf( m_fps_text, "fps: %7.2f", (m_frame_count - m_last_frame_count) / dt );

      m_last_frame_time = current_time;
      m_last_frame_count = m_frame_count;
    } else if( m_frame_count == 1 ) {
      sprintf( m_fps_text, "fps: %7.2f", 0.f );
    }

    drawText( m_fps_text, 10.0f, 10.0f, GLUT_BITMAP_8_BY_13 );
  }

  if( m_print_mem_usage ) {
    // Output memory
    std::ostringstream str;
    DeviceMemoryLogger::logCurrentMemoryUsage(m_scene->getContext(), str);
    drawText( str.str(), 10.0f, 26.0f, GLUT_BITMAP_8_BY_13 );
  }

  //printf("finished frame: %d\n", m_frame_count);
  if( display_requested && (m_cur_continuous_mode == CDBenchmark || m_cur_continuous_mode == CDBenchmarkTimed) ) {
    double current_time;
    sutilCurrentTime(&current_time);

    // Do the timed frames first, since m_benchmark_frame_time may be set by the warmup
    // section below and we don't want to double count the frames.
    if ( m_cur_continuous_mode == CDBenchmarkTimed ) {
      // Count elapsed time, but only if we have moved out of the warmup phase.
      if (m_benchmark_frame_time > 0) {
        m_timed_frames++;
        //printf("_timed_frames = %d\n", m_timed_frames);
        double total_time = current_time-m_benchmark_frame_time;
        if ( total_time > m_benchmark_time ) {
          sutilPrintBenchmark(m_title.c_str(), total_time, m_warmup_frames, m_timed_frames);
          setCurContinuousMode( CDNone );
          quit(0); // We only get here for command line benchmarks, which always end.
        }
      }
    } else {
      // Count frames
      if(m_frame_count == m_benchmark_frame_start+m_warmup_frames+m_timed_frames) {
        double total_time = current_time-m_benchmark_frame_time;
        sutilPrintBenchmark(m_title.c_str(), total_time, m_warmup_frames, m_timed_frames);
        setCurContinuousMode( CDNone );
        quit(0); // We only get here for command line benchmarks, which always end.
      }
    }

    if ( m_cur_continuous_mode == CDBenchmarkTimed ) {
      if ( current_time-m_warmup_start < m_warmup_time) {
        // Under the alloted time, keep counting
        m_warmup_frames++;
        //printf("_warmup_frames = %d\n", m_warmup_frames);
      } else {
        // Over the alloted time, set the m_benchmark_frame_time if it hasn't been set yet.
        if (m_benchmark_frame_time == 0) {
          m_benchmark_frame_time = current_time;
          // Make sure we account for that last frame
          m_warmup_frames++;
          //printf("warmup done (m_warmup_frames = %d) after %g seconds.\n", m_warmup_frames, current_time-m_warmup_start);
        }
      }
    } else {
      // Count frames
      if(m_frame_count-1 == m_benchmark_frame_start+m_warmup_frames) {
        m_benchmark_frame_time = current_time; // Start timing.
      }
    }
  }

  if ( display_requested && m_display_frames ) {
    nvtx::ScopedRange r( "glutSwapBuffers" );
    // Swap buffers
    glutSwapBuffers();
  }
  glutPostRedisplay();
}

void GLUTDisplay::quit(int return_code)
{
  try {
    if(m_scene)
    {
	
      m_scene->cleanUp();
	  std::cout << "Called Cleanup" << std::endl;
      if (m_scene->getContext().get() != 0)
      {
        sutilReportError( "Derived scene class failed to call SampleScene::cleanUp()" );
        exit(2);
      }
    }
	std::cout << "Cleaned up scene" << std::endl;
//    exit(return_code);
//      glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
	  glutDestroyWindow ( Winid);
	  exit(0);
  } catch( Exception& e ) {
    sutilReportError( e.getErrorString().c_str() );
    exit(2);
  }
}
