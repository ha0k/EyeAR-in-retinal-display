/*
processes of the cook scene
*/

#pragma once

//opengl
#ifdef __APPLE__
#  include <OpenGL/gl.h>
#else
#  include <GL/glew.h>
#  include <GL/gl.h>
#  include <GL/glut.h>
#endif
#include <GLUTDisplay.h>


#include <SampleScene.h>
#include "scenemanagment.h"
#include "sharpview.h"

//optix
#include <OptixMesh.h>
#include <optixu/optixpp_namespace.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
#include "radical_inverse_jitter.h"
#include "commonStructs.h"

#include <string>
#include <map>
#include <iterator>

//#include "ExperimentImageLoader.hpp"


class EyeARScene : public SampleScene
{
public:

  struct Parameters
  {
    float lookfrom[3];
    float lookat[3];
    float distance_offset; // How much to offset the focal distance.  It can be
                           // negative, but the value will be clamped to the
                           // eye point minus the scene epsilon.
    float aperture;
    Parameters():
    distance_offset(0),
    aperture(0)
    {
      std::memset(lookfrom, 0, sizeof(lookfrom));
      std::memset(lookat, 0, sizeof(lookat));
    }
    Parameters(float lookfrom_[3], float lookat_[3], float distance_offset_, float aperture_):
    distance_offset(distance_offset_),
    aperture(aperture_)
    {
      //C++11
      //std::copy(std::begin(lookfrom_), std::end(lookfrom_), std::begin(lookfrom));
      //std::copy(std::begin(lookat_), std::end(lookat_), std::begin(lookat));
      std::copy(lookfrom_, lookfrom_+3, lookfrom);
      std::copy(lookat_, lookat_+3, lookat);
    }
  };

  EyeARScene():
    useScreenSample(1),
    visualizeScreen(0),
    doInit(false),
    doSharpView(false),
    doOnce(true),
    doTexture(true),
    loadOnce(true),
    useImage(true),
    pausePlayback(false),
    focusOnCollectionPlane(false),
    firstCreate(true),
    snInverse(0.05),
    //moveX(-0.13f),
    moveX(0.0f),
    moveY(0.0f),
    distanceToScreen(0.4f),
    switchAt(0),
    texId(0),
    frame(0),
    sequence(1),
    generateDatabase(false)
  {
    float lookfrom_[] = {5.0f,  5.0f, 13.7f};
    float lookat_[] = {5.0f, 5.01f,  0.0f};

    parameters[0] = Parameters(lookfrom_, lookat_,  0.0f, 0.100f );
    float lookfrom2_[]  = {5.0f, -3.0f,  5.0f};
    float lookat2_[] = {5.0f, 5.10f,  0.0f};
    parameters[1] = Parameters(lookfrom2_, lookat2_,  -1.5f, 0.150f );
    
    sharpview = new SharpView(0);

    float focusDistance = 0.25f;
    while (focusDistance < 0.55f)
    {
      focusDistances.push_back(focusDistance);
      focusDistance+=0.01f;
    }
    float pupilDiameter = 1.90f;
    while (pupilDiameter < 8.15f)
    {
      appertureSizes.push_back(pupilDiameter);
      pupilDiameter+=0.1f;
    }

  
  displayDistances.push_back(0.35f);
  curFocusDistance = 0;
  curAppertureSize = 0;
  curDisplayDistance = 0;

  //variable added on 23/2/2017
  createMask = false;
  applyMask = true;
  offsetValue = 12;
  };

  EyeARScene( const std::string& texture_path_, bool use_float3_output_buffer ):
    texture_path( texture_path_ ),
    output_buffer_format( use_float3_output_buffer ? RT_FORMAT_FLOAT3 : RT_FORMAT_FLOAT4 ),
    useScreenSample(1),
    visualizeScreen(0),
    doInit(false),
    doSharpView(true),
    doOnce(true),
    doTexture(true),
    loadOnce(true),
    useImage(true),
    pausePlayback(false),
    focusOnCollectionPlane(false),
    firstCreate(true),
    snInverse(0.05),
    //moveX(-0.13f),
    moveX(0.0f),
    moveY(0.0f),
    distanceToScreen(0.4f), 
    switchAt(0),
    texId(0),
    frame(0),
    sequence(1),
    m_old_window_width(-1),
    generateDatabase(false)
    //39, 49, 59cm
  {
    float lookfrom_[] = {5.0f,  5.0f, 13.7f};
    float lookat_[] = {5.0f, 5.01f,  0.0f};

    parameters[0] = Parameters(lookfrom_, lookat_,  0.0f, 0.100f );
    float lookfrom2_[]  = {5.0f, -3.0f,  5.0f};
    float lookat2_[] = {5.0f, 5.10f,  0.0f};
    parameters[1] = Parameters(lookfrom2_, lookat2_,  -1.5f, 0.150f );
    
    sharpview = new SharpView(0);

    float focusDistance = 0.05f;
    focusDistances.push_back(focusDistance);
    while (focusDistance < 2.f)
    {
      focusDistances.push_back(focusDistance);
      focusDistance+=0.05f;
    }
    float pupilDiameter = 6.00f;
    appertureSizes.push_back(pupilDiameter);
  
  displayDistances.push_back(0.3f);
  curFocusDistance = 0;
  curAppertureSize = 0;
  curDisplayDistance = 0;
  
  
  //variable added on 23/2/2017
  createMask = false;
  applyMask = true;
  offsetValue = 12;
  }

  void createLighting();
  virtual void initScene( InitialCameraData& camera_data );
  virtual void trace( const RayGenCameraData& camera_data);
  virtual void trace( const RayGenCameraData& camera_data, bool& disp );
  virtual bool keyPressed(unsigned char key, int x, int y);
  virtual void cleanUp();
  virtual optix::Buffer getOutputBuffer();
  float apertureSize;
  float focusDepth;
  float focalLength;
  cv::Mat textureMat;
  cv::Mat imageMat;
  cv::Mat resultMat;
  cv::Mat crossTexture;
  cv::Mat checkerboardTexture;
  void createMasks(const cv::Mat &image);
  void getFramePixels();
  void loadCheckerboard();

  bool doPausePlayback()
  {
    return pausePlayback;
  }

private:
  //private functions
  void runSharpView(const cv::Mat &input, cv::Mat &output);
  
  void createGeometry();

  optix::Buffer m_light_buffer;
  optix::Buffer m_light_buffer_data;
  Parameters     parameters[2];
  std::map<std::string, std::string> m_configs;
  std::string           texture_path;
  RTformat              output_buffer_format;


  //a bunch of variables...
  int fovH,fovV; //field of view horizontal and vertical
  int useScreenSample;
  int visualizeScreen;

  bool doInit;
  bool doSharpView;
  bool doOnce;
  bool doTexture;
  bool pausePlayback;
  
  bool loadOnce;
  bool useImage;
  bool focusOnCollectionPlane;
  bool firstCreate;

  float moveX;
  float moveY;
  float distanceToScreen;
  int switchAt;

  SharpView *sharpview;
  float snInverse;

  GLuint texId;
 

  unsigned sequence;
  unsigned frame;

  //HACK: should this value be set by the user? Should the user maybe just set the parameters in a function call instead of defining them?
  static const int setup = 1;


  //fullscreen
  int m_old_window_width;
  int m_old_window_height;
  int m_old_window_x;
  int m_old_window_y;

  bool generateDatabase;

  std::vector<float> focusDistances;
  std::vector<float> appertureSizes;
  std::vector<float> displayDistances;
  unsigned int curFocusDistance, curAppertureSize, curDisplayDistance;
  unsigned int counter;


  //variable added on 23/2/2017
  bool createMask, applyMask;
  double lastApertureSize;
  
  int offsetValue;


/*
  ImageLoader *loader;*/
};

