#include "eyearscene.h"



void EyeARScene::trace( const RayGenCameraData& camera_data )
{
  bool disp;
  trace(camera_data, disp);
}

void EyeARScene::trace( const RayGenCameraData& camera_data, bool& disp )
{
  std::vector<int> enabled_devices = m_context->getEnabledDevices();
  m_context->setDevices(enabled_devices.begin(), enabled_devices.begin()+1);
  m_context["useScreenSample"]->setInt(useScreenSample);
  m_context["visualizeScreen"]->setInt(visualizeScreen);
  m_context["eye"]->setFloat( camera_data.eye );
  m_context["U"]->setFloat( camera_data.U ); 
  m_context["V"]->setFloat( camera_data.V );
  m_context["W"]->setFloat( camera_data.W );
  float focal_distance = optix::length(camera_data.W) + parameters[setup].distance_offset;
  focal_distance = optix::fmaxf(focal_distance, m_context["scene_epsilon"]->getFloat());
  float focal_scale = focal_distance / optix::length(camera_data.W);
  m_context["focal_scale"]->setFloat( focal_scale );
  m_context["sampleSize"]->setUint(sampleSize);
  optix::Buffer buffer = getOutputBuffer();
  RTsize buffer_width, buffer_height;
  buffer->getSize( buffer_width, buffer_height );
  buffer->destroy();
  m_context["screen_sizeX"]->setUint(static_cast<unsigned int>(buffer_width));
  m_context["screen_sizeY"]->setUint(static_cast<unsigned int>(buffer_height));

  optix::Group mainGScreen = m_context->createGroup();
  mainGScreen->setChildCount(0);
//  if(!hasAlreadyDone){
	//  hasAlreadyDone=true;
    if (output_buffer_format == RT_FORMAT_FLOAT4 ) {
    m_context["output_buffer_f4"]->setBuffer( createOutputBuffer(RT_FORMAT_FLOAT4, WIDTH, HEIGHT));
    m_context["output_buffer_f3"]->setBuffer( m_context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_FLOAT3, 1, 1));
  }
  else {
    m_context["output_buffer_f3"]->setBuffer( createOutputBuffer(RT_FORMAT_FLOAT3, WIDTH, HEIGHT));
    m_context["output_buffer_f4"]->setBuffer( m_context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_FLOAT4, 1, 1));
  }
 // }
//    //Group mainGTestObject = m_context->createGroup();
  //mainGTestObject->setChildCount(0);
  float w = WIDTH;
  float h = HEIGHT;
  float ratio = w/h;
  //Here we set the scene files
  resetPlane(m_context->getContext(),mainGScreen,m_accel_desc,ratio,fovV,glutFocus);
  mainGScreen->setAcceleration(m_context->createAcceleration(m_accel_desc.builder.c_str(), m_accel_desc.traverser.c_str()));
  m_context["screen_object"]->set(mainGScreen);
  float3 mm = camera_data.W * focusDepth;
  focal_distance = optix::length(mm);
  focal_distance = optix::fmaxf(focal_distance, m_context["scene_epsilon"]->getFloat());
  focal_scale = focal_distance / length(camera_data.W);
  m_context["focal_scale"]->setFloat( focal_scale );
  if(!useEnhancedAperture){
	m_context["aperture_radius"]->setFloat((glutAperture)/1000.0f*2);
  }else{
	m_context["aperture_radius"]->setFloat((glutAperture)/1000.0f*8);
  }
  if(focusOnCollectionPlane){
	  m_context["focal_scale"]->setFloat(distanceToScreen);
  }else{
  float ff = glutFocus;
	  m_context["focal_scale"]->setFloat(ff);
  }

  if( m_camera_changed) {
    srand(1984);
    frame = 0;
    sequence = 1;
    m_camera_changed = false;
	  isNewMeasurement = false;
	  for(int i = 0; i < sampleSize; i++){
	   m_context["frame_number"]->setUint( frame++ );
		  disp = (frame >= 33) || !((frame - 1) & 0x3); // Display frames 1, 5, 9, ... and 33+
		  optix::float4 jitter = radical_inverse_jitter( sequence );

	//	#endif

	   m_context["jitter"]->setFloat( jitter );
	//  if(disp){
	   m_context->launch( 0,
						static_cast<unsigned int>(buffer_width),
						static_cast<unsigned int>(buffer_height)
						);
		//}
	  }
  }
}

void EyeARScene::createGeometry()
{
  std::cerr << "Initializing scene ...";
  std::cerr.flush();
  optix::Group mainG = m_context->createGroup();
  mainG->setChildCount(0);
  optix::Group mainGScreen = m_context->createGroup();
  mainGScreen->setChildCount(0);
  float w = WIDTH;
  float h = HEIGHT;
  float ratio = w/h;
  //Here we set the scene files
  setSceneFromFile(GLUTDisplay::getSceneFileName().c_str(),mainGScreen, mainG, m_context->getContext(), m_configs["obj_folder"].c_str(), m_configs["material_folder"].c_str(),m_accel_desc,ratio,fovV,0.9f);
  mainG->setAcceleration(m_context->createAcceleration(m_accel_desc.builder.c_str(), m_accel_desc.traverser.c_str()));
  mainGScreen->setAcceleration(m_context->createAcceleration(m_accel_desc.builder.c_str(), m_accel_desc.traverser.c_str()));
  optix::Group mainGTestObject = m_context->createGroup();
  mainGTestObject->setChildCount(0);
  //Here we set the scene files
  resetPlane(m_context->getContext(),mainGTestObject,m_accel_desc,ratio,fovV,0.5f);
  mainGTestObject->setAcceleration(m_context->createAcceleration(m_accel_desc.builder.c_str(), m_accel_desc.traverser.c_str()));
  
  m_context["top_object"]->set(mainG);
  m_context["top_shadower"]->set(mainG);
  m_context["screen_object"]->set(mainGScreen);


  m_context["screen_object"]->set(mainGScreen);
  createLighting();

  std::cerr << "finished." << std::endl;   
}

void EyeARScene::loadCheckerboard(){
	  checkerboardTexture = cv::imread("checkerboard.jpg",1);
	  //GLuint tempTex2D;
	  
	  unsigned int TEX_HEIGHT= 256;
	  unsigned int TEX_WIDTH = 256;


	  optix::Buffer buffer = m_context->createBuffer( RT_BUFFER_INPUT, RT_FORMAT_FLOAT, TEX_HEIGHT, TEX_WIDTH );
		float* buffer_data = static_cast<float*>( buffer->map() );
		std::cout << "Doing things" << std::endl;
		unsigned int imgAt = 0;
		std::cout << "Start to do things" << std::endl;
		for(unsigned int j = 0; j < TEX_HEIGHT; j++) {
			for(unsigned int i = 0; i < TEX_WIDTH; i++) {
				float c =((( i & 0x8) == 0) ^(( j & 0x8) == 0)) * 1.0f;
				buffer_data[0] = c;
				buffer_data++;
			}
		}
		
		std::cout << "Done setting things" << std::endl;
		buffer->unmap();
	  m_context["texture_map"]->setBuffer(buffer);
		std::cout << "Finished setting things" << std::endl;
}

void EyeARScene::initScene( InitialCameraData& camera_data )
{
  //loader = new ImageLoader();
	m_configs = getConfig(GLUTDisplay::getVarFileName().c_str());
	camera_data = InitialCameraData( string2float3(m_configs["camera_data_eye"]), // eye
									string2float3(m_configs["camera_data_lookat"]), // lookat
									string2float3(m_configs["camera_data_up"]), // up
									atof(m_configs["camera_data_vfov"].c_str()),//vfov
									atof(m_configs["camera_data_hfov"].c_str()) );                          // hfov
  fovV = atof(m_configs["camera_data_vfov"].c_str());
  fovH = atof(m_configs["camera_data_hfov"].c_str());

  isNewFOV = true;
 // FOVVerMax = 0.53;
//  FOVHorMax = atof(m_configs["camera_data_hfov"].c_str());
  xQuadPos = -0.0;
  yQuadPos = 0.00;
  xAdjustWindowOffset = -0.0;
  yAdjustWindowOffset = 0;//-0.218;
  xWindowOffset = 0.0;
  xAdjustWindowOffset = 0.0;
  curSceneApertureSize = 4.f;  //arp //this is the pupil size
  curSceneFocusDepth = 0.05f;    //arp //42, 52, 62cm; 31, 41, 51  //we set the focus depth here!
    isNewMeasurement = true;

  //arp force create the masks every time the program starts
  //create mask every time the application runs.
  //keyPressed('1',0,0);
  
  // Setup state
  m_context->setRayTypeCount( 2 );
  m_context->setEntryPointCount( 1 );
  m_context->setStackSize( 4440 );
  m_context->setPrintEnabled(true);

  std::cout << "Devices available " << m_context->getDeviceCount() << std::endl;
  std::cout << "Devices using " << m_context->getEnabledDeviceCount() << std::endl;
  // Setup output buffer
  sampleSize = atof(m_configs["sampleSize"].c_str());
  apertureSize = atof(m_configs["aperatureSize"].c_str());
  focalLength = atof(m_configs["focalLength"].c_str());
  focusDepth = atof(m_configs["focusDepth"].c_str());
  loadCheckerboard();

  // Use float3 for testing zero-copy vs. LinearPartitioned.
  m_context["output_format"]->setInt( output_buffer_format );
  if(!doInit){
	  doInit = true;
    if (output_buffer_format == RT_FORMAT_FLOAT4 ) {
      m_context["output_buffer_f4"]->setBuffer( createOutputBuffer(RT_FORMAT_FLOAT4, WIDTH, HEIGHT));
      m_context["output_buffer_f3"]->setBuffer( m_context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_FLOAT3, 1, 1));
    }
    else {
      m_context["output_buffer_f3"]->setBuffer( createOutputBuffer(RT_FORMAT_FLOAT3, WIDTH, HEIGHT));
      m_context["output_buffer_f4"]->setBuffer( m_context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_FLOAT4, 1, 1));
    }
  }
  // Declare camera variables.  The values do not matter, they will be overwritten in trace.
  m_context["eye"]->setFloat( optix::make_float3( 0.0f, 0.0f, 0.0f ) );
  m_context["U"]->setFloat( optix::make_float3( 0.0f, 0.0f, 0.0f ) );
  m_context["V"]->setFloat( optix::make_float3( 0.0f, 0.0f, 0.0f ) );
  m_context["W"]->setFloat( optix::make_float3( 0.0f, 0.0f, 0.0f ) );

  // Context variables
  m_context["radiance_ray_type"]->setUint(0);
  m_context["shadow_ray_type"]->setUint(1);
  m_context["bad_color"]->setFloat( 0.0f, 1.0f, 0.0f );
  m_context["bg_color"]->setFloat( optix::make_float3( 0.f, 0.f, 0.f ) );
  m_context["scene_epsilon"]->setFloat( 1.e-2f );
  m_context["max_depth"]->setInt(5);

  // Camera parameters
  m_context["focal_scale"]->setFloat( 0.0f ); // Value is set in trace()
  m_context["aperture_radius"]->setFloat(parameters[setup].aperture); 

  // Setup programs
  std::string ptx_path = ptxpath( "EyeAR", "dof_camera.cu" );
  optix::Program ray_gen_program = m_context->createProgramFromPTXFile( ptx_path, "dof_camera" );
  m_context->setRayGenerationProgram( 0, ray_gen_program );
  optix::Program exception_program = m_context->createProgramFromPTXFile( ptx_path, "exception" );
  m_context->setExceptionProgram( 0, exception_program );
  m_context->setMissProgram( 0, m_context->createProgramFromPTXFile( ptxpath( "EyeAR", "constantbg.cu" ), "miss" ) );

  // Setup lighting
  //HACK:hardcoded from the cook example
  BasicLight lights[] = { 
    { optix::make_float3( -30.0f, -10.0f, 80.0f ), optix::make_float3( 1.0f, 1.0f, 1.0f ), 1 },
    { optix::make_float3(  10.0f,  30.0f, 20.0f ), optix::make_float3( 1.0f, 1.0f, 1.0f ), 1 }
  };

  optix::Buffer light_buffer = m_context->createBuffer(RT_BUFFER_INPUT);
  light_buffer->setFormat(RT_FORMAT_USER);
  light_buffer->setElementSize(sizeof(BasicLight));
  light_buffer->setSize( sizeof(lights)/sizeof(lights[0]) );
  memcpy(light_buffer->map(), lights, sizeof(lights));
  light_buffer->unmap();

  m_context["lights"]->set(light_buffer);
  m_context["ambient_light_color"]->setFloat( 0.4f, 0.4f, 0.4f );
  
  // Rendering variables
  m_context["frame_number"]->setUint(1);
  m_context["jitter"]->setFloat(0.0f, 0.0f, 0.0f, 0.0f); //TODO: check this - original cook demo has jitter as 0,0,0 and jitter4 as 0,0,0,0
  //xAdjustWindowOffset = -0.121;
  //yAdjustWindowOffset = -0.235;
  xAdjustWindowOffset = 0;
  yAdjustWindowOffset = 0;

  //create the geometry for the scene
  createGeometry();
  optix::TextureSampler m = loadTexture( m_context, "cloth.ppm", optix::make_float3(1,1,1));
  m->setMaxAnisotropy(1.0f);
       RTresult code;
  //m->checkError(code);
  m_context["texture_map2"]->setTextureSampler( m );
  printf("set successfully\n");
  optix::TextureSampler patternPillar = loadTexture( m_context, "flat.ppm", optix::make_float3(1,1,1));
  curSceneFocusDepth = 0.05f;    //arp //42, 52, 62cm; 31, 41, 51  //we set the focus depth here!
  isNewMeasurement = true;

  //patternPillar->checkError(code);
  patternPillar->setMaxAnisotropy(1.0f);
  //patternPillar->checkError(code);
  m_context["texture_map3"]->setTextureSampler( patternPillar );
 
  m_context->validate();
  m_context->compile();
}

optix::Buffer EyeARScene::getOutputBuffer()
{
  return m_context[ output_buffer_format == RT_FORMAT_FLOAT4 ? "output_buffer_f4" : "output_buffer_f3" ]->getBuffer();
}

void EyeARScene::createMasks(const cv::Mat &image){

    //STATIC


    std::cout << std::endl << "Creating masks..." << std::endl << std::endl;

    cv::Mat bW;
    cv::cvtColor(image, bW, cv::COLOR_BGR2GRAY);
    cv::threshold(bW,bW,1,255,cv::THRESH_BINARY_INV);
    cv::Mat rectSmall, rectBig;

    cv::dilate(bW, bW, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*offsetValue + 1, 2*offsetValue + 1), cv::Point(offsetValue , offsetValue)));
    cv::Mat bWBlur = bW.clone();
    //cv::erode(bW, bW, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*offsetValue + 1, 2*offsetValue + 1), cv::Point(offsetValue , offsetValue)));
    cv::GaussianBlur(bW, bWBlur, cv::Size(2*offsetValue + 1, 2 * offsetValue + 1), 0);

    cv::Mat flip;
    cv::flip(bWBlur, flip, 0);
    cv::Mat channels[] = {flip,flip,flip};
    cv::Mat mask3Channel;
    cv::merge(channels, 3, mask3Channel);
    cv::Mat mask2 = 255 - flip;
    cv::Mat channels2[] = {mask2,mask2,mask2};
    cv::Mat mask3Channel2;
    cv::merge(channels2, 3, mask3Channel2);

    cv::imwrite("mask.png",mask3Channel);
    cv::imwrite("mask2.png",mask3Channel2);

    std::stringstream offset;
    offset << offsetValue;
    cv::imwrite(std::string("mask")+ /*offset.str() + */".png",mask3Channel);
    cv::imwrite(std::string("mask2")+ /*offset.str() + */".png",mask3Channel2);
}


void EyeARScene::getFramePixels(){

	cv::Mat outputResultToSend = cv::Mat(HEIGHT, WIDTH, CV_8UC3);
	glPixelStorei(GL_UNPACK_ALIGNMENT, (outputResultToSend.step & 3) ? 1 : 4);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, outputResultToSend.step/outputResultToSend.elemSize());   //set length of one complete row in data (doesn't need to equal image.cols)//
	glReadPixels(0, 0, outputResultToSend.cols, outputResultToSend.rows, GL_BGR, GL_UNSIGNED_BYTE, outputResultToSend.data); //	

//uncommented 10/3/17
  cv::Mat output2 = cv::Mat(HEIGHT,WIDTH, CV_8UC3);
  output2 = outputResultToSend;
  //if (createMask)
  //{
  //  cv::flip(output2, output2,0);
  //  cv::imwrite("test.png", output2);
  //}
//  //added on 23/2/2017
//  if (createMask)
//  {
//    cv::Mat tmp;
//    cv::flip(outputResultToSend, tmp, 0);
//    cv::imwrite("focused.png", tmp);
//  }
//    /*createMasks(outputResultToSend);
//    createMask = false;
//    curSceneApertureSize = lastApertureSize;
//  }  */
//
  //std::ostringstream eyefiles, sharpfiles;
  //eyefiles << "eye_" << curSceneFocusDepth *100 <<".png";
  //sharpfiles << "sharp_" << curSceneFocusDepth *100 <<".png";
  cv::Mat tmp;
  //cv::flip(outputResultToSend, tmp, 0);
  //cv::imwrite(eyefiles.str(), tmp);


//	if(doSharpView){
//	  runSharpView(outputResultToSend, output2);
//      cv::flip(output2, tmp, 0);
//    cv::imwrite(sharpfiles.str(), tmp);
////    curSceneFocusDepth += 0.05; //increment the focus depth by 0.1, to cycle over all targets. You can uncomment this to keep it fixed, or change the stepsize
//        isNewMeasurement = true;
//      if (generateDatabase)
//      {
// std::ostringstream fileNameWithSharpView;
//    fileNameWithSharpView << std::fixed << std::setprecision(2);
//    fileNameWithSharpView << "./imageDatabase/sharpview/" << /*displayDistances[curDisplayDistance] << "_" <<*/ appertureSizes[curAppertureSize] << "_" << focusDistances[curFocusDistance] << ".png";
//    std::cout << fileNameWithSharpView.str() <<"\n";
//          cv::flip(output2, tmp, 0);
//    cv::imwrite(fileNameWithSharpView.str(), tmp);
//      }
//	}else{
//		output2 = outputResultToSend;
//	}
//	cv::flip(output2,output2,1);
  if(doTexture){
	  int tex_width  = textureWidth;
	  int tex_height = textureHeight;
	  int tex_depth  = 3;
	  glEnable(GL_TEXTURE_2D);
	  if(doOnce){
		  glGenTextures(1, &texId);
		  doOnce = false;
	  }
	  glMatrixMode(GL_MODELVIEW);
	  float quadTransform[16] = {
		  (1.0f),0.0f,0.0f,0.0f,
		  0.0f,1.0,0.0f,0.0f,
		  0.0f,0.0f,1.0f,0.0f,
		  moveX,moveY,0.0f,1.0f
		 // -0.4f,-0.3f,0.0f,1.0f
	  };
	  glLoadIdentity();

	  glMultMatrixf(quadTransform);
	  glBindTexture(GL_TEXTURE_2D, texId); // 2d texture (x and y size) 
	  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
	  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR); // scale linearly when image smalled than texture 

	  if(firstCreate){
	//	  firstCreate = false;
	      glTexImage2D(GL_TEXTURE_2D, 0, 3, output2.cols, output2.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, output2.data);
	  }else{
		 // glTexSubImage2D(GL_TEXTURE_2D,0,0,0,output2.cols,output2.rows,GL_BGR,GL_UNSIGNED_BYTE,output2.data);
	  }
	  glBindTexture(GL_TEXTURE_2D, texId); // choose the texture to use.
  }
  if (generateDatabase) //only called if g is pressed
  {
    //if (counter < 10)
    //{
    //  counter ++;
    //  return;
    //}
    curFocusDistance++;
    if (curFocusDistance >= focusDistances.size())
    {
      curFocusDistance = 0;
      curAppertureSize++;
    }
    if (curAppertureSize >= appertureSizes.size())
    {
      generateDatabase = false;
      //curAppertureSize = 0;
      //curDisplayDistance++;
    }
    //if (curDisplayDistance >= displayDistances.size())
    //{
    //  generateDatabase = false; //we should quit here;
    //}
    isNewMeasurement = true;
    curSceneApertureSize = appertureSizes[curAppertureSize];
    curSceneFocusDepth = focusDistances[curFocusDistance];
    //distanceToScreen = displayDistances[curDisplayDistance];
  }
  output2.release();
  outputResultToSend.release();
}

void EyeARScene::runSharpView(const cv::Mat &input, cv::Mat &output){
	//hardcoded sn
  
  //cv::Mat image = cv::imread("C:/Users/Damien Rompapas/Documents/pattern.png");
  //cv::Mat imageTablet;
  //cv::resize(image, imageTablet, cv::Size(), 450.0/image.rows, 450.0/image.rows);
  cv::Mat bW;
  cv::cvtColor(input, bW, cv::COLOR_BGR2GRAY);
  cv::threshold(bW,bW,1,255,cv::THRESH_BINARY);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bW, contours,  cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  cv::Rect rect = cv::boundingRect(contours[0]);
  bW.setTo(0);
  cv::rectangle(bW, rect, cv::Scalar(255), -1);
  
  float a = glutAperture;	
	float u = curSceneFocusDepth; //this value is already in m
	//float u_ = 0.425f; //this is a fixed value!!! //arp
  float u_ = 0.4f; //this is a fixed value!!! //arp new value. Distance Eye-mirror + distance mirror-Screen

  //pixel pitch of the tablet
  double pixelPitch = 25.4/359.0;
  //resolution one can zoom in on and the resolution of the tablet
  double zoomedHeight = 1;//475;//450.0;
  double tabletHeight = 1;//1600.0;
  double tabletToDisplayPixel = zoomedHeight/tabletHeight;


	std::cout << "Values: " << glutAperture<<","<<a << "," << u << std::endl;
  //check that u does not equal 0
  if (abs(u) <1e-6)
  {
    u = 0.01;
  }
	float sigma = (a/2)*abs(1-(u_/u)); //TODO correct this value
  float sigmaPixel = sigma/pixelPitch;
  float sigmaDisplay = sigmaPixel * tabletToDisplayPixel;
  
  std::cout << sigma <<"," << sigmaPixel <<"," << tabletToDisplayPixel << "," << sigmaDisplay <<" \n";
	if (sigmaDisplay <0.5)
  {
    //printf("sigmaDisplay <0.5, setting to 0.5 - requires checking if this is ok!\n");
    sigmaDisplay = 0.5;
    input.copyTo(output);
    cv::Mat merger;
    cv::flip(input, merger, 0);
  }
  if (sigmaDisplay >4)
  {
    //printf("sigmaDisplay <0.5, setting to 0.5 - requires checking if this is ok!\n");
    sigmaDisplay = 4;
  }
  //to only run sharpview for Aitor
  //simplest solution: change picture here
  //sharpview->evaluate(input,output,sigmaDisplay,snInverse);
  
  cv::Mat output2;
  sharpview->evaluate(input,output,sigmaDisplay,snInverse);
  cv::Scalar meanIn = cv::mean(input, bW);
  int offsetX = rect.width%2, offsetY = rect.height%2;
  if (rect.width > 10 && rect.height > 10)
  {
    sharpview->evaluate(input.colRange(rect.x, rect.x+rect.width-offsetX).rowRange(rect.y, rect.y+rect.height-offsetY),output2,sigmaDisplay,snInverse);
    cv::Scalar meanOut = cv::mean(output2);
    output2 *= (meanIn / meanOut)[0];
    output2.copyTo(output.colRange(rect.x, rect.x+rect.width-offsetX).rowRange(rect.y, rect.y+rect.height-offsetY));
  }
  //cv::drawContours(output, contours, 0, cv::Scalar(0,255,0),2);
  //added 20177/02/23
  if (applyMask) //perform blending
  {
    createMasks(input);
    cv::Mat mask = cv::imread("mask.png");
    cv::Mat mask2 = cv::imread("mask2.png");
    cv::flip(mask, mask,0);
    cv::flip(mask2, mask2,0);
    cv::Mat maskResized;
    mask.convertTo(maskResized, CV_32FC3, 1/255.0f);
    cv::Mat maskResized2;
    mask2.convertTo(maskResized2, CV_32FC3, 1/255.0f);
    cv::Mat output32, input32;
    output.convertTo(output32, CV_32FC3,1/255.0f);
    input.convertTo(input32, CV_32FC3, 1/255.0f);
    cv::Mat merger32 = input32.mul(maskResized) + output32.mul(maskResized2);
    cv::Mat merger;
    merger32.convertTo(merger, CV_8UC3, 255.f);
    //merger.copyTo(output);
    cv::flip(merger, merger, 0);
  }

}

void EyeARScene::cleanUp(){
	SampleScene::cleanUp();
	std::cout << "Shutdown Ros" << std::endl;
  delete sharpview;
}

bool EyeARScene::keyPressed(unsigned char key, int x, int y){
  bool val = SampleScene::keyPressed(key,x,y);
  //std_msgs::String completeMessage2;

  std::cout << "Key pressed: " << std::endl;

  switch(key){
    //too many buttons...
    case '1': //create the mask for blending
      createMask = true;
      lastApertureSize = curSceneApertureSize;      
      //curSceneApertureSize = 0.01;
      isNewMeasurement = true;
      val = true;
      break;
    case '2': //switch between blending and non-blending
      applyMask = !applyMask;
      break;
    case '3':
      offsetValue++;
      std::cout << "offset" << offsetValue <<"\n";
      break;
    case '4':
      offsetValue--;
      std::cout << "offset" << offsetValue <<"\n";
      break;
    case 'g':
      generateDatabase = true;
      doSharpView = false;
      counter = 0;
      curFocusDistance= 0;
      curAppertureSize = 0;
      isNewMeasurement = true;
      curSceneApertureSize = appertureSizes[curAppertureSize];
      curSceneFocusDepth = focusDistances[curFocusDistance];
      val = true;
      break;
	  case 'a':
		  snInverse -= 0.01;
		  val = true;
		  break;
	  case 'd':
		  snInverse += 0.01;
		  val = true;
		  break;
	  case 'w':
		  pausePlayback = !pausePlayback;
      isNewMeasurement = true;
      curSceneApertureSize = 2.8f;
      val = true;
		  break;
	  case 's':
		  doSharpView = !doSharpView;
      std::cout << "Sharpview: " << doSharpView << std::endl;
		  val = true;
		  break;
	  case '0':
			std::cout << "Setting the focus on collection plane to: " << !focusOnCollectionPlane << std::endl;
			focusOnCollectionPlane = !focusOnCollectionPlane;
			break;
	  case 'n':
			useScreenSample = (useScreenSample == 0)? 1 : 0;
			break;
	  case 'm':
			visualizeScreen = (visualizeScreen == 0)? 1 : 0;
			break;
	  case 'c':
		  distanceToScreen -= 0.02f;	
		  std::cout << "Setting Collection Plane to: " << distanceToScreen << std::endl;
		  break;
	  case 'v':
		  distanceToScreen += 0.02f;
		  std::cout << "Setting Collection Plane to: " << distanceToScreen << std::endl;
		  break;
	  case 'q':

      break;
	  case '.':
		  curSceneApertureSize += 0.1f;
      std::cout << "Aperture: " << curSceneApertureSize << std::endl;
      isNewMeasurement = true;
		  val = true;
		  break;
	  case ',':
		  curSceneApertureSize -= 0.1f;
      std::cout << "Aperture: " << curSceneApertureSize << std::endl;
		  val = true;
      isNewMeasurement = true;
		  break;
	  case 'z':
		  curSceneFocusDepth -= 0.05f;
		  std::cout << "Focus: " << curSceneFocusDepth << std::endl;
		  isNewMeasurement = true;
		  val = true;
		  break;
	  case 'x':
		  curSceneFocusDepth += 0.05f;
      /*if (abs(curSceneFocusDepth - 0.40f) < 1e-3)       curSceneFocusDepth = 0.45f; 
      else if (abs(curSceneFocusDepth - 0.45) < 1e-3)  curSceneFocusDepth = 0.50f; 
      else if (abs(curSceneFocusDepth - 0.50f) < 1e-3)  curSceneFocusDepth = 0.55f;
      else if (abs(curSceneFocusDepth - 0.55f) < 1e-3)  curSceneFocusDepth = 0.60f;
      else                                  curSceneFocusDepth = 0.40f;*/
      // curSceneFocusDepth = 0.30f;

		  std::cout << "Focus: " << curSceneFocusDepth << std::endl;
		  isNewMeasurement = true;
		  val = true;
		  break;
		case ' ':
			useEnhancedAperture = !useEnhancedAperture;
			val = true;
			break;
		case 'p':
			moveY += 0.01;
			std::cout << moveX << "," << moveY << std::endl;
			val = true;
			break;
		case ';':
			moveY -= 0.01;
			std::cout << moveX << "," << moveY << std::endl;
			val = true;
			break;
		case 'l':
			moveX += 0.005;
			std::cout << moveX << "," << moveY << std::endl;
			val = true;
			break;
		case 'k':
			moveX -= 0.005;
			std::cout << moveX << "," << moveY << std::endl;
			val = true;
			break;
		case 't':
			xAdjustWindowOffset += 0.001;
			std::cout << "Adjusted Window Co-ordinates: " << xAdjustWindowOffset << "," << yAdjustWindowOffset << std::endl;
			val = true;
			break;
		case 'y':
			xAdjustWindowOffset -= 0.001;
			std::cout << "Adjusted Window Co-ordinates: " << xAdjustWindowOffset << "," << yAdjustWindowOffset << std::endl;
			val = true;
			break;
		case 'r':
			yAdjustWindowOffset += 0.001;
			std::cout << "Adjusted Window Co-ordinates: " << xAdjustWindowOffset << "," << yAdjustWindowOffset << std::endl;
			val = true;
			break;
		case 'f':
			yAdjustWindowOffset -= 0.001;
			std::cout << "Adjusted Window Co-ordinates: " << xAdjustWindowOffset << "," << yAdjustWindowOffset << std::endl;
			val = true;
			break;
		case '`':
			break;
   
	}
	return val;
}

void EyeARScene::createLighting(){
		int num_static_lights = atoi(m_configs["num_stc_lights"].c_str());
		std::cout << "Creating " << num_static_lights << " static lights" << std::endl;
		BasicLight* lights = (BasicLight*) std::malloc(sizeof(BasicLight) * num_static_lights);
		int lights_it = 0;
		for(std::map<std::string, std::string>::iterator it = m_configs.begin(); it != m_configs.end(); ++it) {
			std::size_t found = it->first.find("static_light");
			if (found!=std::string::npos) {
				std::cout << "\t" << string2float3(m_configs[it->first]).x << "," << string2float3(m_configs[it->first]).y << "," << string2float3(m_configs[it->first]).z << std::endl;
				BasicLight tempura = {string2float3(m_configs[it->first]), optix::make_float3(0.9f), 1, 0};
				lights[lights_it] = tempura;
				lights_it++;
			}
		}
		m_light_buffer = m_context->createBuffer(RT_BUFFER_INPUT);
		m_light_buffer->setFormat(RT_FORMAT_USER);
		m_light_buffer->setElementSize(sizeof(BasicLight) );
		m_light_buffer->setSize(num_static_lights);
		memcpy(m_light_buffer->map(), lights, sizeof(BasicLight) * num_static_lights);
		m_light_buffer->unmap();
		m_context[ "lights" ]->set(m_light_buffer);
}

