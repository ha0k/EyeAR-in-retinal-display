/*

float xCrossPos = -0.03f;//-0.04,0.17-0.2,0.18 -0.17,-0.21
float yCrossPos = 0.15f;

void CookScene::renderCross(){	
//	if(loadOnce){
//		loadOnce = false;
//		
////		glEnable(GL_TEXTURE_2D);
//		
////		glGenTextures(1, &texId2);
////		glBindTexture(GL_TEXTURE_2D,texId2);
////		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
////		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR); // scale linearly when image smalled than texture
////		crossTexture = cv::imread("cross.jpg",1);
////		glTexImage2D(GL_TEXTURE_2D, 0, 3, crossTexture.cols, crossTexture.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, crossTexture.data);
//	}else{
//		if(!crossTexture.empty()){
//
//		  glClear(GL_DEPTH_BUFFER_BIT);
//		  glBindTexture(GL_TEXTURE_2D, texId2); // choose the texture to use.
//		  float quadTransform[16] = {
//		  0.036f,0.0f,0.0f,0.0f,
//		  0.0f,0.054f,0.0f,0.0f,
//		  0.0f,0.0f,1.0f,0.0f,
//		  0.25+xQuadPos + xWindowOffset+xCrossPos,0.47+yQuadPos+ yWindowOffset+yCrossPos,0.0,1.0f
//		 // 0.65f + xQuadPos,0.8f+yQuadPos,0.1f,1.0f
//		 // -0.4f,-0.3f,0.0f,1.0f
//		  };
//
//		  glMatrixMode(GL_MODELVIEW);
//		  glLoadIdentity();
//		  glMultMatrixf(quadTransform);
//		  glBegin(GL_QUADS);
//    		glTexCoord2f(0.0, 0.0);
//			glVertex3f(0.0f, 0.0f,0.1f);
//			glTexCoord2f(1.0f, 0.0);
//			glVertex3f(1.0f, 0.0f,0.1f);
//			glTexCoord2f(1.0f, 1.0f);
//			glVertex3f(1.0f, 1.0f,0.1f);
//			glTexCoord2f(0.0, 1.0f );
//			glVertex3f(0.0f, 1.0f,0.1f);
//			glEnd();
//		  glMatrixMode(GL_MODELVIEW);
//		  glBindTexture(GL_TEXTURE_2D, texId);
//		  glLoadIdentity();
//		}
//	}
}
*/

/*
void CookScene::SendOutImage(const cv::Mat image){

}
*/
/*
void CookScene::publishResult(){
	cv::Mat outputResultToSend = cv::Mat(HEIGHT, WIDTH, CV_8UC3);
	glPixelStorei(GL_UNPACK_ALIGNMENT, (outputResultToSend.step & 3) ? 1 : 4);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, outputResultToSend.step/outputResultToSend.elemSize());   //set length of one complete row in data (doesn't need to equal image.cols)//
	glReadPixels(0, 0, outputResultToSend.cols, outputResultToSend.rows, GL_BGR, GL_UNSIGNED_BYTE, outputResultToSend.data); //	
	rendered = outputResultToSend;
}
*/

/*
std::string CookScene::texpath( const std::string& base )
{
  return texture_path + "/" + base;
}
*/

/*
int getMilliCount(){
	timeb tb;
	ftime(&tb);
	int nCount = tb.millitm + (tb.time & 0xfffff) * 1000;
	return nCount;
}
*/
/*
int getMilliSpan(int nTimeStart){
	int nSpan = getMilliCount() - nTimeStart;
	if(nSpan < 0)
		nSpan += 0x100000 * 1000;
	return nSpan;
}
*/
/*
inline float deg_to_rad( float degrees )
{
  return degrees * M_PIf / 180.0f;
}
*/
/*
inline float rad_to_deg( float radians )
{
  return radians * 180.0f / M_PIf;
}
*/
/*
inline float random1()
{
  return (float)rand()/(float)RAND_MAX;
}
*/
/*
inline float2 random2()
{
  return make_float2( random1(), random1() );
}
*/
/*
inline float3 random3()
{
  return make_float3( random1(), random1(), random1() );
}
*/
/*
inline float4 random4()
{
  return make_float4( random1(), random1(), random1(), random1() );
}
*/
/*
optix::Matrix3x3 Rotation(float3 rotation)
{
  float alpha = -deg_to_rad( rotation.x );
  float beta  = -deg_to_rad( rotation.y );
  float gamma = -deg_to_rad( rotation.z );

  float s_a = sinf(alpha);
  float c_a = cosf(alpha);

  float s_b = sinf(beta);
  float c_b = cosf(beta);

  float s_g = sinf(gamma);
  float c_g = cosf(gamma);

  float rotate_x[3*3] = {   1,    0,    0,
                            0,   c_a, -s_a,
                            0,   s_a,  c_a };

  float rotate_y[3*3] = {  c_b,   0,   s_b,
                            0,    1,    0,
                          -s_b,   0,   c_b };

  float rotate_z[3*3] = {  c_g, -s_g,   0,
                           s_g,  c_g,   0, 
                            0,    0,    1 };

  optix::Matrix3x3 mat_x(rotate_x);
  optix::Matrix3x3 mat_y(rotate_y);
  optix::Matrix3x3 mat_z(rotate_z);

  optix::Matrix3x3 mat = mat_z * mat_y * mat_x;
    std::map<std::string, std::string> m_configs;
  return mat;
}
*/