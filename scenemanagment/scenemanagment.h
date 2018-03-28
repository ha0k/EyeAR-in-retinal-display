#ifndef OPTIX_SAMPLES_SCENEMANAGMENT_H
#define OPTIX_SAMPLES_SCENEMANAGMENT_H
#include <optix.h>
#include <OptixMesh.h>
#include <optixu/optixpp_namespace.h>
#include <optixu/optixu_math_namespace.h>

#include <ImageLoader.h>
#include <optixu/optixu_aabb_namespace.h>
#include <string>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <map>
#include <glm/matrix.hpp>


#define CFG_NODATA "<None>"
#define CFG_PIXEL "pixel"
#define CFG_SPHERE "sphere"
#define CFG_SCREEN "screen"
#define CFG_PLANE "plane"
#define CFG_CUBE "cube"
#define CFG_OBJ "object"
#define CFG_STATIC_LIGHT "static_light"

#define BRDF_SAMPLING_RES_THETA_H       90
#define BRDF_SAMPLING_RES_THETA_D       90
#define BRDF_SAMPLING_RES_PHI_D         360

std::map<std::string, std::string> getConfig(const char *file_varcfg);
void setSceneFromFile(const char *file_scenecfg,optix::Group screen_ggroup, optix::Group top_ggroup, optix::Context context, 
					const char *obj_folder, const char *brdf_folder,AccelDescriptor m_accel_desc,float fovH,float fovV,float distanceToScreen);
void resetPlane(optix::Context context, optix::Group screen_ggroup,AccelDescriptor m_accel_desc,float fovH,float fovV,float distanceToScreen);
optix::Buffer readBRDF(optix::Buffer &toRet,const char* filename, optix::Context context);

optix::float3 string2float3(std::string in);
glm::mat4 string2TMat(std::string in);


#endif //OPTIX_SAMPLES_SCENEMANAGMENT_H
