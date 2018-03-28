#include <optix.h>
#include <OptixMesh.h>
#include <optixu/optixpp_namespace.h>
#include <optixu/optixu_math_namespace.h>
#include "scenemanagment.h"
#include <string>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <OptixMesh.h>
#include <SampleScene.h>
#define GLM_FORCE_RADIANS
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
using namespace optix;


float degreesToRadians(float x){return x*(3.141592f/180.0f);}
/*
 * Read file to get confugration vars
 * Format in cfg:
 *      var_name = var_value
 *      var_name2 = 1.0,0.0,-2.1
 *      var_name3 = path_to_file
 *
 * How to use the returned map:
 * In main: m_configs = getConfig("path_to_file/file.cfg");
 * When needed:
 *      float var_to_set = atof(m_configs["var_name"].c_str());
 *      float3 color_to_set = string2float3(m_configs["var_name2"]);
 *      file_to_load(m_configs["var_name3"]);
 */
std::map<std::string, std::string> getConfig(const char *file_varcfg) {
    std::map<std::string, std::string> toRet;
    std::string id, eq, val;

    std::ifstream file;
    file.open(file_varcfg);

    if(file.is_open()) {
        std::cout << "[CFG Loader] Reading configurations file: " << file_varcfg << std::endl;
        while(file >> id >> eq >>val) {
            if(id != "") {
                if(val == CFG_NODATA)
                    std::cerr << "[CFG Loader] WARNING: No data for id " << id << std::endl;
                else
                    toRet[id] = val;
            }
        }
        std::cout << "[CFG Loader] Configuration is loaded" << std::endl;
        file.close();
    } else {
        std::cerr << "[CFG Loader] Cannot open configuration file " << file_varcfg << std::endl;
        exit(2);
    }

    file.close();
    return toRet;
}


optix::float3 string2float3(std::string in) {
    std::vector<std::string> tmp;
    boost::split(tmp, in, boost::is_any_of( " ," ), boost::token_compress_on );
    return optix::make_float3(atof(tmp[0].c_str()), atof(tmp[1].c_str()), atof(tmp[2].c_str()));
}

/*
 * TODO Improve BRDF loading (if brdf is already loaded...) but dont know if possible to have multiple material pointing to a single buffer
 * TODO Implement FUNCTION if needed
 *
 * Format (see scene3 for full example):
 *      object
 *          - transformation matrix, 4 floats x 4 lines
 *          - object filename
 *          - MERL BRDF binary filename brightness gamma exposure    // - FUNCTION : will use a function (BRDF calculated)
 *              if FUNCTION: TODO
 *              - ptx_file closest_hit_prgm_name
 *              - ptx_file any_hit_prgm_name
 *
 *      sphere
 *          - transformation matrix, 4 floats x 4 lines
 *          - sphere radius
 *          - MERL BRDF binary filename brightness gamma exposure    // - FUNCTION : will use a function (color or normal sharder)
 *              if FUNCTION: TODO
 *                  - ptx_file closest_hit_prgm_name
 *                  - ptx_file any_hit_prgm_name
 *
 *      plane
 *          - transformation matrix, 4 floats x 4 lines
 *          - anchor point
 *          - vector width v1
 *          - vector height v2
 *          - MERL BRDF binary filename brightness gamma exposure    // - FUNCTION : will use a function (color or normal sharder)
 *              if FUNCTION: TODO
 *                  - ptx_file closest_hit_prgm_name
 *                  - ptx_file any_hit_prgm_name
 *
 *      cube
 *          - transformation matrix, 4 rows
 *          - front bottom left point
 *          - back top right point
 *          - MERL BRDF binary filename brightness gamma exposure    // - FUNCTION : will use a function (color or normal sharder)
 *              if FUNCTION: TODO
 *                  - ptx_file closest_hit_prgm_name
 *                  - ptx_file any_hit_prgm_name
 */
optix::Buffer readBRDF(optix::Buffer &toRet,const char* filename, optix::Context context) {
    double* brdf;
    FILE *f = fopen(filename, "rb");
    if (!f)
        return false;
    int dims[3];
    fread(dims, sizeof(int), 3, f);
    int n = dims[0] * dims[1] * dims[2];
    if (n != BRDF_SAMPLING_RES_THETA_H *
             BRDF_SAMPLING_RES_THETA_D *
             BRDF_SAMPLING_RES_PHI_D / 2)
    {
        fprintf(stderr, "Dimensions don't match\n");
        fclose(f);
    }
    brdf = (double*) malloc (sizeof(double)*3*n);
    float* test = (float*) malloc(sizeof(float)*3*n);
    fread(brdf, sizeof(double), 3*n, f);
    int max = 3*n;
    max = max-1;
    for(int i = 0; i < n; ++i) {
        test[i*3 + 0] = brdf[i*3 + 0];
        test[i*3 + 1] = brdf[i*3 + 1];
        test[i*3 + 2] = brdf[i*3 + 2];
    }
    //Set up the BRDF buffer
    toRet = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT, sizeof(float)*3*n);
    float * buffer_data = static_cast<float*>(toRet->map());
    memcpy(buffer_data, test, sizeof(float)*3*n);
    toRet->unmap();	
    fclose(f);
    return toRet;
}



/*
 * convert a string to a glm::mat4
 * floats are separated by , and no space around
 */
glm::mat4 string2TMat(std::string in) {
    glm::mat4 toRet;
    std::vector<std::string> tmp;
    boost::split(tmp, in, boost::is_any_of( " ," ), boost::token_compress_on );
    int counter = 0;
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            toRet[i][j] = atof(tmp[counter].c_str());
            counter++;
        }
    }
    return toRet;
}
glm::vec3 string2TVec3(std::string in){
	glm::vec3 toRet;
	std::vector<std::string> tmp;
	boost::split(tmp,in,boost::is_any_of(" ," ), boost::token_compress_on);
	toRet.x = atof(tmp[0].c_str());
	toRet.y = atof(tmp[1].c_str());
	toRet.z = atof(tmp[2].c_str());
	return toRet;
}
float4 make_plane( float3 n, float3 p )
{
  n = normalize(n);
  float d = -dot(n, p);
  return make_float4( n, d );
}

void resetPlane(optix::Context context, optix::Group screen_ggroup,AccelDescriptor m_accel_desc,float fovH,float fovV,float distanceToScreen){
	float VV = (distanceToScreen * glm::tan(degreesToRadians(fovV*1.0f)))/2.0f;
				float HV = VV*fovH;
				float3 anchor = make_float3( -(HV/2.0f), -(VV/2.0f), -0.0f);
				float3 v1 = make_float3( HV, 0.0f, -0.0f);
				float3 v2 = make_float3( 0.0f, VV, -0.0f);

				float3 normal = cross( v2, v1 );
				normal = normalize( normal );
				float d = dot( normal, anchor );
				v1 *= 1.0f/dot( v1, v1 );
				v2 *= 1.0f/dot( v2, v2 );
                optix::GeometryGroup local_ggroup = context->createGeometryGroup();
                optix::Geometry screen = context->createGeometry();
                screen->setPrimitiveCount(1);
                screen->setBoundingBoxProgram( context->createProgramFromPTXFile( SampleScene::ptxpath( "EyeAR", "parallelogram.cu" ), "bounds" ) );
                screen->setIntersectionProgram( context->createProgramFromPTXFile( SampleScene::ptxpath( "EyeAR", "parallelogram.cu" ), "intersect" ) );
				float4 plane = make_float4( normal, d );
				screen["plane"]->setFloat( plane );
				screen["v1"]->setFloat( v1 );
				screen["v2"]->setFloat( v2 );
				screen["anchor"]->setFloat( anchor );


                optix::Material obj_material = context->createMaterial();
                optix::Program obj_ch = context->createProgramFromPTXFile(SampleScene::ptxpath("EyeAR", "screen.cu"), "closest_hit_radiance"); // TODO in config file !
                optix::Program obj_ah = context->createProgramFromPTXFile(SampleScene::ptxpath("EyeAR", "screen.cu"), "any_hit_shadow"); // TODO in config file !
                obj_material->setClosestHitProgram(0, obj_ch);
                obj_material->setAnyHitProgram(1, obj_ah);

                optix::GeometryInstance gi_sphere = context->createGeometryInstance(screen, &obj_material, &obj_material + 1);
                local_ggroup->setChildCount(local_ggroup->getChildCount() + 1);
                local_ggroup->setChild(local_ggroup->getChildCount() - 1, gi_sphere);
                local_ggroup->setAcceleration(context->createAcceleration("Bvh", "BvhSingle"));

                glm::mat4 transformation_matrix = glm::mat4(1.0f,0.0f,0.0f,0.0f,
												  0.0f,1.0f,0.0f,0.0f,
												  0.0f,0.0f,1.0f,0.0f,
												  0.0f,0.0f,-distanceToScreen,1.0f);
                optix::Transform t_mat = context->createTransform();
                t_mat->setMatrix(true, glm::value_ptr(transformation_matrix), glm::value_ptr(glm::inverse(transformation_matrix)));
                t_mat->setChild(local_ggroup);
                screen_ggroup->setChildCount(screen_ggroup->getChildCount() + 1);
                screen_ggroup->setChild(screen_ggroup->getChildCount() - 1, t_mat);
}
void setSceneFromFile(const char *file_scenecfg,optix::Group screen_ggroup, optix::Group top_ggroup, optix::Context context, 
					const char *obj_folder, const char *brdf_folder,AccelDescriptor m_accel_desc,float fovH,float fovV,float distanceToScreen) {
    std::ifstream file;
    file.open(file_scenecfg);
    std::string line, trans_mat1, trans_mat2,trans_mat3, trans_mat4, obj_filename,
            brdf_filename, sphere_radius, anchor, v1, v2, box_min, box_max,
            brightness="3.0", gamma="2.2" , exposure="1.0", red_component="1.0", green_component="1.0", blue_component="1.0";

    if(file.is_open()) {
        std::cout << "[Scene Loader	Reading scene configurations file" << std::endl;
        float tmp = 0.0f;
        while(file >> line) {
//            float transformation_matrix[4*4];
            glm::mat4 transformation_matrix;
            // First line is type
            if(line == CFG_OBJ) {
                file >> trans_mat1;
                file >> trans_mat2;
                file >> trans_mat3;
                file >> trans_mat4;
                file >> obj_filename;
                file >> brdf_filename >> brightness >> gamma >> exposure;
				file >> red_component >> green_component >> blue_component;
                std::cout << "[Scene Loader] Adding an obj to scene" << std::endl;
                transformation_matrix = string2TMat(trans_mat1 + "," + trans_mat2 + "," + trans_mat3 + "," + trans_mat4);
                std::cout << "[Scene Loader] Matrix read: " << glm::to_string(transformation_matrix) << std::endl;

                std::cout << "[Scene Loader] Loading model..." << std::endl;

                // Create geometry material
                optix::Material obj_material = context->createMaterial();
                optix::Program obj_ch = context->createProgramFromPTXFile(SampleScene::ptxpath("EyeAR", "distributed_brdf.cu"), "closest_hit_radiance"); // TODO in config file !
                optix::Program obj_ah = context->createProgramFromPTXFile(SampleScene::ptxpath("EyeAR", "distributed_brdf.cu"), "any_hit_shadow"); // TODO in config file !
                obj_material->setClosestHitProgram(0, obj_ch);
                obj_material->setAnyHitProgram(1, obj_ah);
                obj_material["brightness"]->setFloat(atof(brightness.c_str()));
                obj_material["gammag"]->setFloat(atof(gamma.c_str()));
                obj_material["exposure"]->setFloat(atof(exposure.c_str()));
				float colormods[3] = {atof(red_component.c_str()),atof(green_component.c_str()),atof(blue_component.c_str())};
				std::cout << colormods[0] << "," << colormods[1] << "," << colormods[2] << std::endl;
				obj_material["colorModifier"]->setFloat(
					optix::make_float3(colormods[0],colormods[1],colormods[2])
				);
                optix::Buffer temp_buffer = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT, sizeof(float)*3*3);
				obj_material["brdf"]->setBuffer(temp_buffer);

                if(brdf_filename == "FUNCTION") {
                    std::cout << "[Scene Loader] Load BRDF from a function" << std::endl;
                } else {
                    std::cout << "[Scene Loader] Loading BRDF " << brdf_filename << std::endl;
                    optix::Buffer temp_buffer;
					std::cout << brdf_filename.c_str() << std::endl;
					readBRDF(temp_buffer,(brdf_filename).c_str(), context);
					std::cout << obj_material["brdf"]->getBuffer()->getFormat() << std::endl;
					std::cout << temp_buffer->getFormat() << std::endl;
					obj_material["brdf"]->setBuffer(temp_buffer.get());
                }
				std::cout << "[Scene Loader] Loading Model" << std::endl;
               // OptiXMesh loader(context, local_ggroup, obj_material);
                optix::GeometryGroup local_ggroup = context->createGeometryGroup();
				OptiXMesh loader(context,local_ggroup,obj_material,m_accel_desc);
				
				const char* path = obj_filename.c_str();
				std::cout << path << std::endl; 
                loader.loadBegin_Geometry(path);

                loader.loadFinish_Materials();
                std::cout << "[Scene Loader] Model and Material Loaded" << std::endl;
                optix::Transform t_mat = context->createTransform();
//                t_mat->setMatrix(false, transformation_matrix, NULL);
                t_mat->setMatrix(true, glm::value_ptr(transformation_matrix), glm::value_ptr(glm::inverse(transformation_matrix)));
                t_mat->setChild(local_ggroup);

                top_ggroup->setChildCount(top_ggroup->getChildCount() + 1);
                top_ggroup->setChild(top_ggroup->getChildCount() - 1, t_mat);
            }else if(line == CFG_PIXEL) {
                file >> trans_mat1;
                file >> trans_mat2;
                file >> trans_mat3;
                file >> trans_mat4;
                file >> obj_filename;
                std::cout << "[Scene Loader] Adding an obj to scene" << std::endl;
                transformation_matrix = string2TMat(trans_mat1 + "," + trans_mat2 + "," + trans_mat3 + "," + trans_mat4);
                std::cout << "[Scene Loader] Matrix read: " << glm::to_string(transformation_matrix) << std::endl;

                std::cout << "[Scene Loader] Loading model..." << std::endl;

                // Create geometry material
                optix::Material obj_material = context->createMaterial();
                optix::Program obj_ch = context->createProgramFromPTXFile(SampleScene::ptxpath("EyeAR", "pixel.cu"), "closest_hit_radiance"); // TODO in config file !
                optix::Program obj_ah = context->createProgramFromPTXFile(SampleScene::ptxpath("EyeAR", "pixel.cu"), "any_hit_shadow"); // TODO in config file !
                obj_material->setClosestHitProgram(0, obj_ch);
                obj_material->setAnyHitProgram(1, obj_ah);
				std::cout << "[Scene Loader] Loading Model" << std::endl;
               // OptiXMesh loader(context, local_ggroup, obj_material);
                optix::GeometryGroup local_ggroup = context->createGeometryGroup();
				OptiXMesh loader(context,local_ggroup,obj_material,m_accel_desc);
				
				const char* path = obj_filename.c_str();
				std::cout << path << std::endl; 
                loader.loadBegin_Geometry(path);

                loader.loadFinish_Materials();
                std::cout << "[Scene Loader] Model and Material Loaded" << std::endl;
                optix::Transform t_mat = context->createTransform();
//                t_mat->setMatrix(false, transformation_matrix, NULL);
                t_mat->setMatrix(true, glm::value_ptr(transformation_matrix), glm::value_ptr(glm::inverse(transformation_matrix)));
                t_mat->setChild(local_ggroup);

                top_ggroup->setChildCount(top_ggroup->getChildCount() + 1);
                top_ggroup->setChild(top_ggroup->getChildCount() - 1, t_mat);
            }
			else if(line == CFG_SPHERE) {

                file >> trans_mat1;
                file >> trans_mat2;
                file >> trans_mat3;
                file >> trans_mat4;
                file >> sphere_radius;
                file >> brdf_filename >> brightness >> gamma >> exposure;
			
                std::cout << "[Scene Loader] Creating a sphere (size " << sphere_radius << ") to scene" << std::endl;
                optix::GeometryGroup local_ggroup = context->createGeometryGroup();

                optix::Geometry sphere = context->createGeometry();
                sphere->setPrimitiveCount(1);
                sphere->setBoundingBoxProgram( context->createProgramFromPTXFile( SampleScene::ptxpath( "renderer", "sphere.cu" ), "sphere_bounds" ) );
                sphere->setIntersectionProgram( context->createProgramFromPTXFile( SampleScene::ptxpath( "renderer", "sphere.cu" ), "sphere_intersect" ) );
                sphere["sphere_radius"]->setFloat(atof(sphere_radius.c_str()));

                optix::Material obj_material = context->createMaterial();
                optix::Program obj_ch = context->createProgramFromPTXFile(SampleScene::ptxpath("renderer", "tutorial10.cu"), "box_closest_hit_radiance"); // TODO in config file !
                optix::Program obj_ah = context->createProgramFromPTXFile(SampleScene::ptxpath("renderer", "tutorial10.cu"), "any_hit_shadow"); // TODO in config file !
                obj_material->setClosestHitProgram(0, obj_ch);
                obj_material->setAnyHitProgram(1, obj_ah);

                obj_material["brightness"]->setFloat(atof(brightness.c_str())); //arp
                obj_material["gammag"]->setFloat(atof(gamma.c_str())); //arp
                obj_material["exposure"]->setFloat(atof(exposure.c_str())); //arp

                if(brdf_filename == "FUNCTION") {
                    // Here we add some more option to the loader dependings on needs
                    std::cout << "[Scene Loader] Load BRDF from a function" << std::endl;
                    // file >> ch_name;
                    // file >> ah_name;
                    // optix::Program obj_ch = YOUR closest_hit pgrm
                    // optix::Program obj_ah = YOUR any_hit pgrm
                    // optix::Buffer temp_buffer = BRDF_function(....);
                    // obj_material["brdf"]->setBuffer(temp_buffer);
                    // Call a defined BRDF function; Well also can use reflection if multiple functions: std::map<std::string, FnPtr> listOfBRDF_Fct;
                } else {
                    std::cout << "[Scene Loader] Loading BRDF " << brdf_filename << std::endl;
                     optix::Buffer temp_buffer;
					std::cout << brdf_filename.c_str() << std::endl;
					readBRDF(temp_buffer,(brdf_filename).c_str(), context);
					std::cout << obj_material["brdf"]->getBuffer()->getFormat() << std::endl;
					std::cout << temp_buffer->getFormat() << std::endl;
					obj_material["brdf"]->setBuffer(temp_buffer.get());
                }

                optix::GeometryInstance gi_sphere = context->createGeometryInstance(sphere, &obj_material, &obj_material + 1);
                local_ggroup->setChildCount(local_ggroup->getChildCount() + 1);
                local_ggroup->setChild(local_ggroup->getChildCount() - 1, gi_sphere);
                local_ggroup->setAcceleration(context->createAcceleration("Bvh", "BvhSingle"));

                transformation_matrix = string2TMat(trans_mat1 + "," + trans_mat2 + "," + trans_mat3 + "," + trans_mat4);
                optix::Transform t_mat = context->createTransform();
//                t_mat->setMatrix(false, transformation_matrix, NULL);
                t_mat->setMatrix(true, glm::value_ptr(transformation_matrix), glm::value_ptr(glm::inverse(transformation_matrix)));
                t_mat->setChild(local_ggroup);

                top_ggroup->setChildCount(top_ggroup->getChildCount() + 1);
                top_ggroup->setChild(top_ggroup->getChildCount() - 1, t_mat);
            }else if(line == CFG_SCREEN) {
                file >> trans_mat1;
                file >> trans_mat2;
                file >> trans_mat3;
                file >> trans_mat4;
                file >> brdf_filename >> brightness >> gamma >> exposure;



				float VV = (distanceToScreen * glm::tan(degreesToRadians(fovV*1.0f)))/2.0f;
				float HV = VV*fovH;


				std::cout<< "[Scene Loader] creating screen of size " << HV << ", " << VV << std::endl; 
				float3 anchor = make_float3( -(HV/2.0f), -(VV/2.0f), -0.0f);
				float3 v1 = make_float3( HV, 0.0f, -0.0f);
				float3 v2 = make_float3( 0.0f, VV, -0.0f);

				float3 normal = cross( v2, v1 );
				normal = normalize( normal );
				float d = dot( normal, anchor );
				v1 *= 1.0f/dot( v1, v1 );
				v2 *= 1.0f/dot( v2, v2 );
                std::cout << "[Scene Loader] inserting virtual screen to scene" << std::endl;
                optix::GeometryGroup local_ggroup = context->createGeometryGroup();
                optix::Geometry screen = context->createGeometry();
                screen->setPrimitiveCount(1);
                screen->setBoundingBoxProgram( context->createProgramFromPTXFile( SampleScene::ptxpath( "EyeAR", "parallelogram.cu" ), "bounds" ) );
                screen->setIntersectionProgram( context->createProgramFromPTXFile( SampleScene::ptxpath( "EyeAR", "parallelogram.cu" ), "intersect" ) );
				float4 plane = make_float4( normal, d );
				screen["plane"]->setFloat( plane );
				screen["v1"]->setFloat( v1 );
				screen["v2"]->setFloat( v2 );
				screen["anchor"]->setFloat( anchor );


                optix::Material obj_material = context->createMaterial();
                optix::Program obj_ch = context->createProgramFromPTXFile(SampleScene::ptxpath("EyeAR", "screen.cu"), "closest_hit_radiance"); // TODO in config file !
                optix::Program obj_ah = context->createProgramFromPTXFile(SampleScene::ptxpath("EyeAR", "screen.cu"), "any_hit_shadow"); // TODO in config file !
                obj_material->setClosestHitProgram(0, obj_ch);
                obj_material->setAnyHitProgram(1, obj_ah);

                

                optix::GeometryInstance gi_sphere = context->createGeometryInstance(screen, &obj_material, &obj_material + 1);
                local_ggroup->setChildCount(local_ggroup->getChildCount() + 1);
                local_ggroup->setChild(local_ggroup->getChildCount() - 1, gi_sphere);
                local_ggroup->setAcceleration(context->createAcceleration("Bvh", "BvhSingle"));

                transformation_matrix = string2TMat(trans_mat1 + "," + trans_mat2 + "," + trans_mat3 + "," + trans_mat4);
                optix::Transform t_mat = context->createTransform();
                t_mat->setMatrix(true, glm::value_ptr(transformation_matrix), glm::value_ptr(glm::inverse(transformation_matrix)));
                t_mat->setChild(local_ggroup);
                screen_ggroup->setChildCount(screen_ggroup->getChildCount() + 1);
                screen_ggroup->setChild(screen_ggroup->getChildCount() - 1, t_mat);
            }
        }
        std::cout << "[Scene Loader] Scene is loaded" << std::endl;
        file.close();
    } else {
        std::cout << "[Scene Loader] Cannot open configuration file " << file_scenecfg << std::endl;
        exit(2);
    }
}

