
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
#define BRDF_SAMPLING_RES_THETA_H       90
#define BRDF_SAMPLING_RES_THETA_D       90
#define BRDF_SAMPLING_RES_PHI_D         360

#define RED_SCALE (1.0/1500.0)
#define GREEN_SCALE (1.15/1500.0)
#define BLUE_SCALE (1.66/1500.0)
#include <optix.h>
#include <optix_math.h>

#include "commonStructs.h"
#include "helpers.h"


struct PerRayData_radiance
{
  float3 result;
  float importance;
  int depth;
  bool hasHitObject;
  bool hasHitScreen;
  float2 screenUV;
};

struct PerRayData_shadow
{
  float3 attenuation;
};


// global parameters
rtDeclareVariable(rtObject,     top_object, , );
rtDeclareVariable(rtObject,      screen_object, , );
rtDeclareVariable(rtObject,     top_shadower, , );
rtDeclareVariable(int,          max_depth, , );
rtDeclareVariable(float,        scene_epsilon, , );
rtDeclareVariable(float3,       ambient_light_color, , );
rtDeclareVariable(unsigned int, radiance_ray_type, , );
rtDeclareVariable(unsigned int, shadow_ray_type, , );
rtDeclareVariable(float4,       jitter, , );
rtDeclareVariable(float3,        eye, , );
rtDeclareVariable(float3,        U, , );
rtDeclareVariable(float3,        V, , );
rtDeclareVariable(float3,        W, , );
rtDeclareVariable(float3,        bad_color, , );
rtDeclareVariable(float, brightness, ,) = 1.0f;
rtDeclareVariable(float, exposure, , ) = 0.0f;
rtDeclareVariable(float, gammag, ,) = 2.2f;
rtDeclareVariable(float3, upVec,,);
rtDeclareVariable(float3, colorModifier,,);
// ray parameters
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(float, t_hit, rtIntersectionDistance, );

rtDeclareVariable(PerRayData_radiance, prd_radiance, rtPayload, );
rtDeclareVariable(PerRayData_shadow,   prd_shadow,   rtPayload, );

rtTextureSampler<float4, 2>     texture_map2;
rtTextureSampler<float4, 2>     texture_map3;
//
rtBuffer<BasicLight> lights;
rtBuffer<float> brdf;

static __device__ void phongShadowed()
{
  // this material is opaque, so it fully attenuates all shadow rays
  prd_shadow.attenuation = make_float3(0);
  
  rtTerminateRay();
}

static __device__ int theta_half_index(double theta_half) {
    if (theta_half <= 0.0)
        return 0;
    double theta_half_deg = ((theta_half / (M_PI/2.0))*BRDF_SAMPLING_RES_THETA_H);
    double temp = theta_half_deg*BRDF_SAMPLING_RES_THETA_H;
    temp = sqrt(temp);
    int ret_val = (int)temp;
    if (ret_val < 0) ret_val = 0;
    if (ret_val >= BRDF_SAMPLING_RES_THETA_H)
        ret_val = BRDF_SAMPLING_RES_THETA_H-1;
    return ret_val;
}


// Lookup theta_diff index
// In:  [0 .. pi/2]
// Out: [0 .. 89]
static __device__ int theta_diff_index(double theta_diff) {
    int tmp = int(theta_diff / (M_PI * 0.5) * BRDF_SAMPLING_RES_THETA_D);
    if (tmp < 0)
        return 0;
    else if (tmp < BRDF_SAMPLING_RES_THETA_D - 1)
        return tmp;
    else
        return BRDF_SAMPLING_RES_THETA_D - 1;
}

static __device__ int phi_diff_index(float phi_diff) {
    if (phi_diff < 0.0)
        phi_diff += M_PI;

    int tmp = int(phi_diff / M_PI * BRDF_SAMPLING_RES_PHI_D / 2);
    if (tmp < 0)
        return 0;
    else if (tmp < BRDF_SAMPLING_RES_PHI_D / 2 - 1)
        return tmp;
    else
        return BRDF_SAMPLING_RES_PHI_D / 2 - 1;
}

static __device__ float3 lookup_brdf3(float3 toLight, float3 toViewer, float3 normal, float3 tangent, float3 bitangent) {
    float3 H = normalize(toLight + toViewer);
    float3 result;
    float theta_H = acos(clamp(dot(normal, H), 0.0f, 1.0f));
    float theta_diff = acos(clamp(dot(H, toLight), 0.0f, 1.0f));
    float phi_diff=0;

    if (theta_diff < 1e-3) {
        // phi_diff indeterminate, use phi_half instead
        phi_diff = atan2(clamp(-dot(toLight, bitangent), -1.0f, 1.0f), clamp(dot(toLight, tangent), -1.0f, 1.0f));
    }
    else if (theta_H > 1e-3) {
        // use Gram-Schmidt orthonormalization to find diff basis vectors
        float3 u = -normalize(normal - dot(normal,H) * H);
        float3 v = cross(H, u);
        phi_diff = atan2(clamp(dot(toLight,v), -1.0f, 1.0f), clamp(dot(toLight,u), -1.0f, 1.0f));
    }
    else theta_H = 0;

    // Find index.
    // Note that phi_half is ignored, since isotropic BRDFs are assumed
    int ind = phi_diff_index(phi_diff) +
              theta_diff_index(theta_diff) * BRDF_SAMPLING_RES_PHI_D / 2 +
              theta_half_index(theta_H) * BRDF_SAMPLING_RES_PHI_D / 2 *
              BRDF_SAMPLING_RES_THETA_D;

    int redIndex = ind;
    int greenIndex = ind + BRDF_SAMPLING_RES_THETA_H*BRDF_SAMPLING_RES_THETA_D*BRDF_SAMPLING_RES_PHI_D/2;
    int blueIndex = ind + BRDF_SAMPLING_RES_THETA_H*BRDF_SAMPLING_RES_THETA_D*BRDF_SAMPLING_RES_PHI_D;

    result.x = brdf[redIndex] * RED_SCALE;
    result.y = brdf[greenIndex] * GREEN_SCALE;
    result.z = brdf[blueIndex] * BLUE_SCALE;

    result *= brightness; //arp
    result *= pow(2.0f, exposure); //arp

    result.x = pow(result.x, 1.0f/ gammag); //arp
    result.y = pow(result.y, 1.0f/ gammag); //arp
    result.z = pow(result.z, 1.0f/ gammag); //arp

//    return result;
	return clamp(result,make_float3(0.0),make_float3(1.0));
}
 static
__device__ void BRDF(float3 p_normal, float3 texcoord,float3 editedNormal)
{
		float3 hit_point = ray.origin + t_hit * ray.direction;
		float3 result = make_float3(0.0,0.0,0.0);
		// compute direct lighting
		unsigned int num_lights = lights.size();
		float3 lightColor = make_float3(1.0,1.0,1.0);
		float3 n_shading_normal = normalize(p_normal)*normalize(editedNormal);
		float3 camera_up = upVec;
		float3 n_tangent = normalize(cross(camera_up,p_normal*editedNormal));
		float3 n_bi_tangent = normalize(cross(n_tangent,p_normal*editedNormal));
		float3 mm_brdf_diffuse = make_float3(0,0,0); 

		float reflectance = 0;
		for(int i = 0; i < num_lights; ++i) {
		BasicLight light = lights[i];
		float3 L = light.pos - hit_point;
		float3 Ll = light.pos - hit_point;
		float3 LightPositionShadow = make_float3(0.0,1.0,0.5);
		float3 LightPositionShadow2 = make_float3(0.0,-1.0,0.5);
		float2 sample = optix::square_to_disk(make_float2(jitter.x, jitter.y));
	
		// set jittered light direction	
		L += 1.3f * (sample.x * U + sample.y * V);
		Ll += 1.3f * (sample.x * U + sample.y * V);
		LightPositionShadow += 1.3 * (sample.x * U + sample.y * V);
		LightPositionShadow2 += 1.3 * (sample.x * U + sample.y * V);

		float3 dir = normalize(ray.direction);
		float lengthL = length(Ll);
		Ll = normalize(Ll);

		float3 resultBRDF3 = lookup_brdf3(Ll, -dir, p_normal*editedNormal, n_tangent, n_bi_tangent);
		//resultBRDF += resultBRDF2 + resultBRDF3;
		float nDotL = dot(p_normal,Ll);
		float3 U, V, W;
		create_onb(L, U, V, W);
		float Ldist = length(LightPositionShadow);
		L = (1.0f / Ldist) * LightPositionShadow;
		float nDl = max(dot( p_normal*editedNormal, L),0.2);
		// cast shadow ray
		PerRayData_shadow shadow_prd;
		shadow_prd.attenuation = make_float3(1);
		optix::Ray shadow_ray = optix::make_Ray( hit_point, Ll, shadow_ray_type, scene_epsilon, Ldist );//
		rtTrace(top_shadower, shadow_ray, shadow_prd);
		//result += resultBRDF * make_float3(1.0,1.0,1.0) * nDl *lightColor;
		if(fmaxf(shadow_prd.attenuation) > 0) {
			float3 Lc = lightColor * shadow_prd.attenuation;
			//result += length(resultBRDF3)*nDl* make_float3( tex2D( texture_map3, texcoord.x, texcoord.y ) ) * Lc;//colorModifier*Lc;// +  (resultBRDF * make_float3(1.0,1.0,1.0) * nDl *lightColor * colorModifier);// + specularReflection;
      result += length(resultBRDF3)*nDl* colorModifier*Lc;// +  (resultBRDF * make_float3(1.0,1.0,1.0) * nDl *lightColor * colorModifier);// + specularReflection;
		}else{
			float3 Lc = lightColor*0.2;
			//result += length(resultBRDF3)*nDl* make_float3( tex2D( texture_map3, texcoord.x, texcoord.y ) ) * Lc;//colorModifier*Lc;// +  (resultBRDF * make_float3(1.0,1.0,1.0) * nDl *lightColor * colorModifier);// + specularReflection;
      result += length(resultBRDF3)*nDl* colorModifier*Lc;// +  (resultBRDF * make_float3(1.0,1.0,1.0) * nDl *lightColor * colorModifier);// + specularReflection;
		}
		//if(fmaxf(shadow_prd.attenuation) > 0) {
		//	float3 Lc = lightColor * shadow_prd.attenuation;
		//	result += length(resultBRDF3)*nDl* colorModifier*Lc;// +  (resultBRDF * make_float3(1.0,1.0,1.0) * nDl *lightColor * colorModifier);// + specularReflection;
		//}else{
		//	float3 Lc = lightColor*0.2;
		//	result += length(resultBRDF3)*nDl* colorModifier*Lc;// +  (resultBRDF * make_float3(1.0,1.0,1.0) * nDl *lightColor * colorModifier);// + specularReflection;
		//}

		}
		float3 hit_point2 = ray.origin + (t_hit+1) * ray.direction;
		prd_radiance.result =// make_float3( tex2D( texture_map, texcoord.x, texcoord.y ) ) * 
		texcoord * make_float3(result.x,result.y,result.z);	
    //float3 tt = length(make_float3( tex2D( texture_map2, texcoord.x*1.5, texcoord.y*1.5 ) )) * make_float3(1.2f,1.2f,1.2f) + make_float3(0.2,0.2,0.2);
    //prd_radiance.result =  tt * make_float3(result.x,result.y,result.z);
    //prd_radiance.result = make_float3(tex2D( texture_map3, texcoord.x, texcoord.y )); //arp texture only
    //prd_radiance.result = make_float3(result.x,result.y,result.z);	//arp lighting and teture
}