
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

#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include "distributed_brdf.h"

#include <optixu/optixu_aabb.h>




// hitpoint parameters

// hitpoint parameters
rtDeclareVariable(float3, texcoord, attribute texcoord, ); 
rtDeclareVariable(float3, geometric_normal, attribute geometric_normal, ); 
rtDeclareVariable(float3, shading_normal, attribute shading_normal, ); 

//rtTextureSampler<float, 2>     texture_sampler_map;


RT_PROGRAM void any_hit_shadow()
{
  prd_shadow.attenuation = make_float3(0);
  rtTerminateRay();
}

RT_PROGRAM void closest_hit_radiance()
{


	  float3 world_shading_normal = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, shading_normal));
	  float3 world_geometric_normal = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, geometric_normal));
	  float3 ffnormal  = faceforward( world_shading_normal, -ray.direction, world_geometric_normal );
	  //PHONG SHADING
	  //phongShade( kd, ka, ks, ffnormal, phong_exp, reflectivity_n );
	  //BRDF SHADING
	  float3 uvw = texcoord; // testing
	  float3 texCol = make_float3(tex2D(texture_map2,uvw.x*1.5,uvw.y*1.5));
	//  if(uvw.x > 2.0)rtPrintf("LargerX");
	//  if(uvw.y > 2.0)rtPrintf("LargerY");
	//  if(uvw.x < -2.0)rtPrintf("SmallerX");
	//  if(uvw.y < -2.0)rtPrintf("SmallerY");
	//  float x_ = ((uvw.x/2)+0.5f) *256.0f;
	//  float y_ = ((uvw.y/2)+0.5f) *256.0f;
	//  int X = clamp(min(255, (int)x_),0,255);
	//  int Y = clamp(min(255, (int)y_),0,255);
	//  int index = (Y*256)+(X); 
	//  float color = clamp(texture_map[index],0.2f,1.0f);
	//  rtPrintf("X, %d - Y, %d)\n", X, Y);
	 // float3 kas22 = make_float3( color,color,color);
	 float3 textureCol = length(texCol)*make_float3(1.5f,1.5f,1.5f);
	  textureCol += make_float3(0.2,0.2,0.2);
    //textureCol = texcoord; //arp
	  BRDF(ffnormal, textureCol,make_float3(1,1,1));

}