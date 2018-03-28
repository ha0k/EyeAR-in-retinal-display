
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
rtDeclareVariable(rtObject,     top_shadower, , );
rtDeclareVariable(int,          max_depth, , );
rtDeclareVariable(float,        scene_epsilon, , );
rtDeclareVariable(float3,       ambient_light_color, , );
rtDeclareVariable(unsigned int, radiance_ray_type, , );
rtDeclareVariable(unsigned int, shadow_ray_type, , );
rtDeclareVariable(float3,       jitter, , );
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
rtDeclareVariable(float3, intersect_point, attribute intersect_point, ); 
rtDeclareVariable(PerRayData_radiance, prd_radiance, rtPayload, );
rtDeclareVariable(PerRayData_shadow,   prd_shadow,   rtPayload, );


//
rtBuffer<BasicLight> lights;
rtBuffer<float> brdf;

static __device__ void phongShadowed()
{
  // this material is opaque, so it fully attenuates all shadow rays
}

