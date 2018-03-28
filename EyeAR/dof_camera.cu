
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

#include "helpers.h"

using namespace optix;

struct PerRayData_radiance
{
  float3 result;
  float importance;
  int depth;
  bool hasHitObject;
  bool hasHitScreen;
  float2 screenUV;
};

//new variables
rtDeclareVariable(int, visualizeScreen, , ) = 0;
rtDeclareVariable(int, useScreenSample, , ) = 0;
rtDeclareVariable(float3, upVec,,);
rtDeclareVariable(float, _cx, ,);
rtDeclareVariable(float, _cy, ,);
rtDeclareVariable(int, useDepthOfField, , ) = 1;

rtDeclareVariable(unsigned int, sampleSize, , )=2;
rtDeclareVariable(PerRayData_radiance, prd_radiance, rtPayload, );


rtDeclareVariable(float3,        eye, , ) = { 1.0f, 0.0f, 0.0f };
rtDeclareVariable(float3,        U, , )   = { 0.0f, 1.0f, 0.0f };
rtDeclareVariable(float3,        V, , )   = { 0.0f, 0.0f, 1.0f };
rtDeclareVariable(float3,        W, , )   = { -1.0f, 0.0f, 0.0f };
rtDeclareVariable(float3,        bad_color, , );
rtDeclareVariable(float,         scene_epsilon, , ) = 0.1f;
rtDeclareVariable(rtObject,      top_object, , );
//new variable
rtDeclareVariable(rtObject,      screen_object, , );

rtDeclareVariable(unsigned int,  radiance_ray_type, , );
rtDeclareVariable(int, display_mode,,) = 1;

//new variables
rtBuffer<float3, 2>              output_screen_buffer_f3;
rtBuffer<float4, 2>              output_screen_buffer_f4;

rtBuffer<float3, 2>              output_buffer_f3;
rtBuffer<float4, 2>              output_buffer_f4;
rtDeclareVariable(int,  output_format, , ) = RT_FORMAT_FLOAT4;

//new variables
rtDeclareVariable(uint, screen_sizeX, , );
rtDeclareVariable(uint, screen_sizeY, , );

rtDeclareVariable(uint2, launch_index, rtLaunchIndex, );
rtDeclareVariable(float, time_view_scale, , ) = 1e-6f;

//eye parameters
rtDeclareVariable(float, aperture_radius, , );
rtDeclareVariable(float, focal_scale, , );

rtDeclareVariable(unsigned int, frame_number, , );
rtDeclareVariable(float4, jitter, , );

// #define TIME_VIEW


__device__ __forceinline__ void write_output( float3 c )
{
  if ( output_format == RT_FORMAT_FLOAT4 ) {
    output_buffer_f4[launch_index] = make_float4(c, 1.f);
  }
  else {
    output_buffer_f3[launch_index] = c;
  }
}
__device__ __forceinline__ void write_output_screen(uint2 d, float3 c )
{
  if ( output_format == RT_FORMAT_FLOAT4 ) {
    output_buffer_f4[d] = make_float4(c, 1.f);
  }
  else {
    output_buffer_f3[d] = c;
  }
}
__device__ __forceinline__ float3 read_output()
{
  if ( output_format == RT_FORMAT_FLOAT4 ) {
    return make_float3( output_buffer_f4[launch_index] );
  }
  else {
    return output_buffer_f3[launch_index];
  }
}

__device__ __forceinline__ float3 read_output_screen(uint2 d)
{
  if ( output_format == RT_FORMAT_FLOAT4 ) {
    return make_float3( output_buffer_f4[d] );
  }
  else {
    return output_buffer_f3[d];
  }
}





RT_PROGRAM void dof_camera()
{
#ifdef TIME_VIEW
  clock_t t0 = clock(); 
#endif
  size_t2 screen = output_format == RT_FORMAT_FLOAT4 ? output_buffer_f4.size() : output_buffer_f3.size();

  // pixel sampling
  //fixed size float2 that is never used?
  //float2 l_i = make_float2(512,384);
  float2 pixel_sample = make_float2(launch_index) + make_float2(jitter.x, jitter.y);
  float2 d = pixel_sample / make_float2(screen) * 2.f - 1.f;

  // Calculate ray-viewplane intersection point
  float3 ray_origin = eye; 
  float3 ray_direction = d.x*U + d.y*V + W;
  float3 ray_target = ray_origin + focal_scale * ray_direction;

  // lens sampling
  float2 sample = optix::square_to_disk(make_float2(jitter.z, jitter.w));
  if(useDepthOfField == 1){
  ray_origin = ray_origin + aperture_radius * ( sample.x * normalize( U ) +  sample.y * normalize( V ) );
  ray_direction = normalize(ray_target - ray_origin);
  }
  // shoot ray
  optix::Ray ray = optix::make_Ray(ray_origin, ray_direction, radiance_ray_type, scene_epsilon, RT_DEFAULT_MAX);
  PerRayData_radiance prd;
  prd.importance = 1.f;
  prd.depth = 0;
  rtTrace(top_object, ray, prd);

  if (frame_number>1)
  {
    float a = 1.0f / (float)frame_number;
    float b = ((float)frame_number - 1.0f) * a;
    const float3 old_color = read_output();
		write_output(a * prd.result + b * old_color);
  }
  else
  {
	    write_output(prd.result);
  }
}
RT_PROGRAM void envmap_miss()
{
	prd_radiance.result = make_float3(0.0,0.0,0.0);
}
RT_PROGRAM void exception()
{
  write_output(make_float3(0.1f,0.5f,0.1f));
}
