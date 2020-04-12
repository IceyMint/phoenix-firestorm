/** 
 * @file class3/deferred/cloudsF.glsl
 *
 * $LicenseInfo:firstyear=2005&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2005, Linden Research, Inc.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation;
 * version 2.1 of the License only.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 * 
 * Linden Research, Inc., 945 Battery Street, San Francisco, CA  94111  USA
 * $/LicenseInfo$
 */
 
#ifdef DEFINE_GL_FRAGCOLOR
out vec4 frag_color;
#else
#define frag_color gl_FragColor
#endif

uniform sampler2D diffuseMap;

VARYING vec4 pos;
VARYING float target_pos_x;
VARYING float vary_CloudDensity;
VARYING vec2 vary_texcoord0;
VARYING vec2 vary_texcoord1;
VARYING vec2 vary_texcoord2;
VARYING vec2 vary_texcoord3;

uniform sampler2D cloud_noise_texture;
uniform sampler2D cloud_noise_texture_next;
uniform float blend_factor;
uniform vec4 cloud_pos_density1;
uniform vec4 cloud_pos_density2;
uniform vec4 sunlight_color;
uniform vec4 cloud_color;
uniform float cloud_shadow;
uniform float cloud_scale;
uniform float cloud_variance;
uniform vec3 camPosLocal;
uniform vec3 sun_dir;
uniform float sun_size;
uniform float far_z;

#if !defined(DEPTH_CLAMP)
VARYING vec4 post_pos;
#endif

vec4 cloudNoise(vec2 uv)
{
   vec4 a = texture2D(cloud_noise_texture, uv);
   vec4 b = texture2D(cloud_noise_texture_next, uv);
   vec4 cloud_noise_sample = mix(a, b, blend_factor);
   return normalize(cloud_noise_sample);
}

void main()
{
    // Set variables
    vec2 uv1 = vary_texcoord0.xy;
    vec2 uv2 = vary_texcoord1.xy;
    vec2 uv3 = vary_texcoord2.xy;
    float cloudDensity = 2.0 * (cloud_shadow - 0.25);

    if (cloud_scale >= 0.0001)
    {
        vec2 uv4 = vary_texcoord3.xy;
    
        vec2 disturbance  = vec2(cloudNoise(uv1 / 8.0f).x, cloudNoise((uv3 + uv1) / 16.0f).x) * cloud_variance * (1.0f - cloud_scale * 0.25f);
        vec2 disturbance2 = vec2(cloudNoise((uv1 + uv3) / 4.0f).x, cloudNoise((uv4 + uv2) / 8.0f).x) * cloud_variance * (1.0f - cloud_scale * 0.25f);
    
        // Offset texture coords
        uv1 += cloud_pos_density1.xy + (disturbance * 0.2);    //large texture, visible density
        uv2 += cloud_pos_density1.xy;   //large texture, self shadow
        uv3 += cloud_pos_density2.xy;   //small texture, visible density
        uv4 += cloud_pos_density2.xy;   //small texture, self shadow
    
        float density_variance = min(1.0, (disturbance.x* 2.0 + disturbance.y* 2.0 + disturbance2.x + disturbance2.y) * 4.0);
    
        cloudDensity *= 1.0 - (density_variance * density_variance);
    
        // Compute alpha1, the main cloud opacity
        float alpha1 = (cloudNoise(uv1).x - 0.5) + (cloudNoise(uv3).x - 0.5) * cloud_pos_density2.z;
        alpha1 = min(max(alpha1 + cloudDensity, 0.) * 10 * cloud_pos_density1.z, 1.);
    
        // And smooth
        alpha1 = 1. - alpha1 * alpha1;
        alpha1 = 1. - alpha1 * alpha1;  
    
        if (alpha1 < 0.001f)
        {
            discard;
        }
    
        // Compute alpha2, for self shadowing effect
        // (1 - alpha2) will later be used as percentage of incoming sunlight
        float alpha2 = (cloudNoise(uv2).x - 0.5);
        alpha2 = min(max(alpha2 + cloudDensity, 0.) * 2.5 * cloud_pos_density1.z, 1.);
    
        // And smooth
        alpha2 = 1. - alpha2;
        alpha2 = 1. - alpha2 * alpha2;  
    
        frag_color = vec4(alpha1, alpha1, alpha1, 1);
    }
    else
    {
        frag_color = vec4(1);
    }

#if !defined(DEPTH_CLAMP)
    gl_FragDepth = max(post_pos.z/post_pos.w*0.5+0.5, 0.0);
#endif

}
