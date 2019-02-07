/** 
 * @file moonF.glsl
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
 
#extension GL_ARB_texture_rectangle : enable

/*[EXTRA_CODE_HERE]*/

#ifdef DEFINE_GL_FRAGCOLOR
out vec4 frag_data[3];
#else
#define frag_data gl_FragData
#endif

uniform vec4 color;
uniform vec4 sunlight_color;
uniform vec4 moonlight_color;
uniform vec3 lumWeights;
uniform float moon_brightness;
uniform float minLuminance;
uniform sampler2D diffuseMap;
uniform sampler2D altDiffuseMap;
uniform float blend_factor; // interp factor between moon A/B
VARYING vec2 vary_texcoord0;

void main() 
{
    vec4 moonA = texture2D(diffuseMap, vary_texcoord0.xy);
    vec4 moonB = texture2D(altDiffuseMap, vary_texcoord0.xy);
    vec4 c     = mix(moonA, moonB, blend_factor);

    // mix factor which blends when sunlight is brighter
    // and shows true moon color at night
    vec3 luma_weights = vec3(0.2, 0.3, 0.2);

    float mix = 1.0 - dot(normalize(sunlight_color.rgb), luma_weights);

    vec3 exp = vec3(1.0 - mix * moon_brightness) * 2.0  - 1.0;
    c.rgb = pow(c.rgb, exp);
    //c.rgb *= moonlight_color.rgb;

    frag_data[0] = vec4(c.rgb, c.a);
    frag_data[1] = vec4(0.0);
    frag_data[2] = vec4(0.0f);

    gl_FragDepth = 0.9996f;
}

