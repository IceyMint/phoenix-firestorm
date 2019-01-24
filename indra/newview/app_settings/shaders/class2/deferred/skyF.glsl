/** 
 * @file class2/deferred/skyF.glsl
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

uniform mat4 modelview_projection_matrix;

// SKY ////////////////////////////////////////////////////////////////////////
// The vertex shader for creating the atmospheric sky
///////////////////////////////////////////////////////////////////////////////

// Inputs
uniform vec3 camPosLocal;

uniform vec4 lightnorm;
uniform vec4 sunlight_color;
uniform vec4 ambient;
uniform vec4 blue_horizon;
uniform vec4 blue_density;
uniform float haze_horizon;
uniform float haze_density;

uniform float cloud_shadow;
uniform float density_multiplier;
uniform float max_y;

uniform vec4 glow;

uniform vec4 cloud_color;

#ifdef DEFINE_GL_FRAGCOLOR
out vec4 frag_data[3];
#else
#define frag_data gl_FragData
#endif

VARYING vec3 pos;

/////////////////////////////////////////////////////////////////////////
// The fragment shader for the sky
/////////////////////////////////////////////////////////////////////////

uniform vec4 gamma;
uniform sampler2D rainbow_map;
uniform sampler2D halo_map;

uniform float moisture_level;
uniform float droplet_radius;
uniform float ice_level;

vec3 rainbow(float d)
{
   d = clamp(d, -1, 0);
   float rad = (droplet_radius - 5.0f) / 1024.0f;
   return pow(texture2D(rainbow_map, vec2(rad, d)).rgb, vec3(1.8)) * moisture_level;
}

vec3 halo22(float d)
{
   float v = sqrt(max(0, 1 - (d*d)));
   return texture2D(halo_map, vec2(0, v)).rgb * ice_level;
}

/// Soft clips the light with a gamma correction
vec3 scaleSoftClip(vec3 light);

void main()
{

	// World / view / projection
	// Get relative position
	vec3 P = pos.xyz - camPosLocal.xyz + vec3(0,50,0);

	// Set altitude
	if (P.y > 0.)
	{
		P *= (max_y / P.y);
	}
	else
	{
		P *= (-32000. / P.y);
	}

	// Can normalize then
	vec3 Pn = normalize(P);
	float  Plen = length(P);

	// Initialize temp variables
	vec4 temp1 = vec4(0.);
	vec4 temp2 = vec4(0.);
	vec4 blue_weight;
	vec4 haze_weight;
	vec4 sunlight = sunlight_color;
	vec4 light_atten;


	// Sunlight attenuation effect (hue and brightness) due to atmosphere
	// this is used later for sunlight modulation at various altitudes
	light_atten = (blue_density + vec4(haze_density * 0.25)) * (density_multiplier * max_y);

	// Calculate relative weights
	temp1 = blue_density + haze_density;
	blue_weight = blue_density / temp1;
	haze_weight = haze_density / temp1;

	// Compute sunlight from P & lightnorm (for long rays like sky)
	temp2.y = max(0., max(0., Pn.y) * 1.0 + lightnorm.y );
	temp2.y = 1. / temp2.y;
	sunlight *= exp( - light_atten * temp2.y);

	// Distance
	temp2.z = Plen * density_multiplier;

	// Transparency (-> temp1)
	// ATI Bugfix -- can't store temp1*temp2.z in a variable because the ati
	// compiler gets confused.
	temp1 = exp(-temp1 * temp2.z);


	// Compute haze glow
	temp2.x = dot(Pn, lightnorm.xyz);
	temp2.x = 1. - temp2.x;
		// temp2.x is 0 at the sun and increases away from sun
	temp2.x = max(temp2.x, .001);	
		// Set a minimum "angle" (smaller glow.y allows tighter, brighter hotspot)
	temp2.x *= glow.x;
		// Higher glow.x gives dimmer glow (because next step is 1 / "angle")
	temp2.x = pow(temp2.x, glow.z);
		// glow.z should be negative, so we're doing a sort of (1 / "angle") function

	// Add "minimum anti-solar illumination"
	temp2.x += .25;


	// Haze color above cloud
	vec4 color = (	  blue_horizon * blue_weight * (sunlight + ambient)
				+ (haze_horizon * haze_weight) * (sunlight * temp2.x + ambient)
			 );	


	// Increase ambient when there are more clouds
	vec4 tmpAmbient = ambient;
	tmpAmbient += (1. - tmpAmbient) * cloud_shadow * 0.5; 

	// Dim sunlight by cloud shadow percentage
	sunlight *= (1. - cloud_shadow);

	// Haze color below cloud
	vec4 additiveColorBelowCloud = (	  blue_horizon * blue_weight * (sunlight + tmpAmbient)
				+ (haze_horizon * haze_weight) * (sunlight * temp2.x + tmpAmbient)
			 );	

	// Final atmosphere additive
	color *= (1. - temp1);
	
	// Attenuate cloud color by atmosphere
	temp1 = sqrt(temp1);	//less atmos opacity (more transparency) below clouds

	// At horizon, blend high altitude sky color towards the darker color below the clouds
	color += (additiveColorBelowCloud - color) * (1. - sqrt(temp1));
	
    float optic_d = dot(Pn, lightnorm.xyz);

    vec3 halo_22 = halo22(optic_d);

   color.rgb += rainbow(optic_d);

    color.rgb += halo_22;

	color *= 2.;

	/// Gamma correct for WL (soft clip effect).
	frag_data[0] = vec4(scaleSoftClip(color.rgb), 1.0);
	frag_data[1] = vec4(0.0,0.0,0.0,0.0);
	frag_data[2] = vec4(0.5,0.5,0.0,1.0); //1.0 in norm.w masks off fog
}

