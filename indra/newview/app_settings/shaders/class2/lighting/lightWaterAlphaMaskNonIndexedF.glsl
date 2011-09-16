/** 
 * @file lightWaterAlphaMaskNonIndexedF.glsl
 *
 * $LicenseInfo:firstyear=2011&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2011, Linden Research, Inc.
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
 
uniform float minimum_alpha;
uniform float maximum_alpha;

uniform sampler2D diffuseMap;

vec3 atmosLighting(vec3 light);
vec4 applyWaterFog(vec4 color);

VARYING vec4 vertex_color;
VARYING vec2 vary_texcoord0;

void default_lighting_water()
{
	vec4 color = texture2D(diffuseMap,vary_texcoord0.xy) * vertex_color;

	if (color.a < minimum_alpha || color.a > maximum_alpha)
	{
		discard;
	}

	color.rgb = atmosLighting(color.rgb);

	color = applyWaterFog(color);
	
	gl_FragColor = color;
}

