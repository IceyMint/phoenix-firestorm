/** 
 * @file avatarAlphaShadowF.glsl
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

uniform float minimum_alpha;

uniform sampler2D diffuseMap;

#if !defined(DEPTH_CLAMP)
VARYING vec4 post_pos;
#endif

VARYING float pos_w;
VARYING float target_pos_x;
VARYING vec2 vary_texcoord0;
uniform vec4 color;

void main() 
{
	float alpha = texture2D(diffuseMap, vary_texcoord0.xy).a * color.a;

	if (alpha < 0.05) // treat as totally transparent
	{
		discard;
	}

	if (alpha < minimum_alpha) // treat as semi-transparent
	{
	  if (fract(0.5*floor(target_pos_x / pos_w )) < 0.25)
	  {
	    discard;
	  }
	}

	frag_color = vec4(1,1,1,1);
	
#if !defined(DEPTH_CLAMP)
	gl_FragDepth = max(post_pos.z/post_pos.w*0.5+0.5, 0.0);
#endif

}
