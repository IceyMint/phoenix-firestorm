/** 
 * @file shadowAlphaMaskF.glsl
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

/*[EXTRA_CODE_HERE]*/

#ifdef DEFINE_GL_FRAGCOLOR
out vec4 frag_color;
#else
#define frag_color gl_FragColor
#endif

uniform sampler2D diffuseMap;

VARYING vec4 post_pos;
VARYING float target_pos_x;
VARYING vec4 vertex_color;
VARYING vec2 vary_texcoord0;
uniform float minimum_alpha;

void main() 
{
	float alpha = diffuseLookup(vary_texcoord0.xy).a;

    // mask cutoff 0 -> no shadow SL-11051
    if (minimum_alpha == 0)
    {
        discard;
    }

#if !defined(IS_FULLBRIGHT)
    alpha *= vertex_color.a;
#endif

	if (alpha < 0.05) // treat as totally transparent
	{
		discard;
	}

	if (alpha < 0.88) // treat as semi-transparent
	{
	  if (fract(0.5*floor(target_pos_x / post_pos.w )) < 0.25)
	  {
	    discard;
	  }
	}

	frag_color = vec4(1,1,1,1);
	
#if !defined(DEPTH_CLAMP)
	gl_FragDepth = max(post_pos.z/post_pos.w*0.5+0.5, 0.0);
#endif
}
