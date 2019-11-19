/** 
 * @file class2\wl\transportF.glsl
 *
 * $LicenseInfo:firstyear=2007&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2007, Linden Research, Inc.
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
 
//////////////////////////////////////////////////////////
// The fragment shader for the terrain atmospherics
//////////////////////////////////////////////////////////

vec3 getAdditiveColor();
vec3 getAtmosAttenuation();

uniform int no_atmo;

vec3 atmosTransportFrag(vec3 light, vec3 additive, vec3 atten)
{
    if (no_atmo == 1)
    {
        return light * 2.0;
    }
    light *= atten.r;
    light += additive;
    return light * 2.0;
}

vec3 atmosTransport(vec3 light)
{
     return atmosTransportFrag(light, getAdditiveColor(), getAtmosAttenuation());
}

vec3 fullbrightAtmosTransport(vec3 light)
{
    float brightness = dot(light.rgb * 0.5, vec3(0.3333)) + 0.1;
    vec3 attenColor = atmosTransportFrag(light * 0.5, getAdditiveColor() * brightness, getAtmosAttenuation());

    // attenColor is an accurate fog-attenuated result for any brightness
    // But, the pre-EEP shader included a brightness-indexed lerp to a non-attenuated version
    // of the color - effectively a fog 'burn-through' for very bright pixels. To more closely
    // match the pre-EEP behavior, we'll also lerp to the pre-EEP color, based on overall brightness
    float preEepBright = dot(light.rgb, vec3(0.3333));
    retun mix(attenColor, (light.rgb + getAdditiveColor().rgb) * (2.0 - preEepBright), preEepBright * preEepBright);
}

vec3 fullbrightShinyAtmosTransport(vec3 light)
{
    float brightness = dot(light.rgb * 0.5, vec3(0.33333)) + 0.1;
    return atmosTransportFrag(light * 0.5, getAdditiveColor() * (brightness * brightness), getAtmosAttenuation());
}
