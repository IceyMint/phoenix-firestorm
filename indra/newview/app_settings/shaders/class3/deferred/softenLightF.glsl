/**
 * @file class3/deferred/softenLightF.glsl
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

#define DEBUG_PBR_PACKORM0        0 // Rough=0, Metal=0
#define DEBUG_PBR_PACKORM1        0 // Rough=1, Metal=1
#define DEBUG_PBR_TANGENT1        1 // Tangent = 1,0,0
#define DEBUG_PBR_VERT2CAM1       0 // vertex2camera = 0,0,1

#define DEBUG_PBR_RAW_DIFF         0 // Output: use diffuse in G-Buffer
#define DEBUG_PBR_RAW_SPEC         0 // Output: use spec in G-Buffer
#define DEBUG_PBR_IRRADIANCE       0 // Output: Diffuse Irradiance
#define DEBUG_PBR_DIFFUSE          0 // Output: Radiance Lambertian
#define DEBUG_PBR_ORM              0 // Output: Packed Occlusion Roughness Metal
#define DEBUG_PBR_ROUGH_PERCEPTUAL 0 // Output: grayscale Perceptual Roughenss
#define DEBUG_PBR_ROUGH_ALPHA      0 // Output: grayscale Alpha Roughness
#define DEBUG_PBR_METAL            0 // Output: grayscale metal
#define DEBUG_PBR_REFLECTANCE      0 // Output: diffuse reflectance
#define DEBUG_PBR_BRDF_UV          0 // Output: red green BRDF UV         (GGX input)
#define DEBUG_PBR_BRDF_SCALE_BIAS  0 // Output: red green BRDF Scale Bias (GGX output)
#define DEBUG_PBR_SPEC             0 // Output: Final spec
#define DEBUG_PBR_SPEC_REFLECTION  0 // Output: reflection
#define DEBUG_PBR_NORMAL           0 // Output: passed in normal. To see raw normal map: set DEBUG_PBR_RAW_DIFF 1, and in pbropaqueF set DEBUG_NORMAL_RAW
#define DEBUG_PBR_TANGENT          0 // Output: Tangent
#define DEBUG_PBR_BITANGET         0 // Output: Bitangnet
#define DEBUG_PBR_V2C_RAW          0 // Output: vertex2camera
#define DEBUG_PBR_V2C_REMAP        0 // Output: vertex2camera (remap [-1,1] -> [0,1])
#define DEBUG_PBR_BRDF             0 // Output: Environment BRDF
#define DEBUG_PBR_DOT_NV           0 // Output: grayscale dot(Normal,ViewDir)
#define DEBUG_PBR_DOT_TV           0 // Output:
#define DEBUG_PBR_DOT_BV           0 // Output:
#define DEBUG_PBR_FRESNEL          0 // Output: roughness dependent fresnel

#extension GL_ARB_texture_rectangle : enable
#extension GL_ARB_shader_texture_lod : enable

#define FLT_MAX 3.402823466e+38

#define REFMAP_COUNT 256
#define REF_SAMPLE_COUNT 64 //maximum number of samples to consider

#ifdef DEFINE_GL_FRAGCOLOR
out vec4 frag_color;
#else
#define frag_color gl_FragColor
#endif

uniform sampler2DRect diffuseRect;
uniform sampler2DRect specularRect;
uniform sampler2DRect normalMap;

#if defined(HAS_SUN_SHADOW) || defined(HAS_SSAO)
uniform sampler2DRect lightMap;
#endif

uniform sampler2DRect depthMap;
uniform sampler2D     lightFunc;

uniform float blur_size;
uniform float blur_fidelity;

// Inputs
uniform mat3 env_mat;

uniform vec3 sun_dir;
uniform vec3 moon_dir;
uniform int  sun_up_factor;
VARYING vec2 vary_fragcoord;

uniform mat4 inv_proj;
uniform vec2 screen_res;

vec3 getNorm(vec2 pos_screen);
vec4 getPositionWithDepth(vec2 pos_screen, float depth);

void calcAtmosphericVars(vec3 inPositionEye, vec3 light_dir, float ambFactor, out vec3 sunlit, out vec3 amblit, out vec3 additive, out vec3 atten, bool use_ao);
float getAmbientClamp();
vec3  atmosFragLighting(vec3 l, vec3 additive, vec3 atten);
vec3  scaleSoftClipFrag(vec3 l);
vec3  fullbrightAtmosTransportFrag(vec3 light, vec3 additive, vec3 atten);
vec3  fullbrightScaleSoftClip(vec3 light);

// reflection probe interface
void sampleReflectionProbes(inout vec3 ambenv, inout vec3 glossenv, inout vec3 legacyEnv, 
        vec3 pos, vec3 norm, float glossiness, float envIntensity);
void applyGlossEnv(inout vec3 color, vec3 glossenv, vec4 spec, vec3 pos, vec3 norm);
void applyLegacyEnv(inout vec3 color, vec3 legacyenv, vec4 spec, vec3 pos, vec3 norm, float envIntensity);

vec3 linear_to_srgb(vec3 c);
vec3 srgb_to_linear(vec3 c);

#ifdef WATER_FOG
vec4 applyWaterFogView(vec3 pos, vec4 color);
#endif

uniform vec3 view_dir; // PBR

#define getDiffuseLightPBR(n)      ambenv
#define getSpecularPBR(reflection) glossenv

// Approximate Environment BRDF
vec2 getGGXApprox( vec2 uv )
{
    vec2  st    = vec2(1.) - uv;
    float d     = (st.x * st.x * 0.5) * (st.y * st.y);
    float scale = 1.0 - d;
    float bias  = d;
    return vec2( scale, bias );
}

vec2 getGGX( vec2 brdfPoint )
{
    // TODO: use GGXLUT
    // texture2D(GGXLUT, brdfPoint).rg;
    return getGGXApprox( brdfPoint);
}

vec3 calcBaseReflect0(float ior)
{
    vec3   reflect0 = vec3(pow((ior - 1.0) / (ior + 1.0), 2.0));
    return reflect0;
}

void main()
{
    vec2  tc           = vary_fragcoord.xy;
    float depth        = texture2DRect(depthMap, tc.xy).r;
    vec4  pos          = getPositionWithDepth(tc, depth);
    vec4  norm         = texture2DRect(normalMap, tc);
    float envIntensity = norm.z;
    norm.xyz           = getNorm(tc);

    vec3  light_dir   = (sun_up_factor == 1) ? sun_dir : moon_dir;
    float da          = clamp(dot(norm.xyz, light_dir.xyz), 0.0, 1.0);
    float light_gamma = 1.0 / 1.3;
    da                = pow(da, light_gamma);

    vec4 diffuse     = texture2DRect(diffuseRect, tc);
         diffuse.rgb = linear_to_srgb(diffuse.rgb); // SL-14025
    vec4 spec        = texture2DRect(specularRect, vary_fragcoord.xy);


#if defined(HAS_SUN_SHADOW) || defined(HAS_SSAO)
    vec2 scol_ambocc = texture2DRect(lightMap, vary_fragcoord.xy).rg;
    scol_ambocc      = pow(scol_ambocc, vec2(light_gamma));
    float scol       = max(scol_ambocc.r, diffuse.a);
    float ambocc     = scol_ambocc.g;
#else
    float scol = 1.0;
    float ambocc = 1.0;
#endif

    vec3  color = vec3(0);
    float bloom = 0.0;

    vec3 sunlit;
    vec3 amblit;
    vec3 additive;
    vec3 atten;

    calcAtmosphericVars(pos.xyz, light_dir, ambocc, sunlit, amblit, additive, atten, true);

    //vec3 amb_vec = env_mat * norm.xyz;

    vec3 ambenv;
    vec3 glossenv;
    vec3 legacyenv;
    sampleReflectionProbes(ambenv, glossenv, legacyenv, pos.xyz, norm.xyz, spec.a, envIntensity);

    bool hasPBR = GET_GBUFFER_FLAG(GBUFFER_FLAG_HAS_PBR);
    if (hasPBR)
    {
        vec3 colorDiffuse      = vec3(0);
        vec3 colorEmissive     = vec3(0);
        vec3 colorSpec         = vec3(0);
//      vec3 colorClearCoat    = vec3(0);
//      vec3 colorSheen        = vec3(0);
//      vec3 colorTransmission = vec3(0);

        vec3 packedORM        = spec.rgb; // Packed: Occlusion Roughness Metal
#if DEBUG_PBR_PACK_ORM0
             packedORM        = vec3(0,0,0);
#endif
#if DEBUG_PBR_PACK_ORM1
             packedORM        = vec3(1,1,1);
#endif
        float IOR             = 1.5;         // default Index Of Reflection 1.5
        vec3  reflect0        = vec3(0.04);  // -> incidence reflectance 0.04

        IOR = 0.0; // TODO: Set from glb
        reflect0 = calcBaseReflect0(IOR);

        float metal      = packedORM.b;
        vec3  reflect90  = vec3(0);
        vec3  v          = -normalize(pos.xyz);
#if DEBUG_PBR_VERT2CAM1
              v = vec3(0,0,1);
#endif
        vec3  n          = norm.xyz;
//      vec3  t          = texture2DRect(tangentMap, tc).rgb;
#if DEBUG_PBR_TANGENT1
        vec3  t          = vec3(1,0,0);
#endif
        vec3  b          = cross( n,t);
        vec3  reflectVN  = normalize(reflect(-v,n));

        float dotNV = clamp(dot(n,v),0,1);
        float dotTV = clamp(dot(t,v),0,1);
        float dotBV = clamp(dot(b,v),0,1);

        // Reference: getMetallicRoughnessInfo
        float perceptualRough = packedORM.g;
        float alphaRough     = perceptualRough * perceptualRough;
        vec3  colorDiff      = mix( diffuse.rgb, vec3(0)    , metal);
              reflect0       = mix( reflect0   , diffuse.rgb, metal); // reflect at 0 degrees
              reflect90      = vec3(1);                               // reflect at 90 degrees
        float reflectance    = max( max( reflect0.r, reflect0.g ), reflect0.b );

        // Common to RadianceGGX and RadianceLambertian
        float specWeight = 1.0;
        vec2  brdfPoint  = clamp(vec2(dotNV, perceptualRough), vec2(0,0), vec2(1,1));
        vec2  vScaleBias = getGGX( brdfPoint); // Environment BRDF: scale and bias applied to reflect0
        vec3  fresnelR   = max(vec3(1.0 - perceptualRough), reflect0) - reflect0; // roughness dependent fresnel
        vec3  kSpec      = reflect0 + fresnelR*pow(1.0 - dotNV, 5.0);

        // Reference: getIBLRadianceGGX
        vec3 specLight  = getSpecularPBR(reflection);
#if HAS_IBL
        kSpec          = mix( kSpec, iridescenceFresnel, iridescenceFactor);
#endif
        vec3 FssEssRadiance = kSpec*vScaleBias.x + vScaleBias.y;
        colorSpec += specWeight * specLight * FssEssRadiance;

        // Reference: getIBLRadianceLambertian
        vec3  irradiance    = getDiffuseLightPBR(n);
        vec3  FssEssLambert = specWeight * kSpec * vScaleBias.x + vScaleBias.y; // NOTE: Very similar to FssEssRadiance but with extra specWeight term
        float Ems          = (1.0 - vScaleBias.x + vScaleBias.y);
        vec3  avg          = specWeight * (reflect0 + (1.0 - reflect0) / 21.0);
        vec3  AvgEms       = avg * Ems;
        vec3  FmsEms       = AvgEms * FssEssLambert / (1.0 - AvgEms);
        vec3  kDiffuse     = colorDiffuse * (1.0 - FssEssLambert + FmsEms);
        colorDiffuse      += (FmsEms + kDiffuse) * irradiance;

        color.rgb  = colorDiffuse + colorEmissive + colorSpec;

    #if DEBUG_PBR_BRDF_UV
        color.rgb = vec3(brdfPoint,0.0);
    #endif
    #if DEBUG_PBR_BRDF_SCALE_BIAS
        color.rgb = vec3(vScaleBias,0.0);
    #endif
    #if DEBUG_PBR_FRESNEL
        color.rgb = fresnelR;
    #endif
    #if DEBUG_PBR_RAW_DIFF
        color.rgb = diffuse.rgb;
    #endif
    #if DEBUG_PBR_RAW_SPEC
        color.rgb = spec.rgb;
    #endif
    #if DEBUG_PBR_REFLECTANCE
        color.rgb = vec3(reflectance);
    #endif
    #if DEBUG_PBR_IRRADIANCE
        color.rgb = irradiance;
    #endif
    #if DEBUG_PBR_DIFFUSE
        color.rgb = colorDiffuse;
    #endif
    #if DEBUG_PBR_EMISSIVE
        color.rgb = colorEmissive;
    #endif
    #if DEBUG_PBR_METAL
        color.rgb = vec3(metal);
    #endif
    #if DEBUG_PBR_ROUGH_PERCEPTUAL
        color.rgb = vec3(perceptualRough);
    #endif
    #if DEBUG_PBR_ROUGH_ALPHA
        color.rgb = vec3(alphaRough);
    #endif
    #if DEBUG_PBR_SPEC
        color.rgb = colorSpec;
    #endif
    #if DEBUG_PBR_SPEC_REFLECTION
        color.rgb = specLight;
    #endif
    #if DEBUG_PBR_ORM
        color.rgb = packedORM;
    #endif
    #if DEBUG_PBR_NORMAL
        color.rgb = norm.xyz;
    #endif
    #if DEBUG_PBR_TANGENT
        color.rgb = t;
    #endif
    #if DEBUG_PBR_BITANGENT
        color.rgb = b;
    #endif
    #if DEBUG_PBR_V2C_RAW
        color.rgb = v;
    #endif
    #if DEBUG_PBR_V2C_REMAP
        color.rgb = v*0.5 + vec3(0.5);
    #endif
    #if DEBUG_PBR_DOT_NV
        color.rgb = vec3(dotNV);
    #endif
    #if DEBUG_PBR_DOT_TV
        color.rgb = vec3(dotTV);
    #endif
    #if DEBUG_PBR_DOT_BV
        color.rgb = vec3(dotBV);
    #endif
    }
else
{
    amblit = max(ambenv, amblit);
    color.rgb = amblit*ambocc;

    //float ambient = min(abs(dot(norm.xyz, sun_dir.xyz)), 1.0);
    //ambient *= 0.5;
    //ambient *= ambient;
    //ambient = (1.0 - ambient);
    //color.rgb *= ambient;

    vec3 sun_contrib = min(da, scol) * sunlit;
    color.rgb += sun_contrib;
    color.rgb = min(color.rgb, vec3(1,1,1));
    color.rgb *= diffuse.rgb;

    vec3 refnormpersp = reflect(pos.xyz, norm.xyz);

    if (spec.a > 0.0)  // specular reflection
    {
        float sa        = dot(normalize(refnormpersp), light_dir.xyz);
        vec3  dumbshiny = sunlit * scol * (texture2D(lightFunc, vec2(sa, spec.a)).r);

        // add the two types of shiny together
        vec3 spec_contrib = dumbshiny * spec.rgb;
        bloom             = dot(spec_contrib, spec_contrib) / 6;
        color.rgb += spec_contrib;

        // add reflection map - EXPERIMENTAL WORK IN PROGRESS
        applyGlossEnv(color, glossenv, spec, pos.xyz, norm.xyz);
    }

    color.rgb = mix(color.rgb, diffuse.rgb, diffuse.a);

    if (envIntensity > 0.0)
    {  // add environmentmap
        //fudge darker
        legacyenv *= 0.5*diffuse.a+0.5;;
        applyLegacyEnv(color, legacyenv, spec, pos.xyz, norm.xyz, envIntensity);
    }

    if (GET_GBUFFER_FLAG(GBUFFER_FLAG_HAS_ATMOS))
    {
        color = mix(atmosFragLighting(color, additive, atten), fullbrightAtmosTransportFrag(color, additive, atten), diffuse.a);
        color = mix(scaleSoftClipFrag(color), fullbrightScaleSoftClip(color), diffuse.a);
    }

#ifdef WATER_FOG
    vec4 fogged = applyWaterFogView(pos.xyz, vec4(color, bloom));
    color       = fogged.rgb;
    bloom       = fogged.a;
#endif
}
    // convert to linear as fullscreen lights need to sum in linear colorspace
    // and will be gamma (re)corrected downstream...
    //color = vec3(ambocc);
    //color = ambenv;
    //color.b = diffuse.a;
    frag_color.rgb = srgb_to_linear(color.rgb);
    frag_color.a = bloom;
}
