/** 
 * @file llshadermgr.h
 * @brief Shader Manager
 *
 * $LicenseInfo:firstyear=2001&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2010, Linden Research, Inc.
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

#ifndef LL_SHADERMGR_H
#define LL_SHADERMGR_H

#include "llgl.h"
#include "llglslshader.h"

class LLShaderMgr
{
public:
	LLShaderMgr();
	virtual ~LLShaderMgr();

    // clang-format off
    typedef enum
    {                                       // Shader uniform name, set in LLShaderMgr::initAttribsAndUniforms()
        MODELVIEW_MATRIX = 0,               //  "modelview_matrix"
        PROJECTION_MATRIX,                  //  "projection_matrix"
        INVERSE_PROJECTION_MATRIX,          //  "inv_proj"
        MODELVIEW_PROJECTION_MATRIX,        //  "modelview_projection_matrix"
        INVERSE_MODELVIEW_MATRIX,           //  "inv_modelview"
        NORMAL_MATRIX,                      //  "normal_matrix"
        TEXTURE_MATRIX0,                    //  "texture_matrix0"
        TEXTURE_MATRIX1,                    //  "texture_matrix1"
        TEXTURE_MATRIX2,                    //  "texture_matrix2"
        TEXTURE_MATRIX3,                    //  "texture_matrix3"
        OBJECT_PLANE_S,                     //  "object_plane_s"
        OBJECT_PLANE_T,                     //  "object_plane_t"
        VIEWPORT,                           //  "viewport"
        LIGHT_POSITION,                     //  "light_position"
        LIGHT_DIRECTION,                    //  "light_direction"
        LIGHT_ATTENUATION,                  //  "light_attenuation"
        LIGHT_DIFFUSE,                      //  "light_diffuse"
        LIGHT_AMBIENT,                      //  "light_ambient"
        MULTI_LIGHT_COUNT,                  //  "light_count"
        MULTI_LIGHT,                        //  "light"
        MULTI_LIGHT_COL,                    //  "light_col"
        MULTI_LIGHT_FAR_Z,                  //  "far_z"
        PROJECTOR_MATRIX,                   //  "proj_mat"
        PROJECTOR_NEAR,                     //  "proj_near"
        PROJECTOR_P,                        //  "proj_p"
        PROJECTOR_N,                        //  "proj_n"
        PROJECTOR_ORIGIN,                   //  "proj_origin"
        PROJECTOR_RANGE,                    //  "proj_range"
        PROJECTOR_AMBIANCE,                 //  "proj_ambiance"
        PROJECTOR_SHADOW_INDEX,             //  "proj_shadow_idx"
        PROJECTOR_SHADOW_FADE,              //  "shadow_fade"
        PROJECTOR_FOCUS,                    //  "proj_focus"
        PROJECTOR_LOD,                      //  "proj_lod"
        PROJECTOR_AMBIENT_LOD,              //  "proj_ambient_lod"
        DIFFUSE_COLOR,                      //  "color"
        DIFFUSE_MAP,                        //  "diffuseMap"
        ALTERNATE_DIFFUSE_MAP,              //  "altDiffuseMap"
        SPECULAR_MAP,                       //  "specularMap"
        BUMP_MAP,                           //  "bumpMap"
        BUMP_MAP2,                          //  "bumpMap2"
        ENVIRONMENT_MAP,                    //  "environmentMap"
        CLOUD_NOISE_MAP,                    //  "cloud_noise_texture"
        CLOUD_NOISE_MAP_NEXT,               //  "cloud_noise_texture_next"
        FULLBRIGHT,                         //  "fullbright"
        LIGHTNORM,                          //  "lightnorm"
        SUNLIGHT_COLOR,                     //  "sunlight_color"
        AMBIENT,                            //  "ambient_color"
        BLUE_HORIZON,                       //  "blue_horizon"
        BLUE_DENSITY,                       //  "blue_density"
        HAZE_HORIZON,                       //  "haze_horizon"
        HAZE_DENSITY,                       //  "haze_density"
        CLOUD_SHADOW,                       //  "cloud_shadow"
        DENSITY_MULTIPLIER,                 //  "density_multiplier"
        DISTANCE_MULTIPLIER,                //  "distance_multiplier"
        MAX_Y,                              //  "max_y"
        GLOW,                               //  "glow"
        CLOUD_COLOR,                        //  "cloud_color"
        CLOUD_POS_DENSITY1,                 //  "cloud_pos_density1"
        CLOUD_POS_DENSITY2,                 //  "cloud_pos_density2"
        CLOUD_SCALE,                        //  "cloud_scale"
        GAMMA,                              //  "gamma"
        SCENE_LIGHT_STRENGTH,               //  "scene_light_strength"
        LIGHT_CENTER,                       //  "center"
        LIGHT_SIZE,                         //  "size"
        LIGHT_FALLOFF,                      //  "falloff"
        BOX_CENTER,                         //  "box_center"
        BOX_SIZE,                           //  "box_size"

        GLOW_MIN_LUMINANCE,                 //  "minLuminance"
        GLOW_MAX_EXTRACT_ALPHA,             //  "maxExtractAlpha"
        GLOW_LUM_WEIGHTS,                   //  "lumWeights"
        GLOW_WARMTH_WEIGHTS,                //  "warmthWeights"
        GLOW_WARMTH_AMOUNT,                 //  "warmthAmount"
        GLOW_STRENGTH,                      //  "glowStrength"
        GLOW_DELTA,                         //  "glowDelta"

        MINIMUM_ALPHA,                      //  "minimum_alpha"
        EMISSIVE_BRIGHTNESS,                //  "emissive_brightness"

        DEFERRED_SHADOW_MATRIX,             //  "shadow_matrix"
        DEFERRED_ENV_MAT,                   //  "env_mat"
        DEFERRED_SHADOW_CLIP,               //  "shadow_clip"
        DEFERRED_SUN_WASH,                  //  "sun_wash"
        DEFERRED_SHADOW_NOISE,              //  "shadow_noise"
        DEFERRED_BLUR_SIZE,                 //  "blur_size"
        DEFERRED_SSAO_RADIUS,               //  "ssao_radius"
        DEFERRED_SSAO_MAX_RADIUS,           //  "ssao_max_radius"
        DEFERRED_SSAO_FACTOR,               //  "ssao_factor"
        DEFERRED_SSAO_FACTOR_INV,           //  "ssao_factor_inv"
        DEFERRED_SSAO_EFFECT_MAT,           //  "ssao_effect_mat"
        DEFERRED_SCREEN_RES,                //  "screen_res"
        DEFERRED_NEAR_CLIP,                 //  "near_clip"
        DEFERRED_SHADOW_OFFSET,             //  "shadow_offset"
        DEFERRED_SHADOW_BIAS,               //  "shadow_bias"
        DEFERRED_SPOT_SHADOW_BIAS,          //  "spot_shadow_bias"
        DEFERRED_SPOT_SHADOW_OFFSET,        //  "spot_shadow_offset"
        DEFERRED_SUN_DIR,                   //  "sun_dir"
        DEFERRED_MOON_DIR,                  //  "moon_dir"
        DEFERRED_SHADOW_RES,                //  "shadow_res"
        DEFERRED_PROJ_SHADOW_RES,           //  "proj_shadow_res"
        DEFERRED_DEPTH_CUTOFF,              //  "depth_cutoff"
        DEFERRED_NORM_CUTOFF,               //  "norm_cutoff"
        DEFERRED_SHADOW_TARGET_WIDTH,       //  "shadow_target_width"

        FXAA_TC_SCALE,                      //  "tc_scale"
        FXAA_RCP_SCREEN_RES,                //  "rcp_screen_res"
        FXAA_RCP_FRAME_OPT,                 //  "rcp_frame_opt"
        FXAA_RCP_FRAME_OPT2,                //  "rcp_frame_opt2"

        DOF_FOCAL_DISTANCE,                 //  "focal_distance"
        DOF_BLUR_CONSTANT,                  //  "blur_constant"
        DOF_TAN_PIXEL_ANGLE,                //  "tan_pixel_angle"
        DOF_MAGNIFICATION,                  //  "magnification"
        DOF_MAX_COF,                        //  "max_cof"
        DOF_RES_SCALE,                      //  "res_scale"
        DOF_WIDTH,                          //  "dof_width"
        DOF_HEIGHT,                         //  "dof_height"

        DEFERRED_DEPTH,                     //  "depthMap"
        DEFERRED_SHADOW0,                   //  "shadowMap0"
        DEFERRED_SHADOW1,                   //  "shadowMap1"
        DEFERRED_SHADOW2,                   //  "shadowMap2"
        DEFERRED_SHADOW3,                   //  "shadowMap3"
        DEFERRED_SHADOW4,                   //  "shadowMap4"
        DEFERRED_SHADOW5,                   //  "shadowMap5"
        DEFERRED_NORMAL,                    //  "normalMap"
        DEFERRED_POSITION,                  //  "positionMap"
        DEFERRED_DIFFUSE,                   //  "diffuseRect"
        DEFERRED_SPECULAR,                  //  "specularRect"
        DEFERRED_NOISE,                     //  "noiseMap"
        DEFERRED_LIGHTFUNC,                 //  "lightFunc"
        DEFERRED_LIGHT,                     //  "lightMap"
        DEFERRED_BLOOM,                     //  "bloomMap"
        DEFERRED_PROJECTION,                //  "projectionMap"
        DEFERRED_NORM_MATRIX,               //  "norm_mat"
        TEXTURE_GAMMA,                      //  "texture_gamma"
        SPECULAR_COLOR,                     //  "specular_color"
        ENVIRONMENT_INTENSITY,              //  "env_intensity"

        AVATAR_MATRIX,                      //  "matrixPalette"
        AVATAR_TRANSLATION,                 //  "translationPalette"

        // <FS:CR> Import Vignette from Exodus
        EXO_RENDER_VIGNETTE,                //  "exo_vignette"
        EXO_RENDER_SCREEN,                  //  "exo_screen"
        // </FS:CR> Import Vignette from Exodus

        WATER_SCREENTEX,                    //  "screenTex"
        WATER_SCREENDEPTH,                  //  "screenDepth"
        WATER_REFTEX,                       //  "refTex"
        WATER_EYEVEC,                       //  "eyeVec"
        WATER_TIME,                         //  "time"
        WATER_WAVE_DIR1,                    //  "waveDir1"
        WATER_WAVE_DIR2,                    //  "waveDir2"
        WATER_LIGHT_DIR,                    //  "lightDir"
        WATER_SPECULAR,                     //  "specular"
        WATER_SPECULAR_EXP,                 //  "lightExp"
        WATER_FOGCOLOR,                     //  "waterFogColor"
        WATER_FOGDENSITY,                   //  "waterFogDensity"
        WATER_FOGKS,                        //  "waterFogKS"
        WATER_REFSCALE,                     //  "refScale"
        WATER_WATERHEIGHT,                  //  "waterHeight"
        WATER_WATERPLANE,                   //  "waterPlane"
        WATER_NORM_SCALE,                   //  "normScale"
        WATER_FRESNEL_SCALE,                //  "fresnelScale"
        WATER_FRESNEL_OFFSET,               //  "fresnelOffset"
        WATER_BLUR_MULTIPLIER,              //  "blurMultiplier"
        WATER_SUN_ANGLE,                    //  "sunAngle"
        WATER_SCALED_ANGLE,                 //  "scaledAngle"
        WATER_SUN_ANGLE2,                   //  "sunAngle2"

        WL_CAMPOSLOCAL,                     //  "camPosLocal"

        // [RLVa:KB] - @setsphere
        RLV_EFFECT_MODE,                    //  "rlvEffectMode"
        RLV_EFFECT_PARAM1,                  //  "rlvEffectParam1"
        RLV_EFFECT_PARAM2,                  //  "rlvEffectParam2"
        RLV_EFFECT_PARAM3,                  //  "rlvEffectParam3"
        RLV_EFFECT_PARAM4,                  //  "rlvEffectParam4"
        RLV_EFFECT_PARAM5,                  //  "rlvEffectParam5"
        // [/RLVa:KB]

        AVATAR_WIND,                        //  "gWindDir"
        AVATAR_SINWAVE,                     //  "gSinWaveParams"
        AVATAR_GRAVITY,                     //  "gGravity"

        TERRAIN_DETAIL0,                    //  "detail_0"
        TERRAIN_DETAIL1,                    //  "detail_1"
        TERRAIN_DETAIL2,                    //  "detail_2"
        TERRAIN_DETAIL3,                    //  "detail_3"
        TERRAIN_ALPHARAMP,                  //  "alpha_ramp"

        SHINY_ORIGIN,                       //  "origin"
        DISPLAY_GAMMA,                      //  "display_gamma"

        INSCATTER_RT,                       //  "inscatter"
        SUN_SIZE,                           //  "sun_size"
        FOG_COLOR,                          //  "fog_color"

        // precomputed textures
        TRANSMITTANCE_TEX,                  //  "transmittance_texture"
        SCATTER_TEX,                        //  "scattering_texture"
        SINGLE_MIE_SCATTER_TEX,             //  "single_mie_scattering_texture"
        ILLUMINANCE_TEX,                    //  "irradiance_texture"
        BLEND_FACTOR,                       //  "blend_factor"

        NO_ATMO,                            //  "no_atmo"
        MOISTURE_LEVEL,                     //  "moisture_level"
        DROPLET_RADIUS,                     //  "droplet_radius"
        ICE_LEVEL,                          //  "ice_level"
        RAINBOW_MAP,                        //  "rainbow_map"
        HALO_MAP,                           //  "halo_map"

        MOON_BRIGHTNESS,                    //  "moon_brightness"

        CLOUD_VARIANCE,                     //  "cloud_variance"

        SH_INPUT_L1R,                       //  "sh_input_r"
        SH_INPUT_L1G,                       //  "sh_input_g"
        SH_INPUT_L1B,                       //  "sh_input_b"

        SUN_MOON_GLOW_FACTOR,               //  "sun_moon_glow_factor"
        WATER_EDGE_FACTOR,                  //  "water_edge"
        SUN_UP_FACTOR,                      //  "sun_up_factor"
        MOONLIGHT_COLOR,                    //  "moonlight_color"
        END_RESERVED_UNIFORMS
    } eGLSLReservedUniforms;
    // clang-format on

	// singleton pattern implementation
	static LLShaderMgr * instance();

	virtual void initAttribsAndUniforms(void);

	BOOL attachShaderFeatures(LLGLSLShader * shader);
	void dumpObjectLog(GLhandleARB ret, BOOL warns = TRUE, const std::string& filename = "");
    void dumpShaderSource(U32 shader_code_count, GLcharARB** shader_code_text);
	BOOL	linkProgramObject(GLhandleARB obj, BOOL suppress_errors = FALSE);
	BOOL	validateProgramObject(GLhandleARB obj);
	GLhandleARB loadShaderFile(const std::string& filename, S32 & shader_level, GLenum type, std::unordered_map<std::string, std::string>* defines = NULL, S32 texture_index_channels = -1);

	// Implemented in the application to actually point to the shader directory.
	virtual std::string getShaderDirPrefix(void) = 0; // Pure Virtual

	// Implemented in the application to actually update out of date uniforms for a particular shader
	virtual void updateShaderUniforms(LLGLSLShader * shader) = 0; // Pure Virtual

public:
	// Map of shader names to compiled
    std::map<std::string, GLhandleARB> mVertexShaderObjects;
    std::map<std::string, GLhandleARB> mFragmentShaderObjects;

	//global (reserved slot) shader parameters
	std::vector<std::string> mReservedAttribs;

	std::vector<std::string> mReservedUniforms;

	//preprocessor definitions (name/value)
	std::map<std::string, std::string> mDefinitions;

protected:

	// our parameter manager singleton instance
	static LLShaderMgr * sInstance;

}; //LLShaderMgr

#endif
