/**
* @file llsettingssky.cpp
* @author optional
* @brief A base class for asset based settings groups.
*
* $LicenseInfo:2011&license=viewerlgpl$
* Second Life Viewer Source Code
* Copyright (C) 2017, Linden Research, Inc.
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

#include "llsettingssky.h"
#include "indra_constants.h"
#include <algorithm>
#include "lltrace.h"
#include "llfasttimer.h"
#include "v3colorutil.h"

//=========================================================================
namespace
{
    const LLVector3 DUE_EAST(0.0f, 0.0f, 1.0);
    const LLVector3 VECT_ZENITH(0.f, 1.f, 0.f);
    const LLVector3 VECT_NORTHSOUTH(1.f, 0.f, 0.f);

    LLTrace::BlockTimerStatHandle FTM_BLEND_SKYVALUES("Blending Sky Environment");
    LLTrace::BlockTimerStatHandle FTM_UPDATE_SKYVALUES("Update Sky Environment");

    LLQuaternion body_position_from_angles(F32 azimuth, F32 altitude);
    void angles_from_rotation(LLQuaternion quat, F32 &azimuth, F32 &altitude);
}

const F32 LLSettingsSky::DOME_OFFSET(0.96f);
const F32 LLSettingsSky::DOME_RADIUS(15000.f);

const F32 LLSettingsSky::NIGHTTIME_ELEVATION(-8.0f); // degrees
const F32 LLSettingsSky::NIGHTTIME_ELEVATION_COS((F32)sin(NIGHTTIME_ELEVATION*DEG_TO_RAD));

//=========================================================================
const std::string LLSettingsSky::SETTING_AMBIENT("ambient");
const std::string LLSettingsSky::SETTING_BLOOM_TEXTUREID("bloom_id");
const std::string LLSettingsSky::SETTING_BLUE_DENSITY("blue_density");
const std::string LLSettingsSky::SETTING_BLUE_HORIZON("blue_horizon");
const std::string LLSettingsSky::SETTING_CLOUD_COLOR("cloud_color");
const std::string LLSettingsSky::SETTING_CLOUD_POS_DENSITY1("cloud_pos_density1");
const std::string LLSettingsSky::SETTING_CLOUD_POS_DENSITY2("cloud_pos_density2");
const std::string LLSettingsSky::SETTING_CLOUD_SCALE("cloud_scale");
const std::string LLSettingsSky::SETTING_CLOUD_SCROLL_RATE("cloud_scroll_rate");
const std::string LLSettingsSky::SETTING_CLOUD_SHADOW("cloud_shadow");
const std::string LLSettingsSky::SETTING_CLOUD_TEXTUREID("cloud_id");
const std::string LLSettingsSky::SETTING_DENSITY_MULTIPLIER("density_multiplier");
const std::string LLSettingsSky::SETTING_DISTANCE_MULTIPLIER("distance_multiplier");
const std::string LLSettingsSky::SETTING_DOME_OFFSET("dome_offset");
const std::string LLSettingsSky::SETTING_DOME_RADIUS("dome_radius");
const std::string LLSettingsSky::SETTING_GAMMA("gamma");
const std::string LLSettingsSky::SETTING_GLOW("glow");
const std::string LLSettingsSky::SETTING_HAZE_DENSITY("haze_density");
const std::string LLSettingsSky::SETTING_HAZE_HORIZON("haze_horizon");
const std::string LLSettingsSky::SETTING_LIGHT_NORMAL("lightnorm");
const std::string LLSettingsSky::SETTING_MAX_Y("max_y");
const std::string LLSettingsSky::SETTING_MOON_ROTATION("moon_rotation");
const std::string LLSettingsSky::SETTING_MOON_TEXTUREID("moon_id");
const std::string LLSettingsSky::SETTING_STAR_BRIGHTNESS("star_brightness");
const std::string LLSettingsSky::SETTING_SUNLIGHT_COLOR("sunlight_color");
const std::string LLSettingsSky::SETTING_SUN_ROTATION("sun_rotation");
const std::string LLSettingsSky::SETTING_SUN_TEXUTUREID("sun_id");

const std::string LLSettingsSky::SETTING_LEGACY_EAST_ANGLE("east_angle");
const std::string LLSettingsSky::SETTING_LEGACY_ENABLE_CLOUD_SCROLL("enable_cloud_scroll");
const std::string LLSettingsSky::SETTING_LEGACY_SUN_ANGLE("sun_angle");

//=========================================================================
LLSettingsSky::LLSettingsSky(const LLSD &data) :
    LLSettingsBase(data)
{
}

LLSettingsSky::LLSettingsSky():
    LLSettingsBase()
{
}

void LLSettingsSky::blend(const LLSettingsBase::ptr_t &end, F64 blendf) 
{
    LLSettingsSky::ptr_t other = boost::static_pointer_cast<LLSettingsSky>(end);
    LLSD blenddata = interpolateSDMap(mSettings, other->mSettings, blendf);

    replaceSettings(blenddata);
}


void LLSettingsSky::setMoonRotation(F32 azimuth, F32 altitude)
{
    setValue(SETTING_MOON_ROTATION, ::body_position_from_angles(azimuth, altitude));
}

LLSettingsSky::azimalt_t LLSettingsSky::getMoonRotationAzAl() const
{
    azimalt_t res;
    ::angles_from_rotation(getMoonRotation(), res.first, res.second);

    return res;
}

void LLSettingsSky::setSunRotation(F32 azimuth, F32 altitude)
{
    setValue(SETTING_SUN_ROTATION, ::body_position_from_angles(azimuth, altitude));
}

LLSettingsSky::azimalt_t LLSettingsSky::getSunRotationAzAl() const
{
    azimalt_t res;
    ::angles_from_rotation(getSunRotation(), res.first, res.second);

    return res;
}

LLSettingsSky::stringset_t LLSettingsSky::getSlerpKeys() const 
{ 
    static stringset_t slepSet;

    if (slepSet.empty())
    {
        slepSet.insert(SETTING_SUN_ROTATION);
        slepSet.insert(SETTING_MOON_ROTATION);
    }

    return slepSet;
}

LLSettingsSky::validation_list_t LLSettingsSky::getValidationList() const
{
    static validation_list_t validation;

    if (validation.empty())
    {   // Note the use of LLSD(LLSDArray()()()...) This is due to an issue with the 
        // copy constructor for LLSDArray.  Directly binding the LLSDArray as 
        // a parameter without first wrapping it in a pure LLSD object will result 
        // in deeply nested arrays like this [[[[[[[[[[v1,v2,v3]]]]]]]]]]
        
        validation.push_back(Validator(SETTING_AMBIENT, true, LLSD::TypeArray,
            boost::bind(&Validator::verifyVectorMinMax, _1,
                LLSD(LLSDArray(0.0f)(0.0f)(0.0f)("*")),
                LLSD(LLSDArray(3.0f)(3.0f)(3.0f)("*")))));
        validation.push_back(Validator(SETTING_BLOOM_TEXTUREID,     true,  LLSD::TypeUUID));
        validation.push_back(Validator(SETTING_BLUE_DENSITY,        true,  LLSD::TypeArray, 
            boost::bind(&Validator::verifyVectorMinMax, _1,
                LLSD(LLSDArray(0.0f)(0.0f)(0.0f)("*")),
                LLSD(LLSDArray(2.0f)(2.0f)(2.0f)("*")))));
        validation.push_back(Validator(SETTING_BLUE_HORIZON,        true,  LLSD::TypeArray, 
            boost::bind(&Validator::verifyVectorMinMax, _1,
                LLSD(LLSDArray(0.0f)(0.0f)(0.0f)("*")),
                LLSD(LLSDArray(2.0f)(2.0f)(2.0f)("*")))));
        validation.push_back(Validator(SETTING_CLOUD_COLOR,         true,  LLSD::TypeArray, 
            boost::bind(&Validator::verifyVectorMinMax, _1,
                LLSD(LLSDArray(0.0f)(0.0f)(0.0f)("*")),
                LLSD(LLSDArray(1.0f)(1.0f)(1.0f)("*")))));
        validation.push_back(Validator(SETTING_CLOUD_POS_DENSITY1,  true,  LLSD::TypeArray, 
            boost::bind(&Validator::verifyVectorMinMax, _1,
                LLSD(LLSDArray(0.0f)(0.0f)(0.0f)("*")),
                LLSD(LLSDArray(1.68841f)(1.0f)(1.0f)("*")))));
        validation.push_back(Validator(SETTING_CLOUD_POS_DENSITY2,  true,  LLSD::TypeArray, 
            boost::bind(&Validator::verifyVectorMinMax, _1,
                LLSD(LLSDArray(0.0f)(0.0f)(0.0f)("*")),
                LLSD(LLSDArray(1.68841f)(1.0f)(1.0f)("*")))));
        validation.push_back(Validator(SETTING_CLOUD_SCALE,         true,  LLSD::TypeReal,  
            boost::bind(&Validator::verifyFloatRange, _1, LLSD(LLSDArray(0.001f)(0.999f)))));
        validation.push_back(Validator(SETTING_CLOUD_SCROLL_RATE,   true,  LLSD::TypeArray, 
            boost::bind(&Validator::verifyVectorMinMax, _1,
                LLSD(LLSDArray(0.0f)(0.0f)),
                LLSD(LLSDArray(20.0f)(20.0f)))));
        validation.push_back(Validator(SETTING_CLOUD_SHADOW,        true,  LLSD::TypeReal,  
            boost::bind(&Validator::verifyFloatRange, _1, LLSD(LLSDArray(0.0f)(1.0f)))));
        validation.push_back(Validator(SETTING_CLOUD_TEXTUREID,     false, LLSD::TypeUUID));
        validation.push_back(Validator(SETTING_DENSITY_MULTIPLIER,  true,  LLSD::TypeReal,  
            boost::bind(&Validator::verifyFloatRange, _1, LLSD(LLSDArray(0.0f)(0.0009f)))));
        validation.push_back(Validator(SETTING_DISTANCE_MULTIPLIER, true,  LLSD::TypeReal,  
            boost::bind(&Validator::verifyFloatRange, _1, LLSD(LLSDArray(0.0f)(100.0f)))));
        validation.push_back(Validator(SETTING_DOME_OFFSET,         false, LLSD::TypeReal,  
            boost::bind(&Validator::verifyFloatRange, _1, LLSD(LLSDArray(0.0f)(1.0f)))));
        validation.push_back(Validator(SETTING_DOME_RADIUS,         false, LLSD::TypeReal,  
            boost::bind(&Validator::verifyFloatRange, _1, LLSD(LLSDArray(1000.0f)(2000.0f)))));
        validation.push_back(Validator(SETTING_GAMMA,               true,  LLSD::TypeReal,  
            boost::bind(&Validator::verifyFloatRange, _1, LLSD(LLSDArray(0.0f)(10.0f)))));
        validation.push_back(Validator(SETTING_GLOW,                true,  LLSD::TypeArray, 
            boost::bind(&Validator::verifyVectorMinMax, _1,
                LLSD(LLSDArray(0.2f)("*")(-2.5f)("*")),
                LLSD(LLSDArray(20.0f)("*")(0.0f)("*")))));
        validation.push_back(Validator(SETTING_HAZE_DENSITY,        true,  LLSD::TypeReal,  
            boost::bind(&Validator::verifyFloatRange, _1, LLSD(LLSDArray(0.0f)(4.0f)))));
        validation.push_back(Validator(SETTING_HAZE_HORIZON,        true,  LLSD::TypeReal,  
            boost::bind(&Validator::verifyFloatRange, _1, LLSD(LLSDArray(0.0f)(1.0f)))));
        validation.push_back(Validator(SETTING_LIGHT_NORMAL,        false, LLSD::TypeArray, 
            boost::bind(&Validator::verifyVectorNormalized, _1, 3)));
        validation.push_back(Validator(SETTING_MAX_Y,               true,  LLSD::TypeReal,  
            boost::bind(&Validator::verifyFloatRange, _1, LLSD(LLSDArray(0.0f)(4000.0f)))));
        validation.push_back(Validator(SETTING_MOON_ROTATION,       true,  LLSD::TypeArray, &Validator::verifyQuaternionNormal));
        validation.push_back(Validator(SETTING_MOON_TEXTUREID,      false, LLSD::TypeUUID));
        validation.push_back(Validator(SETTING_STAR_BRIGHTNESS,     true,  LLSD::TypeReal, 
            boost::bind(&Validator::verifyFloatRange, _1, LLSD(LLSDArray(0.0f)(2.0f)))));
        validation.push_back(Validator(SETTING_SUNLIGHT_COLOR,      true,  LLSD::TypeArray, 
            boost::bind(&Validator::verifyVectorMinMax, _1,
                LLSD(LLSDArray(0.0f)(0.0f)(0.0f)("*")),
                LLSD(LLSDArray(3.0f)(3.0f)(3.0f)("*")))));
        validation.push_back(Validator(SETTING_SUN_ROTATION,        true,  LLSD::TypeArray, &Validator::verifyQuaternionNormal));
        validation.push_back(Validator(SETTING_SUN_TEXUTUREID,      false, LLSD::TypeUUID));
    }

    return validation;
}


LLSD LLSettingsSky::defaults()
{
    LLSD dfltsetting;

    
    LLQuaternion sunquat;
    sunquat.setEulerAngles(1.39626, 0.0, 0.0); // 80deg Azumith/0deg East
    LLQuaternion moonquat = ~sunquat;

    // Magic constants copied form dfltsetting.xml 
    dfltsetting[SETTING_AMBIENT]            = LLColor4::white.getValue();
    dfltsetting[SETTING_BLUE_DENSITY]       = LLColor4(0.2447, 0.4487, 0.7599, 0.0).getValue();
    dfltsetting[SETTING_BLUE_HORIZON]       = LLColor4(0.4954, 0.4954, 0.6399, 0.0).getValue();
    dfltsetting[SETTING_CLOUD_COLOR]        = LLColor4(0.4099, 0.4099, 0.4099, 0.0).getValue();
    dfltsetting[SETTING_CLOUD_POS_DENSITY1] = LLColor4(1.0000, 0.5260, 1.0000, 0.0).getValue();
    dfltsetting[SETTING_CLOUD_POS_DENSITY2] = LLColor4(1.0000, 0.5260, 1.0000, 0.0).getValue();
    dfltsetting[SETTING_CLOUD_SCALE]        = LLSD::Real(0.4199);
    dfltsetting[SETTING_CLOUD_SCROLL_RATE]  = LLSDArray(10.1999)(10.0109);
    dfltsetting[SETTING_CLOUD_SHADOW]       = LLSD::Real(0.2699);
    dfltsetting[SETTING_DENSITY_MULTIPLIER] = LLSD::Real(0.0001);
    dfltsetting[SETTING_DISTANCE_MULTIPLIER] = LLSD::Real(0.8000);
    dfltsetting[SETTING_DOME_OFFSET]        = LLSD::Real(0.96f);
    dfltsetting[SETTING_DOME_RADIUS]        = LLSD::Real(15000.f);
    dfltsetting[SETTING_GAMMA]              = LLSD::Real(1.0);
    dfltsetting[SETTING_GLOW]               = LLColor4(5.000, 0.0010, -0.4799, 1.0).getValue();
    dfltsetting[SETTING_HAZE_DENSITY]       = LLSD::Real(0.6999);
    dfltsetting[SETTING_HAZE_HORIZON]       = LLSD::Real(0.1899);
    dfltsetting[SETTING_LIGHT_NORMAL]       = LLVector3(0.0000, 0.9126, -0.4086).getValue();
    dfltsetting[SETTING_MAX_Y]              = LLSD::Real(1605);
    dfltsetting[SETTING_MOON_ROTATION]      = moonquat.getValue();
    dfltsetting[SETTING_STAR_BRIGHTNESS]    = LLSD::Real(0.0000);
    dfltsetting[SETTING_SUNLIGHT_COLOR]     = LLColor4(0.7342, 0.7815, 0.8999, 0.0).getValue();
    dfltsetting[SETTING_SUN_ROTATION]       = sunquat.getValue();

    dfltsetting[SETTING_BLOOM_TEXTUREID]    = LLUUID::null;
    dfltsetting[SETTING_CLOUD_TEXTUREID]    = LLUUID::null;
    dfltsetting[SETTING_MOON_TEXTUREID]     = IMG_MOON; // gMoonTextureID;   // These two are returned by the login... wow!
    dfltsetting[SETTING_SUN_TEXUTUREID]     = IMG_SUN;  // gSunTextureID;

    return dfltsetting;
}

LLSD LLSettingsSky::translateLegacySettings(LLSD legacy)
{
    LLSD newsettings(defaults());

    if (legacy.has(SETTING_AMBIENT))
    {
        newsettings[SETTING_AMBIENT] = LLColor3(legacy[SETTING_AMBIENT]).getValue();
    }
    if (legacy.has(SETTING_BLUE_DENSITY))
    {
        newsettings[SETTING_BLUE_DENSITY] = LLColor3(legacy[SETTING_BLUE_DENSITY]).getValue();
    }
    if (legacy.has(SETTING_BLUE_HORIZON))
    {
        newsettings[SETTING_BLUE_HORIZON] = LLColor3(legacy[SETTING_BLUE_HORIZON]).getValue();
    }
    if (legacy.has(SETTING_CLOUD_COLOR))
    {
        newsettings[SETTING_CLOUD_COLOR] = LLColor3(legacy[SETTING_CLOUD_COLOR]).getValue();
    }
    if (legacy.has(SETTING_CLOUD_POS_DENSITY1))
    {
        newsettings[SETTING_CLOUD_POS_DENSITY1] = LLColor3(legacy[SETTING_CLOUD_POS_DENSITY1]).getValue();
    }
    if (legacy.has(SETTING_CLOUD_POS_DENSITY2))
    {
        newsettings[SETTING_CLOUD_POS_DENSITY2] = LLColor3(legacy[SETTING_CLOUD_POS_DENSITY2]).getValue();
    }
    if (legacy.has(SETTING_CLOUD_SCALE))
    {
        newsettings[SETTING_CLOUD_SCALE] = LLSD::Real(legacy[SETTING_CLOUD_SCALE][0].asReal());
    }
    if (legacy.has(SETTING_CLOUD_SCROLL_RATE))
    {
        LLVector2 cloud_scroll(legacy[SETTING_CLOUD_SCROLL_RATE]);

        if (legacy.has(SETTING_LEGACY_ENABLE_CLOUD_SCROLL))
        {
            LLSD enabled = legacy[SETTING_LEGACY_ENABLE_CLOUD_SCROLL];
            if (!enabled[0].asBoolean())
                cloud_scroll.mV[0] = 0.0f;
            if (!enabled[1].asBoolean())
                cloud_scroll.mV[1] = 0.0f;
        }

        newsettings[SETTING_CLOUD_SCROLL_RATE] = cloud_scroll.getValue();
    }
    if (legacy.has(SETTING_CLOUD_SHADOW))
    {
        newsettings[SETTING_CLOUD_SHADOW] = LLSD::Real(legacy[SETTING_CLOUD_SHADOW][0].asReal());
    }
    if (legacy.has(SETTING_DENSITY_MULTIPLIER))
    {
        newsettings[SETTING_DENSITY_MULTIPLIER] = LLSD::Real(legacy[SETTING_DENSITY_MULTIPLIER][0].asReal());
    }
    if (legacy.has(SETTING_DISTANCE_MULTIPLIER))
    {
        newsettings[SETTING_DISTANCE_MULTIPLIER] = LLSD::Real(legacy[SETTING_DISTANCE_MULTIPLIER][0].asReal());
    }
    if (legacy.has(SETTING_GAMMA))
    {
        newsettings[SETTING_GAMMA] = legacy[SETTING_GAMMA][0].asReal();
    }
    if (legacy.has(SETTING_GLOW))
    {
        newsettings[SETTING_GLOW] = LLColor3(legacy[SETTING_GLOW]).getValue();
    }
    if (legacy.has(SETTING_HAZE_DENSITY))
    {
        newsettings[SETTING_HAZE_DENSITY] = LLSD::Real(legacy[SETTING_HAZE_DENSITY][0].asReal());
    }
    if (legacy.has(SETTING_HAZE_HORIZON))
    {
        newsettings[SETTING_HAZE_HORIZON] = LLSD::Real(legacy[SETTING_HAZE_HORIZON][0].asReal());
    }
    if (legacy.has(SETTING_LIGHT_NORMAL))
    {
        newsettings[SETTING_LIGHT_NORMAL] = LLVector3(legacy[SETTING_LIGHT_NORMAL]).getValue();
    }
    if (legacy.has(SETTING_MAX_Y))
    {
        newsettings[SETTING_MAX_Y] = LLSD::Real(legacy[SETTING_MAX_Y][0].asReal());
    }
    if (legacy.has(SETTING_STAR_BRIGHTNESS))
    {
        newsettings[SETTING_STAR_BRIGHTNESS] = LLSD::Real(legacy[SETTING_STAR_BRIGHTNESS].asReal());
    }
    if (legacy.has(SETTING_SUNLIGHT_COLOR))
    {
        newsettings[SETTING_SUNLIGHT_COLOR] = LLColor4(legacy[SETTING_SUNLIGHT_COLOR]).getValue();
    }

    if (legacy.has(SETTING_LEGACY_EAST_ANGLE) && legacy.has(SETTING_LEGACY_SUN_ANGLE))
    {   // convert the east and sun angles into a quaternion.
        F32 azimuth = legacy[SETTING_LEGACY_EAST_ANGLE].asReal();
        F32 altitude = legacy[SETTING_LEGACY_SUN_ANGLE].asReal();

        LLQuaternion sunquat = ::body_position_from_angles(azimuth, altitude);
        LLQuaternion moonquat = ::body_position_from_angles(azimuth + F_PI, -altitude);

        F32 az(0), al(0);
        ::angles_from_rotation(sunquat, az, al);

        newsettings[SETTING_SUN_ROTATION] = sunquat.getValue();
        newsettings[SETTING_MOON_ROTATION] = moonquat.getValue();
    }

    return newsettings;
}

void LLSettingsSky::updateSettings()
{
    LL_RECORD_BLOCK_TIME(FTM_UPDATE_SKYVALUES);
    //LL_INFOS("WINDLIGHT", "SKY", "EEP") << "WL Parameters are dirty.  Reticulating Splines..." << LL_ENDL;

    // base class clears dirty flag so as to not trigger recursive update
    LLSettingsBase::updateSettings();

    calculateHeavnlyBodyPositions();
    calculateLightSettings();
}

void LLSettingsSky::calculateHeavnlyBodyPositions()
{
    mSunDirection = DUE_EAST * getSunRotation();
    mSunDirection.normalize();
    mMoonDirection = DUE_EAST * getMoonRotation();
    mMoonDirection.normalize();

    // is the normal from the sun or the moon
    if (mSunDirection.mV[1] >= 0.0)
    {
        mLightDirection = mSunDirection;
    }
    else if (mSunDirection.mV[1] < 0.0 && mSunDirection.mV[1] > NIGHTTIME_ELEVATION_COS)
    {
        // clamp v1 to 0 so sun never points up and causes weirdness on some machines
        LLVector3 vec(mSunDirection);
        vec.mV[1] = 0.0;
        vec.normalize();
        mLightDirection = vec;
    }
    else
    {
        mLightDirection = mMoonDirection;
    }

    // calculate the clamp lightnorm for sky (to prevent ugly banding in sky
    // when haze goes below the horizon
    mClampedLightDirection = mLightDirection;

    if (mClampedLightDirection.mV[1] < -0.1f)
    {
        mClampedLightDirection.mV[1] = -0.1f;
        mClampedLightDirection.normalize();
    }
}

void LLSettingsSky::calculateLightSettings()
{
    LLColor3 vary_HazeColor;
    LLColor3 vary_SunlightColor;
    LLColor3 vary_AmbientColor;
    {
        // Initialize temp variables
        LLColor3    sunlight = getSunlightColor();
        LLColor3    ambient = getAmbientColor();
        F32         gamma = getGamma();
        LLColor3    blue_density = getBlueDensity();
        LLColor3    blue_horizon = getBlueHorizon();
        F32         haze_density = getHazeDensity();
        F32         haze_horizon = getHazeHorizon();
        F32         density_multiplier = getDensityMultiplier();
        F32         max_y = getMaxY();
        F32         cloud_shadow = getCloudShadow();
        LLVector3   lightnorm = getLightDirection();

        // Sunlight attenuation effect (hue and brightness) due to atmosphere
        // this is used later for sunlight modulation at various altitudes
        LLColor3 light_atten =
            (blue_density * 1.0 + smear(haze_density * 0.25f)) * (density_multiplier * max_y);

        // Calculate relative weights
        LLColor3 temp2(0.f, 0.f, 0.f);
        LLColor3 temp1 = blue_density + smear(haze_density);
        LLColor3 blue_weight = componentDiv(blue_density, temp1);
        LLColor3 haze_weight = componentDiv(smear(haze_density), temp1);

        // Compute sunlight from P & lightnorm (for long rays like sky)
        /// USE only lightnorm.
        // temp2[1] = llmax(0.f, llmax(0.f, Pn[1]) * 1.0f + lightnorm[1] );

        // and vary_sunlight will work properly with moon light
        F32 lighty = lightnorm[1];
        if (lighty < NIGHTTIME_ELEVATION_COS)
        {
            lighty = -lighty;
        }

        temp2.mV[1] = llmax(0.f, lighty);
        if(temp2.mV[1] > 0.f)
        {
            temp2.mV[1] = 1.f / temp2.mV[1];
        }
        componentMultBy(sunlight, componentExp((light_atten * -1.f) * temp2.mV[1]));

        // Distance
        temp2.mV[2] = density_multiplier;

        // Transparency (-> temp1)
        temp1 = componentExp((temp1 * -1.f) * temp2.mV[2]);

        // vary_AtmosAttenuation = temp1; 

        //increase ambient when there are more clouds
        LLColor3 tmpAmbient = ambient + (smear(1.f) - ambient) * cloud_shadow * 0.5f;

        //haze color
        vary_HazeColor =
            (blue_horizon * blue_weight * (sunlight*(1.f - cloud_shadow) + tmpAmbient)	
            + componentMult(haze_horizon * haze_weight, sunlight*(1.f - cloud_shadow) * temp2.mV[0] + tmpAmbient)
            );	

        //brightness of surface both sunlight and ambient
        vary_SunlightColor = componentMult(sunlight, temp1) * 1.f;
        vary_SunlightColor.clamp();
        vary_SunlightColor = smear(1.0f) - vary_SunlightColor;
        vary_SunlightColor = componentPow(vary_SunlightColor, gamma);
        vary_SunlightColor = smear(1.0f) - vary_SunlightColor;
        vary_AmbientColor = componentMult(tmpAmbient, temp1) * 0.5;
        vary_AmbientColor.clamp();
        vary_AmbientColor = smear(1.0f) - vary_AmbientColor;
        vary_AmbientColor = componentPow(vary_AmbientColor, gamma);
        vary_AmbientColor = smear(1.0f) - vary_AmbientColor;

        componentMultBy(vary_HazeColor, LLColor3(1.f, 1.f, 1.f) - temp1);

    }

    mSunDiffuse = vary_SunlightColor;
    mSunAmbient = vary_AmbientColor;
    mMoonDiffuse = vary_SunlightColor;
    mMoonAmbient = vary_AmbientColor;

    mTotalAmbient = LLColor4(vary_AmbientColor, 1.0f);

    mFadeColor = mTotalAmbient + (mSunDiffuse + mMoonDiffuse) * 0.5f;
    mFadeColor.setAlpha(0);
}


//=========================================================================
namespace
{
    LLQuaternion body_position_from_angles(F32 azimuth, F32 altitude)
    {
        // Azimuth is traditionally calculated from North, we are going from East.
        LLQuaternion rot_azi;
        LLQuaternion rot_alt;

        rot_azi.setAngleAxis(azimuth, VECT_ZENITH);
        rot_alt.setAngleAxis(-altitude, VECT_NORTHSOUTH);

        LLQuaternion body_quat = rot_alt * rot_azi;
        body_quat.normalize();

        //LLVector3 sun_vector = (DUE_EAST * body_quat);
        //_WARNS("RIDER") << "Azimuth=" << azimuth << " Altitude=" << altitude << " Body Vector=" << sun_vector.getValue() << LL_ENDL;
        return body_quat;
    }

    void angles_from_rotation(LLQuaternion quat, F32 &azimuth, F32 &altitude)
    {
        LLVector3 body_vector = (DUE_EAST * quat);

        LLVector3 body_az(body_vector[0], 0.f, body_vector[2]);
        LLVector3 body_al(0.f, body_vector[1], body_vector[2]);
        
        if (fabs(body_az.normalize()) > 0.001)
            azimuth = angle_between(DUE_EAST, body_az);
        else
            azimuth = 0.0f;

        if (fabs(body_al.normalize()) > 0.001)
            altitude = angle_between(DUE_EAST, body_al);
        else
            altitude = 0.0f;
    }
}


