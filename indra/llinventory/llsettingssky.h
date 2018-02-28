/**
* @file llsettingssky.h
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

#ifndef LL_SETTINGS_SKY_H
#define LL_SETTINGS_SKY_H

#include "llsettingsbase.h"
#include "v4coloru.h"

#define SUPPORT_LEGACY_ATMOSPHERICS 1

class LLSettingsSky: public LLSettingsBase
{
public:
    static const std::string SETTING_AMBIENT;
    static const std::string SETTING_BLOOM_TEXTUREID;
    static const std::string SETTING_BLUE_DENSITY;
    static const std::string SETTING_BLUE_HORIZON;
    static const std::string SETTING_DENSITY_MULTIPLIER;
    static const std::string SETTING_DISTANCE_MULTIPLIER;
    static const std::string SETTING_HAZE_DENSITY;
    static const std::string SETTING_HAZE_HORIZON;
    static const std::string SETTING_BLOOM_TEXTUREID;
    static const std::string SETTING_CLOUD_COLOR;
    static const std::string SETTING_CLOUD_POS_DENSITY1;
    static const std::string SETTING_CLOUD_POS_DENSITY2;
    static const std::string SETTING_CLOUD_SCALE;
    static const std::string SETTING_CLOUD_SCROLL_RATE;
    static const std::string SETTING_CLOUD_SHADOW;
    static const std::string SETTING_CLOUD_TEXTUREID;
    static const std::string SETTING_DOME_OFFSET;
    static const std::string SETTING_DOME_RADIUS;
    static const std::string SETTING_GAMMA;
    static const std::string SETTING_GLOW;    
    static const std::string SETTING_LIGHT_NORMAL;
    static const std::string SETTING_MAX_Y;
    static const std::string SETTING_MOON_ROTATION;
    static const std::string SETTING_MOON_TEXTUREID;
    static const std::string SETTING_STAR_BRIGHTNESS;
    static const std::string SETTING_SUNLIGHT_COLOR;
    static const std::string SETTING_SUN_ROTATION;
    static const std::string SETTING_SUN_TEXTUREID;

    static const std::string SETTING_PLANET_RADIUS;
    static const std::string SETTING_SKY_BOTTOM_RADIUS;
    static const std::string SETTING_SKY_TOP_RADIUS;
    static const std::string SETTING_SUN_ARC_RADIANS;

    static const std::string SETTING_RAYLEIGH_CONFIG;
    static const std::string SETTING_MIE_CONFIG;
    static const std::string SETTING_ABSORPTION_CONFIG;

    static const std::string KEY_DENSITY_PROFILE;
        static const std::string SETTING_DENSITY_PROFILE_WIDTH;
        static const std::string SETTING_DENSITY_PROFILE_EXP_TERM;
        static const std::string SETTING_DENSITY_PROFILE_EXP_SCALE_FACTOR;
        static const std::string SETTING_DENSITY_PROFILE_LINEAR_TERM;
        static const std::string SETTING_DENSITY_PROFILE_CONSTANT_TERM;
        static const std::string SETTING_MIE_ANISOTROPY_FACTOR;

    typedef std::shared_ptr<LLSettingsSky> ptr_t;
    typedef std::pair<F32, F32> azimalt_t;

    //---------------------------------------------------------------------
    LLSettingsSky(const LLSD &data);
    virtual ~LLSettingsSky() { };

    virtual ptr_t   buildClone() = 0;

    //---------------------------------------------------------------------
    virtual std::string getSettingType() const override { return std::string("sky"); }
    virtual LLSettingsType getSettingTypeValue() const override { return LLSettingsType::ST_SKY; }


    // Settings status 
    virtual void blend(const LLSettingsBase::ptr_t &end, F64 blendf);
    
    static LLSD defaults();

    LLUUID getBloomTextureId() const
    {
        return mSettings[SETTING_BLOOM_TEXTUREID].asUUID();
    }

    //---------------------------------------------------------------------
#if SUPPORT_LEGACY_ATMOSPHERICS
    LLColor3 getAmbientColor() const
    {
        return LLColor3(mSettings[SETTING_AMBIENT]);
    }

    void setAmbientColor(const LLColor3 &val)
    {
        setValue(SETTING_AMBIENT, val);
    }

    LLUUID getBloomTextureId() const
    {
        return mSettings[SETTING_BLOOM_TEXTUREID].asUUID();
    }

    LLColor3 getBlueDensity() const
    {
        return LLColor3(mSettings[SETTING_BLUE_DENSITY]);
    }

    void setBlueDensity(const LLColor3 &val)
    {
        setValue(SETTING_BLUE_DENSITY, val);
    }

    LLColor3 getBlueHorizon() const
    {
        return LLColor3(mSettings[SETTING_BLUE_HORIZON]);
    }

    void setBlueHorizon(const LLColor3 &val)
    {
        setValue(SETTING_BLUE_HORIZON, val);
    }

    F32 getDensityMultiplier() const
    {
        return mSettings[SETTING_DENSITY_MULTIPLIER].asReal();
    }

    void setDensityMultiplier(F32 val)
    {
        setValue(SETTING_DENSITY_MULTIPLIER, val);
    }

    F32 getDistanceMultiplier() const
    {
        return mSettings[SETTING_DISTANCE_MULTIPLIER].asReal();
    }

    void setDistanceMultiplier(F32 val)
    {
        setValue(SETTING_DISTANCE_MULTIPLIER, val);
    }

    F32 getHazeDensity() const
    {
        return mSettings[SETTING_HAZE_DENSITY].asReal();
    }

    void setHazeDensity(F32 val)
    {
        setValue(SETTING_HAZE_DENSITY, val);
    }

    F32 getHazeHorizon() const
    {
        return mSettings[SETTING_HAZE_HORIZON].asReal();
    }

    void setHazeHorizon(F32 val)
    {
        setValue(SETTING_HAZE_HORIZON, val);
    }
#endif

    LLColor3 getCloudColor() const
    {
        return LLColor3(mSettings[SETTING_CLOUD_COLOR]);
    }

    void setCloudColor(const LLColor3 &val)
    {
        setValue(SETTING_CLOUD_COLOR, val);
    }

    LLUUID getCloudNoiseTextureId() const
    {
        return mSettings[SETTING_CLOUD_TEXTUREID].asUUID();
    }

    LLColor3 getCloudPosDensity1() const
    {
        return LLColor3(mSettings[SETTING_CLOUD_POS_DENSITY1]);
    }

    void setCloudPosDensity1(const LLColor3 &val)
    {
        setValue(SETTING_CLOUD_POS_DENSITY1, val);
    }

    LLColor3 getCloudPosDensity2() const
    {
        return LLColor3(mSettings[SETTING_CLOUD_POS_DENSITY2]);
    }

    void setCloudPosDensity2(const LLColor3 &val)
    {
        setValue(SETTING_CLOUD_POS_DENSITY2, val);
    }

    F32 getCloudScale() const
    {
        return mSettings[SETTING_CLOUD_SCALE].asReal();
    }

    void setCloudScale(F32 val)
    {
        setValue(SETTING_CLOUD_SCALE, val);
    }

    LLVector2 getCloudScrollRate() const
    {
        return LLVector2(mSettings[SETTING_CLOUD_SCROLL_RATE]);
    }

    void setCloudScrollRate(const LLVector2 &val)
    {
        setValue(SETTING_CLOUD_SCROLL_RATE, val);
    }

    void setCloudScrollRateX(F32 val)
    {
        mSettings[SETTING_CLOUD_SCROLL_RATE][0] = val;
        setDirtyFlag(true);
    }

    void setCloudScrollRateY(F32 val)
    {
        mSettings[SETTING_CLOUD_SCROLL_RATE][1] = val;
        setDirtyFlag(true);
    }

    F32 getCloudShadow() const
    {
        return mSettings[SETTING_CLOUD_SHADOW].asReal();
    }

    void setCloudShadow(F32 val)
    {
        setValue(SETTING_CLOUD_SHADOW, val);
    }

    
    F32 getDomeOffset() const
    {
        return DOME_OFFSET;
        //return mSettings[SETTING_DOME_OFFSET].asReal();
    }

    F32 getDomeRadius() const
    {
        return DOME_RADIUS;
        //return mSettings[SETTING_DOME_RADIUS].asReal();
    }

    F32 getGamma() const
    {
        return mSettings[SETTING_GAMMA].asReal();
    }

    void setGamma(F32 val)
    {
        mSettings[SETTING_GAMMA] = LLSD::Real(val);
        setDirtyFlag(true);
    }

    LLColor3 getGlow() const
    {
        return LLColor3(mSettings[SETTING_GLOW]);
    }

    void setGlow(const LLColor3 &val)
    {
        setValue(SETTING_GLOW, val);
    }

    LLVector3 getLightNormal() const
    {
        return LLVector3(mSettings[SETTING_LIGHT_NORMAL]);
    }

    void setLightNormal(const LLVector3 &val) 
    {
        setValue(SETTING_LIGHT_NORMAL, val);
    }

    F32 getMaxY() const
    {
        return mSettings[SETTING_MAX_Y].asReal();
    }

    LLQuaternion getMoonRotation() const
    {
        return LLQuaternion(mSettings[SETTING_MOON_ROTATION]);
    }

    void setMoonRotation(const LLQuaternion &val)
    {
        setValue(SETTING_MOON_ROTATION, val);
    }

    azimalt_t getMoonRotationAzAl() const;

    void setMoonRotation(F32 azimuth, F32 altitude);

    void setMoonRotation(const azimalt_t &azialt)
    {
        setMoonRotation(azialt.first, azialt.second);
    }

    LLUUID getMoonTextureId() const
    {
        return mSettings[SETTING_MOON_TEXTUREID].asUUID();
    }

    F32 getStarBrightness() const
    {
        return mSettings[SETTING_STAR_BRIGHTNESS].asReal();
    }

    void setStarBrightness(F32 val)
    {
        setValue(SETTING_STAR_BRIGHTNESS, val);
    }

    LLColor3 getSunlightColor() const
    {
        return LLColor3(mSettings[SETTING_SUNLIGHT_COLOR]);
    }

    void setSunlightColor(const LLColor3 &val)
    {
        setValue(SETTING_SUNLIGHT_COLOR, val);
    }

    LLQuaternion getSunRotation() const
    {
        return LLQuaternion(mSettings[SETTING_SUN_ROTATION]);
    }

    azimalt_t getSunRotationAzAl() const;

    void setSunRotation(const LLQuaternion &val) 
    {
        setValue(SETTING_SUN_ROTATION, val);
    }

    void setSunRotation(F32 azimuth, F32 altitude);

    void setSunRotation(const azimalt_t & azimalt)
    {
        setSunRotation(azimalt.first, azimalt.second);
    }

    LLUUID getSunTextureId() const
    {
        return mSettings[SETTING_SUN_TEXTUREID].asUUID();
    }

    // Internal/calculated settings
    LLVector3 getLightDirection() const
    {
        update();
        return mLightDirection;
    };

    LLVector3 getClampedLightDirection() const
    {
        update();
        return mClampedLightDirection;
    };

    LLVector3   getSunDirection() const
    {
        update();
        return mSunDirection;
    }

    LLVector3   getMoonDirection() const
    {
        update();
        return mMoonDirection;
    }

    LLColor4U   getFadeColor() const
    {
        update();
        return mFadeColor;
    }

    LLColor4    getMoonAmbient() const
    {
        update();
        return mMoonAmbient;
    }

    LLColor3    getMoonDiffuse() const
    {
        update();
        return mMoonDiffuse;
    }

    LLColor4    getSunAmbient() const
    {
        update();
        return mSunAmbient;
    }

    LLColor3    getSunDiffuse() const
    {
        update();
        return mSunDiffuse;
    }

    LLColor4    getTotalAmbient() const
    {
        update();
        return mTotalAmbient;
    }

    virtual validation_list_t getValidationList() const;
    static validation_list_t validationList();

    static LLSD     translateLegacySettings(LLSD legacy);

    static LLSD settingValidation(LLSD &settings, validation_list_t &validations);

protected:
    static const std::string SETTING_LEGACY_EAST_ANGLE;
    static const std::string SETTING_LEGACY_ENABLE_CLOUD_SCROLL;
    static const std::string SETTING_LEGACY_SUN_ANGLE;

    LLSettingsSky();

    virtual stringset_t getSlerpKeys() const;

    virtual void    updateSettings();

private:
    // validations for structured sections of sky settings data
    static validation_list_t rayleighValidationList();
    static validation_list_t absorptionValidationList();
    static validation_list_t mieValidationList();

    static LLSD rayleighConfigDefault();
    static LLSD absorptionConfigDefault();
    static LLSD mieConfigDefault();

    static const F32         NIGHTTIME_ELEVATION;
    static const F32         NIGHTTIME_ELEVATION_COS;

    void        calculateHeavnlyBodyPositions();
    void        calculateLightSettings();

    LLVector3   mSunDirection;
    LLVector3   mMoonDirection;
    LLVector3   mLightDirection;
    LLVector3   mClampedLightDirection;

    static const F32 DOME_RADIUS;
    static const F32 DOME_OFFSET;

    LLColor4U   mFadeColor;
    LLColor4    mMoonAmbient;
    LLColor3    mMoonDiffuse;
    LLColor4    mSunAmbient;
    LLColor3    mSunDiffuse;

    LLColor4    mTotalAmbient;

    typedef std::map<std::string, S32> mapNameToUniformId_t;

    static mapNameToUniformId_t sNameToUniformMapping;
};

#endif
