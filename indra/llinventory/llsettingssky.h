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

const F32 EARTH_RADIUS  =      6.370e6f;
const F32 SUN_RADIUS    =    695.508e6f;
const F32 SUN_DIST      = 149598.260e6f;
const F32 MOON_RADIUS   =      1.737e6f;
const F32 MOON_DIST     =    384.400e6f;

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
    static const std::string SETTING_MIE_ANISOTROPY_FACTOR;

    static const std::string SETTING_RAYLEIGH_CONFIG;
    static const std::string SETTING_MIE_CONFIG;
    static const std::string SETTING_ABSORPTION_CONFIG;

    static const std::string KEY_DENSITY_PROFILE;
        static const std::string SETTING_DENSITY_PROFILE_WIDTH;
        static const std::string SETTING_DENSITY_PROFILE_EXP_TERM;
        static const std::string SETTING_DENSITY_PROFILE_EXP_SCALE_FACTOR;
        static const std::string SETTING_DENSITY_PROFILE_LINEAR_TERM;
        static const std::string SETTING_DENSITY_PROFILE_CONSTANT_TERM;
        

    static const std::string SETTING_LEGACY_HAZE;

    typedef PTR_NAMESPACE::shared_ptr<LLSettingsSky> ptr_t;

    //---------------------------------------------------------------------
    LLSettingsSky(const LLSD &data);
    virtual ~LLSettingsSky() { };

    virtual ptr_t   buildClone() = 0;

    //---------------------------------------------------------------------
    virtual std::string getSettingsType() const SETTINGS_OVERRIDE { return std::string("sky"); }
    virtual LLSettingsType::type_e getSettingsTypeValue() const SETTINGS_OVERRIDE { return LLSettingsType::ST_SKY; }

    // Settings status 
    virtual void blend(const LLSettingsBase::ptr_t &end, F64 blendf) SETTINGS_OVERRIDE;
    
    virtual void replaceSettings(LLSD settings) SETTINGS_OVERRIDE;

    static LLSD defaults(const LLSettingsBase::TrackPosition& position = 0.0f);

    F32 getPlanetRadius() const;
    F32 getSkyBottomRadius() const;
    F32 getSkyTopRadius() const;
    F32 getSunArcRadians() const;
    F32 getMieAnisotropy() const;   
    LLSD getRayleighConfigs() const;
    LLSD getMieConfigs() const;

    LLSD getAbsorptionConfigs() const;
    LLUUID getBloomTextureId() const;

    //---------------------------------------------------------------------
    LLColor3 getAmbientColor() const;
    void setAmbientColor(const LLColor3 &val);

    LLColor3 getCloudColor() const;
    void setCloudColor(const LLColor3 &val);

    LLUUID getCloudNoiseTextureId() const;
    void setCloudNoiseTextureId(const LLUUID &id);

    LLColor3 getCloudPosDensity1() const;
    void setCloudPosDensity1(const LLColor3 &val);

    LLColor3 getCloudPosDensity2() const;
    void setCloudPosDensity2(const LLColor3 &val);

    F32 getCloudScale() const;
    void setCloudScale(F32 val);

    LLVector2 getCloudScrollRate() const;
    void setCloudScrollRate(const LLVector2 &val);

    void setCloudScrollRateX(F32 val);
    void setCloudScrollRateY(F32 val);

    F32 getCloudShadow() const;
    void setCloudShadow(F32 val);
    
    F32 getDomeOffset() const;
    F32 getDomeRadius() const;

    F32 getGamma() const;

    void setGamma(F32 val);

    LLColor3 getGlow() const;
    void setGlow(const LLColor3 &val);

    F32 getMaxY() const;

    void setMaxY(F32 val);

    LLQuaternion getMoonRotation() const;
    void setMoonRotation(const LLQuaternion &val);

    LLUUID getMoonTextureId() const;
    void setMoonTextureId(LLUUID id);

    F32 getStarBrightness() const;
    void setStarBrightness(F32 val);

    LLColor3 getSunlightColor() const;
    void setSunlightColor(const LLColor3 &val);

    LLQuaternion getSunRotation() const;
    void setSunRotation(const LLQuaternion &val) ;

    LLUUID getSunTextureId() const;
    void setSunTextureId(LLUUID id);

    //=====================================================================
    // transient properties used in animations.
    LLUUID getNextSunTextureId() const;
    LLUUID getNextMoonTextureId() const;
    LLUUID getNextCloudNoiseTextureId() const;
    LLUUID getNextBloomTextureId() const;

    //=====================================================================
    virtual void loadTextures() { };

    //=====================================================================
    virtual validation_list_t getValidationList() const SETTINGS_OVERRIDE;
    static validation_list_t validationList();

    static LLSD translateLegacySettings(const LLSD& legacy);

// LEGACY_ATMOSPHERICS
    static LLSD translateLegacyHazeSettings(const LLSD& legacy);

    LLColor3 getLightAttenuation(F32 distance) const;
    LLColor3 getLightTransmittance() const;
    LLColor3 gammaCorrect(const LLColor3& in) const;

    LLColor3 getBlueDensity() const;
    LLColor3 getBlueHorizon() const;
    F32 getHazeDensity() const;
    F32 getHazeHorizon() const;
    F32 getDensityMultiplier() const;
    F32 getDistanceMultiplier() const;

    void setBlueDensity(const LLColor3 &val);
    void setBlueHorizon(const LLColor3 &val);
    void setDensityMultiplier(F32 val);
    void setDistanceMultiplier(F32 val);
    void setHazeDensity(F32 val);
    void setHazeHorizon(F32 val);

// Internal/calculated settings
    bool getIsSunUp() const;
    bool getIsMoonUp() const;

    LLVector3 getLightDirection() const;
    LLVector3 getSunDirection() const;
    LLVector3 getMoonDirection() const;
    LLColor4U getFadeColor() const;
    LLColor4  getMoonAmbient() const;
    LLColor3  getMoonDiffuse() const;
    LLColor4  getSunAmbient() const;
    LLColor3  getSunDiffuse() const;
    LLColor4  getTotalAmbient() const;

    virtual LLSettingsBase::ptr_t buildDerivedClone() SETTINGS_OVERRIDE { return buildClone(); }

    static LLUUID GetDefaultAssetId();
    static LLUUID GetDefaultSunTextureId();
    static LLUUID GetDefaultMoonTextureId();
    static LLUUID GetDefaultCloudNoiseTextureId();
    static LLUUID GetDefaultBloomTextureId();

protected:
    static const std::string SETTING_LEGACY_EAST_ANGLE;
    static const std::string SETTING_LEGACY_ENABLE_CLOUD_SCROLL;
    static const std::string SETTING_LEGACY_SUN_ANGLE;

    LLSettingsSky();

    virtual stringset_t getSlerpKeys() const SETTINGS_OVERRIDE;
    virtual stringset_t getSkipInterpolateKeys() const SETTINGS_OVERRIDE;
    virtual void    updateSettings() SETTINGS_OVERRIDE;

private:
    static LLSD rayleighConfigDefault();
    static LLSD absorptionConfigDefault();
    static LLSD mieConfigDefault();

    void        calculateHeavenlyBodyPositions() const;
    void        calculateLightSettings() const;

    mutable LLVector3   mSunDirection;
    mutable LLVector3   mMoonDirection;
    mutable LLVector3   mLightDirection;

    static const F32 DOME_RADIUS;
    static const F32 DOME_OFFSET;

    mutable LLColor4U   mFadeColor;
    mutable LLColor4    mMoonAmbient;
    mutable LLColor3    mMoonDiffuse;
    mutable LLColor4    mSunAmbient;
    mutable LLColor3    mSunDiffuse;
    mutable LLColor4    mTotalAmbient;

    LLUUID      mNextSunTextureId;
    LLUUID      mNextMoonTextureId;
    LLUUID      mNextCloudTextureId;
    LLUUID      mNextBloomTextureId;

    typedef std::map<std::string, S32> mapNameToUniformId_t;

    static mapNameToUniformId_t sNameToUniformMapping;
};

#endif
