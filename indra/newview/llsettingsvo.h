/**
* @file llsettingsvo.h
* @author Rider Linden
* @brief Subclasses for viewer specific settings behaviors.
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

#ifndef LL_SETTINGS_VO_H
#define LL_SETTINGS_VO_H

#include "llsettingsbase.h"
#include "llsettingssky.h"
#include "llsettingswater.h"
#include "llsettingsdaycycle.h"

#include "llsdserialize.h"

#include "llextendedstatus.h"

//=========================================================================
class LLSettingsVOBase : public LLSettingsBase
{
public:
#if 0
    static void     storeAsAsset(const LLSettingsBase::ptr_t &settings);
#endif

    static void     createInventoryItem(const LLSettingsBase::ptr_t &settings);
    
    static void     uploadSettingsAsset(const LLSettingsBase::ptr_t &settings, LLUUID inv_item_id);
    static void     uploadSettingsAsset(const LLSettingsBase::ptr_t &settings, LLUUID object_id, LLUUID inv_item_id);


    static bool     exportFile(const LLSettingsBase::ptr_t &settings, const std::string &filename, LLSDSerialize::ELLSD_Serialize format = LLSDSerialize::LLSD_NOTATION);
    static LLSettingsBase::ptr_t importFile(const std::string &filename);

private:
    struct SettingsSaveData
    {
        typedef std::shared_ptr<SettingsSaveData> ptr_t;
        std::string             mType;
        std::string             mTempFile;
        LLSettingsBase::ptr_t   mSettings;
        LLTransactionID         mTransId;
    };

    LLSettingsVOBase() {}

    static void     onInventoryItemCreated(const LLUUID &inventoryId, LLSettingsBase::ptr_t settings);

#if 0
    static void     onSaveNewAssetComplete(const LLUUID& new_asset_id, const SettingsSaveData::ptr_t &savedata, S32 status, LLExtStat ext_status);
#endif
    static void     onAgentAssetUploadComplete(LLUUID itemId, LLUUID newAssetId, LLUUID newItemId, LLSD response, LLSettingsBase::ptr_t psettings);
    static void     onTaskAssetUploadComplete(LLUUID itemId, LLUUID taskId, LLUUID newAssetId, LLSD response, LLSettingsBase::ptr_t psettings);
};

//=========================================================================
class LLSettingsVOSky : public LLSettingsSky
{
public:
    LLSettingsVOSky(const LLSD &data);

    static ptr_t    buildSky(LLSD settings);

    static ptr_t    buildFromLegacyPreset(const std::string &name, const LLSD &oldsettings);
    static ptr_t    buildDefaultSky();
    virtual ptr_t   buildClone() override;

    static LLSD     convertToLegacy(const ptr_t &);
protected:
    LLSettingsVOSky();

    virtual void    updateSettings() override;

    virtual void    applySpecial(void *) override;

    virtual parammapping_t getParameterMap() const override;

};

//=========================================================================
class LLSettingsVOWater : public LLSettingsWater
{
public:
    LLSettingsVOWater(const LLSD &data);

    static ptr_t    buildWater(LLSD settings);

    static ptr_t    buildFromLegacyPreset(const std::string &name, const LLSD &oldsettings);
    static ptr_t    buildDefaultWater();
    virtual ptr_t   buildClone() override;

    static LLSD     convertToLegacy(const ptr_t &);
protected:
    LLSettingsVOWater();

    virtual void    updateSettings() override;
    virtual void    applySpecial(void *) override;

    virtual parammapping_t getParameterMap() const override;


private:
    static const F32 WATER_FOG_LIGHT_CLAMP;

};

//=========================================================================
class LLSettingsVODay : public LLSettingsDay
{
public:
    LLSettingsVODay(const LLSD &data);

    static ptr_t    buildDay(LLSD settings);

    static ptr_t    buildFromLegacyPreset(const std::string &name, const LLSD &oldsettings);
    static ptr_t    buildFromLegacyMessage(const LLUUID &regionId, LLSD daycycle, LLSD skys, LLSD water);
    static ptr_t    buildDefaultDayCycle();
    static ptr_t    buildFromEnvironmentMessage(LLSD settings);
    virtual ptr_t   buildClone() override;

    static LLSD     convertToLegacy(const ptr_t &);
    
    virtual LLSettingsSkyPtr_t      getDefaultSky() const override;
    virtual LLSettingsWaterPtr_t    getDefaultWater() const override;
    virtual LLSettingsSkyPtr_t      buildSky(LLSD) const override;
    virtual LLSettingsWaterPtr_t    buildWater(LLSD) const override;
    virtual LLSettingsSkyPtr_t      getNamedSky(const std::string &) const override;
    virtual LLSettingsWaterPtr_t    getNamedWater(const std::string &) const override;

protected:
    LLSettingsVODay();
};


#endif
