/**
* @file llsettingsvo.cpp
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

#include "llviewerprecompiledheaders.h"
#include "llviewercontrol.h"
#include "llsettingsvo.h"

#include "pipeline.h"

#include <algorithm>
#include <cstdio>
#include <boost/make_shared.hpp>
#include "lltrace.h"
#include "llfasttimer.h"
#include "v3colorutil.h"

#include "llglslshader.h"
#include "llviewershadermgr.h"

#include "llagent.h"
#include "llassettype.h"
#include "llfloaterperms.h"
#include "llnotificationsutil.h"

#include "llviewerregion.h"
#include "llviewerassetupload.h"
#include "llviewerinventory.h"

#include "llenvironment.h"
#include "llsky.h"

#include "llpermissions.h"

#include "llinventorymodel.h"
#include "llassetstorage.h"
#include "llvfile.h"
#include "lldrawpoolwater.h"

#include <boost/algorithm/string/replace.hpp>
#include "llinventoryobserver.h"
#include "llinventorydefines.h"

#undef  VERIFY_LEGACY_CONVERSION

//=========================================================================
namespace 
{
    LLSD ensure_array_4(LLSD in, F32 fill);
    LLSD read_legacy_preset_data(const std::string &name, const std::string& path);

    //-------------------------------------------------------------------------
    class LLSettingsInventoryCB : public LLInventoryCallback
    {
    public:
        typedef std::function<void(const LLUUID &)> callback_t;

        LLSettingsInventoryCB(callback_t cbfn) :
            mCbfn(cbfn)
        { }

        void fire(const LLUUID& inv_item) override  { if (mCbfn) mCbfn(inv_item); }

    private:
        callback_t  mCbfn;
    };

    //-------------------------------------------------------------------------
}


//=========================================================================
void LLSettingsVOBase::createNewInventoryItem(LLSettingsType::type_e stype, const LLUUID &parent_id, inventory_result_fn callback)
{
    LLTransactionID tid;
    U32 nextOwnerPerm = LLFloaterPerms::getNextOwnerPerms("Settings");
    nextOwnerPerm |= PERM_COPY;

    if (!LLEnvironment::instance().isInventoryEnabled())
    {
        LL_WARNS("SETTINGS") << "Region does not support settings inventory objects." << LL_ENDL;
        LLNotificationsUtil::add("SettingsUnsuported");
        return;
    }

    tid.generate();

    LLPointer<LLInventoryCallback> cb = new LLSettingsInventoryCB([callback](const LLUUID &inventoryId) {
        LLSettingsVOBase::onInventoryItemCreated(inventoryId, LLSettingsBase::ptr_t(), callback);
    });

    create_inventory_settings(gAgent.getID(), gAgent.getSessionID(),
        parent_id, LLTransactionID::tnull,
        LLSettingsType::getDefaultName(stype), "",
        stype, nextOwnerPerm, cb);
}


void LLSettingsVOBase::createInventoryItem(const LLSettingsBase::ptr_t &settings, const LLUUID &parent_id, std::string settings_name, inventory_result_fn callback)
{
    LLTransactionID tid;
    U32             nextOwnerPerm = LLPermissions::DEFAULT.getMaskNextOwner();

    if (!LLEnvironment::instance().isInventoryEnabled())
    {
        LL_WARNS("SETTINGS") << "Region does not support settings inventory objects." << LL_ENDL;
        LLNotificationsUtil::add("SettingsUnsuported");
        return;
    }

    tid.generate();

    LLPointer<LLInventoryCallback> cb = new LLSettingsInventoryCB([settings, callback](const LLUUID &inventoryId) {
            LLSettingsVOBase::onInventoryItemCreated(inventoryId, settings, callback);
        });

    if (settings_name.empty())
    {
        settings_name = settings->getName();
    }
    create_inventory_settings(gAgent.getID(), gAgent.getSessionID(),
        parent_id, tid,
        settings_name, "",
        settings->getSettingsTypeValue(), nextOwnerPerm, cb);
}

void LLSettingsVOBase::onInventoryItemCreated(const LLUUID &inventoryId, LLSettingsBase::ptr_t settings, inventory_result_fn callback)
{
    if (!settings)
    {   // The item was created as new with no settings passed in.  Simulator should have given it the default for the type... check ID, 
        // no need to upload asset.
        LLUUID asset_id;
        LLViewerInventoryItem *pitem = gInventory.getItem(inventoryId);

        if (pitem)
        {
            asset_id = pitem->getAssetUUID();
        }
        if (callback)
            callback(asset_id, inventoryId, LLUUID::null, LLSD());
        return;
    }
    // We need to update some inventory stuff here.... maybe.
    updateInventoryItem(settings, inventoryId, callback, false);
}

void LLSettingsVOBase::updateInventoryItem(const LLSettingsBase::ptr_t &settings, LLUUID inv_item_id, inventory_result_fn callback, bool update_name)
{
    const LLViewerRegion* region = gAgent.getRegion();
    if (!region)
    {
        LL_WARNS("SETTINGS") << "Not connected to a region, cannot save setting." << LL_ENDL;
        return;
    }

    std::string agent_url(region->getCapability("UpdateSettingsAgentInventory"));

    if (!LLEnvironment::instance().isInventoryEnabled())
    {
        LL_WARNS("SETTINGS") << "Region does not support settings inventory objects." << LL_ENDL;
        LLNotificationsUtil::add("SettingsUnsuported");
        return;
    }

    LLViewerInventoryItem *inv_item = gInventory.getItem(inv_item_id);
    if (inv_item)
    {
        bool need_update(false);
        LLPointer<LLViewerInventoryItem> new_item = new LLViewerInventoryItem(inv_item);

        if (settings->getFlag(LLSettingsBase::FLAG_NOTRANS) && new_item->getPermissions().allowOperationBy(PERM_TRANSFER, gAgent.getID()))
        {
            LLPermissions perm(inv_item->getPermissions());
            perm.setBaseBits(LLUUID::null, FALSE, PERM_TRANSFER);
            perm.setOwnerBits(LLUUID::null, FALSE, PERM_TRANSFER);
            new_item->setPermissions(perm);
            need_update |= true;
        }
        if (update_name && (settings->getName() != new_item->getName()))
        {
            new_item->rename(settings->getName());
            settings->setName(new_item->getName()); // account for corrections
            need_update |= true;
        }
        if (need_update)
        {
            new_item->updateServer(FALSE);
            gInventory.updateItem(new_item);
            gInventory.notifyObservers();
        }
    }

    std::stringstream buffer; 
    LLSD settingdata(settings->getSettings());
    LLSDSerialize::serialize(settingdata, buffer, LLSDSerialize::LLSD_NOTATION);

    LLResourceUploadInfo::ptr_t uploadInfo = std::make_shared<LLBufferedAssetUploadInfo>(inv_item_id, LLAssetType::AT_SETTINGS, buffer.str(), 
        [settings, callback](LLUUID itemId, LLUUID newAssetId, LLUUID newItemId, LLSD response) {
            LLSettingsVOBase::onAgentAssetUploadComplete(itemId, newAssetId, newItemId, response, settings, callback);
        });

    LLViewerAssetUpload::EnqueueInventoryUpload(agent_url, uploadInfo);
}

void LLSettingsVOBase::updateInventoryItem(const LLSettingsBase::ptr_t &settings, LLUUID object_id, LLUUID inv_item_id, inventory_result_fn callback)
{
    const LLViewerRegion* region = gAgent.getRegion();
    if (!region)
    {
        LL_WARNS("SETTINGS") << "Not connected to a region, cannot save setting." << LL_ENDL;
        return;
    }

    std::string agent_url(region->getCapability("UpdateSettingsAgentInventory"));

    if (!LLEnvironment::instance().isInventoryEnabled())
    {
        LL_WARNS("SETTINGS") << "Region does not support settings inventory objects." << LL_ENDL;
        LLNotificationsUtil::add("SettingsUnsuported");
        return;
    }

    std::stringstream buffer;
    LLSD settingdata(settings->getSettings());

    LLSDSerialize::serialize(settingdata, buffer, LLSDSerialize::LLSD_NOTATION);

    LLResourceUploadInfo::ptr_t uploadInfo = std::make_shared<LLBufferedAssetUploadInfo>(object_id, inv_item_id, LLAssetType::AT_SETTINGS, buffer.str(),
        [settings, callback](LLUUID itemId, LLUUID taskId, LLUUID newAssetId, LLSD response) {
        LLSettingsVOBase::onTaskAssetUploadComplete(itemId, taskId, newAssetId, response, settings, callback);
    });

    LLViewerAssetUpload::EnqueueInventoryUpload(agent_url, uploadInfo);
}

void LLSettingsVOBase::onAgentAssetUploadComplete(LLUUID itemId, LLUUID newAssetId, LLUUID newItemId, LLSD response, LLSettingsBase::ptr_t psettings, inventory_result_fn callback)
{
    LL_INFOS("SETTINGS") << "itemId:" << itemId << " newAssetId:" << newAssetId << " newItemId:" << newItemId << " response:" << response << LL_ENDL;
    psettings->setAssetId(newAssetId);
    if (callback)
        callback( newAssetId, itemId, LLUUID::null, response );

#if 0
    std::string exprtFile = gDirUtilp->getTempDir() + gDirUtilp->getDirDelimiter() + newAssetId.asString() + ".llsd";

    LLSettingsVOBase::exportFile(psettings, exprtFile, LLSDSerialize::LLSD_NOTATION);

    LL_WARNS("LAPRAS") << "SETTINGS File written as: '" << exprtFile << "'" << LL_ENDL;
#endif
}

void LLSettingsVOBase::onTaskAssetUploadComplete(LLUUID itemId, LLUUID taskId, LLUUID newAssetId, LLSD response, LLSettingsBase::ptr_t psettings, inventory_result_fn callback)
{
    LL_INFOS("SETTINGS") << "Upload to task complete!" << LL_ENDL;
    psettings->setAssetId(newAssetId);
    if (callback)
        callback(newAssetId, itemId, taskId, response);
}


void LLSettingsVOBase::getSettingsAsset(const LLUUID &assetId, LLSettingsVOBase::asset_download_fn callback)
{
    gAssetStorage->getAssetData(assetId, LLAssetType::AT_SETTINGS,
        [callback](LLVFS *vfs, const LLUUID &asset_id, LLAssetType::EType, void *, S32 status, LLExtStat ext_status) 
            { onAssetDownloadComplete(vfs, asset_id, status, ext_status, callback); },
        nullptr, true);

}

void LLSettingsVOBase::onAssetDownloadComplete(LLVFS *vfs, const LLUUID &asset_id, S32 status, LLExtStat ext_status, LLSettingsVOBase::asset_download_fn callback)
{
    LLSettingsBase::ptr_t settings;
    if (!status)
    {
        LLVFile file(vfs, asset_id, LLAssetType::AT_SETTINGS, LLVFile::READ);
        S32 size = file.getSize();

        std::string buffer(size + 1, '\0');
        file.read((U8 *)buffer.data(), size);

        std::stringstream llsdstream(buffer);
        LLSD llsdsettings;

        if (LLSDSerialize::deserialize(llsdsettings, llsdstream, -1))
        {
            settings = createFromLLSD(llsdsettings);
        }

        if (!settings)
        {
            status = 1;
            LL_WARNS("SETTINGS") << "Unable to create settings object." << LL_ENDL;
        }
        else
        {
            LL_WARNS("LAPRAS") << "Setting asset ID to " << asset_id << LL_ENDL;
            settings->setAssetId(asset_id);
        }
    }
    else
    {
        LL_WARNS("SETTINGS") << "Error retrieving asset " << asset_id << ". Status code=" << status << "(" << LLAssetStorage::getErrorString(status) << ") ext_status=" << ext_status << LL_ENDL;
    }
    if (callback)
        callback(asset_id, settings, status, ext_status);
}

void LLSettingsVOBase::getSettingsInventory(const LLUUID &inventoryId, inventory_download_fn callback)
{

}

bool LLSettingsVOBase::exportFile(const LLSettingsBase::ptr_t &settings, const std::string &filename, LLSDSerialize::ELLSD_Serialize format)
{
    try
    {
        std::ofstream file(filename, std::ios::out | std::ios::trunc);
        file.exceptions(std::ios_base::failbit | std::ios_base::badbit);

        if (!file)
        {
            LL_WARNS("SETTINGS") << "Unable to open '" << filename << "' for writing." << LL_ENDL;
            return false;
        }

        LLSDSerialize::serialize(settings->getSettings(), file, format);
    }
    catch (const std::ios_base::failure &e)
    {
        LL_WARNS("SETTINGS") << "Unable to save settings to file '" << filename << "': " << e.what() << LL_ENDL;
        return false;
    }

    return true;
}

LLSettingsBase::ptr_t LLSettingsVOBase::importFile(const std::string &filename)
{
    LLSD settings;

    try
    {
        std::ifstream file(filename, std::ios::in);
        file.exceptions(std::ios_base::failbit | std::ios_base::badbit);

        if (!file)
        {
            LL_WARNS("SETTINGS") << "Unable to open '" << filename << "' for reading." << LL_ENDL;
            return LLSettingsBase::ptr_t();
        }

        if (!LLSDSerialize::deserialize(settings, file, -1))
        {
            LL_WARNS("SETTINGS") << "Unable to deserialize settings from '" << filename << "'" << LL_ENDL;
            return LLSettingsBase::ptr_t();
        }
    }
    catch (const std::ios_base::failure &e)
    {
        LL_WARNS("SETTINGS") << "Unable to save settings to file '" << filename << "': " << e.what() << LL_ENDL;
        return LLSettingsBase::ptr_t();
    }

    return createFromLLSD(settings);
}

LLSettingsBase::ptr_t LLSettingsVOBase::createFromLLSD(const LLSD &settings)
{
    if (!settings.has(SETTING_TYPE))
    {
        LL_WARNS("SETTINGS") << "No settings type in LLSD" << LL_ENDL;
        return LLSettingsBase::ptr_t();
    }

    std::string settingtype = settings[SETTING_TYPE].asString();

    LLSettingsBase::ptr_t psetting;

    if (settingtype == "water")
    {
        return LLSettingsVOWater::buildWater(settings);
    }
    else if (settingtype == "sky")
    {
        return LLSettingsVOSky::buildSky(settings);
    }
    else if (settingtype == "daycycle")
    {
        return LLSettingsVODay::buildDay(settings);
    }

    LL_WARNS("SETTINGS") << "Unable to determine settings type for '" << settingtype << "'." << LL_ENDL;
    return LLSettingsBase::ptr_t();

}

//=========================================================================
LLSettingsVOSky::LLSettingsVOSky(const LLSD &data, bool isAdvanced)
: LLSettingsSky(data)
, m_isAdvanced(isAdvanced)
{
}

LLSettingsVOSky::LLSettingsVOSky()
: LLSettingsSky()
, m_isAdvanced(false)
{
}

//-------------------------------------------------------------------------
LLSettingsSky::ptr_t LLSettingsVOSky::buildSky(LLSD settings)
{
    LLSettingsSky::validation_list_t validations = LLSettingsSky::validationList();

    LLSD results = LLSettingsBase::settingValidation(settings, validations);

    if (!results["success"].asBoolean())
    {
        LL_WARNS("SETTINGS") << "Sky setting validation failed!\n" << results << LL_ENDL;
        LLSettingsSky::ptr_t();
    }

    return std::make_shared<LLSettingsVOSky>(settings, true);
}


LLSettingsSky::ptr_t LLSettingsVOSky::buildFromLegacyPreset(const std::string &name, const LLSD &legacy)
{

    LLSD newsettings = LLSettingsSky::translateLegacySettings(legacy);

    newsettings[SETTING_NAME] = name;

    LLSettingsSky::validation_list_t validations = LLSettingsSky::validationList();
    LLSD results = LLSettingsBase::settingValidation(newsettings, validations);
    if (!results["success"].asBoolean())
    {
        LL_WARNS("SETTINGS") << "Sky setting validation failed!\n" << results << LL_ENDL;
        LLSettingsSky::ptr_t();
    }

    LLSettingsSky::ptr_t skyp = std::make_shared<LLSettingsVOSky>(newsettings);

#ifdef VERIFY_LEGACY_CONVERSION
    LLSD oldsettings = LLSettingsVOSky::convertToLegacy(skyp, isAdvanced());

    if (!llsd_equals(legacy, oldsettings))
    {
        LL_WARNS("SKY") << "Conversion to/from legacy does not match!\n" 
            << "Old: " << legacy
            << "new: " << oldsettings << LL_ENDL;
    }

#endif

    return skyp;
}

LLSettingsSky::ptr_t LLSettingsVOSky::buildFromLegacyPresetFile(const std::string &name, const std::string &path)
{
    LLSD legacy_data = read_legacy_preset_data(name, path);

    if (!legacy_data)
    {
        LL_WARNS("SETTINGS") << "Could not load legacy Windlight \"" << name << "\" from " << path << LL_ENDL;
        return ptr_t();
    }

    return buildFromLegacyPreset(name, legacy_data);
}


LLSettingsSky::ptr_t LLSettingsVOSky::buildDefaultSky()
{
    static LLSD default_settings;

    if (!default_settings.size())
    {
        default_settings = LLSettingsSky::defaults();

        default_settings[SETTING_NAME] = std::string("_default_");

        LLSettingsSky::validation_list_t validations = LLSettingsSky::validationList();
        LLSD results = LLSettingsBase::settingValidation(default_settings, validations);
        if (!results["success"].asBoolean())
        {
            LL_WARNS("SETTINGS") << "Sky setting validation failed!\n" << results << LL_ENDL;
            LLSettingsSky::ptr_t();
        }
    }

    LLSettingsSky::ptr_t skyp = std::make_shared<LLSettingsVOSky>(default_settings);
    return skyp;
}

LLSettingsSky::ptr_t LLSettingsVOSky::buildClone() const
{
    LLSD settings = cloneSettings();

    LLSettingsSky::validation_list_t validations = LLSettingsSky::validationList();
    LLSD results = LLSettingsBase::settingValidation(settings, validations);
    if (!results["success"].asBoolean())
    {
        LL_WARNS("SETTINGS") << "Sky setting validation failed!\n" << results << LL_ENDL;
        LLSettingsSky::ptr_t();
    }

    LLSettingsSky::ptr_t skyp = std::make_shared<LLSettingsVOSky>(settings);
    return skyp;
}

void LLSettingsVOSky::convertAtmosphericsToLegacy(LLSD& legacy, LLSD& settings)
{
    // These will need to be inferred from new settings' density profiles
    if (settings.has(SETTING_LEGACY_HAZE))
    {
        LLSD legacyhaze = settings[SETTING_LEGACY_HAZE];
        legacy[SETTING_AMBIENT]             = ensure_array_4(legacyhaze[SETTING_AMBIENT], 1.0f);
        legacy[SETTING_BLUE_DENSITY]        = ensure_array_4(legacyhaze[SETTING_BLUE_DENSITY], 1.0);
        legacy[SETTING_BLUE_HORIZON]        = ensure_array_4(legacyhaze[SETTING_BLUE_HORIZON], 1.0);
        legacy[SETTING_DENSITY_MULTIPLIER]  = LLSDArray(legacyhaze[SETTING_DENSITY_MULTIPLIER].asReal())(0.0f)(0.0f)(1.0f);
        legacy[SETTING_DISTANCE_MULTIPLIER] = LLSDArray(legacyhaze[SETTING_DISTANCE_MULTIPLIER].asReal())(0.0f)(0.0f)(1.0f);
        legacy[SETTING_HAZE_DENSITY]        = LLSDArray(legacyhaze[SETTING_HAZE_DENSITY])(0.0f)(0.0f)(1.0f);
        legacy[SETTING_HAZE_HORIZON]        = LLSDArray(legacyhaze[SETTING_HAZE_HORIZON])(0.0f)(0.0f)(1.0f);
    }
}

LLSD LLSettingsVOSky::convertToLegacy(const LLSettingsSky::ptr_t &psky, bool isAdvanced)
{
    LLSD legacy(LLSD::emptyMap());
    LLSD settings = psky->getSettings();
    
    convertAtmosphericsToLegacy(legacy, settings);

    legacy[SETTING_CLOUD_COLOR] = ensure_array_4(settings[SETTING_CLOUD_COLOR], 1.0);
    legacy[SETTING_CLOUD_POS_DENSITY1] = ensure_array_4(settings[SETTING_CLOUD_POS_DENSITY1], 1.0);
    legacy[SETTING_CLOUD_POS_DENSITY2] = ensure_array_4(settings[SETTING_CLOUD_POS_DENSITY2], 1.0);
    legacy[SETTING_CLOUD_SCALE] = LLSDArray(settings[SETTING_CLOUD_SCALE])(LLSD::Real(0.0))(LLSD::Real(0.0))(LLSD::Real(1.0));       
    legacy[SETTING_CLOUD_SCROLL_RATE] = settings[SETTING_CLOUD_SCROLL_RATE];
    legacy[SETTING_LEGACY_ENABLE_CLOUD_SCROLL] = LLSDArray(LLSD::Boolean(!is_approx_zero(settings[SETTING_CLOUD_SCROLL_RATE][0].asReal())))
        (LLSD::Boolean(!is_approx_zero(settings[SETTING_CLOUD_SCROLL_RATE][1].asReal())));     
    legacy[SETTING_CLOUD_SHADOW] = LLSDArray(settings[SETTING_CLOUD_SHADOW].asReal())(0.0f)(0.0f)(1.0f);    
    legacy[SETTING_GAMMA] = LLSDArray(settings[SETTING_GAMMA])(0.0f)(0.0f)(1.0f);
    legacy[SETTING_GLOW] = ensure_array_4(settings[SETTING_GLOW], 1.0);
    legacy[SETTING_LIGHT_NORMAL] = ensure_array_4(psky->getLightDirection().getValue(), 0.0f);
    legacy[SETTING_MAX_Y] = LLSDArray(settings[SETTING_MAX_Y])(0.0f)(0.0f)(1.0f);
    legacy[SETTING_STAR_BRIGHTNESS] = settings[SETTING_STAR_BRIGHTNESS];
    legacy[SETTING_SUNLIGHT_COLOR] = ensure_array_4(settings[SETTING_SUNLIGHT_COLOR], 1.0f);
    
    LLQuaternion sunquat = psky->getSunRotation();

    F32 roll;
    F32 pitch;
    F32 yaw;

    // get euler angles in right-handed X right, Y up, Z at
    sunquat.getEulerAngles(&roll, &pitch, &yaw);
    
    legacy[SETTING_LEGACY_EAST_ANGLE] = yaw;
    legacy[SETTING_LEGACY_SUN_ANGLE]  = -pitch;

    return legacy;    
}

//-------------------------------------------------------------------------
void LLSettingsVOSky::updateSettings()
{
    LLSettingsSky::updateSettings();
    LLVector3 sun_direction  = getSunDirection();
    LLVector3 moon_direction = getMoonDirection();

    F32 dp = getLightDirection() * LLVector3(0.0f, 0.0f, 1.0f);
	if (dp < 0)
	{
		dp = 0;
	}
    dp = llmax(dp, 0.1f);

	// Since WL scales everything by 2, there should always be at least a 2:1 brightness ratio
	// between sunlight and point lights in windlight to normalize point lights.
	F32 sun_dynamic_range = llmax(gSavedSettings.getF32("RenderSunDynamicRange"), 0.0001f);
    mSceneLightStrength = 2.0f * (1.0f + sun_dynamic_range * dp);

    gSky.setSunAndMoonDirectionsCFR(sun_direction, moon_direction);
    gSky.setSunTextures(getSunTextureId(), getNextSunTextureId());
    gSky.setMoonTextures(getMoonTextureId(), getNextMoonTextureId());
    gSky.setCloudNoiseTextures(getCloudNoiseTextureId(), getNextCloudNoiseTextureId());
    gSky.setBloomTextures(getBloomTextureId(), getNextBloomTextureId());

    gSky.setSunScale(getSunScale());
    gSky.setMoonScale(getMoonScale());
}

void LLSettingsVOSky::applySpecial(void *ptarget)
{
    LLGLSLShader *shader = (LLGLSLShader *)ptarget;

    LLVector4 light_direction = LLEnvironment::instance().getClampedSunNorm();

    if (shader->mShaderGroup == LLGLSLShader::SG_DEFAULT)
	{        
        shader->uniform4fv(LLViewerShaderMgr::LIGHTNORM, 1, light_direction.mV);
		shader->uniform3fv(LLShaderMgr::WL_CAMPOSLOCAL, 1, LLViewerCamera::getInstance()->getOrigin().mV);
	} 
	else if (shader->mShaderGroup == LLGLSLShader::SG_SKY)
	{
        shader->uniform4fv(LLViewerShaderMgr::LIGHTNORM, 1, light_direction.mV);        

        LLVector4 vect_c_p_d1(mSettings[SETTING_CLOUD_POS_DENSITY1]);
        vect_c_p_d1 += LLVector4(LLEnvironment::instance().getCloudScrollDelta());
        shader->uniform4fv(LLShaderMgr::CLOUD_POS_DENSITY1, 1, vect_c_p_d1.mV);
	}

    shader->uniform1f(LLShaderMgr::SCENE_LIGHT_STRENGTH, mSceneLightStrength);
    shader->uniform4f(LLShaderMgr::GAMMA, getGamma(), 0.0, 0.0, 1.0);
}

LLSettingsSky::parammapping_t LLSettingsVOSky::getParameterMap() const
{
    static parammapping_t param_map;

    if (param_map.empty())
    {
// LEGACY_ATMOSPHERICS
        param_map[SETTING_AMBIENT] = LLShaderMgr::AMBIENT;
        param_map[SETTING_BLUE_DENSITY] = LLShaderMgr::BLUE_DENSITY;
        param_map[SETTING_BLUE_HORIZON] = LLShaderMgr::BLUE_HORIZON;
        param_map[SETTING_HAZE_DENSITY] = LLShaderMgr::HAZE_DENSITY;
        param_map[SETTING_HAZE_HORIZON] = LLShaderMgr::HAZE_HORIZON;
        param_map[SETTING_DENSITY_MULTIPLIER] = LLShaderMgr::DENSITY_MULTIPLIER;
        param_map[SETTING_DISTANCE_MULTIPLIER] = LLShaderMgr::DISTANCE_MULTIPLIER;

        param_map[SETTING_CLOUD_COLOR] = LLShaderMgr::CLOUD_COLOR;
        param_map[SETTING_CLOUD_POS_DENSITY2] = LLShaderMgr::CLOUD_POS_DENSITY2;
        param_map[SETTING_CLOUD_SCALE] = LLShaderMgr::CLOUD_SCALE;
        param_map[SETTING_CLOUD_SHADOW] = LLShaderMgr::CLOUD_SHADOW;       
        param_map[SETTING_CLOUD_VARIANCE] = LLShaderMgr::CLOUD_VARIANCE;
        param_map[SETTING_GLOW] = LLShaderMgr::GLOW;        
        param_map[SETTING_MAX_Y] = LLShaderMgr::MAX_Y;
        param_map[SETTING_SUNLIGHT_COLOR] = LLShaderMgr::SUNLIGHT_COLOR;
        param_map[SETTING_MOON_BRIGHTNESS] = LLShaderMgr::MOON_BRIGHTNESS;
        param_map[SETTING_SKY_MOISTURE_LEVEL] = LLShaderMgr::MOISTURE_LEVEL;
        param_map[SETTING_SKY_DROPLET_RADIUS] = LLShaderMgr::DROPLET_RADIUS;
        param_map[SETTING_SKY_ICE_LEVEL] = LLShaderMgr::ICE_LEVEL;

// AdvancedAtmospherics TODO
// Provide mappings for new shader params here
    }

    return param_map;
}

//=========================================================================
const F32 LLSettingsVOWater::WATER_FOG_LIGHT_CLAMP(0.3f);

//-------------------------------------------------------------------------
LLSettingsVOWater::LLSettingsVOWater(const LLSD &data) :
    LLSettingsWater(data)
{

}

LLSettingsVOWater::LLSettingsVOWater() :
    LLSettingsWater()
{

}

LLSettingsWater::ptr_t LLSettingsVOWater::buildWater(LLSD settings)
{
    LLSettingsWater::validation_list_t validations = LLSettingsWater::validationList();
    LLSD results = LLSettingsWater::settingValidation(settings, validations);
    if (!results["success"].asBoolean())
    {
        LL_WARNS("SETTINGS") << "Water setting validation failed!\n" << results << LL_ENDL;
        LLSettingsWater::ptr_t();
    }

    return std::make_shared<LLSettingsVOWater>(settings);
}

//-------------------------------------------------------------------------
LLSettingsWater::ptr_t LLSettingsVOWater::buildFromLegacyPreset(const std::string &name, const LLSD &legacy)
{
    LLSD newsettings(LLSettingsWater::translateLegacySettings(legacy));

    newsettings[SETTING_NAME] = name; 
    LLSettingsWater::validation_list_t validations = LLSettingsWater::validationList();
    LLSD results = LLSettingsWater::settingValidation(newsettings, validations);
    if (!results["success"].asBoolean())
    {
        LL_WARNS("SETTINGS") << "Water setting validation failed!: " << results << LL_ENDL;
        return LLSettingsWater::ptr_t();
    }

    LLSettingsWater::ptr_t waterp = std::make_shared<LLSettingsVOWater>(newsettings);

#ifdef VERIFY_LEGACY_CONVERSION
    LLSD oldsettings = LLSettingsVOWater::convertToLegacy(waterp);

    if (!llsd_equals(legacy, oldsettings))
    {
        LL_WARNS("WATER") << "Conversion to/from legacy does not match!\n"
            << "Old: " << legacy
            << "new: " << oldsettings << LL_ENDL;
    }

#endif
    return waterp;
}

LLSettingsWater::ptr_t LLSettingsVOWater::buildFromLegacyPresetFile(const std::string &name, const std::string &path)
{
    LLSD legacy_data = read_legacy_preset_data(name, path);

    if (!legacy_data)
    {
        LL_WARNS("SETTINGS") << "Could not load legacy Windlight \"" << name << "\" from " << path << LL_ENDL;
        return ptr_t();
    }

    return buildFromLegacyPreset(name, legacy_data);
}


LLSettingsWater::ptr_t LLSettingsVOWater::buildDefaultWater()
{
    static LLSD default_settings;

    if (!default_settings.size())
    {
        default_settings = LLSettingsWater::defaults();

        default_settings[SETTING_NAME] = std::string("_default_");

        LLSettingsWater::validation_list_t validations = LLSettingsWater::validationList();
        LLSD results = LLSettingsWater::settingValidation(default_settings, validations);
        if (!results["success"].asBoolean())
        {
            LL_WARNS("SETTINGS") << "Water setting validation failed!: " << results << LL_ENDL;
            return LLSettingsWater::ptr_t();
        }
    }

    LLSettingsWater::ptr_t waterp = std::make_shared<LLSettingsVOWater>(default_settings);

    return waterp;
}

LLSettingsWater::ptr_t LLSettingsVOWater::buildClone() const
{
    LLSD settings = cloneSettings();
    LLSettingsWater::validation_list_t validations = LLSettingsWater::validationList();
    LLSD results = LLSettingsWater::settingValidation(settings, validations);
    if (!results["success"].asBoolean())
    {
        LL_WARNS("SETTINGS") << "Water setting validation failed!: " << results << LL_ENDL;
        return LLSettingsWater::ptr_t();
    }

    LLSettingsWater::ptr_t waterp = std::make_shared<LLSettingsVOWater>(settings);

    return waterp;
}

LLSD LLSettingsVOWater::convertToLegacy(const LLSettingsWater::ptr_t &pwater)
{
    LLSD legacy(LLSD::emptyMap());
    LLSD settings = pwater->getSettings();

    legacy[SETTING_LEGACY_BLUR_MULTIPLIER] = settings[SETTING_BLUR_MULTIPLIER];
    legacy[SETTING_LEGACY_FOG_COLOR] = ensure_array_4(settings[SETTING_FOG_COLOR], 1.0f);
    legacy[SETTING_LEGACY_FOG_DENSITY] = settings[SETTING_FOG_DENSITY];
    legacy[SETTING_LEGACY_FOG_MOD] = settings[SETTING_FOG_MOD];
    legacy[SETTING_LEGACY_FRESNEL_OFFSET] = settings[SETTING_FRESNEL_OFFSET];
    legacy[SETTING_LEGACY_FRESNEL_SCALE] = settings[SETTING_FRESNEL_SCALE];
    legacy[SETTING_LEGACY_NORMAL_MAP] = settings[SETTING_NORMAL_MAP];
    legacy[SETTING_LEGACY_NORMAL_SCALE] = settings[SETTING_NORMAL_SCALE];
    legacy[SETTING_LEGACY_SCALE_ABOVE] = settings[SETTING_SCALE_ABOVE];
    legacy[SETTING_LEGACY_SCALE_BELOW] = settings[SETTING_SCALE_BELOW];
    legacy[SETTING_LEGACY_WAVE1_DIR] = settings[SETTING_WAVE1_DIR];
    legacy[SETTING_LEGACY_WAVE2_DIR] = settings[SETTING_WAVE2_DIR];
    
    //_WARNS("LAPRAS") << "Legacy water: " << legacy << LL_ENDL;
    return legacy;
}
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
void LLSettingsVOWater::applySpecial(void *ptarget)
{
    LLGLSLShader *shader = (LLGLSLShader *)ptarget;

    if (shader->mShaderGroup == LLGLSLShader::SG_WATER)
	{
        //transform water plane to eye space
        glh::vec3f norm(0.f, 0.f, 1.f);
        glh::vec3f p(0.f, 0.f, LLEnvironment::instance().getWaterHeight() + 0.1f);

        F32 modelView[16];
        for (U32 i = 0; i < 16; i++)
        {
            modelView[i] = (F32)gGLModelView[i];
        }

        glh::matrix4f mat(modelView);
        glh::matrix4f invtrans = mat.inverse().transpose();
        glh::vec3f enorm;
        glh::vec3f ep;
        invtrans.mult_matrix_vec(norm, enorm);
        enorm.normalize();
        mat.mult_matrix_vec(p, ep);

        LLVector4 waterPlane(enorm.v[0], enorm.v[1], enorm.v[2], -ep.dot(enorm));

        shader->uniform4fv(LLShaderMgr::WATER_WATERPLANE, 1, waterPlane.mV);

        LLVector4 light_direction = LLEnvironment::instance().getClampedLightNorm();

        F32 waterFogKS = 1.f / llmax(light_direction.mV[2], WATER_FOG_LIGHT_CLAMP);

        shader->uniform1f(LLShaderMgr::WATER_FOGKS, waterFogKS);

        F32 blend_factor = LLEnvironment::instance().getCurrentWater()->getBlendFactor();
        shader->uniform1f(LLShaderMgr::BLEND_FACTOR, blend_factor);

        LLVector4 rotated_light_direction = LLEnvironment::instance().getRotatedLightNorm();
        shader->uniform4fv(LLViewerShaderMgr::LIGHTNORM, 1, rotated_light_direction.mV);
        shader->uniform3fv(LLShaderMgr::WL_CAMPOSLOCAL, 1, LLViewerCamera::getInstance()->getOrigin().mV);
        shader->uniform1f(LLViewerShaderMgr::DISTANCE_MULTIPLIER, 0);
    }
}

void LLSettingsVOWater::updateSettings()
{
    // base class clears dirty flag so as to not trigger recursive update
    LLSettingsBase::updateSettings();

    LLDrawPoolWater* pwaterpool = (LLDrawPoolWater*)gPipeline.getPool(LLDrawPool::POOL_WATER);
    if (pwaterpool)
    {
        pwaterpool->setTransparentTextures(getTransparentTextureID(), getNextTransparentTextureID());
        pwaterpool->setOpaqueTexture(GetDefaultOpaqueTextureAssetId());
        pwaterpool->setNormalMaps(getNormalMapID(), getNextNormalMapID());
    }
}

LLSettingsWater::parammapping_t LLSettingsVOWater::getParameterMap() const
{
    static parammapping_t param_map;

    if (param_map.empty())
    {
        param_map[SETTING_FOG_COLOR] = LLShaderMgr::WATER_FOGCOLOR;
        param_map[SETTING_FOG_DENSITY] = LLShaderMgr::WATER_FOGDENSITY;
    }
    return param_map;
}

//=========================================================================
LLSettingsVODay::LLSettingsVODay(const LLSD &data):
    LLSettingsDay(data)
{}

LLSettingsVODay::LLSettingsVODay():
    LLSettingsDay()
{}

LLSettingsDay::ptr_t LLSettingsVODay::buildDay(LLSD settings)
{
    LLSettingsDay::validation_list_t validations = LLSettingsDay::validationList();
    LLSD results = LLSettingsDay::settingValidation(settings, validations);
    if (!results["success"].asBoolean())
    {
        LL_WARNS("SETTINGS") << "Day setting validation failed!\n" << results << LL_ENDL;
        LLSettingsDay::ptr_t();
    }

    LLSettingsDay::ptr_t pday = std::make_shared<LLSettingsVODay>(settings);
    if (pday)
        pday->initialize();

    return pday;
}

//-------------------------------------------------------------------------
LLSettingsDay::ptr_t LLSettingsVODay::buildFromLegacyPreset(const std::string &name, const std::string &path, const LLSD &oldsettings)
{
    LLSD newsettings(defaults());
    std::set<std::string> framenames;
    std::set<std::string> notfound;

    std::string base_path(gDirUtilp->getDirName(path));
    std::string water_path(base_path);
    std::string sky_path(base_path);

    gDirUtilp->append(water_path, "water");
    gDirUtilp->append(sky_path, "skies");

    newsettings[SETTING_NAME] = name;

    LLSD watertrack = LLSDArray(
        LLSDMap(SETTING_KEYKFRAME, LLSD::Real(0.0f))
        (SETTING_KEYNAME, "water:Default"));

    LLSD skytrack = LLSD::emptyArray();

    for (LLSD::array_const_iterator it = oldsettings.beginArray(); it != oldsettings.endArray(); ++it)
    {
        std::string framename = (*it)[1].asString();
        LLSD entry = LLSDMap(SETTING_KEYKFRAME, (*it)[0].asReal())
            (SETTING_KEYNAME, "sky:" + framename);
        framenames.insert(framename);
        skytrack.append(entry);
    }

    newsettings[SETTING_TRACKS] = LLSDArray(watertrack)(skytrack);

    LLSD frames(LLSD::emptyMap());

    {
        LLSettingsWater::ptr_t pwater = LLSettingsVOWater::buildFromLegacyPresetFile("Default", water_path);
        if (pwater)
            frames["water:Default"] = pwater->getSettings();
    }

    for (std::set<std::string>::iterator itn = framenames.begin(); itn != framenames.end(); ++itn)
    {
        LLSettingsSky::ptr_t psky = LLSettingsVOSky::buildFromLegacyPresetFile((*itn), sky_path);
        if (psky)
            frames["sky:" + (*itn)] = psky->getSettings();
    }

    newsettings[SETTING_FRAMES] = frames;

    LLSettingsDay::validation_list_t validations = LLSettingsDay::validationList();
    LLSD results = LLSettingsDay::settingValidation(newsettings, validations);
    if (!results["success"].asBoolean())
    {
        LL_WARNS("SETTINGS") << "Day setting validation failed!: " << results << LL_ENDL;
        return LLSettingsDay::ptr_t();
    }

    LLSettingsDay::ptr_t dayp = std::make_shared<LLSettingsVODay>(newsettings);

#ifdef VERIFY_LEGACY_CONVERSION
    LLSD testsettings = LLSettingsVODay::convertToLegacy(dayp);

    if (!llsd_equals(oldsettings, testsettings))
    {
        LL_WARNS("DAYCYCLE") << "Conversion to/from legacy does not match!\n" 
            << "Old: " << oldsettings
            << "new: " << testsettings << LL_ENDL;
    }

#endif

    dayp->initialize();

    return dayp;
}

LLSettingsDay::ptr_t LLSettingsVODay::buildFromLegacyPresetFile(const std::string &name, const std::string &path)
{
    LLSD legacy_data = read_legacy_preset_data(name, path);

    if (!legacy_data)
    {
        LL_WARNS("SETTINGS") << "Could not load legacy Windlight \"" << name << "\" from " << path << LL_ENDL;
        return ptr_t();
    }

    return buildFromLegacyPreset(name, path, legacy_data);
}



LLSettingsDay::ptr_t LLSettingsVODay::buildFromLegacyMessage(const LLUUID &regionId, LLSD daycycle, LLSD skydefs, LLSD waterdef)
{
    LLSD frames(LLSD::emptyMap());

    for (LLSD::map_iterator itm = skydefs.beginMap(); itm != skydefs.endMap(); ++itm)
    {
        std::string newname = "sky:" + (*itm).first;
        LLSD newsettings = LLSettingsSky::translateLegacySettings((*itm).second);
        
        newsettings[SETTING_NAME] = newname;
        frames[newname] = newsettings;

        LL_WARNS("SETTINGS") << "created region sky '" << newname << "'" << LL_ENDL;
    }

    LLSD watersettings = LLSettingsWater::translateLegacySettings(waterdef);
    std::string watername = "water:"+ watersettings[SETTING_NAME].asString();
    watersettings[SETTING_NAME] = watername;
    frames[watername] = watersettings;

    LLSD watertrack = LLSDArray(
            LLSDMap(SETTING_KEYKFRAME, LLSD::Real(0.0f))
            (SETTING_KEYNAME, watername));

    LLSD skytrack(LLSD::emptyArray());
    for (LLSD::array_const_iterator it = daycycle.beginArray(); it != daycycle.endArray(); ++it)
    {
        LLSD entry = LLSDMap(SETTING_KEYKFRAME, (*it)[0].asReal())
            (SETTING_KEYNAME, "sky:" + (*it)[1].asString());
        skytrack.append(entry);
    }

    LLSD newsettings = LLSDMap
        ( SETTING_NAME, "Region (legacy)" )
        ( SETTING_TRACKS, LLSDArray(watertrack)(skytrack))
        ( SETTING_FRAMES, frames )
        ( SETTING_TYPE, "daycycle" );

    LL_WARNS("LAPRAS") << "newsettings=" << newsettings << LL_ENDL;

    LLSettingsSky::validation_list_t validations = LLSettingsDay::validationList();
    LLSD results = LLSettingsDay::settingValidation(newsettings, validations);
    if (!results["success"].asBoolean())
    {
        LL_WARNS("SETTINGS") << "Day setting validation failed!:" << results << LL_ENDL;
        return LLSettingsDay::ptr_t();
    }

    LLSettingsDay::ptr_t dayp = std::make_shared<LLSettingsVODay>(newsettings);
    
    if (dayp)
    {
        // true for validation - either validate here, or when cloning for floater.
        dayp->initialize(true);
    }
    return dayp;
}



LLSettingsDay::ptr_t LLSettingsVODay::buildDefaultDayCycle()
{
    static LLSD default_settings;

    if (!default_settings.size())
    {
        default_settings = LLSettingsDay::defaults();
        default_settings[SETTING_NAME] = std::string("_default_");

        LLSettingsDay::validation_list_t validations = LLSettingsDay::validationList();
        LLSD results = LLSettingsDay::settingValidation(default_settings, validations);
        if (!results["success"].asBoolean())
        {
            LL_WARNS("SETTINGS") << "Day setting validation failed!\n" << results << LL_ENDL;
            LLSettingsDay::ptr_t();
        }
    }

    LLSettingsDay::ptr_t dayp = std::make_shared<LLSettingsVODay>(default_settings);

    dayp->initialize();
    return dayp;
}

LLSettingsDay::ptr_t LLSettingsVODay::buildFromEnvironmentMessage(LLSD settings)
{
    LLSettingsDay::validation_list_t validations = LLSettingsDay::validationList();
    LLSD results = LLSettingsDay::settingValidation(settings, validations);
    if (!results["success"].asBoolean())
    {
        LL_WARNS("SETTINGS") << "Day setting validation failed!\n" << results << LL_ENDL;
        LLSettingsDay::ptr_t();
    }

    LLSettingsDay::ptr_t dayp = std::make_shared<LLSettingsVODay>(settings);

    dayp->initialize();
    return dayp;
}


void LLSettingsVODay::buildFromOtherSetting(LLSettingsBase::ptr_t settings, LLSettingsVODay::asset_built_fn cb)
{
    if (settings->getSettingsType() == "daycycle")
    {
        if (cb)
            cb(std::static_pointer_cast<LLSettingsDay>(settings));
    }
    else
    {
        LLSettingsVOBase::getSettingsAsset(LLSettingsDay::GetDefaultAssetId(),
            [settings, cb](LLUUID, LLSettingsBase::ptr_t pday, S32, LLExtStat){ combineIntoDayCycle(std::static_pointer_cast<LLSettingsDay>(pday), settings, cb); });
    }
}

void LLSettingsVODay::combineIntoDayCycle(LLSettingsDay::ptr_t pday, LLSettingsBase::ptr_t settings, asset_built_fn cb)
{
    if (settings->getSettingsType() == "sky")
    {
        pday->setName("sky: " + settings->getName());
        pday->clearCycleTrack(1);
        pday->setSettingsAtKeyframe(settings, 0.0, 1);
    }
    else if (settings->getSettingsType() == "water")
    {
        pday->setName("water: " + settings->getName());
        pday->clearCycleTrack(0);
        pday->setSettingsAtKeyframe(settings, 0.0, 0);
    }
    else
    {
        pday.reset();
    }

    if (cb)
        cb(pday);
}


LLSettingsDay::ptr_t LLSettingsVODay::buildClone() const
{
    LLSD settings = cloneSettings();

    LLSettingsDay::validation_list_t validations = LLSettingsDay::validationList();
    LLSD results = LLSettingsDay::settingValidation(settings, validations);
    if (!results["success"].asBoolean())
    {
        LL_WARNS("SETTINGS") << "Day setting validation failed!\n" << results << LL_ENDL;
        LLSettingsDay::ptr_t();
    }

    LLSettingsDay::ptr_t dayp = std::make_shared<LLSettingsVODay>(settings);

    U32 flags = getFlags();
    if (flags)
        dayp->setFlags(flags);

    dayp->initialize();
    return dayp;
}

LLSettingsDay::ptr_t LLSettingsVODay::buildDeepCloneAndUncompress() const
{
    // no need for SETTING_TRACKS or SETTING_FRAMES, so take base LLSD
    LLSD settings = llsd_clone(mSettings);

    LLSettingsDay::ptr_t day_clone = std::make_shared<LLSettingsVODay>(settings);

    for (S32 i = 0; i < LLSettingsDay::TRACK_MAX; ++i)
    {
        const LLSettingsDay::CycleTrack_t& track = getCycleTrackConst(i);
        LLSettingsDay::CycleTrack_t::const_iterator iter = track.begin();
        while (iter != track.end())
        {
            // 'Unpack', usually for editing
            // - frames 'share' settings multiple times
            // - settings can reuse LLSDs they were initialized from
            // We do not want for edited frame to change multiple frames in same track, so do a clone
            day_clone->setSettingsAtKeyframe(iter->second->buildDerivedClone(), iter->first, i);
            iter++;
        }
    }
    return day_clone;
}

LLSD LLSettingsVODay::convertToLegacy(const LLSettingsVODay::ptr_t &pday)
{
    CycleTrack_t &trackwater = pday->getCycleTrack(TRACK_WATER);

    LLSettingsWater::ptr_t pwater;
    if (!trackwater.empty())
    {
        pwater = std::static_pointer_cast<LLSettingsWater>((*trackwater.begin()).second);
    }

    if (!pwater)
        pwater = LLSettingsVOWater::buildDefaultWater();
    
    LLSD llsdwater = LLSettingsVOWater::convertToLegacy(pwater);
    
    CycleTrack_t &tracksky = pday->getCycleTrack(1);   // first sky track
    std::map<std::string, LLSettingsSky::ptr_t> skys;
    
    LLSD llsdcycle(LLSD::emptyArray());
    
    for(CycleTrack_t::iterator it = tracksky.begin(); it != tracksky.end(); ++it)
    {
        size_t hash = (*it).second->getHash();
        std::stringstream name;
        
        name << hash;
        
        skys[name.str()] = std::static_pointer_cast<LLSettingsSky>((*it).second);
        
        F32 frame = ((tracksky.size() == 1) && (it == tracksky.begin())) ? -1.0f : (*it).first;
        llsdcycle.append( LLSDArray(LLSD::Real(frame))(name.str()) );
    }
    //_WARNS("LAPRAS") << "Cycle created with " << llsdcycle.size() << "entries: " << llsdcycle << LL_ENDL;

    LLSD llsdskylist(LLSD::emptyMap());
    
    for (std::map<std::string, LLSettingsSky::ptr_t>::iterator its = skys.begin(); its != skys.end(); ++its)
    {
        LLSD llsdsky = LLSettingsVOSky::convertToLegacy((*its).second, false);
        llsdsky[SETTING_NAME] = (*its).first;
        
        llsdskylist[(*its).first] = llsdsky;
    }

    //_WARNS("LAPRAS") << "Sky map with " << llsdskylist.size() << " entries created: " << llsdskylist << LL_ENDL;
    
    return LLSDArray(LLSD::emptyMap())(llsdcycle)(llsdskylist)(llsdwater);
}

LLSettingsSkyPtr_t  LLSettingsVODay::getDefaultSky() const
{
    return LLSettingsVOSky::buildDefaultSky();
}

LLSettingsWaterPtr_t LLSettingsVODay::getDefaultWater() const
{
    return LLSettingsVOWater::buildDefaultWater();
}

LLSettingsSkyPtr_t LLSettingsVODay::buildSky(LLSD settings) const
{
    LLSettingsSky::ptr_t skyp = std::make_shared<LLSettingsVOSky>(settings);

    if (skyp->validate())
        return skyp;

    return LLSettingsSky::ptr_t();
}

LLSettingsWaterPtr_t LLSettingsVODay::buildWater(LLSD settings) const
{
    LLSettingsWater::ptr_t waterp = std::make_shared<LLSettingsVOWater>(settings);

    if (waterp->validate())
        return waterp;

    return LLSettingsWater::ptr_t();
}

//=========================================================================
namespace
{
    LLSD ensure_array_4(LLSD in, F32 fill)
    {
        if (in.size() >= 4)
            return in;

        LLSD out(LLSD::emptyArray());

        for (S32 idx = 0; idx < in.size(); ++idx)
        {
            out.append(in[idx]);
        }

        while (out.size() < 4)
        {
            out.append(LLSD::Real(fill));
        }
        return out;
    }

    // This is a disturbing hack
    std::string legacy_name_to_filename(const std::string &name, bool convertdash = false)
    {
        std::string fixedname(LLURI::escape(name));

        if (convertdash)
            boost::algorithm::replace_all(fixedname, "-", "%2D");

        return fixedname;
    }

    //---------------------------------------------------------------------
    LLSD read_legacy_preset_data(const std::string &name, const std::string& path)
    {
        llifstream xml_file;

        std::string full_path(path);
        std::string full_name(name);
        full_name += ".xml";
        gDirUtilp->append(full_path, full_name);

        xml_file.open(full_path.c_str());
        if (!xml_file)
        {
            std::string bad_path(full_path);
            full_path = path;
            full_name = legacy_name_to_filename(name);
            full_name += ".xml";
            gDirUtilp->append(full_path, full_name);

            LL_INFOS("LEGACYSETTING") << "Could not open \"" << bad_path << "\" trying escaped \"" << full_path << "\"" << LL_ENDL;

            xml_file.open(full_path.c_str());
            if (!xml_file)
            {
                LL_WARNS("LEGACYSETTING") << "Unable to open legacy windlight \"" << name << "\" from " << path << LL_ENDL;

                full_path = path;
                full_name = legacy_name_to_filename(name, true);
                full_name += ".xml";
                gDirUtilp->append(full_path, full_name);
                xml_file.open(full_path.c_str());
                if (!xml_file)
                {
                    LL_WARNS("LEGACYSETTING") << "Unable to open legacy windlight \"" << name << "\" from " << path << LL_ENDL;
                    return LLSD();
                }
            }
        }

        LLSD params_data;
        LLPointer<LLSDParser> parser = new LLSDXMLParser();
        parser->parse(xml_file, params_data, LLSDSerialize::SIZE_UNLIMITED);
        xml_file.close();

        return params_data;
    }
}
