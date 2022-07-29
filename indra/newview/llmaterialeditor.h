/** 
 * @file llmaterialeditor.h
 * @brief LLMaterialEditor class header file
 *
 * $LicenseInfo:firstyear=2022&license=viewerlgpl$
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

#pragma once

#include "llpreview.h"
#include "llvoinventorylistener.h"
#include "llimagej2c.h"
#include "llviewertexture.h"

class LLTextureCtrl;

namespace tinygltf
{
    class Model;
}

class LLGLTFMaterial;

class LLMaterialEditor : public LLPreview, public LLVOInventoryListener
{
public:
	LLMaterialEditor(const LLSD& key);

    bool setFromGltfModel(tinygltf::Model& model, bool set_textures = false);

    // open a file dialog and select a gltf/glb file for import
    void importMaterial();

    // for live preview, apply current material to currently selected object
    void applyToSelection();

    void getGLTFMaterial(LLGLTFMaterial* mat);

    void setFromGLTFMaterial(LLGLTFMaterial* mat);

    void loadAsset();

    static void onLoadComplete(const LLUUID& asset_uuid, LLAssetType::EType type, void* user_data, S32 status, LLExtStat ext_status);

    void inventoryChanged(LLViewerObject* object, LLInventoryObject::object_list_t* inventory, S32 serial_num, void* user_data) override;

    typedef std::function<void(LLUUID newAssetId, LLSD response)> upload_callback_f;
    void saveTexture(LLImageJ2C* img, const std::string& name, const LLUUID& asset_id, upload_callback_f cb);

    // save textures to inventory if needed
    // returns amount of scheduled uploads
    S32 saveTextures();

    void onClickSave();

    // get a dump of the json representation of the current state of the editor UI in GLTF format
    std::string getGLTFJson(bool prettyprint = true);

    void getGLBData(std::vector<U8>& data);

    void getGLTFModel(tinygltf::Model& model);

    std::string getEncodedAsset();

    bool decodeAsset(const std::vector<char>& buffer);

    bool saveIfNeeded();
    static bool saveToInventoryItem(const std::string &buffer, const LLUUID &item_id, const LLUUID &task_id);

    static void finishInventoryUpload(LLUUID itemId, LLUUID newAssetId, LLUUID newItemId);

    static void finishTaskUpload(LLUUID itemId, LLUUID newAssetId, LLUUID taskId);

    static void finishSaveAs(const LLUUID &oldItemId, const LLUUID &newItemId, const std::string &buffer);

    void refreshFromInventory(const LLUUID& new_item_id = LLUUID::null);

    void onClickSaveAs();
    void onSaveAsMsgCallback(const LLSD& notification, const LLSD& response);
    void onClickCancel();
    void onCancelMsgCallback(const LLSD& notification, const LLSD& response);

	// llpanel
	BOOL postBuild() override;
    void onClickCloseBtn(bool app_quitting = false) override;

    LLUUID getAlbedoId();
    void setAlbedoId(const LLUUID& id);
    void setAlbedoUploadId(const LLUUID& id);

    LLColor4 getAlbedoColor();

    // sets both albedo color and transparency
    void    setAlbedoColor(const LLColor4& color);

    F32     getTransparency();
    void     setTransparency(F32 transparency);

    std::string getAlphaMode();
    void setAlphaMode(const std::string& alpha_mode);

    F32 getAlphaCutoff();
    void setAlphaCutoff(F32 alpha_cutoff);
    
    void setMaterialName(const std::string &name);

    LLUUID getMetallicRoughnessId();
    void setMetallicRoughnessId(const LLUUID& id);
    void setMetallicRoughnessUploadId(const LLUUID& id);

    F32 getMetalnessFactor();
    void setMetalnessFactor(F32 factor);

    F32 getRoughnessFactor();
    void setRoughnessFactor(F32 factor);

    LLUUID getEmissiveId();
    void setEmissiveId(const LLUUID& id);
    void setEmissiveUploadId(const LLUUID& id);

    LLColor4 getEmissiveColor();
    void setEmissiveColor(const LLColor4& color);

    LLUUID getNormalId();
    void setNormalId(const LLUUID& id);
    void setNormalUploadId(const LLUUID& id);

    bool getDoubleSided();
    void setDoubleSided(bool double_sided);

    void setHasUnsavedChanges(bool value);
    void setCanSaveAs(BOOL value);
    void setCanSave(BOOL value);

    void onCommitAlbedoTexture(LLUICtrl* ctrl, const LLSD& data);
    void onCommitMetallicTexture(LLUICtrl* ctrl, const LLSD& data);
    void onCommitEmissiveTexture(LLUICtrl* ctrl, const LLSD& data);
    void onCommitNormalTexture(LLUICtrl* ctrl, const LLSD& data);

private:
    friend class LLMaterialFilePicker;

    LLUUID mAssetID;
    LLUUID mObjectID;

    LLTextureCtrl* mAlbedoTextureCtrl;
    LLTextureCtrl* mMetallicTextureCtrl;
    LLTextureCtrl* mEmissiveTextureCtrl;
    LLTextureCtrl* mNormalTextureCtrl;

    // 'Default' texture, unless it's null or from inventory is the one with the fee
    LLUUID mAlbedoTextureUploadId;
    LLUUID mMetallicTextureUploadId;
    LLUUID mEmissiveTextureUploadId;
    LLUUID mNormalTextureUploadId;

    // last known name of each texture
    std::string mAlbedoName;
    std::string mNormalName;
    std::string mMetallicRoughnessName;
    std::string mEmissiveName;

    // keep pointers to fetched textures or viewer will remove them
    // if user temporary selects something else with 'apply now'
    LLPointer<LLViewerFetchedTexture> mAlbedoFetched;
    LLPointer<LLViewerFetchedTexture> mNormalFetched;
    LLPointer<LLViewerFetchedTexture> mMetallicRoughnessFetched;
    LLPointer<LLViewerFetchedTexture> mEmissiveFetched;

    // J2C versions of packed buffers for uploading
    LLPointer<LLImageJ2C> mAlbedoJ2C;
    LLPointer<LLImageJ2C> mNormalJ2C;
    LLPointer<LLImageJ2C> mMetallicRoughnessJ2C;
    LLPointer<LLImageJ2C> mEmissiveJ2C;

    bool mHasUnsavedChanges;
    S32 mUploadingTexturesCount;
    S32 mExpectedUploadCost;
    std::string mMaterialName;
};

