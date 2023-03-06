/** 
 * @file llfloaterchangeitemthumbnail.cpp
 * @brief LLFloaterChangeItemThumbnail class implementation
 *
 * $LicenseInfo:firstyear=2023&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2023, Linden Research, Inc.
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

#include "llfloaterchangeitemthumbnail.h"

#include "llbutton.h"
#include "llclipboard.h"
#include "lliconctrl.h"
#include "llinventoryfunctions.h"
#include "llinventoryicon.h"
#include "llinventorymodel.h"
#include "llinventoryobserver.h"
#include "llfloaterreg.h"
#include "llfloatersimplesnapshot.h"
#include "lllineeditor.h"
#include "llnotificationsutil.h"
#include "lltextbox.h"
#include "lltexturectrl.h"
#include "llthumbnailctrl.h"
#include "llviewerfoldertype.h"
#include "llviewermenufile.h"
#include "llviewerobjectlist.h"
#include "llwindow.h"
#include "lltrans.h"


class LLThumbnailImagePicker : public LLFilePickerThread
{
public:
    LLThumbnailImagePicker(const LLUUID &item_id);
    LLThumbnailImagePicker(const LLUUID &item_id, const LLUUID &task_id);
    ~LLThumbnailImagePicker();
    void notify(const std::vector<std::string>& filenames) override;

private:
    LLUUID mInventoryId;
    LLUUID mTaskId;
};

LLThumbnailImagePicker::LLThumbnailImagePicker(const LLUUID &item_id)
    : LLFilePickerThread(LLFilePicker::FFLOAD_IMAGE)
    , mInventoryId(item_id)
{
}

LLThumbnailImagePicker::LLThumbnailImagePicker(const LLUUID &item_id, const LLUUID &task_id)
    : LLFilePickerThread(LLFilePicker::FFLOAD_IMAGE)
    , mInventoryId(item_id)
    , mTaskId(task_id)
{
}

LLThumbnailImagePicker::~LLThumbnailImagePicker()
{
}

void LLThumbnailImagePicker::notify(const std::vector<std::string>& filenames)
{
    if (filenames.empty())
    {
        return;
    }
    std::string file_path = filenames[0];
    if (file_path.empty())
    {
        return;
    }
    
    LLFloaterSimpleSnapshot::uploadThumbnail(file_path, mInventoryId, mTaskId);
}

LLFloaterChangeItemThumbnail::LLFloaterChangeItemThumbnail(const LLSD& key)
    : LLFloater(key)
    , mObserverInitialized(false)
    , mTooltipState(TOOLTIP_NONE)
{
}

LLFloaterChangeItemThumbnail::~LLFloaterChangeItemThumbnail()
{
    gInventory.removeObserver(this);
    removeVOInventoryListener();
}

BOOL LLFloaterChangeItemThumbnail::postBuild()
{
    mItemNameText = getChild<LLUICtrl>("item_name");
    mItemTypeIcon = getChild<LLIconCtrl>("item_type_icon");
    mThumbnailCtrl = getChild<LLThumbnailCtrl>("item_thumbnail");
    mToolTipTextBox = getChild<LLTextBox>("tooltip_text");

    LLSD tooltip_text;
    mToolTipTextBox->setValue(tooltip_text);

    LLButton *upload_local = getChild<LLButton>("upload_local");
    upload_local->setClickedCallback(onUploadLocal, (void*)this);
    upload_local->setMouseEnterCallback(boost::bind(&LLFloaterChangeItemThumbnail::onButtonMouseEnter, this, _1, _2, TOOLTIP_UPLOAD_LOCAL));
    upload_local->setMouseLeaveCallback(boost::bind(&LLFloaterChangeItemThumbnail::onButtonMouseLeave, this, _1, _2, TOOLTIP_UPLOAD_LOCAL));

    LLButton *upload_snapshot = getChild<LLButton>("upload_snapshot");
    upload_snapshot->setClickedCallback(onUploadSnapshot, (void*)this);
    upload_snapshot->setMouseEnterCallback(boost::bind(&LLFloaterChangeItemThumbnail::onButtonMouseEnter, this, _1, _2, TOOLTIP_UPLOAD_SNAPSHOT));
    upload_snapshot->setMouseLeaveCallback(boost::bind(&LLFloaterChangeItemThumbnail::onButtonMouseLeave, this, _1, _2, TOOLTIP_UPLOAD_SNAPSHOT));

    LLButton *use_texture = getChild<LLButton>("use_texture");
    use_texture->setClickedCallback(onUseTexture, (void*)this);
    use_texture->setMouseEnterCallback(boost::bind(&LLFloaterChangeItemThumbnail::onButtonMouseEnter, this, _1, _2, TOOLTIP_USE_TEXTURE));
    use_texture->setMouseLeaveCallback(boost::bind(&LLFloaterChangeItemThumbnail::onButtonMouseLeave, this, _1, _2, TOOLTIP_USE_TEXTURE));

    mCopyToClipboardBtn = getChild<LLButton>("copy_to_clipboard");
    mCopyToClipboardBtn->setClickedCallback(onCopyToClipboard, (void*)this);
    mCopyToClipboardBtn->setMouseEnterCallback(boost::bind(&LLFloaterChangeItemThumbnail::onButtonMouseEnter, this, _1, _2, TOOLTIP_COPY_TO_CLIPBOARD));
    mCopyToClipboardBtn->setMouseLeaveCallback(boost::bind(&LLFloaterChangeItemThumbnail::onButtonMouseLeave, this, _1, _2, TOOLTIP_COPY_TO_CLIPBOARD));

    mPasteFromClipboardBtn = getChild<LLButton>("paste_from_clipboard");
    mPasteFromClipboardBtn->setClickedCallback(onPasteFromClipboard, (void*)this);
    mPasteFromClipboardBtn->setMouseEnterCallback(boost::bind(&LLFloaterChangeItemThumbnail::onButtonMouseEnter, this, _1, _2, TOOLTIP_COPY_FROM_CLIPBOARD));
    mPasteFromClipboardBtn->setMouseLeaveCallback(boost::bind(&LLFloaterChangeItemThumbnail::onButtonMouseLeave, this, _1, _2, TOOLTIP_COPY_FROM_CLIPBOARD));

    mRemoveImageBtn = getChild<LLButton>("remove_image");
    mRemoveImageBtn->setClickedCallback(onRemove, (void*)this);
    mRemoveImageBtn->setMouseEnterCallback(boost::bind(&LLFloaterChangeItemThumbnail::onButtonMouseEnter, this, _1, _2, TOOLTIP_REMOVE));
    mRemoveImageBtn->setMouseLeaveCallback(boost::bind(&LLFloaterChangeItemThumbnail::onButtonMouseLeave, this, _1, _2, TOOLTIP_REMOVE));

    return LLFloater::postBuild();
}

void LLFloaterChangeItemThumbnail::onOpen(const LLSD& key)
{
    if (!key.has("item_id") && !key.isUUID())
    {
        closeFloater();
    }

    if (key.isUUID())
    {
        mItemId = key.asUUID();
    }
    else
    {
        mItemId = key["item_id"].asUUID();
        mTaskId = key["task_id"].asUUID();
    }

    refreshFromInventory();
}

void LLFloaterChangeItemThumbnail::changed(U32 mask)
{
    //LLInventoryObserver

    if (mTaskId.notNull() || mItemId.isNull())
    {
        // Task inventory or not set up yet
        return;
    }

    const std::set<LLUUID>& mChangedItemIDs = gInventory.getChangedIDs();
    std::set<LLUUID>::const_iterator it;

    for (it = mChangedItemIDs.begin(); it != mChangedItemIDs.end(); it++)
    {
        // set dirty for 'item profile panel' only if changed item is the item for which 'item profile panel' is shown (STORM-288)
        if (*it == mItemId)
        {
            // if there's a change we're interested in.
            if ((mask & (LLInventoryObserver::LABEL | LLInventoryObserver::INTERNAL | LLInventoryObserver::REMOVE)) != 0)
            {
                refreshFromInventory();
            }
        }
    }
}

void LLFloaterChangeItemThumbnail::inventoryChanged(LLViewerObject* object,
    LLInventoryObject::object_list_t* inventory,
    S32 serial_num,
    void* user_data)
{
    //LLVOInventoryListener
    refreshFromInventory();
}

LLInventoryObject* LLFloaterChangeItemThumbnail::getInventoryObject()
{
    LLInventoryObject* obj = NULL;
    if (mTaskId.isNull())
    {
        // it is in agent inventory
        if (!mObserverInitialized)
        {
            gInventory.addObserver(this);
            mObserverInitialized = true;
        }

        obj = gInventory.getObject(mItemId);
    }
    else
    {
        LLViewerObject* object = gObjectList.findObject(mTaskId);
        if (object)
        {
            if (!mObserverInitialized)
            {
                registerVOInventoryListener(object, NULL);
                mObserverInitialized = false;
            }

            obj = object->getInventoryObject(mItemId);
        }
    }
    return obj;
}

void LLFloaterChangeItemThumbnail::refreshFromInventory()
{
    LLInventoryObject* obj = getInventoryObject();
    if (!obj)
    {
        closeFloater();
    }

    if (obj)
    {
        const LLUUID trash_id = gInventory.findCategoryUUIDForType(LLFolderType::FT_TRASH);
        bool in_trash = (obj->getUUID() == trash_id) || gInventory.isObjectDescendentOf(obj->getUUID(), trash_id);
        if (in_trash)
        {
            // Close properties when moving to trash
            // Aren't supposed to view properties from trash
            closeFloater();
        }
        else
        {
            refreshFromObject(obj);
        }
    }
    else
    {
        closeFloater();
    }
}

class LLIsOutfitTextureType : public LLInventoryCollectFunctor
{
public:
    LLIsOutfitTextureType() {}
    virtual ~LLIsOutfitTextureType() {}
    virtual bool operator()(LLInventoryCategory* cat,
        LLInventoryItem* item);
};

bool LLIsOutfitTextureType::operator()(LLInventoryCategory* cat, LLInventoryItem* item)
{
    return item && (item->getType() == LLAssetType::AT_TEXTURE);
}

void LLFloaterChangeItemThumbnail::refreshFromObject(LLInventoryObject* obj)
{
    LLUIImagePtr icon_img;
    LLUUID thumbnail_id = obj->getThumbnailUUID();

    LLViewerInventoryItem* item = dynamic_cast<LLViewerInventoryItem*>(obj);
    if (item)
    {
        // This floater probably shouldn't be be possible to open
        // for imcomplete items
        llassert(item->isFinished());

        icon_img = LLInventoryIcon::getIcon(item->getType(), item->getInventoryType(), item->getFlags(), FALSE);
        mRemoveImageBtn->setEnabled(thumbnail_id.notNull() && ((item->getActualType() != LLAssetType::AT_TEXTURE) || (item->getAssetUUID() != thumbnail_id)));
    }
    else
    {
        LLViewerInventoryCategory* cat = dynamic_cast<LLViewerInventoryCategory*>(obj);

        if (cat)
        {
            icon_img = LLUI::getUIImage(LLViewerFolderType::lookupIconName(cat->getPreferredType(), true));

            if (thumbnail_id.isNull() && (cat->getPreferredType() == LLFolderType::FT_OUTFIT))
            {
                // Legacy support, check if there is an image inside

                LLInventoryModel::cat_array_t cats;
                LLInventoryModel::item_array_t items;
                // Not LLIsOfAssetType, because we allow links
                LLIsOutfitTextureType f;
                gInventory.getDirectDescendentsOf(mItemId, cats, items, f);

                if (1 == items.size())
                {
                    LLViewerInventoryItem* item = items.front();
                    if (item && item->getIsLinkType())
                    {
                        item = item->getLinkedItem();
                    }
                    if (item)
                    {
                        LL_INFOS() << "Setting image from outfit as a thumbnail" << LL_ENDL;
                        thumbnail_id = item->getAssetUUID();

                        if (validateAsset(thumbnail_id))
                        {
                            // per SL-19188, set this image as a thumbnail
                            setThumbnailId(thumbnail_id);
                        }
                    }
                }
            }

            mRemoveImageBtn->setEnabled(thumbnail_id.notNull());
        }
    }
    mItemTypeIcon->setImage(icon_img);
    mItemNameText->setValue(obj->getName());

    mThumbnailCtrl->setValue(thumbnail_id);

    mCopyToClipboardBtn->setEnabled(thumbnail_id.notNull());

    // todo: some elements might not support setting thumbnails
    // since they already have them
}

void LLFloaterChangeItemThumbnail::onUploadLocal(void *userdata)
{
    LLFloaterChangeItemThumbnail *self = (LLFloaterChangeItemThumbnail*)userdata;

    (new LLThumbnailImagePicker(self->mItemId, self->mTaskId))->getFile();

    LLFloater* floaterp = self->mPickerHandle.get();
    if (floaterp)
    {
        floaterp->closeFloater();
    }
    floaterp = self->mSnapshotHandle.get();
    if (floaterp)
    {
        floaterp->closeFloater();
    }
}

void LLFloaterChangeItemThumbnail::onUploadSnapshot(void *userdata)
{
    LLFloaterChangeItemThumbnail *self = (LLFloaterChangeItemThumbnail*)userdata;

    LLFloater* floaterp = self->mSnapshotHandle.get();
    // Show the dialog
    if (floaterp)
    {
        floaterp->openFloater();
    }
    else
    {
        LLSD key;
        key["item_id"] = self->mItemId;
        key["task_id"] = self->mTaskId;
        LLFloaterSimpleSnapshot* snapshot_floater = (LLFloaterSimpleSnapshot*)LLFloaterReg::showInstance("simple_snapshot", key, true);
        if (snapshot_floater)
        {
            self->addDependentFloater(snapshot_floater);
            self->mSnapshotHandle = snapshot_floater->getHandle();
            snapshot_floater->setOwner(self);
        }
    }

    floaterp = self->mPickerHandle.get();
    if (floaterp)
    {
        floaterp->closeFloater();
    }
}

void LLFloaterChangeItemThumbnail::onUseTexture(void *userdata)
{
    LLFloaterChangeItemThumbnail *self = (LLFloaterChangeItemThumbnail*)userdata;
    LLInventoryObject* obj = self->getInventoryObject();
    if (obj)
    {
        self->showTexturePicker(obj->getThumbnailUUID());
    }

    LLFloater* floaterp = self->mSnapshotHandle.get();
    if (floaterp)
    {
        floaterp->closeFloater();
    }
}

void LLFloaterChangeItemThumbnail::onCopyToClipboard(void *userdata)
{
    LLFloaterChangeItemThumbnail *self = (LLFloaterChangeItemThumbnail*)userdata;
    LLInventoryObject* obj = self->getInventoryObject();
    if (obj)
    {
        LLClipboard::instance().addToClipboard(obj->getThumbnailUUID());
    }
}

void LLFloaterChangeItemThumbnail::onPasteFromClipboard(void *userdata)
{
    LLFloaterChangeItemThumbnail *self = (LLFloaterChangeItemThumbnail*)userdata;
    std::vector<LLUUID> objects;
    LLClipboard::instance().pasteFromClipboard(objects);
    if (objects.size() > 0)
    {
        LLUUID asset_id = objects[0];
        if (validateAsset(asset_id))
        {
            self->setThumbnailId(asset_id);
        }
        else
        {
            LLNotificationsUtil::add("ThumbnailDimantionsLimit");
        }
    }
}

void LLFloaterChangeItemThumbnail::onRemove(void *userdata)
{
    LLFloaterChangeItemThumbnail *self = (LLFloaterChangeItemThumbnail*)userdata;

    LLSD payload;
    payload["item_id"] = self->mItemId;
    payload["object_id"] = self->mTaskId;
    LLNotificationsUtil::add("DeleteThumbnail", LLSD(), payload, boost::bind(&LLFloaterChangeItemThumbnail::onRemovalConfirmation, _1, _2, self->getHandle()));
}

// static 
void LLFloaterChangeItemThumbnail::onRemovalConfirmation(const LLSD& notification, const LLSD& response, LLHandle<LLFloater> handle)
{
    S32 option = LLNotificationsUtil::getSelectedOption(notification, response);
    if (option == 0 && !handle.isDead() && !handle.get()->isDead())
    {
        LLFloaterChangeItemThumbnail* self = (LLFloaterChangeItemThumbnail*)handle.get();
        self->setThumbnailId(LLUUID::null);
    }
}

bool LLFloaterChangeItemThumbnail::validateAsset(const LLUUID &asset_id)
{
    LLPointer<LLViewerFetchedTexture> texturep = LLViewerTextureManager::getFetchedTexture(asset_id);

    if (!texturep)
    {
        return false;
    }

    if (texturep->getFullWidth() != texturep->getFullHeight())
    {
        return false;
    }

    if (texturep->getFullWidth() > LLFloaterSimpleSnapshot::THUMBNAIL_SNAPSHOT_DIM_MAX
        || texturep->getFullHeight() > LLFloaterSimpleSnapshot::THUMBNAIL_SNAPSHOT_DIM_MAX)
    {
        return false;
    }

    if (texturep->getFullWidth() < LLFloaterSimpleSnapshot::THUMBNAIL_SNAPSHOT_DIM_MIN
        || texturep->getFullHeight() < LLFloaterSimpleSnapshot::THUMBNAIL_SNAPSHOT_DIM_MIN)
    {
        return false;
    }
    return true;
}

void LLFloaterChangeItemThumbnail::showTexturePicker(const LLUUID &thumbnail_id)
{
    // show hourglass cursor when loading inventory window
    getWindow()->setCursor(UI_CURSOR_WAIT);

    LLFloater* floaterp = mPickerHandle.get();
    // Show the dialog
    if (floaterp)
    {
        floaterp->openFloater();
    }
    else
    {
        floaterp = new LLFloaterTexturePicker(
            this,
            thumbnail_id,
            thumbnail_id,
            thumbnail_id,
            FALSE,
            TRUE,
            LLTrans::getString("TexturePickerOutfitHeader"), // "SELECT PHOTO", // <FS:Ansariel> Localizable floater header
            PERM_NONE,
            PERM_NONE,
            PERM_NONE,
            FALSE,
            NULL);

        mPickerHandle = floaterp->getHandle();

        LLFloaterTexturePicker* texture_floaterp = dynamic_cast<LLFloaterTexturePicker*>(floaterp);
        if (texture_floaterp)
        {
            //texture_floaterp->setTextureSelectedCallback();
            //texture_floaterp->setOnUpdateImageStatsCallback();
            texture_floaterp->setOnFloaterCommitCallback([this](LLTextureCtrl::ETexturePickOp op, LLUUID id)
            {
                if (op == LLTextureCtrl::TEXTURE_SELECT)
                {
                    onTexturePickerCommit(id);
                }
            }
            );

            texture_floaterp->setLocalTextureEnabled(FALSE);
            texture_floaterp->setBakeTextureEnabled(FALSE);
            texture_floaterp->setCanApplyImmediately(false);
            texture_floaterp->setCanApply(false, true);

            addDependentFloater(texture_floaterp);
        }

        floaterp->openFloater();
    }
    floaterp->setFocus(TRUE);
}

void LLFloaterChangeItemThumbnail::onTexturePickerCommit(LLUUID id)
{
    LLFloaterTexturePicker* floaterp = (LLFloaterTexturePicker*)mPickerHandle.get();

    if (floaterp)
    {
        LLUUID asset_id = floaterp->getAssetID();
        if (validateAsset(asset_id))
        {
            setThumbnailId(asset_id);
        }
        else
        {
            LLNotificationsUtil::add("ThumbnailDimantionsLimit");
        }
    }
}


void LLFloaterChangeItemThumbnail::setThumbnailId(const LLUUID &new_thumbnail_id)
{
    LLInventoryObject* obj = getInventoryObject();
    if (!obj)
    {
        return;
    }

    if (mTaskId.notNull())
    {
        LL_ERRS() << "Not implemented yet" << LL_ENDL;
    }
    else if (obj->getThumbnailUUID() != new_thumbnail_id)
    {
        LLSD updates;
        updates["thumbnail"] = LLSD().with("asset_id", new_thumbnail_id);
        LLViewerInventoryCategory* view_folder = dynamic_cast<LLViewerInventoryCategory*>(obj);
        if (view_folder)
        {
            update_inventory_category(mItemId, updates, NULL);
        }
        LLViewerInventoryItem* view_item = dynamic_cast<LLViewerInventoryItem*>(obj);
        if (view_item)
        {
            update_inventory_item(mItemId, updates, NULL);
        }
    }
}

void LLFloaterChangeItemThumbnail::onButtonMouseEnter(LLUICtrl* button, const LLSD& param, EToolTipState state)
{
    mTooltipState = state;

    std::string tooltip_text;
    std::string tooltip_name = "tooltip_" + button->getName();
    if (hasString(tooltip_name))
    {
        tooltip_text = getString(tooltip_name);
    }

    mToolTipTextBox->setValue(tooltip_text);
}

void LLFloaterChangeItemThumbnail::onButtonMouseLeave(LLUICtrl* button, const LLSD& param, EToolTipState state)
{
    if (mTooltipState == state)
    {
        mTooltipState = TOOLTIP_NONE;
        LLSD tooltip_text;
        mToolTipTextBox->setValue(tooltip_text);
    }
}

