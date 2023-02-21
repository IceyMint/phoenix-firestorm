/** 
* @file llfloatersimplesnapshot.cpp
* @brief Snapshot preview window for saving as a thumbnail
*
* $LicenseInfo:firstyear=2022&license=viewerlgpl$
* Second Life Viewer Source Code
* Copyright (C) 2022, Linden Research, Inc.
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

#include "llfloatersimplesnapshot.h"

#include "llfloaterreg.h"
#include "llimagefiltersmanager.h"
#include "llinventorymodel.h"
#include "llstatusbar.h" // can_afford_transaction()
#include "llnotificationsutil.h"
#include "llagent.h"
#include "llagentbenefits.h"
#include "llviewercontrol.h"
#include "llviewertexturelist.h"



LLSimpleSnapshotFloaterView* gSimpleSnapshotFloaterView = NULL;

const S32 THUMBNAIL_SNAPSHOT_DIM = 256;

// Thumbnail posting coro

static const std::string THUMBNAIL_ITEM_UPLOAD_CAP = "InventoryItemThumbnailUpload";
static const std::string THUMBNAIL_CATEGORY_UPLOAD_CAP = "InventoryCategoryThumbnailUpload";

void post_thumbnail_image_coro(std::string cap_url, std::string path_to_image, LLSD first_data)
{
    LLCore::HttpRequest::policy_t httpPolicy(LLCore::HttpRequest::DEFAULT_POLICY_ID);
    LLCoreHttpUtil::HttpCoroutineAdapter::ptr_t
        httpAdapter(new LLCoreHttpUtil::HttpCoroutineAdapter("post_profile_image_coro", httpPolicy));
    LLCore::HttpRequest::ptr_t httpRequest(new LLCore::HttpRequest);
    LLCore::HttpHeaders::ptr_t httpHeaders;

    LLCore::HttpOptions::ptr_t httpOpts(new LLCore::HttpOptions);
    httpOpts->setFollowRedirects(true);

    LLSD result = httpAdapter->postAndSuspend(httpRequest, cap_url, first_data, httpOpts, httpHeaders);

    LLSD httpResults = result[LLCoreHttpUtil::HttpCoroutineAdapter::HTTP_RESULTS];
    LLCore::HttpStatus status = LLCoreHttpUtil::HttpCoroutineAdapter::getStatusFromLLSD(httpResults);

    if (!status)
    {
        // todo: notification?
        LL_WARNS("AvatarProperties") << "Failed to get uploader cap " << status.toString() << LL_ENDL;
        return;
    }
    if (!result.has("uploader"))
    {
        // todo: notification?
        LL_WARNS("AvatarProperties") << "Failed to get uploader cap, response contains no data." << LL_ENDL;
        return;
    }
    std::string uploader_cap = result["uploader"].asString();
    if (uploader_cap.empty())
    {
        LL_WARNS("AvatarProperties") << "Failed to get uploader cap, cap invalid." << LL_ENDL;
        return;
    }

    // Upload the image

    LLCore::HttpRequest::ptr_t uploaderhttpRequest(new LLCore::HttpRequest);
    LLCore::HttpHeaders::ptr_t uploaderhttpHeaders(new LLCore::HttpHeaders);
    LLCore::HttpOptions::ptr_t uploaderhttpOpts(new LLCore::HttpOptions);
    S64 length;

    {
        llifstream instream(path_to_image.c_str(), std::iostream::binary | std::iostream::ate);
        if (!instream.is_open())
        {
            LL_WARNS("AvatarProperties") << "Failed to open file " << path_to_image << LL_ENDL;
            return;
        }
        length = instream.tellg();
    }

    uploaderhttpHeaders->append(HTTP_OUT_HEADER_CONTENT_TYPE, "application/jp2"); // optional
    uploaderhttpHeaders->append(HTTP_OUT_HEADER_CONTENT_LENGTH, llformat("%d", length)); // required!
    uploaderhttpOpts->setFollowRedirects(true);

    result = httpAdapter->postFileAndSuspend(uploaderhttpRequest, uploader_cap, path_to_image, uploaderhttpOpts, uploaderhttpHeaders);

    httpResults = result[LLCoreHttpUtil::HttpCoroutineAdapter::HTTP_RESULTS];
    status = LLCoreHttpUtil::HttpCoroutineAdapter::getStatusFromLLSD(httpResults);

    LL_DEBUGS("Thumbnail") << result << LL_ENDL;

    if (!status)
    {
        LL_WARNS("Thumbnail") << "Failed to upload image " << status.toString() << LL_ENDL;
        return;
    }

    if (result["state"].asString() != "complete")
    {
        if (result.has("message"))
        {
            LL_WARNS("Thumbnail") << "Failed to upload image, state " << result["state"] << " message: " << result["message"] << LL_ENDL;
        }
        else
        {
            LL_WARNS("Thumbnail") << "Failed to upload image " << result << LL_ENDL;
        }
        return;
    }

    // todo: issue an inventory udpate?
    //return result["new_asset"].asUUID();
}

///----------------------------------------------------------------------------
/// Class LLFloaterSimpleSnapshot::Impl
///----------------------------------------------------------------------------

LLSnapshotModel::ESnapshotFormat LLFloaterSimpleSnapshot::Impl::getImageFormat(LLFloaterSnapshotBase* floater)
{
    return LLSnapshotModel::SNAPSHOT_FORMAT_PNG;
}

LLSnapshotModel::ESnapshotLayerType LLFloaterSimpleSnapshot::Impl::getLayerType(LLFloaterSnapshotBase* floater)
{
    return LLSnapshotModel::SNAPSHOT_TYPE_COLOR;
}

void LLFloaterSimpleSnapshot::Impl::updateControls(LLFloaterSnapshotBase* floater)
{
    LLSnapshotLivePreview* previewp = getPreviewView();
    updateResolution(floater);
    if (previewp)
    {
        previewp->setSnapshotType(LLSnapshotModel::ESnapshotType::SNAPSHOT_TEXTURE);
        previewp->setSnapshotFormat(LLSnapshotModel::ESnapshotFormat::SNAPSHOT_FORMAT_PNG);
        previewp->setSnapshotBufferType(LLSnapshotModel::ESnapshotLayerType::SNAPSHOT_TYPE_COLOR);
    }
}

std::string LLFloaterSimpleSnapshot::Impl::getSnapshotPanelPrefix()
{
    return "panel_outfit_snapshot_";
}

void LLFloaterSimpleSnapshot::Impl::updateResolution(void* data)
{
    LLFloaterSimpleSnapshot *view = (LLFloaterSimpleSnapshot *)data;

    if (!view)
    {
        llassert(view);
        return;
    }

    S32 width = THUMBNAIL_SNAPSHOT_DIM;
    S32 height = THUMBNAIL_SNAPSHOT_DIM;

    LLSnapshotLivePreview* previewp = getPreviewView();
    if (previewp)
    {
        S32 original_width = 0, original_height = 0;
        previewp->getSize(original_width, original_height);

        if (gSavedSettings.getBOOL("RenderHUDInSnapshot"))
        { //clamp snapshot resolution to window size when showing UI HUD in snapshot
            width = llmin(width, gViewerWindow->getWindowWidthRaw());
            height = llmin(height, gViewerWindow->getWindowHeightRaw());
        }

        llassert(width > 0 && height > 0);

        previewp->setSize(width, height);

        if (original_width != width || original_height != height)
        {
            // hide old preview as the aspect ratio could be wrong
            checkAutoSnapshot(previewp, FALSE);
            previewp->updateSnapshot(TRUE);
        }
    }
}

void LLFloaterSimpleSnapshot::Impl::setStatus(EStatus status, bool ok, const std::string& msg)
{
    switch (status)
    {
    case STATUS_READY:
        mFloater->setCtrlsEnabled(true);
        break;
    case STATUS_WORKING:
        mFloater->setCtrlsEnabled(false);
        break;
    case STATUS_FINISHED:
        mFloater->setCtrlsEnabled(true);
        break;
    }

    mStatus = status;
}

///----------------------------------------------------------------re------------
/// Class LLFloaterSimpleSnapshot
///----------------------------------------------------------------------------

LLFloaterSimpleSnapshot::LLFloaterSimpleSnapshot(const LLSD& key)
    : LLFloaterSnapshotBase(key)
{
    impl = new Impl(this);
}

LLFloaterSimpleSnapshot::~LLFloaterSimpleSnapshot()
{
}

BOOL LLFloaterSimpleSnapshot::postBuild()
{
    childSetAction("new_snapshot_btn", ImplBase::onClickNewSnapshot, this);
    childSetAction("save_btn", boost::bind(&LLFloaterSimpleSnapshot::onSend, this));
    childSetAction("cancel_btn", boost::bind(&LLFloaterSimpleSnapshot::onCancel, this));

    mThumbnailPlaceholder = getChild<LLUICtrl>("thumbnail_placeholder");

    // create preview window
    LLRect full_screen_rect = getRootView()->getRect();
    LLSnapshotLivePreview::Params p;
    p.rect(full_screen_rect);
    LLSnapshotLivePreview* previewp = new LLSnapshotLivePreview(p);
    LLView* parent_view = gSnapshotFloaterView->getParent();

    parent_view->addChild(previewp);

    //move snapshot floater to special purpose snapshotfloaterview
    gFloaterView->removeChild(this);
    gSnapshotFloaterView->addChild(this);

    impl->mPreviewHandle = previewp->getHandle();
    previewp->setContainer(this);
    impl->updateControls(this);
    impl->setAdvanced(true);
    impl->setSkipReshaping(true);

    previewp->mKeepAspectRatio = FALSE;
    previewp->setThumbnailPlaceholderRect(getThumbnailPlaceholderRect());
    previewp->setAllowRenderUI(false);
    previewp->setThumbnailSubsampled(TRUE);

    return TRUE;
}

const S32 PREVIEW_OFFSET_Y = 70;

void LLFloaterSimpleSnapshot::draw()
{
    LLSnapshotLivePreview* previewp = getPreviewView();

    if (previewp && (previewp->isSnapshotActive() || previewp->getThumbnailLock()))
    {
        // don't render snapshot window in snapshot, even if "show ui" is turned on
        return;
    }

    LLFloater::draw();

    if (previewp && !isMinimized() && mThumbnailPlaceholder->getVisible())
    {		
        if(previewp->getThumbnailImage())
        {
            bool working = impl->getStatus() == ImplBase::STATUS_WORKING;
            const S32 thumbnail_w = previewp->getThumbnailWidth();
            const S32 thumbnail_h = previewp->getThumbnailHeight();

            LLRect local_rect = getLocalRect();
            S32 offset_x = (local_rect.getWidth() - thumbnail_w) / 2;
            S32 offset_y = PREVIEW_OFFSET_Y;

            gGL.matrixMode(LLRender::MM_MODELVIEW);
            // Apply floater transparency to the texture unless the floater is focused.
            F32 alpha = getTransparencyType() == TT_ACTIVE ? 1.0f : getCurrentTransparency();
            LLColor4 color = working ? LLColor4::grey4 : LLColor4::white;
            gl_draw_scaled_image(offset_x, offset_y, 
                thumbnail_w, thumbnail_h,
                previewp->getThumbnailImage(), color % alpha);
        }
    }
    impl->updateLayout(this);
}

void LLFloaterSimpleSnapshot::onOpen(const LLSD& key)
{
    LLSnapshotLivePreview* preview = getPreviewView();
    if (preview)
    {
        preview->updateSnapshot(TRUE);
    }
    focusFirstItem(FALSE);
    gSnapshotFloaterView->setEnabled(TRUE);
    gSnapshotFloaterView->setVisible(TRUE);
    gSnapshotFloaterView->adjustToFitScreen(this, FALSE);

    impl->updateControls(this);
    impl->setStatus(ImplBase::STATUS_READY);
}

void LLFloaterSimpleSnapshot::onCancel()
{
    closeFloater();
}

void LLFloaterSimpleSnapshot::onSend()
{
    LLSnapshotLivePreview* previewp = getPreviewView();

    std::string temp_file = gDirUtilp->getTempFilename();
    if (previewp->createUploadFile(temp_file, THUMBNAIL_SNAPSHOT_DIM))
    {
        uploadImageUploadFile(temp_file, mInventoryId, mTaskId);
    }
    else
    {
        LLSD notif_args;
        notif_args["REASON"] = LLImage::getLastError().c_str();
        LLNotificationsUtil::add("CannotUploadTexture", notif_args);
    }
}

void LLFloaterSimpleSnapshot::postSave()
{
    impl->setStatus(ImplBase::STATUS_WORKING);
}

// static
void LLFloaterSimpleSnapshot::uploadThumbnail(const std::string &file_path, const LLUUID &inventory_id, const LLUUID &task_id)
{
    // generate a temp texture file for coroutine
    std::string temp_file = gDirUtilp->getTempFilename();
    U32 codec = LLImageBase::getCodecFromExtension(gDirUtilp->getExtension(file_path));
    if (!LLViewerTextureList::createUploadFile(file_path, temp_file, codec, THUMBNAIL_SNAPSHOT_DIM))
    {
        LLSD notif_args;
        notif_args["REASON"] = LLImage::getLastError().c_str();
        LLNotificationsUtil::add("CannotUploadTexture", notif_args);
        LL_WARNS("Thumbnail") << "Failed to upload thumbnail for " << inventory_id << " " << task_id << ", reason: " << notif_args["REASON"].asString() << LL_ENDL;
        return;
    }
    uploadImageUploadFile(temp_file, inventory_id, task_id);
}

// static
void LLFloaterSimpleSnapshot::uploadImageUploadFile(const std::string &temp_file, const LLUUID &inventory_id, const LLUUID &task_id)
{
    std::string cap_name;
    LLSD data;

    if (task_id.notNull())
    {
        cap_name = THUMBNAIL_ITEM_UPLOAD_CAP;
        data["item_id"] = inventory_id;
        data["task_id"] = task_id;
    }
    else if (gInventory.getCategory(inventory_id))
    {
        cap_name = THUMBNAIL_CATEGORY_UPLOAD_CAP;
        data["category_id"] = inventory_id;
    }
    else
    {
        cap_name = THUMBNAIL_ITEM_UPLOAD_CAP;
        data["item_id"] = inventory_id;
    }

    std::string cap_url = gAgent.getRegionCapability(cap_name);
    if (cap_url.empty())
    {
        LLSD args;
        args["CAPABILITY"] = cap_url;
        LLNotificationsUtil::add("RegionCapabilityRequestError", args);
        LL_WARNS("Thumbnail") << "Failed to upload profile image for item " << inventory_id << " " << task_id << ", no cap found" << LL_ENDL;
        return;
    }

    LLCoros::instance().launch("postAgentUserImageCoro",
        boost::bind(post_thumbnail_image_coro, cap_url, temp_file, data));
}

// static 
void LLFloaterSimpleSnapshot::update()
{
    LLFloaterSimpleSnapshot* inst = findInstance();
    if (inst != NULL)
    {
        inst->impl->updateLivePreview();
    }
}


// static
LLFloaterSimpleSnapshot* LLFloaterSimpleSnapshot::findInstance()
{
    return LLFloaterReg::findTypedInstance<LLFloaterSimpleSnapshot>("simple_snapshot");
}

// static
LLFloaterSimpleSnapshot* LLFloaterSimpleSnapshot::getInstance()
{
    return LLFloaterReg::getTypedInstance<LLFloaterSimpleSnapshot>("simple_snapshot");
}

void LLFloaterSimpleSnapshot::saveTexture()
{
    LLSnapshotLivePreview* previewp = getPreviewView();
    if (!previewp)
    {
        llassert(previewp != NULL);
        return;
    }

    previewp->saveTexture(TRUE, getInventoryId().asString());
    closeFloater();
}

///----------------------------------------------------------------------------
/// Class LLSimpleOutfitSnapshotFloaterView
///----------------------------------------------------------------------------

LLSimpleSnapshotFloaterView::LLSimpleSnapshotFloaterView(const Params& p) : LLFloaterView(p)
{
}

LLSimpleSnapshotFloaterView::~LLSimpleSnapshotFloaterView()
{
}
