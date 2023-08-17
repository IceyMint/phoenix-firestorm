/**
 * @file llfloaterbulkythumbs.h
 * @author Callum Prentice
 * @brief Helper floater for bulk processing of inventory thumbnails
 *
 * $LicenseInfo:firstyear=2008&license=viewerlgpl$
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

#ifndef LL_LLFLOATERBULKYTHUMBS_H
#define LL_LLFLOATERBULKYTHUMBS_H

#include "llfloater.h"
class LLTextEditor;
class LLMediaCtrl;
class LLViewerInventoryItem;
class LLUUID;

class LLFloaterBulkyThumbs:
    public LLFloater
{
        friend class LLFloaterReg;
    private:
        LLFloaterBulkyThumbs(const LLSD& key);
        BOOL postBuild() override;
        ~LLFloaterBulkyThumbs();

        LLUICtrl* mPasteItemsBtn;
        void onPasteItems();

        LLUICtrl* mPasteTexturesBtn;
        void onPasteTextures();

        LLTextEditor* mOutputLog;

        LLUICtrl* mMergeItemsTexturesBtn;
        void onMergeItemsTextures();

        LLUICtrl* mWriteThumbnailsBtn;
        void onWriteThumbnails();

        LLUICtrl* mDisplayThumbnaillessItemsBtn;
        void onDisplayThumbnaillessItems();

        void recordInventoryItemEntry(LLViewerInventoryItem* item);
        void recordTextureItemEntry(LLViewerInventoryItem* item);

        std::map<std::string, LLUUID> mItemNamesIDs;
        std::map<std::string, LLUUID> mTextureNamesIDs;

        std::map<std::string, std::pair< LLUUID, LLUUID>> mNameItemIDTextureId;
};

#endif // LL_LLFLOATERBULKYTHUMBS_H
