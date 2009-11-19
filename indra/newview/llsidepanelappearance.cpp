/**
 * @file llsidepanelappearance.cpp
 * @brief Side Bar "Appearance" panel
 *
 * $LicenseInfo:firstyear=2009&license=viewergpl$
 *
 * Copyright (c) 2004-2009, Linden Research, Inc.
 *
 * Second Life Viewer Source Code
 * The source code in this file ("Source Code") is provided by Linden Lab
 * to you under the terms of the GNU General Public License, version 2.0
 * ("GPL"), unless you have obtained a separate licensing agreement
 * ("Other License"), formally executed by you and Linden Lab.  Terms of
 * the GPL can be found in doc/GPL-license.txt in this distribution, or
 * online at http://secondlifegrid.net/programs/open_source/licensing/gplv2
 *
 * There are special exceptions to the terms and conditions of the GPL as
 * it is applied to this Source Code. View the full text of the exception
 * in the file doc/FLOSS-exception.txt in this software distribution, or
 * online at http://secondlifegrid.net/programs/open_source/licensing/flossexception
 *
 * By copying, modifying or distributing this software, you acknowledge
 * that you have read and understood your obligations described above,
 * and agree to abide by those obligations.
 *
 * ALL LINDEN LAB SOURCE CODE IS PROVIDED "AS IS." LINDEN LAB MAKES NO
 * WARRANTIES, EXPRESS, IMPLIED OR OTHERWISE, REGARDING ITS ACCURACY,
 * COMPLETENESS OR PERFORMANCE.
 * $/LicenseInfo$
 */

#include "llviewerprecompiledheaders.h"
#include "llsidepanelappearance.h"

#include "llagent.h"
#include "llagentwearables.h"
#include "llfiltereditor.h"
#include "llfloaterreg.h"
#include "llfloaterworldmap.h"
#include "llpaneleditwearable.h"
#include "llpaneloutfitsinventory.h"
#include "lltextbox.h"
#include "lluictrlfactory.h"
#include "llviewerregion.h"
#include "llvoavatarself.h"
#include "llwearable.h"

static LLRegisterPanelClassWrapper<LLSidepanelAppearance> t_appearance("sidepanel_appearance");

class LLCurrentlyWornFetchObserver : public LLInventoryFetchObserver
{
public:
	LLCurrentlyWornFetchObserver(LLSidepanelAppearance *panel) :
		mPanel(panel)
	{}
	~LLCurrentlyWornFetchObserver() {}
	virtual void done()
	{
		mPanel->inventoryFetched();
		gInventory.removeObserver(this);
	}
private:
	LLSidepanelAppearance *mPanel;
};

LLSidepanelAppearance::LLSidepanelAppearance() :
	LLPanel(),
	mFilterSubString(LLStringUtil::null),
	mFilterEditor(NULL),
	mLookInfo(NULL),
	mCurrLookPanel(NULL)
{
	//LLUICtrlFactory::getInstance()->buildPanel(this, "panel_appearance.xml"); // Called from LLRegisterPanelClass::defaultPanelClassBuilder()
	mFetchWorn = new LLCurrentlyWornFetchObserver(this);
}

LLSidepanelAppearance::~LLSidepanelAppearance()
{
}

// virtual
BOOL LLSidepanelAppearance::postBuild()
{
	mEditAppearanceBtn = getChild<LLButton>("editappearance_btn");
	mEditAppearanceBtn->setClickedCallback(boost::bind(&LLSidepanelAppearance::onEditAppearanceButtonClicked, this));

	mWearBtn = getChild<LLButton>("wear_btn");
	mWearBtn->setClickedCallback(boost::bind(&LLSidepanelAppearance::onWearButtonClicked, this));

	mEditBtn = getChild<LLButton>("edit_btn");
	mEditBtn->setClickedCallback(boost::bind(&LLSidepanelAppearance::onEditButtonClicked, this));

	mNewLookBtn = getChild<LLButton>("newlook_btn");
	mNewLookBtn->setClickedCallback(boost::bind(&LLSidepanelAppearance::onNewOutfitButtonClicked, this));
	mNewLookBtn->setEnabled(false);

	mOverflowBtn = getChild<LLButton>("overflow_btn");

	mFilterEditor = getChild<LLFilterEditor>("Filter");
	if (mFilterEditor)
	{
		mFilterEditor->setCommitCallback(boost::bind(&LLSidepanelAppearance::onFilterEdit, this, _2));
	}

	mPanelOutfitsInventory = dynamic_cast<LLPanelOutfitsInventory *>(getChild<LLPanel>("panel_outfits_inventory"));
	mPanelOutfitsInventory->setParent(this);

	mLookInfo = dynamic_cast<LLPanelLookInfo*>(getChild<LLPanel>("panel_look_info"));
	if (mLookInfo)
	{
		LLButton* back_btn = mLookInfo->getChild<LLButton>("back_btn");
		if (back_btn)
		{
			back_btn->setClickedCallback(boost::bind(&LLSidepanelAppearance::onBackButtonClicked, this));
		}

		// *TODO: Assign the action to an appropriate event.
		// mOverflowBtn->setClickedCallback(boost::bind(&LLSidepanelAppearance::toggleMediaPanel, this));
	}

	mEditWearable = dynamic_cast<LLPanelEditWearable*>(getChild<LLPanel>("panel_edit_wearable"));
	if (mEditWearable)
	{
		LLButton* edit_wearable_back_btn = mEditWearable->getChild<LLButton>("back_btn");
		if (edit_wearable_back_btn)
		{
			edit_wearable_back_btn->setClickedCallback(boost::bind(&LLSidepanelAppearance::onEditWearBackClicked, this));
		}
	}

	mCurrentLookName = getChild<LLTextBox>("currentlook_name");
	
	mCurrLookPanel = getChild<LLPanel>("panel_currentlook");

	return TRUE;
}

// virtual
void LLSidepanelAppearance::onOpen(const LLSD& key)
{
	fetchInventory();
	refreshCurrentOutfitName();

	if(key.size() == 0)
		return;

	toggleLookInfoPanel(TRUE);
	updateVerbs();
	
	mLookInfoType = key["type"].asString();

	if (mLookInfoType == "look")
	{
		LLInventoryCategory *pLook = gInventory.getCategory(key["id"].asUUID());
		if (pLook)
			mLookInfo->displayLookInfo(pLook);
	}
}

void LLSidepanelAppearance::onFilterEdit(const std::string& search_string)
{
	if (mFilterSubString != search_string)
	{
		mFilterSubString = search_string;

		// Searches are case-insensitive
		LLStringUtil::toUpper(mFilterSubString);
		LLStringUtil::trimHead(mFilterSubString);

		mPanelOutfitsInventory->onSearchEdit(mFilterSubString);
	}
}

void LLSidepanelAppearance::onWearButtonClicked()
{
	if (!mLookInfo->getVisible())
	{
		mPanelOutfitsInventory->onWear();
	}
}

void LLSidepanelAppearance::onEditAppearanceButtonClicked()
{
	if (gAgentWearables.areWearablesLoaded())
	{
		gAgent.changeCameraToCustomizeAvatar();
	}
}

void LLSidepanelAppearance::onEditButtonClicked()
{
	toggleLookInfoPanel(FALSE);
	toggleWearableEditPanel(TRUE, NULL);
	/*if (mLookInfo->getVisible())
	  {
	  }
	  else
	  {
	  mPanelOutfitsInventory->onEdit();
	  }*/
}

void LLSidepanelAppearance::onNewOutfitButtonClicked()
{
	if (!mLookInfo->getVisible())
	{
		mPanelOutfitsInventory->onNew();
	}
}


void LLSidepanelAppearance::onBackButtonClicked()
{
	toggleLookInfoPanel(FALSE);
}

void LLSidepanelAppearance::onEditWearBackClicked()
{
	mEditWearable->saveChanges();
	toggleWearableEditPanel(FALSE, NULL);
	toggleLookInfoPanel(TRUE);
}

void LLSidepanelAppearance::toggleLookInfoPanel(BOOL visible)
{
	if (!mLookInfo)
		return;

	mLookInfo->setVisible(visible);
	mPanelOutfitsInventory->setVisible(!visible);
	mFilterEditor->setVisible(!visible);
	mWearBtn->setVisible(!visible);
	mEditBtn->setVisible(!visible);
	mNewLookBtn->setVisible(!visible);
	mOverflowBtn->setVisible(!visible);
	mCurrLookPanel->setVisible(!visible);
}

void LLSidepanelAppearance::toggleWearableEditPanel(BOOL visible, LLWearable *wearable)
{
	if (!wearable)
	{
		wearable = gAgentWearables.getWearable(WT_SHAPE, 0);
	}
	if (!mEditWearable || !wearable)
	{
		return;
	}

	mEditWearable->setVisible(visible);
	mFilterEditor->setVisible(!visible);
	mPanelOutfitsInventory->setVisible(!visible);
}

void LLSidepanelAppearance::updateVerbs()
{
	bool is_look_info_visible = mLookInfo->getVisible();
	mOverflowBtn->setEnabled(false);

	if (!is_look_info_visible)
	{
		const bool is_correct_type = (mPanelOutfitsInventory->getCorrectListenerForAction() != NULL);
		mEditBtn->setEnabled(is_correct_type);
		mWearBtn->setEnabled(is_correct_type);
	}
	else
	{
		mEditBtn->setEnabled(FALSE);
		mWearBtn->setEnabled(FALSE);
	}
}

void LLSidepanelAppearance::refreshCurrentOutfitName(const std::string name)
{
	if (name == "")
	{
		const LLUUID current_outfit_cat = gInventory.findCategoryUUIDForType(LLFolderType::FT_CURRENT_OUTFIT);
		LLInventoryModel::cat_array_t cat_array;
		LLInventoryModel::item_array_t item_array;
		// Can't search on AT_OUTFIT since links to categories return AT_CATEGORY for type since they don't
		// return preferred type.
		LLIsType is_category( LLAssetType::AT_CATEGORY ); 
		gInventory.collectDescendentsIf(current_outfit_cat,
										cat_array,
										item_array,
										false,
										is_category,
										false);
		for (LLInventoryModel::item_array_t::const_iterator iter = item_array.begin();
			 iter != item_array.end();
			 iter++)
		{
			const LLViewerInventoryItem *item = (*iter);
			const LLViewerInventoryCategory *cat = item->getLinkedCategory();
			if (cat && cat->getPreferredType() == LLFolderType::FT_OUTFIT)
			{
				mCurrentLookName->setText(cat->getName());
				return;
			}
		}
		mCurrentLookName->setText(std::string(""));
	}
	else
	{
		mCurrentLookName->setText(name);
	}
}

//static
void LLSidepanelAppearance::editWearable(LLWearable *wearable, void *data)
{
	LLSidepanelAppearance *panel = (LLSidepanelAppearance*) data;
	panel->toggleLookInfoPanel(FALSE);
	panel->toggleWearableEditPanel(TRUE, wearable);
}

// Fetch currently worn items and only enable the New Look button after everything's been
// fetched.  Alternatively, we could stuff this logic into llagentwearables::makeNewOutfitLinks.
void LLSidepanelAppearance::fetchInventory()
{

	mNewLookBtn->setEnabled(false);
	LLInventoryFetchObserver::item_ref_t ids;
	LLUUID item_id;
	for(S32 type = (S32)WT_SHAPE; type < (S32)WT_COUNT; ++type)
	{
		// MULTI_WEARABLE:
		item_id = gAgentWearables.getWearableItemID((EWearableType)type,0);
		if(item_id.notNull())
		{
			ids.push_back(item_id);
		}
	}

	LLVOAvatarSelf* avatar = gAgent.getAvatarObject();
	if( avatar )
	{
		for (LLVOAvatar::attachment_map_t::const_iterator iter = avatar->mAttachmentPoints.begin(); 
			 iter != avatar->mAttachmentPoints.end(); ++iter)
		{
			LLViewerJointAttachment* attachment = iter->second;
			if (!attachment) continue;
			for (LLViewerJointAttachment::attachedobjs_vec_t::iterator attachment_iter = attachment->mAttachedObjects.begin();
				 attachment_iter != attachment->mAttachedObjects.end();
				 ++attachment_iter)
			{
				LLViewerObject* attached_object = (*attachment_iter);
				if (!attached_object) continue;
				const LLUUID& item_id = attached_object->getItemID();
				if (item_id.isNull()) continue;
				ids.push_back(item_id);
			}
		}
	}

	mFetchWorn->fetchItems(ids);
	// If no items to be fetched, done will never be triggered.
	// TODO: Change LLInventoryFetchObserver::fetchItems to trigger done() on this condition.
	if (mFetchWorn->isEverythingComplete())
	{
		mFetchWorn->done();
	}
	else
	{
		gInventory.addObserver(mFetchWorn);
	}
}

void LLSidepanelAppearance::inventoryFetched()
{
	mNewLookBtn->setEnabled(true);
}
