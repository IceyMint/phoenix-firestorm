/**
 * @file llpanelwearing.cpp
 * @brief List of agent's worn items.
 *
 * $LicenseInfo:firstyear=2010&license=viewerlgpl$
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

#include "llviewerprecompiledheaders.h"

#include "llpanelwearing.h"

#include "lltoggleablemenu.h"

#include "llagent.h"
#include "llaccordionctrl.h"
#include "llaccordionctrltab.h"
#include "llappearancemgr.h"
#include "llfloatersidepanelcontainer.h"
#include "llinventoryfunctions.h"
#include "llinventoryicon.h"
#include "llinventorymodel.h"
#include "llinventoryobserver.h"
#include "llmenubutton.h"
#include "llscrolllistctrl.h"
#include "llviewermenu.h"
#include "llviewerregion.h"
#include "llwearableitemslist.h"
#include "llsdserialize.h"
#include "llclipboard.h"
// [RLVa:KB] - Checked: 2012-07-28 (RLVa-1.4.7)
#include "rlvactions.h"
#include "rlvcommon.h"
#include "rlvhandler.h"
#include "rlvlocks.h"
// [/RLVa:KB]
#include "lltextbox.h"
#include "llresmgr.h"

// Context menu and Gear menu helper.
static void edit_outfit()
{
	LLFloaterSidePanelContainer::showPanel("appearance", LLSD().with("type", "edit_outfit"));
}

// [SL:KB] - Patch: Inventory-AttachmentEdit - Checked: 2010-09-04 (Catznip-2.2.0a) | Added: Catznip-2.1.2a
static void edit_item(const LLUUID& idItem)
{
	const LLViewerInventoryItem* pItem = gInventory.getItem(idItem);
	if (!pItem)
		return;

	switch (pItem->getType())
	{
		case LLAssetType::AT_BODYPART:
		case LLAssetType::AT_CLOTHING:
			LLAgentWearables::editWearable(idItem);
			break;
		case LLAssetType::AT_OBJECT:
			handle_attachment_edit(idItem);
			break;
		default:
			break;
	}
}
// [/SL:KB]

//////////////////////////////////////////////////////////////////////////

class LLWearingGearMenu
{
public:
	LLWearingGearMenu(LLPanelWearing* panel_wearing)
	:	mMenu(NULL), mPanelWearing(panel_wearing)
	{
		LLUICtrl::CommitCallbackRegistry::ScopedRegistrar registrar;
		LLUICtrl::EnableCallbackRegistry::ScopedRegistrar enable_registrar;

		registrar.add("Gear.Edit", boost::bind(&edit_outfit));
		registrar.add("Gear.TakeOff", boost::bind(&LLPanelWearing::onRemoveItem, mPanelWearing));
		registrar.add("Gear.Copy", boost::bind(&LLPanelWearing::copyToClipboard, mPanelWearing));

		enable_registrar.add("Gear.OnEnable", boost::bind(&LLPanelWearing::isActionEnabled, mPanelWearing, _2));

		mMenu = LLUICtrlFactory::getInstance()->createFromFile<LLToggleableMenu>(
			"menu_wearing_gear.xml", gMenuHolder, LLViewerMenuHolderGL::child_registry_t::instance());
		llassert(mMenu);
	}

	LLToggleableMenu* getMenu() { return mMenu; }

private:

	LLToggleableMenu*		mMenu;
	LLPanelWearing* 		mPanelWearing;
};

//////////////////////////////////////////////////////////////////////////

class LLWearingContextMenu : public LLListContextMenu
{
protected:
	/* virtual */ LLContextMenu* createMenu()
	{
		LLUICtrl::CommitCallbackRegistry::ScopedRegistrar registrar;

		registrar.add("Wearing.Edit", boost::bind(&edit_outfit));
// [SL:KB] - Patch: Inventory-AttachmentEdit - Checked: 2010-09-04 (Catznip-2.2.0a) | Added: Catznip-2.1.2a
		registrar.add("Wearing.EditItem", boost::bind(handleMultiple, edit_item, mUUIDs));
// [/SL:KB]
		registrar.add("Wearing.ShowOriginal", boost::bind(show_item_original, mUUIDs.front()));
		registrar.add("Wearing.TakeOff",
					  boost::bind(&LLAppearanceMgr::removeItemsFromAvatar, LLAppearanceMgr::getInstance(), mUUIDs));
		registrar.add("Wearing.Detach", 
					  boost::bind(&LLAppearanceMgr::removeItemsFromAvatar, LLAppearanceMgr::getInstance(), mUUIDs));
// [SL:KB] - Patch: Inventory-AttachmentEdit - Checked: 2010-09-04 (Catznip-2.2.0a) | Added: Catznip-2.1.2a
		registrar.add("Wearing.TakeOffDetach", 
					  boost::bind(&LLAppearanceMgr::removeItemsFromAvatar, LLAppearanceMgr::getInstance(), mUUIDs));
// [/SL:KB]
		LLContextMenu* menu = createFromFile("menu_wearing_tab.xml");

		updateMenuItemsVisibility(menu);

		return menu;
	}

	void updateMenuItemsVisibility(LLContextMenu* menu)
	{
		bool bp_selected			= false;	// true if body parts selected
		bool clothes_selected		= false;
		bool attachments_selected	= false;
// [RLVa:KB] - Checked: 2012-07-28 (RLVa-1.4.7)
		S32 rlv_locked_count = 0;
// [/RLVa:KB]

		// See what types of wearables are selected.
		for (uuid_vec_t::const_iterator it = mUUIDs.begin(); it != mUUIDs.end(); ++it)
		{
			LLViewerInventoryItem* item = gInventory.getItem(*it);

			if (!item)
			{
				LL_WARNS() << "Invalid item" << LL_ENDL;
				continue;
			}

			LLAssetType::EType type = item->getType();
			if (type == LLAssetType::AT_CLOTHING)
			{
				clothes_selected = true;
			}
			else if (type == LLAssetType::AT_BODYPART)
			{
				bp_selected = true;
			}
			else if (type == LLAssetType::AT_OBJECT || type == LLAssetType::AT_GESTURE)
			{
				attachments_selected = true;
			}
// [RLVa:KB] - Checked: 2012-07-28 (RLVa-1.4.7)
			if ( (rlv_handler_t::isEnabled()) && (!rlvPredCanRemoveItem(*it)) )
			{
				rlv_locked_count++;
			}
// [/RLVa:KB]
		}

		// Enable/disable some menu items depending on the selection.
// [RLVa:KB] - Checked: 2012-07-28 (RLVa-1.4.7)
		bool rlv_blocked = (mUUIDs.size() == rlv_locked_count);
// [/RLVa:KB]
		bool allow_detach = !bp_selected && !clothes_selected && attachments_selected;
		bool allow_take_off = !bp_selected && clothes_selected && !attachments_selected;

		menu->setItemVisible("take_off",	allow_take_off);
		menu->setItemVisible("detach",		allow_detach);
// [SL:KB] - Patch: Inventory-AttachmentEdit - Checked: 2010-09-04 (Catznip-2.2.0a) | Added: Catznip-2.1.2a
		menu->setItemVisible("take_off_or_detach", (!allow_detach) && (!allow_take_off) && (clothes_selected) && (attachments_selected));
// [/SL:KB]
// [RLVa:KB] - Checked: 2012-07-28 (RLVa-1.4.7)
		menu->setItemEnabled("take_off",	!rlv_blocked);
		menu->setItemEnabled("detach",		!rlv_blocked);
// [/RLVa:KB]
		menu->setItemVisible("edit_outfit_separator", allow_take_off || allow_detach);
		menu->setItemVisible("show_original", mUUIDs.size() == 1);
//		menu->setItemVisible("edit_item", FALSE);
// [SL:KB] - Patch: Inventory-AttachmentEdit - Checked: 2010-09-04 (Catznip-2.2.0a) | Added: Catznip-2.1.2a
		menu->setItemVisible("edit_item",	bp_selected || clothes_selected || attachments_selected);
		menu->setItemEnabled("edit_item",	1 == mUUIDs.size());
// [/SL:KB]
	}
};

//////////////////////////////////////////////////////////////////////////

class LLTempAttachmentsContextMenu : public LLListContextMenu
{
public:
	LLTempAttachmentsContextMenu(LLPanelWearing* panel_wearing)
		:	mPanelWearing(panel_wearing)
	{}
protected:
	/* virtual */ LLContextMenu* createMenu()
	{
		LLUICtrl::CommitCallbackRegistry::ScopedRegistrar registrar;

		registrar.add("Wearing.EditItem", boost::bind(&LLPanelWearing::onEditAttachment, mPanelWearing));
		registrar.add("Wearing.Detach", boost::bind(&LLPanelWearing::onRemoveAttachment, mPanelWearing));
		LLContextMenu* menu = createFromFile("menu_wearing_tab.xml");

		updateMenuItemsVisibility(menu);

		return menu;
	}

	void updateMenuItemsVisibility(LLContextMenu* menu)
	{
		menu->setItemVisible("take_off", FALSE);
		menu->setItemVisible("detach", TRUE);
// [SL:KB] - Patch: Inventory-AttachmentEdit - Checked: 2010-09-04 (Catznip-2.2.0a) | Added: Catznip-2.1.2a
		menu->setItemVisible("take_off_or_detach", FALSE);
// [/SL:KB]
		menu->setItemVisible("edit_outfit_separator", TRUE);
		menu->setItemVisible("show_original", FALSE);
		menu->setItemVisible("edit_item", TRUE);
		menu->setItemVisible("edit", FALSE);
	}

	LLPanelWearing* 		mPanelWearing;
};

//////////////////////////////////////////////////////////////////////////

std::string LLPanelAppearanceTab::sFilterSubString = LLStringUtil::null;

static LLPanelInjector<LLPanelWearing> t_panel_wearing("panel_wearing");

LLPanelWearing::LLPanelWearing()
	:	LLPanelAppearanceTab()
	,	mCOFItemsList(NULL)
	,	mAvatarComplexityLabel(NULL) // <FS:Ansariel> Show avatar complexity in appearance floater
	,	mIsInitialized(false)
	,	mAttachmentsChangedConnection()
{
	mCategoriesObserver = new LLInventoryCategoriesObserver();

	mGearMenu = new LLWearingGearMenu(this);
	mContextMenu = new LLWearingContextMenu();
	mAttachmentsMenu = new LLTempAttachmentsContextMenu(this);
}

LLPanelWearing::~LLPanelWearing()
{
	delete mGearMenu;
	delete mContextMenu;
	delete mAttachmentsMenu;

	if (gInventory.containsObserver(mCategoriesObserver))
	{
		gInventory.removeObserver(mCategoriesObserver);
	}
	delete mCategoriesObserver;

	if (mAttachmentsChangedConnection.connected())
	{
		mAttachmentsChangedConnection.disconnect();
	}
}

BOOL LLPanelWearing::postBuild()
{
	mAccordionCtrl = getChild<LLAccordionCtrl>("wearables_accordion");
	mWearablesTab = getChild<LLAccordionCtrlTab>("tab_wearables");
	mWearablesTab->setIgnoreResizeNotification(true);
	mAttachmentsTab = getChild<LLAccordionCtrlTab>("tab_temp_attachments");
	mAttachmentsTab->setDropDownStateChangedCallback(boost::bind(&LLPanelWearing::onAccordionTabStateChanged, this));

	mCOFItemsList = getChild<LLWearableItemsList>("cof_items_list");
	mCOFItemsList->setRightMouseDownCallback(boost::bind(&LLPanelWearing::onWearableItemsListRightClick, this, _1, _2, _3));
	mCOFItemsList->setDoubleClickCallback(boost::bind(&LLPanelWearing::onDoubleClick, this)); // <FS:Ansariel> FIRE-22484: Double-click wear in outfits list

	mTempItemsList = getChild<LLScrollListCtrl>("temp_attachments_list");
	mTempItemsList->setFgUnselectedColor(LLColor4::white);
	mTempItemsList->setRightMouseDownCallback(boost::bind(&LLPanelWearing::onTempAttachmentsListRightClick, this, _1, _2, _3));
	mTempItemsList->setDoubleClickCallback(boost::bind(&LLPanelWearing::onRemoveAttachment, this)); // <FS:Ansariel> FIRE-22484: Double-click wear in outfits list

	// <FS:Ansariel> Show avatar complexity in appearance floater
	mAvatarComplexityLabel = getChild<LLTextBox>("avatar_complexity_label");

	LLMenuButton* menu_gear_btn = getChild<LLMenuButton>("options_gear_btn");

	menu_gear_btn->setMenu(mGearMenu->getMenu());

	return TRUE;
}

//virtual
void LLPanelWearing::onOpen(const LLSD& /*info*/)
{
	if (!mIsInitialized)
	{
		// *TODO: I'm not sure is this check necessary but it never match while developing.
		if (!gInventory.isInventoryUsable())
			return;

		const LLUUID cof = gInventory.findCategoryUUIDForType(LLFolderType::FT_CURRENT_OUTFIT);

		// *TODO: I'm not sure is this check necessary but it never match while developing.
		LLViewerInventoryCategory* category = gInventory.getCategory(cof);
		if (!category)
			return;

		gInventory.addObserver(mCategoriesObserver);

		// Start observing changes in Current Outfit category.
		mCategoriesObserver->addCategory(cof, boost::bind(&LLWearableItemsList::updateList, mCOFItemsList, cof));

		// Fetch Current Outfit contents and refresh the list to display
		// initially fetched items. If not all items are fetched now
		// the observer will refresh the list as soon as the new items
		// arrive.
		category->fetch();

		mCOFItemsList->updateList(cof);

		mIsInitialized = true;
	}
}

void LLPanelWearing::draw()
{
	if (mUpdateTimer.getStarted() && (mUpdateTimer.getElapsedTimeF32() > 0.1f))
	{
		mUpdateTimer.stop();
		updateAttachmentsList();
	}
	LLPanel::draw();
}

void LLPanelWearing::onAccordionTabStateChanged()
{
	if(mAttachmentsTab->isExpanded())
	{
		startUpdateTimer();
		mAttachmentsChangedConnection = LLAppearanceMgr::instance().setAttachmentsChangedCallback(boost::bind(&LLPanelWearing::startUpdateTimer, this));
	}
	else
	{
		if (mAttachmentsChangedConnection.connected())
		{
			mAttachmentsChangedConnection.disconnect();
		}
	}
}

void LLPanelWearing::startUpdateTimer()
{
	if (!mUpdateTimer.getStarted())
	{
		mUpdateTimer.start();
	}
	else
	{
		mUpdateTimer.reset();
	}
}

// virtual
void LLPanelWearing::setFilterSubString(const std::string& string)
{
	sFilterSubString = string;
	mCOFItemsList->setFilterSubString(sFilterSubString);
}

// virtual
bool LLPanelWearing::isActionEnabled(const LLSD& userdata)
{
	const std::string command_name = userdata.asString();

	if (command_name == "save_outfit")
	{
		bool outfit_locked = LLAppearanceMgr::getInstance()->isOutfitLocked();
		bool outfit_dirty = LLAppearanceMgr::getInstance()->isOutfitDirty();
		// allow save only if outfit isn't locked and is dirty
		return !outfit_locked && outfit_dirty;
	}

	if (command_name == "take_off")
	{
		if (mWearablesTab->isExpanded())
		{
			return hasItemSelected() && canTakeOffSelected();
		}
		else
		{
			LLScrollListItem* item = mTempItemsList->getFirstSelected();
			if (item && item->getUUID().notNull())
			{
				return true;
			}
		}
	}

	return false;
}

void LLPanelWearing::updateAttachmentsList()
{
	std::vector<LLViewerObject*> attachs = LLAgentWearables::getTempAttachments();
	mTempItemsList->deleteAllItems();
	mAttachmentsMap.clear();
	if(!attachs.empty())
	{
		if(!populateAttachmentsList())
		{
			requestAttachmentDetails();
		}
	}
	else
	{
		std::string no_attachments = getString("no_attachments");
		LLSD row;
		row["columns"][0]["column"] = "text";
		row["columns"][0]["value"] = no_attachments;
		row["columns"][0]["font"] = "SansSerifBold";
		mTempItemsList->addElement(row);
	}
}

bool LLPanelWearing::populateAttachmentsList(bool update)
{
	bool populated = true;
	if(mTempItemsList)
	{
		mTempItemsList->deleteAllItems();
		mAttachmentsMap.clear();
		std::vector<LLViewerObject*> attachs = LLAgentWearables::getTempAttachments();

		std::string icon_name = LLInventoryIcon::getIconName(LLAssetType::AT_OBJECT, LLInventoryType::IT_OBJECT);
		for (std::vector<LLViewerObject*>::iterator iter = attachs.begin();
				iter != attachs.end(); ++iter)
		{
			LLViewerObject *attachment = *iter;
			LLSD row;
			row["id"] = attachment->getID();
			row["columns"][0]["column"] = "icon";
			row["columns"][0]["type"] = "icon";
			row["columns"][0]["value"] = icon_name;
			row["columns"][1]["column"] = "text";
			if(mObjectNames.count(attachment->getID()) && !mObjectNames[attachment->getID()].empty())
			{
				row["columns"][1]["value"] = mObjectNames[attachment->getID()];
			}
			else if(update)
			{
				row["columns"][1]["value"] = attachment->getID();
				populated = false;
			}
			else
			{
				// <FS:Ansariel> Translation fix
				//row["columns"][1]["value"] = "Loading...";
				row["columns"][1]["value"] = LLTrans::getString("LoadingData");
				// </FS:Ansariel>
				populated = false;
			}
			// <FS:Ansariel> Show avatar complexity in appearance floater
			std::string complexity_string;
			LLLocale locale("");
			LLResMgr::getInstance()->getIntegerString(complexity_string, mTempItemComplexityMap[attachment->getID()]);
			row["columns"][2]["column"] = "weight";
			row["columns"][2]["value"] = complexity_string;
			row["columns"][2]["halign"] = "right";
			// </FS:Ansariel>
			mTempItemsList->addElement(row);
			mAttachmentsMap[attachment->getID()] = attachment;
		}
	}
	return populated;
}

void LLPanelWearing::requestAttachmentDetails()
{
	LLSD body;
	std::string url = gAgent.getRegionCapability("AttachmentResources");
	if (!url.empty())
	{
		LLCoros::instance().launch("LLPanelWearing::getAttachmentLimitsCoro",
		boost::bind(&LLPanelWearing::getAttachmentLimitsCoro, this, url));
	}
}

void LLPanelWearing::getAttachmentLimitsCoro(std::string url)
{
	LLCore::HttpRequest::policy_t httpPolicy(LLCore::HttpRequest::DEFAULT_POLICY_ID);
	LLCoreHttpUtil::HttpCoroutineAdapter::ptr_t
	httpAdapter(new LLCoreHttpUtil::HttpCoroutineAdapter("getAttachmentLimitsCoro", httpPolicy));
	LLCore::HttpRequest::ptr_t httpRequest(new LLCore::HttpRequest);

	LLSD result = httpAdapter->getAndSuspend(httpRequest, url);

	LLSD httpResults = result[LLCoreHttpUtil::HttpCoroutineAdapter::HTTP_RESULTS];
	LLCore::HttpStatus status = LLCoreHttpUtil::HttpCoroutineAdapter::getStatusFromLLSD(httpResults);

	if (!status)
	{
		LL_WARNS() << "Unable to retrieve attachment limits." << LL_ENDL;
		return;
	}

	result.erase(LLCoreHttpUtil::HttpCoroutineAdapter::HTTP_RESULTS);
	setAttachmentDetails(result);
}


void LLPanelWearing::setAttachmentDetails(LLSD content)
{
	mObjectNames.clear();
	S32 number_attachments = content["attachments"].size();
	for(int i = 0; i < number_attachments; i++)
	{
		S32 number_objects = content["attachments"][i]["objects"].size();
		for(int j = 0; j < number_objects; j++)
		{
			LLUUID task_id = content["attachments"][i]["objects"][j]["id"].asUUID();
			std::string name = content["attachments"][i]["objects"][j]["name"].asString();
			mObjectNames[task_id] = name;
		}
	}
	if(!mObjectNames.empty())
	{
		populateAttachmentsList(true);
	}
}

boost::signals2::connection LLPanelWearing::setSelectionChangeCallback(commit_callback_t cb)
{
	if (!mCOFItemsList) return boost::signals2::connection();

	return mCOFItemsList->setCommitCallback(cb);
}

void LLPanelWearing::onWearableItemsListRightClick(LLUICtrl* ctrl, S32 x, S32 y)
{
	LLWearableItemsList* list = dynamic_cast<LLWearableItemsList*>(ctrl);
	if (!list) return;

	uuid_vec_t selected_uuids;

	list->getSelectedUUIDs(selected_uuids);

	mContextMenu->show(ctrl, selected_uuids, x, y);
}

void LLPanelWearing::onTempAttachmentsListRightClick(LLUICtrl* ctrl, S32 x, S32 y)
{
	LLScrollListCtrl* list = dynamic_cast<LLScrollListCtrl*>(ctrl);
	if (!list) return;
	list->selectItemAt(x, y, MASK_NONE);
	uuid_vec_t selected_uuids;

	if(list->getCurrentID().notNull())
	{
		selected_uuids.push_back(list->getCurrentID());
		mAttachmentsMenu->show(ctrl, selected_uuids, x, y);
	}
}

bool LLPanelWearing::hasItemSelected()
{
	return mCOFItemsList->getSelectedItem() != NULL;
}

void LLPanelWearing::getSelectedItemsUUIDs(uuid_vec_t& selected_uuids) const
{
	mCOFItemsList->getSelectedUUIDs(selected_uuids);
}

void LLPanelWearing::onEditAttachment()
{
	LLScrollListItem* item = mTempItemsList->getFirstSelected();
	if (item)
	{
		LLSelectMgr::getInstance()->deselectAll();
		LLSelectMgr::getInstance()->selectObjectAndFamily(mAttachmentsMap[item->getUUID()]);
		handle_object_edit();
	}
}

void LLPanelWearing::onRemoveAttachment()
{
	LLScrollListItem* item = mTempItemsList->getFirstSelected();
	if (item && item->getUUID().notNull())
	{
		LLSelectMgr::getInstance()->deselectAll();
		LLSelectMgr::getInstance()->selectObjectAndFamily(mAttachmentsMap[item->getUUID()]);
		LLSelectMgr::getInstance()->sendDropAttachment();
	}
}

void LLPanelWearing::onRemoveItem()
{
	if (mWearablesTab->isExpanded())
	{
		uuid_vec_t selected_uuids;
		getSelectedItemsUUIDs(selected_uuids);
		LLAppearanceMgr::instance().removeItemsFromAvatar(selected_uuids);
	}
	else
	{
		onRemoveAttachment();
	}
}


void LLPanelWearing::copyToClipboard()
{
	std::string text;
	std::vector<LLSD> data;
	mCOFItemsList->getValues(data);

	for(std::vector<LLSD>::const_iterator iter = data.begin(); iter != data.end();)
	{
		LLSD uuid = (*iter);
		LLViewerInventoryItem* item = gInventory.getItem(uuid);

		iter++;
		if (item != NULL)
		{
			// Append a newline to all but the last line
			text += iter != data.end() ? item->getName() + "\n" : item->getName();
		}
	}

	LLClipboard::instance().copyToClipboard(utf8str_to_wstring(text),0,text.size());
}

// <FS:Ansariel> Show avatar complexity in appearance floater
void LLPanelWearing::updateAvatarComplexity(U32 complexity, const std::map<LLUUID, U32>& item_complexity, const std::map<LLUUID, U32>& temp_item_complexity, U32 body_parts_complexity)
{
	std::string complexity_string;
	LLLocale locale("");
	LLResMgr::getInstance()->getIntegerString(complexity_string, complexity);

	mAvatarComplexityLabel->setTextArg("[WEIGHT]", complexity_string);
	
	mCOFItemsList->updateItemComplexity(item_complexity, body_parts_complexity);

	mTempItemComplexityMap = temp_item_complexity;
	updateAttachmentsList();
}
// </FS:Ansariel>

// <FS:Ansariel> FIRE-22484: Double-click wear in outfits list
void LLPanelWearing::onDoubleClick()
{
	LLUUID selected_item_id = mCOFItemsList->getSelectedUUID();
	if (selected_item_id.notNull())
	{
		uuid_vec_t ids;
		ids.push_back(selected_item_id);
		LLViewerInventoryItem* item = gInventory.getItem(selected_item_id);

		if ((item->getType() == LLAssetType::AT_CLOTHING && (!RlvActions::isRlvEnabled() || gRlvWearableLocks.canRemove(item))) ||
			((item->getType() == LLAssetType::AT_OBJECT) && (!RlvActions::isRlvEnabled() || gRlvAttachmentLocks.canDetach(item))))
		{
			LLAppearanceMgr::instance().removeItemsFromAvatar(ids);
		}
	}
}
// </FS:Ansariel>

// EOF
