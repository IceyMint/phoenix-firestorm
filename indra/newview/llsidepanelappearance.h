/** 
 * @file llsidepanelappearance.h
 * @brief Side Bar "Appearance" panel
 *
 * $LicenseInfo:firstyear=2009&license=viewerlgpl$
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

#ifndef LL_LLSIDEPANELAPPEARANCE_H
#define LL_LLSIDEPANELAPPEARANCE_H

#include "llpanel.h"
#include "llinventoryobserver.h"

#include "llinventory.h"
#include "llpaneloutfitedit.h"

class LLFilterEditor;
class LLCurrentlyWornFetchObserver;
class LLPanelEditWearable;
class LLViewerWearable;
class LLPanelOutfitsInventory;

class LLSidepanelAppearance : public LLPanel
{
	LOG_CLASS(LLSidepanelAppearance);
public:
	LLSidepanelAppearance();
	virtual ~LLSidepanelAppearance();

	/*virtual*/ BOOL postBuild();
	/*virtual*/ void onOpen(const LLSD& key);	
	// <FS:Ansariel> CTRL-F focusses local search editor
	/*virtual*/ BOOL handleKeyHere(KEY key, MASK mask);
	/*virtual*/ bool hasAccelerators() const { return true; }
	// </FS:Ansariel>

	void refreshCurrentOutfitName(const std::string& name = "");

	static void editWearable(LLViewerWearable *wearable, LLView *data, BOOL disable_camera_switch = FALSE);

	void fetchInventory();
	void inventoryFetched();
	void onNewOutfitButtonClicked();

	void showOutfitsInventoryPanel();
	void showOutfitEditPanel();
	void showWearableEditPanel(LLViewerWearable *wearable = NULL, BOOL disable_camera_switch = FALSE);
	void setWearablesLoading(bool val);
	void showDefaultSubpart();
	void updateScrollingPanelList();
	void updateToVisibility( const LLSD& new_visibility );
	LLPanelEditWearable* getWearable(){ return mEditWearable; }

// [RLVa:KB] - Checked: 2010-09-16 (RLVa-1.2.1a) | Added: RLVa-1.2.1a
	bool isOutfitEditPanelVisible() const;
	bool isWearableEditPanelVisible() const;

	LLPanelOutfitEdit*	 getOutfitEditPanel() { return mOutfitEdit; }
	LLPanelEditWearable* getWearableEditPanel() { return mEditWearable; }
// [/RLVa:KB]

	// <FS:Ansariel> Show avatar complexity in appearance floater
	static void updateAvatarComplexity(U32 complexity, const std::map<LLUUID, U32>& item_complexity, const std::map<LLUUID, U32>& temp_item_complexity, U32 body_parts_complexity);

private:
	void onFilterEdit(const std::string& search_string);
	void onVisibilityChanged ( const LLSD& new_visibility );

	void onOpenOutfitButtonClicked();
	void onEditAppearanceButtonClicked();

	void toggleMyOutfitsPanel(BOOL visible);
	void toggleOutfitEditPanel(BOOL visible, BOOL disable_camera_switch = FALSE);
	void toggleWearableEditPanel(BOOL visible, LLViewerWearable* wearable = NULL, BOOL disable_camera_switch = FALSE);

	LLFilterEditor*			mFilterEditor;
	LLPanelOutfitsInventory* mPanelOutfitsInventory;
	LLPanelOutfitEdit*		mOutfitEdit;
	LLPanelEditWearable*	mEditWearable;

	LLButton*					mOpenOutfitBtn;
	LLButton*					mEditAppearanceBtn;
	//LLButton*					mNewOutfitBtn; // <FS:Ansariel> Disabled as of 12-09-2014
	LLPanel*					mCurrOutfitPanel;

	LLTextBox*					mCurrentLookName;
	LLTextBox*					mOutfitStatus;

	// Search string for filtering landmarks and teleport
	// history locations
	std::string					mFilterSubString;

	// Gets set to true when we're opened for the first time.
	bool mOpened;

	// <FS:Ansariel> Show avatar complexity in appearance floater
	U32 mLastAvatarComplexity;
};

#endif //LL_LLSIDEPANELAPPEARANCE_H
