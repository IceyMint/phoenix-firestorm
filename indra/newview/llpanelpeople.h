/** 
 * @file llpanelpeople.h
 * @brief Side tray "People" panel
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

#ifndef LL_LLPANELPEOPLE_H
#define LL_LLPANELPEOPLE_H

#include <llpanel.h>

#include "llcallingcard.h" // for avatar tracker
#include "llconversationmodel.h"
#include "llevents.h"
#include "llfloaterwebcontent.h"
#include "llvoiceclient.h"

class LLAvatarList;
class LLAvatarListSocial;
class LLAvatarName;
class LLFilterEditor;
class LLGroupList;
class LLSocialList;
class LLMenuButton;
class LLTabContainer;
class LLFolderView;

class LLPersonTabModel;
class LLPersonTabView;
class LLPersonView;
class LLPersonModel;

typedef std::map<LLUUID, LLPersonTabModel *> person_folder_model_map;
typedef std::map<LLUUID, LLPersonTabView *> person_folder_view_map;

class LLPanelPeople 
	: public LLPanel
	, public LLVoiceClientStatusObserver
{
	LOG_CLASS(LLPanelPeople);
public:
	LLPanelPeople();
	virtual ~LLPanelPeople();

	/*virtual*/ BOOL 	postBuild();
	/*virtual*/ void	onOpen(const LLSD& key);
	/*virtual*/ bool	notifyChildren(const LLSD& info);
	// Implements LLVoiceClientStatusObserver::onChange() to enable call buttons
	// when voice is available
	/*virtual*/ void onChange(EStatusType status, const std::string &channelURI, bool proximal);

	static void idle(void * user_data);

	void openFacebookWeb(LLFloaterWebContent::Params& p);
	void showFacebookFriends(const LLSD& friends);
	void addTestParticipant();
	void addParticipantToModel(LLPersonTabModel * session_model, const LLUUID& agent_id, const std::string& name);
	void hideFacebookFriends();
	void loadFacebookFriends();
	void tryToReconnectToFacebook();
	void connectToFacebook(const std::string& auth_code);
	void disconnectFromFacebook();
	
	std::string getFacebookConnectURL(const std::string& route = "");
	std::string getFacebookRedirectURL();
	
	bool mConnectedToFbc;
	bool mTryToConnectToFbc;

	// internals
	class Updater;

private:

	typedef enum e_sort_oder {
		E_SORT_BY_NAME = 0,
		E_SORT_BY_STATUS = 1,
		E_SORT_BY_MOST_RECENT = 2,
		E_SORT_BY_DISTANCE = 3,
		E_SORT_BY_RECENT_SPEAKERS = 4,
	} ESortOrder;

    void				    removePicker();

	// methods indirectly called by the updaters
	void					updateFriendListHelpText();
	void					updateFriendList();
	void					updateNearbyList();
	void					updateRecentList();
	void					updateFbcTestList();

	bool					isItemsFreeOfFriends(const uuid_vec_t& uuids);

	void					updateButtons();
	std::string				getActiveTabName() const;
	LLUUID					getCurrentItemID() const;
	void					getCurrentItemIDs(uuid_vec_t& selected_uuids) const;
	void					showGroupMenu(LLMenuGL* menu);
	void					setSortOrder(LLAvatarList* list, ESortOrder order, bool save = true);

	// UI callbacks
	void					onFilterEdit(const std::string& search_string);
	void					onTabSelected(const LLSD& param);
	void					onAddFriendButtonClicked();
	void					onAddFriendWizButtonClicked();
	void					onDeleteFriendButtonClicked();
	void					onChatButtonClicked();
	void					onGearButtonClicked(LLUICtrl* btn);
	void					onImButtonClicked();
	void					onMoreButtonClicked();
	void					onAvatarListDoubleClicked(LLUICtrl* ctrl);
	void					onAvatarListCommitted(LLAvatarList* list);
	bool					onGroupPlusButtonValidate();
	void					onGroupMinusButtonClicked();
	void					onGroupPlusMenuItemClicked(const LLSD& userdata);

	void					onFriendsViewSortMenuItemClicked(const LLSD& userdata);
	void					onNearbyViewSortMenuItemClicked(const LLSD& userdata);
	void					onGroupsViewSortMenuItemClicked(const LLSD& userdata);
	void					onRecentViewSortMenuItemClicked(const LLSD& userdata);

	void					onLoginFbcButtonClicked();
	void					onFacebookAppRequestClicked();
	void					onFacebookAppSendClicked();
	void					onFacebookTestAddClicked();

	bool					onFriendsViewSortMenuItemCheck(const LLSD& userdata);
	bool					onRecentViewSortMenuItemCheck(const LLSD& userdata);
	bool					onNearbyViewSortMenuItemCheck(const LLSD& userdata);

	// misc callbacks
	static void				onAvatarPicked(const uuid_vec_t& ids, const std::vector<LLAvatarName> names);

	void					onFriendsAccordionExpandedCollapsed(LLUICtrl* ctrl, const LLSD& param, LLAvatarList* avatar_list);

	void					showAccordion(const std::string name, bool show);

	void					showFriendsAccordionsIfNeeded();

	void					onFriendListRefreshComplete(LLUICtrl*ctrl, const LLSD& param);

	void					setAccordionCollapsedByUser(LLUICtrl* acc_tab, bool collapsed);
	void					setAccordionCollapsedByUser(const std::string& name, bool collapsed);
	bool					isAccordionCollapsedByUser(LLUICtrl* acc_tab);
	bool					isAccordionCollapsedByUser(const std::string& name);

	bool					onConversationModelEvent(const LLSD& event);
	LLPersonView * createConversationViewParticipant(LLPersonModel * item);

	LLTabContainer*			mTabContainer;
	LLAvatarList*			mOnlineFriendList;
	LLAvatarList*			mAllFriendList;
	LLAvatarList*			mNearbyList;
	LLAvatarList*			mRecentList;
	LLGroupList*			mGroupList;
	LLSocialList*			mFacebookFriends;
	LLNetMap*				mMiniMap;

	std::vector<std::string> mSavedOriginalFilters;
	std::vector<std::string> mSavedFilters;
	LLHandle<LLView>		mFBCMenuHandle;
	LLHandle<LLFloater>		mFbcTestBrowserHandle;

	Updater*				mFriendListUpdater;
	Updater*				mNearbyListUpdater;
	Updater*				mRecentListUpdater;
	Updater*				mFbcTestListUpdater;
	Updater*				mButtonsUpdater;
	LLMenuButton*			mFBCGearButton;
    LLHandle< LLFloater >	mPicker;

	person_folder_model_map mPersonFolderModelMap;
	person_folder_view_map mPersonFolderViewMap;
	LLConversationViewModel mConversationViewModel;
	LLFolderView* mConversationsRoot;
	LLEventStream mConversationsEventStream;
};

#endif //LL_LLPANELPEOPLE_H
