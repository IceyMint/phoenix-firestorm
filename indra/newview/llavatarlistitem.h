/** 
 * @file llavatarlistitem.h
 * @avatar list item header file
 *
 * $LicenseInfo:firstyear=2009&license=viewergpl$
 * 
 * Copyright (c) 2009, Linden Research, Inc.
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
 * online at
 * http://secondlifegrid.net/programs/open_source/licensing/flossexception
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

#ifndef LL_LLAVATARLISTITEM_H
#define LL_LLAVATARLISTITEM_H

#include "llpanel.h"
#include "lloutputmonitorctrl.h"
#include "llbutton.h"
#include "lltextbox.h"
#include "llstyle.h"

#include "llcallingcard.h" // for LLFriendObserver

class LLAvatarIconCtrl;

class LLAvatarListItem : public LLPanel, public LLFriendObserver
{
public:
	typedef enum e_item_state_type {
		IS_DEFAULT,
		IS_VOICE_INVITED,
		IS_VOICE_JOINED,
		IS_VOICE_LEFT,
		IS_ONLINE,
		IS_OFFLINE,
	} EItemState;

	class ContextMenu
	{
	public:
		virtual void show(LLView* spawning_view, const std::vector<LLUUID>& selected_uuids, S32 x, S32 y) = 0;
	};

	/**
	 * Creates an instance of LLAvatarListItem.
	 *
	 * It is not registered with LLDefaultChildRegistry. It is built via LLUICtrlFactory::buildPanel
	 * or via registered LLCallbackMap depend on passed parameter.
	 * 
	 * @param not_from_ui_factory if true instance will be build with LLUICtrlFactory::buildPanel 
	 * otherwise it should be registered via LLCallbackMap before creating.
	 */
	LLAvatarListItem(bool not_from_ui_factory = true);
	virtual ~LLAvatarListItem();

	virtual BOOL postBuild();
	virtual void onMouseLeave(S32 x, S32 y, MASK mask);
	virtual void onMouseEnter(S32 x, S32 y, MASK mask);
	virtual void setValue(const LLSD& value);
	virtual void changed(U32 mask); // from LLFriendObserver

	void setOnline(bool online);
	void setName(const std::string& name);
	void setHighlight(const std::string& highlight);
	void setState(EItemState item_style);
	void setAvatarId(const LLUUID& id, bool ignore_status_changes = false);
	void setLastInteractionTime(U32 secs_since);
	//Show/hide profile/info btn, translating speaker indicator and avatar name coordinates accordingly
	void setShowProfileBtn(bool show);
	void setShowInfoBtn(bool show);
	void showSpeakingIndicator(bool show);
	void showLastInteractionTime(bool show);
	void setAvatarIconVisible(bool visible);
	
	const LLUUID& getAvatarId() const;
	const std::string getAvatarName() const;

	void onInfoBtnClick();
	void onProfileBtnClick();

protected:
	/**
	 * Contains indicator to show voice activity. 
	 */
	LLOutputMonitorCtrl* mSpeakingIndicator;

	LLAvatarIconCtrl* mAvatarIcon;

private:

	typedef enum e_online_status {
		E_OFFLINE,
		E_ONLINE,
		E_UNKNOWN,
	} EOnlineStatus;

	/**
	 * Enumeration of item elements in order from right to left.
	 * 
	 * updateChildren() assumes that indexes are in the such order to process avatar icon easier.
	 *
	 * @see updateChildren()
	 */
	typedef enum e_avatar_item_child {
		ALIC_PROFILE_BUTTON,
		ALIC_INFO_BUTTON,
		ALIC_SPEAKER_INDICATOR,
		ALIC_INTERACTION_TIME,
		ALIC_NAME,
		ALIC_ICON,
		ALIC_COUNT,
	} EAvatarListItemChildIndex;

	void setNameInternal(const std::string& name, const std::string& highlight);
	void onNameCache(const std::string& first_name, const std::string& last_name);

	std::string formatSeconds(U32 secs);

	typedef std::map<EItemState, LLStyle::Params> item_style_map_t;
	static item_style_map_t& getItemStylesParams();

	typedef std::map<EItemState, LLColor4> icon_color_map_t;
	static icon_color_map_t& getItemIconColorMap();

	/**
	 * Initializes widths of all children to use them while changing visibility of any of them.
	 *
	 * @see updateChildren()
	 */
	static void initChildrenWidths(LLAvatarListItem* self);

	/**
	 * Updates position and rectangle of visible children to fit all available item's width.
	 */
	void updateChildren();

	/**
	 * Gets child view specified by index.
	 *
	 * This method implemented via switch by all EAvatarListItemChildIndex values.
	 * It is used to not store children in array or vector to avoid of increasing memory usage.
	 */
	LLView* getItemChildView(EAvatarListItemChildIndex child_index);

	LLTextBox* mAvatarName;
	LLTextBox* mLastInteractionTime;
	LLStyle::Params mAvatarNameStyle;
	
	LLButton* mInfoBtn;
	LLButton* mProfileBtn;

	LLUUID mAvatarId;
	std::string mHighlihtSubstring; // substring to highlight
	EOnlineStatus mOnlineStatus;
	//Flag indicating that info/profile button shouldn't be shown at all.
	//Speaker indicator and avatar name coords are translated accordingly
	bool mShowInfoBtn;
	bool mShowProfileBtn;

	static bool	sStaticInitialized; // this variable is introduced to improve code readability
	static S32	sIconWidth; // icon width + padding
	static S32  sInfoBtnWidth; //info btn width + padding
	static S32  sProfileBtnWidth; //profile btn width + padding
	static S32  sSpeakingIndicatorWidth; //speaking indicator width + padding
	static S32  sLeftPadding; // padding to first left visible child (icon or name)
	static S32  sRightNamePadding; // right padding from name to next visible child

	/**
	 * Contains widths of each child specified by EAvatarListItemChildIndex
	 * including padding to the next right one.
	 *
	 * @see initChildrenWidths()
	 */
	static S32 sChildrenWidths[ALIC_COUNT];

};

#endif //LL_LLAVATARLISTITEM_H
