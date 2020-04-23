/*
 * @file fspanelcontactsets.cpp
 * @brief Contact sets UI
 *
 * (C) 2013 Cinder Roxley @ Second Life <cinder.roxley@phoenixviewer.com>
 *
 * Permission is hereby granted, free of charge, to any person or organization
 * obtaining a copy of the software and accompanying documentation covered by
 * this license (the "Software") to use, reproduce, display, distribute,
 * execute, and transmit the Software, and to prepare derivative works of the
 * Software, and to permit third-parties to whom the Software is furnished to
 * do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer,
 * must be included in all copies of the Software, in whole or in part, and
 * all derivative works of the Software, unless such copies or derivative
 * works are solely in the form of machine-executable object code generated by
 * a source language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "llviewerprecompiledheaders.h"

#include "fspanelcontactsets.h"

#include "fsfloatercontacts.h"
#include "fsfloatercontactsetconfiguration.h"
#include "lggcontactsets.h"
#include "llavataractions.h"
#include "llcallingcard.h"
#include "llfloateravatarpicker.h"
#include "llfloaterreg.h"
#include "llnotificationsutil.h"
#include "llpanelpeoplemenus.h"
#include "llslurl.h"

const U32 MAX_SELECTIONS = 20;
static LLPanelInjector<FSPanelContactSets> t_panel_contact_sets("contact_sets_panel");

FSPanelContactSets::FSPanelContactSets() : LLPanel()
, mContactSetCombo(NULL)
, mAvatarList(NULL)
{
	mContactSetChangedConnection = LGGContactSets::getInstance()->setContactSetChangeCallback(boost::bind(&FSPanelContactSets::updateSets, this, _1));
}

FSPanelContactSets::~FSPanelContactSets()
{
	if (mContactSetChangedConnection.connected())
	{
		mContactSetChangedConnection.disconnect();
	}
}

BOOL FSPanelContactSets::postBuild()
{
	childSetAction("add_set_btn",			boost::bind(&FSPanelContactSets::onClickAddSet,			this));
	childSetAction("remove_set_btn",		boost::bind(&FSPanelContactSets::onClickRemoveSet,		this));
	childSetAction("config_btn",			boost::bind(&FSPanelContactSets::onClickConfigureSet,	this, _1));
	childSetAction("add_btn",				boost::bind(&FSPanelContactSets::onClickAddAvatar,		this, _1));
	childSetAction("remove_btn",			boost::bind(&FSPanelContactSets::onClickRemoveAvatar,	this));
	childSetAction("profile_btn",			boost::bind(&FSPanelContactSets::onClickOpenProfile,	this));
	childSetAction("start_im_btn",			boost::bind(&FSPanelContactSets::onClickStartIM,		this));
	childSetAction("offer_teleport_btn",	boost::bind(&FSPanelContactSets::onClickOfferTeleport,	this));
	childSetAction("set_pseudonym_btn",		boost::bind(&FSPanelContactSets::onClickSetPseudonym,	this));
	childSetAction("remove_pseudonym_btn",	boost::bind(&FSPanelContactSets::onClickRemovePseudonym,	this));
	childSetAction("remove_displayname_btn", boost::bind(&FSPanelContactSets::onClickRemoveDisplayName,	this));

	mContactSetCombo = getChild<LLComboBox>("combo_sets");
	if (mContactSetCombo)
	{
		mContactSetCombo->setCommitCallback(boost::bind(&FSPanelContactSets::refreshSetList, this));
		refreshContactSets();
	}

	mAvatarList = getChild<LLAvatarList>("contact_list");
	if (mAvatarList)
	{
		mAvatarList->setCommitCallback(boost::bind(&FSPanelContactSets::onSelectAvatar, this));
		mAvatarList->setNoItemsCommentText(getString("empty_list"));
		mAvatarList->setContextMenu(&LLPanelPeopleMenus::gPeopleContextMenu);
		generateAvatarList(mContactSetCombo->getValue().asString());
	}

	return TRUE;
}

void FSPanelContactSets::onSelectAvatar()
{
	mAvatarSelections.clear();
	mAvatarList->getSelectedUUIDs(mAvatarSelections);
	resetControls();
}

void FSPanelContactSets::generateAvatarList(const std::string& contact_set)
{
	if (!mAvatarList)
	{
		return;
	}

	mAvatarList->clear();
	mAvatarList->setDirty(true, true);

	uuid_vec_t& avatars = mAvatarList->getIDs();

	if (contact_set == CS_SET_ALL_SETS)
	{
		avatars = LGGContactSets::getInstance()->getListOfNonFriends();
		
		// "All sets" includes buddies
		LLAvatarTracker::buddy_map_t all_buddies;
		LLAvatarTracker::instance().copyBuddyList(all_buddies);
		for (LLAvatarTracker::buddy_map_t::const_iterator buddy = all_buddies.begin();
			 buddy != all_buddies.end();
			 ++buddy)
		{
			avatars.push_back(buddy->first);
		}
	}
	else if (contact_set == CS_SET_NO_SETS)
	{
		LLAvatarTracker::buddy_map_t all_buddies;
		LLAvatarTracker::instance().copyBuddyList(all_buddies);
		for (LLAvatarTracker::buddy_map_t::const_iterator buddy = all_buddies.begin();
			 buddy != all_buddies.end();
			 ++buddy)
		{
			// Only show our buddies who aren't in a set, by request.
			if (!LGGContactSets::getInstance()->isFriendInSet(buddy->first))
				avatars.push_back(buddy->first);
		}
	}
	else if (contact_set == CS_SET_PSEUDONYM)
	{
		avatars = LGGContactSets::getInstance()->getListOfPseudonymAvs();
	}
	else if (contact_set == CS_SET_EXTRA_AVS)
	{
		avatars = LGGContactSets::getInstance()->getListOfNonFriends();
	}
	else if (!LGGContactSets::getInstance()->isInternalSetName(contact_set))
	{
		LGGContactSets::ContactSet* group = LGGContactSets::getInstance()->getContactSet(contact_set);	// UGLY!
		for (auto const& id : group->mFriends)
		{
			avatars.push_back(id);
		}
	}
	getChild<LLTextBox>("member_count")->setTextArg("[COUNT]", llformat("%d", avatars.size()));
	mAvatarList->setDirty();
	resetControls();
}

void FSPanelContactSets::resetControls()
{
	bool mutable_set = (!LGGContactSets::getInstance()->isInternalSetName(mContactSetCombo->getValue().asString()));
	bool has_selection = (!mAvatarSelections.empty()
						  // Set a maximum number of avatars users are allowed to operate on
						  && mAvatarSelections.size() <= MAX_SELECTIONS);
	childSetEnabled("remove_set_btn", mutable_set);
	childSetEnabled("config_btn", mutable_set);
	childSetEnabled("add_btn", mutable_set);
	childSetEnabled("remove_btn", (mutable_set && has_selection));
	childSetEnabled("profile_btn", has_selection);
	childSetEnabled("start_im_btn", has_selection);
	childSetEnabled("offer_teleport_btn", has_selection);	// Should probably check if they're online...
	childSetEnabled("set_pseudonym_btn", (mAvatarSelections.size() == 1));
	childSetEnabled("remove_pseudonym_btn", (has_selection
											 && LGGContactSets::getInstance()->hasPseudonym(mAvatarSelections)));
	childSetEnabled("remove_displayname_btn", (has_selection
											   && !LGGContactSets::getInstance()->hasDisplayNameRemoved(mAvatarSelections)));
}

void FSPanelContactSets::updateSets(LGGContactSets::EContactSetUpdate type)
{
	switch (type)
	{
		case LGGContactSets::UPDATED_LISTS:
			refreshContactSets();
		case LGGContactSets::UPDATED_MEMBERS:
			refreshSetList();
			break;
	}
}

void FSPanelContactSets::refreshContactSets()
{
	if (!mContactSetCombo)
	{
		return;
	}

	mContactSetCombo->clearRows();
	std::vector<std::string> contact_sets = LGGContactSets::getInstance()->getAllContactSets();
	if (!contact_sets.empty())
	{
		for(auto const& set_name : contact_sets)
		{
			mContactSetCombo->add(set_name);
		}
		mContactSetCombo->addSeparator(ADD_BOTTOM);
	}
	mContactSetCombo->add(getString("all_sets"), LLSD(CS_SET_ALL_SETS), ADD_BOTTOM);
	mContactSetCombo->add(getString("no_sets"), LLSD(CS_SET_NO_SETS), ADD_BOTTOM);
	mContactSetCombo->add(getString("pseudonyms"), LLSD(CS_SET_PSEUDONYM), ADD_BOTTOM);
	mContactSetCombo->add(getString("non_friends"), LLSD(CS_SET_EXTRA_AVS), ADD_BOTTOM);
	resetControls();
}

void FSPanelContactSets::refreshSetList()
{
	mAvatarList->refreshNames();
	generateAvatarList(mContactSetCombo->getValue().asString());
	resetControls();
}

void FSPanelContactSets::onClickAddAvatar(LLUICtrl* ctrl)
{
	LLFloater* root_floater = gFloaterView->getParentFloater(this);
	LLFloater* avatar_picker = LLFloaterAvatarPicker::show(boost::bind(&FSPanelContactSets::handlePickerCallback, this, _1, mContactSetCombo->getValue().asString()),
														   TRUE, TRUE, TRUE, root_floater->getName(), ctrl);
	if (root_floater && avatar_picker)
	{
		root_floater->addDependentFloater(avatar_picker);
	}
}

void FSPanelContactSets::handlePickerCallback(const uuid_vec_t& ids, const std::string& set)
{
	if (ids.empty() || !mContactSetCombo)
	{
		return;
	}

	LGGContactSets::instance().addToSet(ids, set);
}

void FSPanelContactSets::onClickRemoveAvatar()
{
	if (!(mAvatarList && mContactSetCombo))
	{
		return;
	}

	LLSD payload, args;
	std::string set = mContactSetCombo->getValue().asString();
	S32 selected_size = mAvatarSelections.size();
	args["SET_NAME"] = set;
	args["TARGET"] = (selected_size > 1 ? llformat("%d", selected_size) : LLSLURL("agent", mAvatarSelections.front(), "about").getSLURLString());
	payload["contact_set"] = set;
	for (auto const& id : mAvatarSelections)
	{
		payload["ids"].append(id);
	}
	LLNotificationsUtil::add((selected_size > 1 ? "RemoveContactsFromSet" : "RemoveContactFromSet"), args, payload, &LGGContactSets::handleRemoveAvatarFromSetCallback);
}

void FSPanelContactSets::onClickAddSet()
{
	LLNotificationsUtil::add("AddNewContactSet", LLSD(), LLSD(), &LGGContactSets::handleAddContactSetCallback);
}

void FSPanelContactSets::onClickRemoveSet()
{
	LLSD payload, args;
	std::string set = mContactSetCombo->getValue().asString();
	args["SET_NAME"] = set;
	payload["contact_set"] = set;
	LLNotificationsUtil::add("RemoveContactSet", args, payload, &LGGContactSets::handleRemoveContactSetCallback);
}

void FSPanelContactSets::onClickConfigureSet(LLUICtrl* ctrl)
{
	LLFloater* root_floater = gFloaterView->getParentFloater(this);
	FSFloaterContactSetConfiguration* config_floater = LLFloaterReg::showTypedInstance<FSFloaterContactSetConfiguration>("fs_contact_set_config", LLSD(mContactSetCombo->getValue().asString()));
	config_floater->setFrustumOrigin(ctrl);
	if (root_floater && config_floater)
	{
		root_floater->addDependentFloater(config_floater);
	}
}

void FSPanelContactSets::onClickOpenProfile()
{
	for (auto const& id : mAvatarSelections)
	{
		LLAvatarActions::showProfile(id);
	}
}

void FSPanelContactSets::onClickStartIM()
{
	if ( mAvatarSelections.size() == 1 )
	{
		LLAvatarActions::startIM(mAvatarSelections[0]);
	}
	else if ( mAvatarSelections.size() > 1 )
	{
		LLAvatarActions::startConference(mAvatarSelections);
	}
}

void FSPanelContactSets::onClickOfferTeleport()
{
	LLAvatarActions::offerTeleport(mAvatarSelections);
}

void FSPanelContactSets::onClickSetPseudonym()
{
	LLSD payload, args;
	args["AVATAR"] = LLSLURL("agent", mAvatarSelections.front(), "about").getSLURLString();
	payload["id"] = mAvatarSelections.front();
	LLNotificationsUtil::add("SetAvatarPseudonym", args, payload, &LGGContactSets::handleSetAvatarPseudonymCallback);
}

void FSPanelContactSets::onClickRemovePseudonym()
{
	for (auto const& id : mAvatarSelections)
	{
		if (LGGContactSets::getInstance()->hasPseudonym(id))
		{
			LGGContactSets::getInstance()->clearPseudonym(id);
		}
	}
}

void FSPanelContactSets::onClickRemoveDisplayName()
{
	for (auto const& id : mAvatarSelections)
	{
		if (!LGGContactSets::getInstance()->hasDisplayNameRemoved(id))
		{
			LGGContactSets::getInstance()->removeDisplayName(id);
		}
	}
}
