/** 
 * @file llpanelvolumepulldown.cpp
 * @author Tofu Linden
 * @brief A floater showing the master volume pull-down
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

#include "llviewerprecompiledheaders.h"

#include "llpanelvolumepulldown.h"

// Viewer libs
#include "llviewercontrol.h"
#include "llstatusbar.h"

// Linden libs
#include "llbutton.h"
#include "llcheckboxctrl.h"
#include "lltabcontainer.h"
#include "llfloaterreg.h"
#include "llfloaterpreference.h"
#include "llsliderctrl.h"

#include "llcheckboxctrl.h"
#include "llviewercontrol.h"

/* static */ const F32 LLPanelVolumePulldown::sAutoCloseFadeStartTimeSec = 2.0f;
/* static */ const F32 LLPanelVolumePulldown::sAutoCloseTotalTimeSec = 3.0f;

///----------------------------------------------------------------------------
/// Class LLPanelVolumePulldown
///----------------------------------------------------------------------------

// Default constructor
LLPanelVolumePulldown::LLPanelVolumePulldown()
{
	mHoverTimer.stop();

	/*//<FS:KC> Handled centrally now
	mCommitCallbackRegistrar.add("Vol.setControlFalse", boost::bind(&LLPanelVolumePulldown::setControlFalse, this, _2));
	mCommitCallbackRegistrar.add("Vol.SetSounds", boost::bind(&LLPanelVolumePulldown::onClickSetSounds, this));
	mCommitCallbackRegistrar.add("Vol.updateMediaAutoPlayCheckbox",	boost::bind(&LLPanelVolumePulldown::updateMediaAutoPlayCheckbox, this, _1));
	mCommitCallbackRegistrar.add("Vol.GoAudioPrefs", boost::bind(&LLPanelVolumePulldown::onAdvancedButtonClick, this, _2));
	// <FS:Ansariel> Missing callback function
	mCommitCallbackRegistrar.add("Vol.SetSounds", boost::bind(&LLPanelVolumePulldown::setSounds, this));
	*/

	buildFromFile( "panel_volume_pulldown.xml");
}

BOOL LLPanelVolumePulldown::postBuild()
{
	// <FS:PP> FIRE-9856: Mute sound effects disable plays sound from collisions and plays sound from gestures checkbox not disable after restart/relog
	bool mute_sound_effects = gSavedSettings.getBOOL("MuteSounds");
	bool mute_all_sounds = gSavedSettings.getBOOL("MuteAudio");
	LLCheckBoxCtrl* gesture_audio_play_btn = getChild<LLCheckBoxCtrl>("gesture_audio_play_btn");
	gesture_audio_play_btn->setEnabled(!(mute_sound_effects || mute_all_sounds));
	LLCheckBoxCtrl* collisions_audio_play_btn = getChild<LLCheckBoxCtrl>("collisions_audio_play_btn");
	collisions_audio_play_btn->setEnabled(!(mute_sound_effects || mute_all_sounds));
	// </FS:PP> 

	return LLPanel::postBuild();
}

/*virtual*/
void LLPanelVolumePulldown::onMouseEnter(S32 x, S32 y, MASK mask)
{
	mHoverTimer.stop();
	LLPanel::onMouseEnter(x,y,mask);
}

/*virtual*/
void LLPanelVolumePulldown::onTopLost()
{
	setVisible(FALSE);
}

/*virtual*/
void LLPanelVolumePulldown::onMouseLeave(S32 x, S32 y, MASK mask)
{
	mHoverTimer.start();
	LLPanel::onMouseLeave(x,y,mask);
}

/*virtual*/ 
void LLPanelVolumePulldown::onVisibilityChange ( BOOL new_visibility )
{
	if (new_visibility)	
	{
		mHoverTimer.start(); // timer will be stopped when mouse hovers over panel
	}
	else
	{
		mHoverTimer.stop();

	}
}

//<FS:KC> Handled centrally now
/*
void LLPanelVolumePulldown::onAdvancedButtonClick(const LLSD& user_data)
{
	// close the global volume minicontrol, we're bringing up the big one
	setVisible(FALSE);

	// bring up the prefs floater
	LLFloaterPreference* prefsfloater = dynamic_cast<LLFloaterPreference*>
		(LLFloaterReg::showInstance("preferences"));
	if (prefsfloater)
	{
		// grab the 'audio' panel from the preferences floater and
		// bring it the front!
		LLTabContainer* tabcontainer = prefsfloater->getChild<LLTabContainer>("pref core");
		LLPanel* audiopanel = prefsfloater->getChild<LLPanel>("audio");
		if (tabcontainer && audiopanel)
		{
			tabcontainer->selectTabPanel(audiopanel);
		}
	}
}

void LLPanelVolumePulldown::setControlFalse(const LLSD& user_data)
{
	std::string control_name = user_data.asString();
	LLControlVariable* control = findControl(control_name);
	
	if (control)
		control->set(LLSD(FALSE));
}

void LLPanelVolumePulldown::updateMediaAutoPlayCheckbox(LLUICtrl* ctrl)
{
	std::string name = ctrl->getName();

	// Disable "Allow Media to auto play" only when both
	// "Streaming Music" and "Media" are unchecked. STORM-513.
	if ((name == "enable_music") || (name == "enable_media"))
	{
		bool music_enabled = getChild<LLCheckBoxCtrl>("enable_music")->get();
		bool media_enabled = getChild<LLCheckBoxCtrl>("enable_media")->get();

		getChild<LLCheckBoxCtrl>("media_auto_play_btn")->setEnabled(music_enabled || media_enabled);
	}
}

void LLPanelVolumePulldown::onClickSetSounds()
{
	// Disable Enable gesture sounds checkbox if the master sound is disabled 
	// or if sound effects are disabled.
	getChild<LLCheckBoxCtrl>("gesture_audio_play_btn")->setEnabled(!gSavedSettings.getBOOL("MuteSounds"));
}
*/

//virtual
void LLPanelVolumePulldown::draw()
{
	F32 alpha = mHoverTimer.getStarted() 
		? clamp_rescale(mHoverTimer.getElapsedTimeF32(), sAutoCloseFadeStartTimeSec, sAutoCloseTotalTimeSec, 1.f, 0.f)
		: 1.0f;
	LLViewDrawContext context(alpha);

	LLPanel::draw();

	if (alpha == 0.f)
	{
		setVisible(FALSE);
	}
}

//<FS:KC> Handled centrally now
/*
// <FS:Ansariel> Missing callback function
void LLPanelVolumePulldown::setSounds()
{
	// Disable Enable gesture/collisions sounds checkbox if the master sound is disabled
	// or if sound effects are disabled.
	getChild<LLCheckBoxCtrl>("gesture_audio_play_btn")->setEnabled(!gSavedSettings.getBOOL("MuteSounds"));
	getChild<LLCheckBoxCtrl>("collisions_audio_play_btn")->setEnabled(!gSavedSettings.getBOOL("MuteSounds"));
}
// </FS:Ansariel> Missing callback function
*/
