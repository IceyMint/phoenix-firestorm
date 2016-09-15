/** 
 * @file llpanelsnapshotlocal.cpp
 * @brief The panel provides UI for saving snapshot to a local folder.
 *
 * $LicenseInfo:firstyear=2011&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2011, Linden Research, Inc.
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

#include "llcombobox.h"
#include "llsidetraypanelcontainer.h"
#include "llsliderctrl.h"
#include "llspinctrl.h"

#include "llfloatersnapshot.h" // FIXME: replace with a snapshot storage model
#include "llpanelsnapshot.h"
#include "llsnapshotlivepreview.h"
#include "llviewercontrol.h" // gSavedSettings
#include "llviewerwindow.h"

#include "llnotificationsutil.h"

/**
 * The panel provides UI for saving snapshot to a local folder.
 */
class LLPanelSnapshotLocal
:	public LLPanelSnapshot
{
	LOG_CLASS(LLPanelSnapshotLocal);

public:
	LLPanelSnapshotLocal();
	/*virtual*/ ~LLPanelSnapshotLocal(); // <FS:Ansariel> Store settings at logout
	/*virtual*/ BOOL postBuild();
	/*virtual*/ void onOpen(const LLSD& key);

private:
	/*virtual*/ std::string getWidthSpinnerName() const		{ return "local_snapshot_width"; }
	/*virtual*/ std::string getHeightSpinnerName() const	{ return "local_snapshot_height"; }
	/*virtual*/ std::string getAspectRatioCBName() const	{ return "local_keep_aspect_check"; }
	/*virtual*/ std::string getImageSizeComboName() const	{ return "local_size_combo"; }
	/*virtual*/ std::string getImageSizePanelName() const	{ return "local_image_size_lp"; }
	/*virtual*/ LLSnapshotModel::ESnapshotFormat getImageFormat() const;
	/*virtual*/ LLSnapshotModel::ESnapshotType getSnapshotType();
	/*virtual*/ void updateControls(const LLSD& info);

	// <FS:Ansariel> Threaded filepickers
	void saveLocalCallback(bool success);

	S32 mLocalFormat;

	void onFormatComboCommit(LLUICtrl* ctrl);
	void onQualitySliderCommit(LLUICtrl* ctrl);
	void onSaveFlyoutCommit(LLUICtrl* ctrl);
};

static LLPanelInjector<LLPanelSnapshotLocal> panel_class("llpanelsnapshotlocal");

LLPanelSnapshotLocal::LLPanelSnapshotLocal()
{
	mLocalFormat = gSavedSettings.getS32("SnapshotFormat");
	mCommitCallbackRegistrar.add("Local.Cancel",	boost::bind(&LLPanelSnapshotLocal::cancel,		this));
}

// virtual
BOOL LLPanelSnapshotLocal::postBuild()
{
	getChild<LLUICtrl>("image_quality_slider")->setCommitCallback(boost::bind(&LLPanelSnapshotLocal::onQualitySliderCommit, this, _1));
	getChild<LLUICtrl>("local_format_combo")->setCommitCallback(boost::bind(&LLPanelSnapshotLocal::onFormatComboCommit, this, _1));
	getChild<LLUICtrl>("save_btn")->setCommitCallback(boost::bind(&LLPanelSnapshotLocal::onSaveFlyoutCommit, this, _1));

	// <FS:Ansariel> Store settings at logout
	getImageSizeComboBox()->setCurrentByIndex(gSavedSettings.getS32("LastSnapshotToDiskResolution"));
	getWidthSpinner()->setValue(gSavedSettings.getS32("LastSnapshotToDiskWidth"));
	getHeightSpinner()->setValue(gSavedSettings.getS32("LastSnapshotToDiskHeight"));
	// </FS:Ansariel>

	return LLPanelSnapshot::postBuild();
}

// virtual
void LLPanelSnapshotLocal::onOpen(const LLSD& key)
{
	// <FS:Ansariel> FIRE-7090: Snapshot format for disk changes when selecting snapshot to inventory or email
	//if(gSavedSettings.getS32("SnapshotFormat") != mLocalFormat)
	//{
	//	getChild<LLComboBox>("local_format_combo")->selectNthItem(mLocalFormat);
	//}
	S32 index = gSavedSettings.getS32("FSSnapshotLocalFormat");
	gSavedSettings.setS32("SnapshotFormat", index);
	getChild<LLComboBox>("local_format_combo")->setCurrentByIndex(index);
	// </FS:Ansariel>

	LLPanelSnapshot::onOpen(key);
}

// virtual
LLSnapshotModel::ESnapshotFormat LLPanelSnapshotLocal::getImageFormat() const
{
	LLSnapshotModel::ESnapshotFormat fmt = LLSnapshotModel::SNAPSHOT_FORMAT_PNG;

	LLComboBox* local_format_combo = getChild<LLComboBox>("local_format_combo");
	const std::string id  = local_format_combo->getValue().asString();
	if (id == "PNG")
	{
		fmt = LLSnapshotModel::SNAPSHOT_FORMAT_PNG;
	}
	else if (id == "JPEG")
	{
		fmt = LLSnapshotModel::SNAPSHOT_FORMAT_JPEG;
	}
	else if (id == "BMP")
	{
		fmt = LLSnapshotModel::SNAPSHOT_FORMAT_BMP;
	}

	return fmt;
}

// virtual
void LLPanelSnapshotLocal::updateControls(const LLSD& info)
{
	LLSnapshotModel::ESnapshotFormat fmt =
		(LLSnapshotModel::ESnapshotFormat) gSavedSettings.getS32("SnapshotFormat");
	getChild<LLComboBox>("local_format_combo")->selectNthItem((S32) fmt);

	const bool show_quality_ctrls = (fmt == LLSnapshotModel::SNAPSHOT_FORMAT_JPEG);
	getChild<LLUICtrl>("image_quality_slider")->setVisible(show_quality_ctrls);
	getChild<LLUICtrl>("image_quality_level")->setVisible(show_quality_ctrls);

	getChild<LLUICtrl>("image_quality_slider")->setValue(gSavedSettings.getS32("SnapshotQuality"));
	updateImageQualityLevel();

	const bool have_snapshot = info.has("have-snapshot") ? info["have-snapshot"].asBoolean() : true;
	getChild<LLUICtrl>("save_btn")->setEnabled(have_snapshot);
}

void LLPanelSnapshotLocal::onFormatComboCommit(LLUICtrl* ctrl)
{
	mLocalFormat = getImageFormat();
	// <FS:Ansariel> FIRE-7090: Snapshot format for disk changes when selecting snapshot to inventory or email
	gSavedSettings.setS32("FSSnapshotLocalFormat", getChild<LLComboBox>("local_format_combo")->getCurrentIndex());

	// will call updateControls()
	LLFloaterSnapshot::getInstance()->notify(LLSD().with("image-format-change", true));
}

void LLPanelSnapshotLocal::onQualitySliderCommit(LLUICtrl* ctrl)
{
	updateImageQualityLevel();

	LLSliderCtrl* slider = (LLSliderCtrl*)ctrl;
	S32 quality_val = llfloor((F32)slider->getValue().asReal());
	LLSD info;
	info["image-quality-change"] = quality_val;
	LLFloaterSnapshot::getInstance()->notify(info);
}

void LLPanelSnapshotLocal::onSaveFlyoutCommit(LLUICtrl* ctrl)
{
	if (ctrl->getValue().asString() == "save as")
	{
		gViewerWindow->resetSnapshotLoc();
	}

	LLFloaterSnapshot* floater = LLFloaterSnapshot::getInstance();

	floater->notify(LLSD().with("set-working", true));
	// <FS:Ansariel> Threaded filepickers
	//BOOL saved = floater->saveLocal();
	//if (saved)
	//{
	//	LLFloaterSnapshot::postSave();
	//	floater->notify(LLSD().with("set-finished", LLSD().with("ok", true).with("msg", "local")));
	//}
	//else
	//{
	//	cancel();
	//}
	floater->saveLocal(boost::bind(&LLPanelSnapshotLocal::saveLocalCallback, this, _1));
	// </FS:Ansariel>
}

// <FS:Ansariel> Store settings at logout
LLPanelSnapshotLocal::~LLPanelSnapshotLocal()
{
	gSavedSettings.setS32("LastSnapshotToDiskResolution", getImageSizeComboBox()->getCurrentIndex());
	gSavedSettings.setS32("LastSnapshotToDiskWidth", getTypedPreviewWidth());
	gSavedSettings.setS32("LastSnapshotToDiskHeight", getTypedPreviewHeight());
}
// </FS:Ansariel>

// <FS:Ansariel> Threaded filepickers
void LLPanelSnapshotLocal::saveLocalCallback(bool success)
{
	LLFloaterSnapshot* floater = LLFloaterSnapshot::getInstance();

	if (success)
	{
		mSnapshotFloater->postSave();
		floater->notify(LLSD().with("set-finished", LLSD().with("ok", true).with("msg", "local")));
	}
	else
	{
		LLNotificationsUtil::add("CannotSaveSnapshot");
		floater->notify(LLSD().with("set-ready", true));
	}
}

LLSnapshotModel::ESnapshotType LLPanelSnapshotLocal::getSnapshotType()
{
	return LLSnapshotModel::SNAPSHOT_LOCAL;
}
// </FS:Ansariel>
