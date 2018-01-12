/**
 * @file llpresetsmanager.cpp
 * @brief Implementation for the LLPresetsManager class.
 *
 * $LicenseInfo:firstyear=2007&license=viewerlgpl$
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

#include <boost/assign/list_of.hpp>

#include "llpresetsmanager.h"

#include "lldiriterator.h"
#include "llfloater.h"
#include "llsdserialize.h"
#include "lltrans.h"
#include "lluictrlfactory.h"
#include "llviewercontrol.h"
#include "llfloaterpreference.h"
#include "llfloaterreg.h"
#include "llfeaturemanager.h"
#include "llagentcamera.h"

LLPresetsManager::LLPresetsManager()
{
}

LLPresetsManager::~LLPresetsManager()
{
	mCameraChangedSignal.disconnect();
}

void LLPresetsManager::triggerChangeCameraSignal()
{
	mPresetListChangeCameraSignal();
}

void LLPresetsManager::triggerChangeSignal()
{
	mPresetListChangeSignal();
}

void LLPresetsManager::createMissingDefault(const std::string& subdirectory)
{
	if(gDirUtilp->getLindenUserDir().empty())
	{
		return;
	}

	std::string default_file = gDirUtilp->getExpandedFilename(LL_PATH_PER_SL_ACCOUNT, PRESETS_DIR,
															  subdirectory, PRESETS_DEFAULT + ".xml");
	if (!gDirUtilp->fileExists(default_file))
	{
		LL_INFOS() << "No default preset found -- creating one at " << default_file << LL_ENDL;

		// Write current settings as the default
        savePreset(subdirectory, PRESETS_DEFAULT, true);
	}
    else
    {
        LL_DEBUGS() << "default preset exists; no-op" << LL_ENDL;
    }
}

void LLPresetsManager::startWatching(const std::string& subdirectory)
{
	if (PRESETS_CAMERA == subdirectory)
	{
		std::vector<std::string> name_list;
		getControlNames(name_list);

		for (std::vector<std::string>::iterator it = name_list.begin(); it != name_list.end(); ++it)
		{
			std::string ctrl_name = *it;
LL_WARNS() << "DBG starting watch on " << ctrl_name << LL_ENDL;
			if (gSavedSettings.controlExists(ctrl_name))
			{
				LLPointer<LLControlVariable> cntrl_ptr = gSavedSettings.getControl(ctrl_name);
				if (cntrl_ptr.isNull())
				{
					LL_WARNS("Init") << "Unable to set signal on global setting '" << ctrl_name
									<< "'" << LL_ENDL;
				}
				else
				{
					mCameraChangedSignal = cntrl_ptr->getCommitSignal()->connect(boost::bind(&settingChanged));
				}
			}
		}
	}
}

std::string LLPresetsManager::getPresetsDir(const std::string& subdirectory)
{
	std::string presets_path = gDirUtilp->getExpandedFilename(LL_PATH_PER_SL_ACCOUNT, PRESETS_DIR);
	std::string full_path;

	if (!gDirUtilp->fileExists(presets_path))
	{
		LLFile::mkdir(presets_path);
	}

	full_path = gDirUtilp->getExpandedFilename(LL_PATH_PER_SL_ACCOUNT, PRESETS_DIR, subdirectory);
	if (!gDirUtilp->fileExists(full_path))
	{
		LLFile::mkdir(full_path);
	}

	return full_path;
}

void LLPresetsManager::loadPresetNamesFromDir(const std::string& dir, preset_name_list_t& presets, EDefaultOptions default_option)
{
	LL_INFOS("AppInit") << "Loading list of preset names from " << dir << LL_ENDL;

	mPresetNames.clear();

	LLDirIterator dir_iter(dir, "*.xml");
	bool found = true;
	while (found)
	{
		std::string file;
		found = dir_iter.next(file);

		if (found)
		{
			std::string path = gDirUtilp->add(dir, file);
			std::string name = LLURI::unescape(gDirUtilp->getBaseFileName(path, /*strip_exten = */ true));
            LL_DEBUGS() << "  Found preset '" << name << "'" << LL_ENDL;

			if (PRESETS_DEFAULT != name)
			{
				mPresetNames.push_back(name);
			}
			else
			{
				switch (default_option)
				{
					case DEFAULT_SHOW:
						mPresetNames.push_back(LLTrans::getString(PRESETS_DEFAULT));
						break;

					case DEFAULT_TOP:
						mPresetNames.push_front(LLTrans::getString(PRESETS_DEFAULT));
						break;

					case DEFAULT_HIDE:
					default:
						break;
				}
			}
		}
	}

	presets = mPresetNames;
}

bool LLPresetsManager::mCameraDirty = false;

void LLPresetsManager::setCameraDirty(bool dirty)
{
	mCameraDirty = dirty;
}

bool LLPresetsManager::isCameraDirty()
{
	return mCameraDirty;
}

void LLPresetsManager::settingChanged()
{
LL_WARNS() << "DBG setting changed" << LL_ENDL;
	setCameraDirty(true);
}

void LLPresetsManager::getControlNames(std::vector<std::string>& names)
{
	names = boost::assign::list_of
		("CameraOffsetCustom0")
		("FocusOffsetCustom0")
		// From panel_preferences_move.xml
		("CameraAngle")
		("CameraOffsetScale")
		("CameraOpacity")
		("EditCameraMovement")
		("AppearanceCameraMovement")
		// From llagentcamera.cpp
		("RenderFarClip")
		("CameraOffsetBuild")
		("CameraPreset")
		("CameraOffsetRearView")
		("CameraOffsetFrontView")
		("CameraOffsetGroupView")
		("FocusOffsetRearView")
		("FocusOffsetFrontView")
		("FocusOffsetGroupView")
		("CameraOffsetScale")
		("TrackFocusObject");
}

bool LLPresetsManager::savePreset(const std::string& subdirectory, std::string name, bool createDefault)
{
	if (LLTrans::getString(PRESETS_DEFAULT) == name)
	{
		name = PRESETS_DEFAULT;
	}

	bool saved = false;
	std::vector<std::string> name_list;

	if(PRESETS_GRAPHIC == subdirectory)
	{
		LLFloaterPreference* instance = LLFloaterReg::findTypedInstance<LLFloaterPreference>("preferences");
		if (instance && !createDefault)
		{
			gSavedSettings.setString("PresetGraphicActive", name);
			instance->getControlNames(name_list);
			LL_DEBUGS() << "saving preset '" << name << "'; " << name_list.size() << " names" << LL_ENDL;
			name_list.push_back("PresetGraphicActive");
		}
		else
        {
			LL_WARNS() << "preferences floater instance not found" << LL_ENDL;
		}
	}
	else if(PRESETS_CAMERA == subdirectory)
	{
		gSavedSettings.setString("PresetGraphicActive", name);
//		gSavedSettings.setU32("CameraPreset", CAMERA_PRESET_CUSTOM0);

//		gSavedSettings.setVector3d("CameraOffsetCustom0", gAgentCamera.calcCameraPositionTargetGlobal());
//		gSavedSettings.setVector3d("FocusOffsetCustom0", gAgentCamera.calcFocusPositionTargetGlobal());

		getControlNames(name_list);
		name_list.push_back("PresetCameraActive");
	}
	else
	{
		LL_ERRS() << "Invalid presets directory '" << subdirectory << "'" << LL_ENDL;
	}
 
	// make an empty llsd
	LLSD paramsData(LLSD::emptyMap());

	// Create a default graphics preset from hw recommended settings 
	if (createDefault && name == PRESETS_DEFAULT && subdirectory == PRESETS_GRAPHIC)
	{
		paramsData = LLFeatureManager::getInstance()->getRecommendedSettingsMap();
		if (gSavedSettings.getU32("RenderAvatarMaxComplexity") == 0)
		{
			// use the recommended setting as an initial one (MAINT-6435)
			gSavedSettings.setU32("RenderAvatarMaxComplexity", paramsData["RenderAvatarMaxComplexity"]["Value"].asInteger());
		}
	}
	else
	{
		for (std::vector<std::string>::iterator it = name_list.begin(); it != name_list.end(); ++it)
		{
			std::string ctrl_name = *it;
			LLControlVariable* ctrl = gSavedSettings.getControl(ctrl_name).get();
			std::string comment = ctrl->getComment();
			std::string type = LLControlGroup::typeEnumToString(ctrl->type());
			LLSD value = ctrl->getValue();

			paramsData[ctrl_name]["Comment"] = comment;
			paramsData[ctrl_name]["Persist"] = 1;
			paramsData[ctrl_name]["Type"] = type;
			paramsData[ctrl_name]["Value"] = value;
		}
	}

	std::string pathName(getPresetsDir(subdirectory) + gDirUtilp->getDirDelimiter() + LLURI::escape(name) + ".xml");

 // If the active preset name is the only thing in the list, don't save the list
	if (paramsData.size() > 1)
	{
		// write to file
		llofstream presetsXML(pathName.c_str());
		if (presetsXML.is_open())
		{
			LLPointer<LLSDFormatter> formatter = new LLSDXMLFormatter();
			formatter->format(paramsData, presetsXML, LLSDFormatter::OPTIONS_PRETTY);
			presetsXML.close();
			saved = true;
            
			LL_DEBUGS() << "saved preset '" << name << "'; " << paramsData.size() << " parameters" << LL_ENDL;

			if (subdirectory == PRESETS_GRAPHIC)
			{
				gSavedSettings.setString("PresetGraphicActive", name);
				// signal interested parties
				triggerChangeSignal();
			}

			if (subdirectory == PRESETS_CAMERA)
			{
				gSavedSettings.setString("PresetCameraActive", name);
				setCameraDirty(false);
				// signal interested parties
				triggerChangeCameraSignal();
			}
		}
		else
		{
			LL_WARNS("Presets") << "Cannot open for output preset file " << pathName << LL_ENDL;
		}
	}
    else
	{
		LL_INFOS() << "No settings available to be saved" << LL_ENDL;
	}
    
	return saved;
}

bool LLPresetsManager::setPresetNamesInComboBox(const std::string& subdirectory, LLComboBox* combo, EDefaultOptions default_option)
{
	bool sts = true;

	combo->clearRows();

	std::string presets_dir = getPresetsDir(subdirectory);

	if (!presets_dir.empty())
	{
		std::list<std::string> preset_names;
		loadPresetNamesFromDir(presets_dir, preset_names, default_option);

		std::string preset_graphic_active = gSavedSettings.getString("PresetGraphicActive");

		if (preset_names.begin() != preset_names.end())
		{
			for (std::list<std::string>::const_iterator it = preset_names.begin(); it != preset_names.end(); ++it)
			{
				const std::string& name = *it;
				combo->add(name, LLSD().with(0, name));
			}
		}
		else
		{
			combo->setLabel(LLTrans::getString("preset_combo_label"));
			sts = false;
		}
	}
	return sts;
}

void LLPresetsManager::loadPreset(const std::string& subdirectory, std::string name)
{
	if (LLTrans::getString(PRESETS_DEFAULT) == name)
	{
		name = PRESETS_DEFAULT;
	}

	std::string full_path(getPresetsDir(subdirectory) + gDirUtilp->getDirDelimiter() + LLURI::escape(name) + ".xml");

    LL_DEBUGS() << "attempting to load preset '"<<name<<"' from '"<<full_path<<"'" << LL_ENDL;

	if(gSavedSettings.loadFromFile(full_path, false, true) > 0)
	{
		if(PRESETS_GRAPHIC == subdirectory)
		{
			gSavedSettings.setString("PresetGraphicActive", name);

			LLFloaterPreference* instance = LLFloaterReg::findTypedInstance<LLFloaterPreference>("preferences");
			if (instance)
			{
				instance->refreshEnabledGraphics();
			}
			triggerChangeSignal();
		}
		if(PRESETS_CAMERA == subdirectory)
		{
			gSavedSettings.setString("PresetCameraActive", name);
			triggerChangeCameraSignal();
		}
	}
    else
    {
        LL_WARNS() << "failed to load preset '"<<name<<"' from '"<<full_path<<"'" << LL_ENDL;
    }
}

bool LLPresetsManager::deletePreset(const std::string& subdirectory, std::string name)
{
	if (LLTrans::getString(PRESETS_DEFAULT) == name)
	{
		name = PRESETS_DEFAULT;
	}

	bool sts = true;

	if (PRESETS_DEFAULT == name)
	{
		// This code should never execute
		LL_WARNS("Presets") << "You are not allowed to delete the default preset." << LL_ENDL;
		sts = false;
	}

	if (gDirUtilp->deleteFilesInDir(getPresetsDir(subdirectory), LLURI::escape(name) + ".xml") < 1)
	{
		LL_WARNS("Presets") << "Error removing preset " << name << " from disk" << LL_ENDL;
		sts = false;
	}

	// If you delete the preset that is currently marked as loaded then also indicate that no preset is loaded.
	if(PRESETS_GRAPHIC == subdirectory)
	{
		if (gSavedSettings.getString("PresetGraphicActive") == name)
		{
			gSavedSettings.setString("PresetGraphicActive", "");
		}
		// signal interested parties
		triggerChangeSignal();
	}

	if(PRESETS_CAMERA == subdirectory)
	{
		if (gSavedSettings.getString("PresetCameraActive") == name)
		{
			gSavedSettings.setString("PresetCameraActive", "");
		}
		// signal interested parties
		triggerChangeCameraSignal();
	}

	return sts;
}

boost::signals2::connection LLPresetsManager::setPresetListChangeCameraCallback(const preset_list_signal_t::slot_type& cb)
{
	return mPresetListChangeCameraSignal.connect(cb);
}

boost::signals2::connection LLPresetsManager::setPresetListChangeCallback(const preset_list_signal_t::slot_type& cb)
{
	return mPresetListChangeSignal.connect(cb);
}
