/** 
 * @file llkeyconflict.cpp
 * @brief 
 *
 * $LicenseInfo:firstyear=2019&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2019, Linden Research, Inc.
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

/*
 * App-wide preferences.  Note that these are not per-user,
 * because we need to load many preferences before we have
 * a login name.
 */

#include "llviewerprecompiledheaders.h"

#include "llkeyconflict.h"

#include "llinitparam.h"
#include "llkeyboard.h"
#include "llviewercontrol.h"
#include "llviewerinput.h"
#include "llxuiparser.h"
//#include "llstring.h"

static const std::string saved_settings_key_controls[] = { "placeholder" };


// LLKeyboard::stringFromMask is meant for UI and is OS dependent,
// so this class uses it's own version
std::string string_from_mask(MASK mask)
{
    std::string res;
    if ((mask & MASK_CONTROL) != 0)
    {
        res = "CTL";
    }
    if ((mask & MASK_ALT) != 0)
    {
        if (!res.empty()) res += "_";
        res += "ALT";
    }
    if ((mask & MASK_SHIFT) != 0)
    {
        if (!res.empty()) res += "_";
        res += "SHIFT";
    }

    if (mask == MASK_NONE)
    {
        res = "NONE";
    }
    return res;
}

std::string string_from_mouse(EMouseClickType click)
{
    std::string res;
    switch (click)
    {
    case CLICK_LEFT:
        res = "LMB";
        break;
    case CLICK_MIDDLE:
        res = "MMB";
        break;
    case CLICK_RIGHT:
        res = "RMB";
        break;
    case CLICK_BUTTON4:
        res = "MB4";
        break;
    case CLICK_BUTTON5:
        res = "MB5";
        break;
    case CLICK_DOUBLELEFT:
        res = "Double LMB";
        break;
    default:
        break;
    }
    return res;
}

EMouseClickType mouse_from_string(const std::string& input)
{
    if (input == "LMB")
    {
        return CLICK_LEFT;
    }
    if (input == "MMB")
    {
        return CLICK_MIDDLE;
    }
    if (input == "RMB")
    {
        return CLICK_RIGHT;
    }
    if (input == "MB4")
    {
        return CLICK_BUTTON4;
    }
    if (input == "MB5")
    {
        return CLICK_BUTTON5;
    }
    return CLICK_NONE;
}

// LLKeyConflictHandler

LLKeyConflictHandler::LLKeyConflictHandler()
    : mHasUnsavedChanges(false)
{
}

LLKeyConflictHandler::LLKeyConflictHandler(ESourceMode mode)
    : mHasUnsavedChanges(false),
    mLoadMode(mode)
{
    loadFromSettings(mode);
}

bool LLKeyConflictHandler::canHandleControl(const std::string &control_name, EMouseClickType mouse_ind, KEY key, MASK mask)
{
    return mControlsMap[control_name].canHandle(mouse_ind, key, mask);
}

bool LLKeyConflictHandler::canHandleKey(const std::string &control_name, KEY key, MASK mask)
{
    return canHandleControl(control_name, CLICK_NONE, key, mask);
}

bool LLKeyConflictHandler::canHandleMouse(const std::string &control_name, EMouseClickType mouse_ind, MASK mask)
{
    return canHandleControl(control_name, mouse_ind, KEY_NONE, mask);
}

bool LLKeyConflictHandler::canHandleMouse(const std::string &control_name, S32 mouse_ind, MASK mask)
{
    return canHandleControl(control_name, (EMouseClickType)mouse_ind, KEY_NONE, mask);
}

bool LLKeyConflictHandler::canAssignControl(const std::string &control_name)
{
    control_map_t::iterator iter = mControlsMap.find(control_name);
    if (iter != mControlsMap.end())
    {
        return iter->second.mAssignable;
    }
    // If we don't know this control means it wasn't assigned by user yet and thus is editable
    return true;
}

bool LLKeyConflictHandler::registerControl(const std::string &control_name, U32 index, EMouseClickType mouse, KEY key, MASK mask, bool ignore_mask)
{
    if (control_name.empty())
    {
        return false;
    }
    LLKeyConflict &type_data = mControlsMap[control_name];
    if (!type_data.mAssignable)
    {
        LL_ERRS() << "Error in code, user or system should not be able to change certain controls" << LL_ENDL;
    }
    LLKeyData data(mouse, key, mask, ignore_mask);
    if (type_data.mKeyBind.getKeyData(index) == data)
    {
        return true;
    }
    if (removeConflicts(data, type_data.mConflictMask))
    {
        type_data.mKeyBind.replaceKeyData(data, index);
        mHasUnsavedChanges = true;
        return true;
    }
    // control already in use/blocked
    return false;
}

LLKeyData LLKeyConflictHandler::getControl(const std::string &control_name, U32 index)
{
    if (control_name.empty())
    {
        return LLKeyData();
    }
    return mControlsMap[control_name].getKeyData(index);
}

// static
std::string LLKeyConflictHandler::getStringFromKeyData(const LLKeyData& keydata)
{
    std::string result;

    if (keydata.mMask != MASK_NONE && keydata.mKey != KEY_NONE)
    {
        result = LLKeyboard::stringFromAccelerator(keydata.mMask, keydata.mKey);
    }
    else if (keydata.mKey != KEY_NONE)
    {
        result = LLKeyboard::stringFromKey(keydata.mKey);
    }
    else if (keydata.mMask != MASK_NONE)
    {
        result = LLKeyboard::stringFromAccelerator(keydata.mMask);
    }
    else if (keydata.mIgnoreMasks)
    {
        result = "acc+";
    }

    result += string_from_mouse(keydata.mMouse);

    return result;
}

std::string LLKeyConflictHandler::getControlString(const std::string &control_name, U32 index)
{
    if (control_name.empty())
    {
        return "";
    }
    return getStringFromKeyData(mControlsMap[control_name].getKeyData(index));
}

void LLKeyConflictHandler::loadFromControlSettings(const std::string &name)
{
    LLControlVariablePtr var = gSavedSettings.getControl(name);
    if (var)
    {
        LLKeyBind bind(var->getValue());
        LLKeyConflict key(bind, true, 0);
        mControlsMap[name] = key;
    }
}

void LLKeyConflictHandler::loadFromSettings(const LLViewerInput::KeyMode& keymode, control_map_t *destination)
{
    for (LLInitParam::ParamIterator<LLViewerInput::KeyBinding>::const_iterator it = keymode.bindings.begin(),
        end_it = keymode.bindings.end();
        it != end_it;
    ++it)
    {
        KEY key;
        MASK mask;
        EMouseClickType mouse = it->mouse.isProvided() ? mouse_from_string(it->mouse) : CLICK_NONE;
        bool ignore = it->ignore.isProvided() ? it->ignore.getValue() : false;
        if (it->key.getValue().empty())
        {
            key = KEY_NONE;
        }
        else
        {
            LLKeyboard::keyFromString(it->key, &key);
        }
        LLKeyboard::maskFromString(it->mask, &mask);
        // Note: it->command is also the name of UI element, howhever xml we are loading from
        // might not know all the commands, so UI will have to know what to fill by its own
        LLKeyConflict &type_data = (*destination)[it->command];
        type_data.mAssignable = true;
        type_data.mKeyBind.addKeyData(mouse, key, mask, ignore);
    }
}

bool LLKeyConflictHandler::loadFromSettings(const ESourceMode &load_mode, const std::string &filename, control_map_t *destination)
{
    if (filename.empty())
    {
        return false;
    }

    bool res = false;

    LLViewerInput::Keys keys;
    LLSimpleXUIParser parser;

    if (parser.readXUI(filename, keys)
        && keys.validateBlock())
    {
        switch (load_mode)
        {
        case MODE_FIRST_PERSON:
            if (keys.first_person.isProvided())
            {
                loadFromSettings(keys.first_person, destination);
                res = true;
            }
            break;
        case MODE_THIRD_PERSON:
            if (keys.third_person.isProvided())
            {
                loadFromSettings(keys.third_person, destination);
                res = true;
            }
            break;
        case MODE_EDIT:
            if (keys.edit.isProvided())
            {
                loadFromSettings(keys.edit, destination);
                res = true;
            }
            break;
        case MODE_EDIT_AVATAR:
            if (keys.edit_avatar.isProvided())
            {
                loadFromSettings(keys.edit_avatar, destination);
                res = true;
            }
            break;
        case MODE_SITTING:
            if (keys.sitting.isProvided())
            {
                loadFromSettings(keys.sitting, destination);
                res = true;
            }
            break;
        default:
            break;
        }
    }
    return res;
}

void LLKeyConflictHandler::loadFromSettings(ESourceMode load_mode)
{
    mControlsMap.clear();
    mDefaultsMap.clear();

    // E.X. In case we need placeholder keys for conflict resolution.
    generatePlaceholders(load_mode);

    if (load_mode == MODE_SAVED_SETTINGS)
    {
        // load settings clss knows about, but it also possible to load settings by name separately
        const S32 size = std::extent<decltype(saved_settings_key_controls)>::value;
        for (U32 i = 0; i < size; i++)
        {
            loadFromControlSettings(saved_settings_key_controls[i]);
        }
    }
    else
    {
        // load defaults
        std::string filename = gDirUtilp->getExpandedFilename(LL_PATH_APP_SETTINGS, "keys.xml");
        if (!loadFromSettings(load_mode, filename, &mDefaultsMap))
        {
            LL_WARNS() << "Failed to load default settings, aborting" << LL_ENDL;
            return;
        }

        // load user's
        filename = gDirUtilp->getExpandedFilename(LL_PATH_USER_SETTINGS, "keys.xml");
        if (!gDirUtilp->fileExists(filename) || loadFromSettings(load_mode, filename, &mControlsMap))
        {
            // mind placeholders
            mControlsMap.insert(mDefaultsMap.begin(), mDefaultsMap.end());
        }
    }
    mLoadMode = load_mode;
}

void LLKeyConflictHandler::saveToSettings()
{
    if (mControlsMap.empty())
    {
        return;
    }

    if (mLoadMode == MODE_SAVED_SETTINGS)
    {
        control_map_t::iterator iter = mControlsMap.begin();
        control_map_t::iterator end = mControlsMap.end();

        for (; iter != end; ++iter)
        {
            if (iter->first.empty())
            {
                continue;
            }

            LLKeyConflict &key = iter->second;
            key.mKeyBind.trimEmpty();
            if (!key.mAssignable)
            {
                continue;
            }

            if (gSavedSettings.controlExists(iter->first))
            {
                gSavedSettings.setLLSD(iter->first, key.mKeyBind.asLLSD());
            }
            else if (!key.mKeyBind.empty())
            {
                // Note: this is currently not in use, might be better for load mechanics to ask for and retain control group
                // otherwise settings loaded from other control groups will end in this one
                LL_INFOS() << "Creating new keybinding " << iter->first << LL_ENDL;
                gSavedSettings.declareLLSD(iter->first, key.mKeyBind.asLLSD(), "comment", LLControlVariable::PERSIST_ALWAYS);
            }
        }
    }
    else
    {
        // loaded full copy of original file
        std::string filename = gDirUtilp->findFile("keys.xml",
            gDirUtilp->getExpandedFilename(LL_PATH_USER_SETTINGS, ""),
            gDirUtilp->getExpandedFilename(LL_PATH_APP_SETTINGS, ""));

        LLViewerInput::Keys keys;
        LLSimpleXUIParser parser;

        if (parser.readXUI(filename, keys)
            && keys.validateBlock())
        {
            // replace category we edited

            // mode is a HACK to correctly reset bindings without reparsing whole file and avoid doing
            // own param container (which will face issues with inasseesible members of LLInitParam)
            LLViewerInput::KeyMode mode;
            LLViewerInput::KeyBinding binding;

            control_map_t::iterator iter = mControlsMap.begin();
            control_map_t::iterator end = mControlsMap.end();
            for (; iter != end; ++iter)
            {
                // By default xml have (had) up to 6 elements per function
                // eventually it will be cleaned up and UI will only shows 3 per function,
                // so make sure to cleanup.
                // Also this helps in keeping file small.
                iter->second.mKeyBind.trimEmpty();
                U32 size = iter->second.mKeyBind.getDataCount();
                for (U32 i = 0; i < size; ++i)
                {
                    if (iter->first.empty())
                    {
                        continue;
                    }

                    LLKeyConflict &key = iter->second;
                    key.mKeyBind.trimEmpty();
                    if (key.mKeyBind.empty() || !key.mAssignable)
                    {
                        continue;
                    }

                    LLKeyData data = key.mKeyBind.getKeyData(i);
                    // Still write empty LLKeyData to make sure we will maintain UI position
                    if (data.mKey == KEY_NONE)
                    {
                        binding.key = "";
                    }
                    else
                    {
                        // Note: this is UI string, we might want to hardcode our own for 'fixed' use in keys.xml
                        binding.key = LLKeyboard::stringFromKey(data.mKey);
                    }
                    binding.mask = string_from_mask(data.mMask);
                    binding.mouse.set(string_from_mouse(data.mMouse), true); //set() because 'optional', for compatibility purposes
                    binding.ignore.set(data.mIgnoreMasks, true);
                    binding.command = iter->first;
                    mode.bindings.add(binding);
                }
            }

            switch (mLoadMode)
            {
            case MODE_FIRST_PERSON:
                if (keys.first_person.isProvided())
                {
                    keys.first_person.bindings.set(mode.bindings, true);
                }
                break;
            case MODE_THIRD_PERSON:
                if (keys.third_person.isProvided())
                {
                    keys.third_person.bindings.set(mode.bindings, true);
                }
                break;
            case MODE_EDIT:
                if (keys.edit.isProvided())
                {
                    keys.edit.bindings.set(mode.bindings, true);
                }
                break;
            case MODE_EDIT_AVATAR:
                if (keys.edit_avatar.isProvided())
                {
                    keys.edit_avatar.bindings.set(mode.bindings, true);
                }
                break;
            case MODE_SITTING:
                if (keys.sitting.isProvided())
                {
                    keys.sitting.bindings.set(mode.bindings, true);
                }
                break;
            default:
                break;
            }

            // write back to user's xml;
            std::string filename = gDirUtilp->getExpandedFilename(LL_PATH_USER_SETTINGS, "keys.xml");

            LLXMLNodePtr output_node = new LLXMLNode("keys", false);
            LLXUIParser parser;
            parser.writeXUI(output_node, keys);

            // Write the resulting XML to file
            if (!output_node->isNull())
            {
                LLFILE *fp = LLFile::fopen(filename, "w");
                if (fp != NULL)
                {
                    LLXMLNode::writeHeaderToFile(fp);
                    output_node->writeToFile(fp);
                    fclose(fp);
                }
            }
            // Now force a rebind for keyboard
            if (gDirUtilp->fileExists(filename))
            {
                gViewerInput.loadBindingsXML(filename);
            }
        }
    }
    mHasUnsavedChanges = false;
}

LLKeyData LLKeyConflictHandler::getDefaultControl(const std::string &control_name, U32 index)
{
    if (control_name.empty())
    {
        return LLKeyData();
    }
    if (mLoadMode == MODE_SAVED_SETTINGS)
    {
        LLControlVariablePtr var = gSavedSettings.getControl(control_name);
        if (var)
        {
            return LLKeyBind(var->getDefault()).getKeyData(index);
        }
        return LLKeyData();
    }
    else
    {
        control_map_t::iterator iter = mDefaultsMap.find(control_name);
        if (iter != mDefaultsMap.end())
        {
            return iter->second.mKeyBind.getKeyData(index);
        }
        return LLKeyData();
    }
}

void LLKeyConflictHandler::resetToDefault(const std::string &control_name, U32 index)
{
    if (control_name.empty())
    {
        return;
    }
    LLKeyData data = getDefaultControl(control_name, index);

    if (data != mControlsMap[control_name].getKeyData(index))
    {
        // reset controls that might have been switched to our current control
        removeConflicts(data, mControlsMap[control_name].mConflictMask);
        mControlsMap[control_name].setKeyData(data, index);
    }
}

void LLKeyConflictHandler::resetToDefaultAndResolve(const std::string &control_name, bool ignore_conflicts)
{
    if (control_name.empty())
    {
        return;
    }
    if (mLoadMode == MODE_SAVED_SETTINGS)
    {
        LLControlVariablePtr var = gSavedSettings.getControl(control_name);
        if (var)
        {
            LLKeyBind bind(var->getDefault());
            if (!ignore_conflicts)
            {
                for (S32 i = 0; i < bind.getDataCount(); ++i)
                {
                    removeConflicts(bind.getKeyData(i), mControlsMap[control_name].mConflictMask);
                }
            }
            mControlsMap[control_name].mKeyBind = bind;
        }
        else
        {
            mControlsMap[control_name].mKeyBind.clear();
        }
    }
    else
    {
        control_map_t::iterator iter = mDefaultsMap.find(control_name);
        if (iter != mDefaultsMap.end())
        {
            if (!ignore_conflicts)
            {
                for (S32 i = 0; i < iter->second.mKeyBind.getDataCount(); ++i)
                {
                    removeConflicts(iter->second.mKeyBind.getKeyData(i), mControlsMap[control_name].mConflictMask);
                }
            }
            mControlsMap[control_name].mKeyBind = iter->second.mKeyBind;
        }
        else
        {
            mControlsMap[control_name].mKeyBind.clear();
        }
    }
}

void LLKeyConflictHandler::resetToDefault(const std::string &control_name)
{
    // reset specific binding without ignoring conflicts
    resetToDefaultAndResolve(control_name, false);
}

void LLKeyConflictHandler::resetToDefaults(ESourceMode mode)
{
    if (mode == MODE_SAVED_SETTINGS)
    {
        control_map_t::iterator iter = mControlsMap.begin();
        control_map_t::iterator end = mControlsMap.end();

        for (; iter != end; ++iter)
        {
            resetToDefaultAndResolve(iter->first, true);
        }
    }
    else
    {
        mControlsMap.clear();
        generatePlaceholders(mode);
        mControlsMap.insert(mDefaultsMap.begin(), mDefaultsMap.end());
    }

    mHasUnsavedChanges = true;
}

void LLKeyConflictHandler::resetToDefaults()
{
    if (!empty())
    {
        resetToDefaults(mLoadMode);
    }
    else
    {
        // not optimal since:
        // 1. We are not sure that mLoadMode was set
        // 2. We are not sure if there are any changes in comparison to default
        // 3. We are loading 'current' only to replace it
        // but it is reliable and works Todo: consider optimizing.
        loadFromSettings(mLoadMode);
        resetToDefaults(mLoadMode);
    }
}

void LLKeyConflictHandler::clear()
{
    mHasUnsavedChanges = false;
    mControlsMap.clear();
    mDefaultsMap.clear();
}

void LLKeyConflictHandler::resetKeyboardBindings()
{
    std::string filename = gDirUtilp->findFile("keys.xml",
        gDirUtilp->getExpandedFilename(LL_PATH_USER_SETTINGS, ""),
        gDirUtilp->getExpandedFilename(LL_PATH_APP_SETTINGS, ""));
    
    gViewerInput.loadBindingsXML(filename);
}

void LLKeyConflictHandler::generatePlaceholders(ESourceMode load_mode)
{
    // These controls are meant to cause conflicts when user tries to assign same control somewhere else
    // also this can be used to pre-record controls that should not conflict or to assign conflict groups/masks
    /*registerTemporaryControl(CONTROL_RESERVED_MENU, CLICK_RIGHT, KEY_NONE, MASK_NONE, 0);
    registerTemporaryControl(CONTROL_DELETE, CLICK_NONE, KEY_DELETE, MASK_NONE, 0);*/
}

bool LLKeyConflictHandler::removeConflicts(const LLKeyData &data, const U32 &conlict_mask)
{
    if (conlict_mask == CONFLICT_NOTHING)
    {
        // Can't conflict
        return true;
    }
    std::map<std::string, S32> conflict_list;
    control_map_t::iterator cntrl_iter = mControlsMap.begin();
    control_map_t::iterator cntrl_end = mControlsMap.end();
    for (; cntrl_iter != cntrl_end; ++cntrl_iter)
    {
        S32 index = cntrl_iter->second.mKeyBind.findKeyData(data);
        if (index >= 0
            && cntrl_iter->second.mConflictMask != CONFLICT_NOTHING
            && (cntrl_iter->second.mConflictMask & conlict_mask) != 0)
        {
            if (cntrl_iter->second.mAssignable)
            {
                // Potentially we can have multiple conflict flags conflicting
                // including unassignable keys.
                // So record the conflict and find all others before doing any changes.
                // Assume that there is only one conflict per bind
                conflict_list[cntrl_iter->first] = index;
            }
            else
            {
                return false;
            }
        }
    }

    std::map<std::string, S32>::iterator cnflct_iter = conflict_list.begin();
    std::map<std::string, S32>::iterator cnflct_end = conflict_list.end();
    for (; cnflct_iter != cnflct_end; ++cnflct_iter)
    {
        mControlsMap[cnflct_iter->first].mKeyBind.resetKeyData(cnflct_iter->second);
    }
    return true;
}

void LLKeyConflictHandler::registerTemporaryControl(const std::string &control_name, EMouseClickType mouse, KEY key, MASK mask, U32 conflict_mask)
{
    LLKeyConflict *type_data = &mControlsMap[control_name];
    type_data->mAssignable = false;
    type_data->mConflictMask = conflict_mask;
    type_data->mKeyBind.addKeyData(mouse, key, mask, false);
}

