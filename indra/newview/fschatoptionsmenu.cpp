/** 
 * @file fschatoptionsmenu.cpp
 * @brief Handler for chat options menu
 *
 * $LicenseInfo:firstyear=2015&license=viewerlgpl$
 * Phoenix Firestorm Viewer Source Code
 * Copyright (c) 2015 Ansariel Hiller
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
 * The Phoenix Firestorm Project, Inc., 1831 Oakwood Drive, Fairmont, Minnesota 56031-3225 USA
 * http://www.firestormviewer.org
 * $/LicenseInfo$
 */

#include "llviewerprecompiledheaders.h"

#include "fschatoptionsmenu.h"

#include "fsareasearch.h"
#include "llfloaterreg.h"
#include "llfloatersidepanelcontainer.h"
#include "llviewercontrol.h"

void FSChatOptionsMenu::onMenuItemClick(const LLSD& userdata, LLUICtrl* source)
{
	std::string option = userdata.asString();

	if (option == "blocklist")
	{
		if (gSavedSettings.getBOOL("FSUseStandaloneBlocklistFloater"))
		{
			LLFloaterReg::toggleInstance("fs_blocklist");
		}
		else
		{
			LLPanel* panel = LLFloaterSidePanelContainer::getPanel("people", "panel_people");
			if (!panel)
			{
				return;
			}

			if (panel->isInVisibleChain())
			{
				LLFloaterReg::hideInstance("people");
			}
			else
			{
				LLFloaterSidePanelContainer::showPanel("people", "panel_people", LLSD().with("people_panel_tab_name", "blocked_panel"));
			}
		}
	}
}

bool FSChatOptionsMenu::onMenuItemEnable(const LLSD& userdata, LLUICtrl* source)
{
	return false;
}

bool FSChatOptionsMenu::onMenuItemVisible(const LLSD& userdata, LLUICtrl* source)
{
	return false;
}

bool FSChatOptionsMenu::onMenuItemCheck(const LLSD& userdata, LLUICtrl* source)
{
	std::string option = userdata.asString();

	if (option == "blocklist")
	{
		if (gSavedSettings.getBOOL("FSUseStandaloneBlocklistFloater"))
		{
			return LLFloaterReg::instanceVisible("fs_blocklist");
		}
	}

	return false;
}
