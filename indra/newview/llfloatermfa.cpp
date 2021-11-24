/**
 * @file llfloatermfa.cpp
 * @brief Multi-Factor Auth token submission dialog
 *
 * $LicenseInfo:firstyear=2003&license=viewerlgpl$
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

#include "llfloatermfa.h"

// viewer includes
#include "llevents.h"


LLFloaterMFA::LLFloaterMFA(const LLSD& data)
:   LLModalDialog("mfa_challenge"),
    mMessage(data["message"].asStringRef()),
    mReplyPumpName(data["reply_pump"].asStringRef())
{

}

LLFloaterMFA::~LLFloaterMFA()
{
}

BOOL LLFloaterMFA::postBuild()
{
    childSetAction("Continue", onContinue, this);
    childSetAction("Cancel", onCancel, this);

    if (hasChild("token_prompt"))
    {
        // this displays the prompt message
        LLUICtrl *token_prompt = getChild<LLUICtrl>("token_prompt");
        token_prompt->setEnabled( FALSE );
        token_prompt->setFocus(TRUE);
        token_prompt->setValue(LLSD(mMessage));

        return TRUE;
    }

    return TRUE;
}

// static
void LLFloaterMFA::onContinue( void* userdata )
{
    LLFloaterMFA* self = (LLFloaterMFA*) userdata;
    LL_INFOS("MFA") << "User submits MFA token for challenge." << LL_ENDL;

    std::string token{"8675309"};

    if (self->hasChild("token"))
    {
        // this displays the prompt message
        LLUICtrl *token_ctrl = self->getChild<LLUICtrl>("token");

        token = token_ctrl->getValue().asStringRef();
    }

    if(self->mReplyPumpName != "")
    {
        LLEventPumps::instance().obtain(self->mReplyPumpName).post(LLSD(token));
    }

    self->closeFloater(); // destroys this object
}

// static
void LLFloaterMFA::onCancel( void* userdata )
{
    LLFloaterMFA* self = (LLFloaterMFA*) userdata;
    LL_INFOS("MFA") << "User cancels MFA challenge attempt." << LL_ENDL;

    if(self->mReplyPumpName != "")
    {
        LL_DEBUGS("MFA") << self->mReplyPumpName << LL_ENDL;
        LLEventPumps::instance().obtain(self->mReplyPumpName).post(LLSD());
    }

    // destroys this object
    self->closeFloater();
}
