/**
 * @file pieseparator.h
 * @brief Pie menu separator slice class
 *
 * $LicenseInfo:firstyear=2001&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2010, Linden Research, Inc.
 * Copyright (C) 2011, Zi Ree @ Second Life
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

#ifndef PIESEPARATOR_H
#define PIESEPARATOR_H

#include "lluictrl.h"

// a pie slice that does nothing and is not highlighting by mouse hover
class PieSeparator : public LLUICtrl
{
public:
	// parameter block for the XUI factory
	struct Params : public LLInitParam::Block<Params, LLUICtrl::Params>
	{
		Params();
	};

	PieSeparator(const Params& p);
};

#endif // PIESEPARATOR_H
