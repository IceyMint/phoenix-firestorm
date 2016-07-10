/** 
 * @file lleventpoll.h
 * @brief LLEvDescription of the LLEventPoll class.
 *
 * $LicenseInfo:firstyear=2006&license=viewerlgpl$
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

#ifndef LL_LLEVENTPOLL_H
#define LL_LLEVENTPOLL_H

#include "boost/move/unique_ptr.hpp"

namespace boost
{
    using ::boost::movelib::unique_ptr; // move unique_ptr into the boost namespace.
}

class LLHost;

namespace LLEventPolling
{
namespace Details
{
    class LLEventPollImpl;
}
}


class LLEventPoll
	///< implements the viewer side of server-to-viewer pushed events.
{
public:
	LLEventPoll(const std::string& pollURL, const LLHost& sender);
		///< Start polling the URL.

	virtual ~LLEventPoll();
		///< will stop polling, canceling any poll in progress.


private:
    // <FS:ND> FIRE-19557; Hold on to LLEventPollImpl while the coroutine runs, otherwise the this pointer can get deleted while the coroutine is still active.
    // boost::unique_ptr<LLEventPolling::Details::LLEventPollImpl>    mImpl;
    boost::shared_ptr<LLEventPolling::Details::LLEventPollImpl>    mImpl;
    // </FS:ND>
};


#endif // LL_LLEVENTPOLL_H
