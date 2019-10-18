/**
 * @file   lleventcoro.cpp
 * @author Nat Goodspeed
 * @date   2009-04-29
 * @brief  Implementation for lleventcoro.
 * 
 * $LicenseInfo:firstyear=2009&license=viewerlgpl$
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

// Precompiled header
#include "linden_common.h"
// associated header
#include "lleventcoro.h"
// STL headers
#include <chrono>
// std headers
// external library headers
#include <boost/fiber/operations.hpp>
// other Linden headers
#include "llsdserialize.h"
#include "llerror.h"
#include "llcoros.h"

namespace
{

/**
 * suspendUntilEventOn() permits a coroutine to temporarily listen on an
 * LLEventPump any number of times. We don't really want to have to ask
 * the caller to label each such call with a distinct string; the whole
 * point of suspendUntilEventOn() is to present a nice sequential interface to
 * the underlying LLEventPump-with-named-listeners machinery. So we'll use
 * LLEventPump::inventName() to generate a distinct name for each
 * temporary listener. On the other hand, because a given coroutine might
 * call suspendUntilEventOn() any number of times, we don't really want to
 * consume an arbitrary number of generated inventName()s: that namespace,
 * though large, is nonetheless finite. So we memoize an invented name for
 * each distinct coroutine instance.
 */
std::string listenerNameForCoro()
{
    // If this coroutine was launched by LLCoros::launch(), find that name.
    std::string name(LLCoros::instance().getName());
    if (! name.empty())
    {
        return name;
    }
    // this is the first time we've been called for this coroutine instance
    name = LLEventPump::inventName("coro");
    LL_INFOS("LLEventCoro") << "listenerNameForCoro(): inventing coro name '"
                            << name << "'" << LL_ENDL;
    return name;
}

/**
 * Implement behavior described for postAndSuspend()'s @a replyPumpNamePath
 * parameter:
 *
 * * If <tt>path.isUndefined()</tt>, do nothing.
 * * If <tt>path.isString()</tt>, @a dest is an LLSD map: store @a value
 *   into <tt>dest[path.asString()]</tt>.
 * * If <tt>path.isInteger()</tt>, @a dest is an LLSD array: store @a
 *   value into <tt>dest[path.asInteger()]</tt>.
 * * If <tt>path.isArray()</tt>, iteratively apply the rules above to step
 *   down through the structure of @a dest. The last array entry in @a
 *   path specifies the entry in the lowest-level structure in @a dest
 *   into which to store @a value.
 *
 * @note
 * In the degenerate case in which @a path is an empty array, @a dest will
 * @em become @a value rather than @em containing it.
 */
void storeToLLSDPath(LLSD& dest, const LLSD& rawPath, const LLSD& value)
{
    if (rawPath.isUndefined())
    {
        // no-op case
        return;
    }

    // Arrange to treat rawPath uniformly as an array. If it's not already an
    // array, store it as the only entry in one.
    LLSD path;
    if (rawPath.isArray())
    {
        path = rawPath;
    }
    else
    {
        path.append(rawPath);
    }

    // Need to indicate a current destination -- but that current destination
    // needs to change as we step through the path array. Where normally we'd
    // use an LLSD& to capture a subscripted LLSD lvalue, this time we must
    // instead use a pointer -- since it must be reassigned.
    LLSD* pdest = &dest;

    // Now loop through that array
    for (LLSD::Integer i = 0; i < path.size(); ++i)
    {
        if (path[i].isString())
        {
            // *pdest is an LLSD map
            pdest = &((*pdest)[path[i].asString()]);
        }
        else if (path[i].isInteger())
        {
            // *pdest is an LLSD array
            pdest = &((*pdest)[path[i].asInteger()]);
        }
        else
        {
            // What do we do with Real or Array or Map or ...?
            // As it's a coder error -- not a user error -- rub the coder's
            // face in it so it gets fixed.
            LL_ERRS("lleventcoro") << "storeToLLSDPath(" << dest << ", " << rawPath << ", " << value
                                   << "): path[" << i << "] bad type " << path[i].type() << LL_ENDL;
        }
    }

    // Here *pdest is where we should store value.
    *pdest = value;
}

} // anonymous

void llcoro::suspend()
{
    boost::this_fiber::yield();
}

void llcoro::suspendUntilTimeout(float seconds)
{
    // The fact that we accept non-integer seconds means we should probably
    // use granularity finer than one second. However, given the overhead of
    // the rest of our processing, it seems silly to use granularity finer
    // than a millisecond.
    boost::this_fiber::sleep_for(std::chrono::milliseconds(long(seconds * 1000)));
}

namespace
{

LLBoundListener postAndSuspendSetup(const std::string& callerName,
                                    const std::string& listenerName,
                                    LLCoros::Promise<LLSD>& promise,
                                    const LLSD& event,
                                    const LLEventPumpOrPumpName& requestPumpP,
                                    const LLEventPumpOrPumpName& replyPumpP,
                                    const LLSD& replyPumpNamePath)
{
    // Get the consuming attribute for THIS coroutine, the one that's about to
    // suspend. Don't call get_consuming() in the lambda body: that would
    // return the consuming attribute for some other coroutine, most likely
    // the main routine.
    bool consuming(LLCoros::get_consuming());
    // listen on the specified LLEventPump with a lambda that will assign a
    // value to the promise, thus fulfilling its future
    llassert_always_msg(replyPumpP, ("replyPump required for " + callerName));
    LLEventPump& replyPump(replyPumpP.getPump());
    LLBoundListener connection(
        replyPump.listen(
            listenerName,
            [&promise, consuming, listenerName](const LLSD& result)
            {
                try
                {
                    promise.set_value(result);
                    // We did manage to propagate the result value to the
                    // (real) listener. If we're supposed to indicate that
                    // we've consumed it, do so.
                    return consuming;
                }
                catch(boost::fibers::promise_already_satisfied & ex)
                {
                    LL_DEBUGS("lleventcoro") << "promise already satisfied in '"
                        << listenerName << "': "  << ex.what() << LL_ENDL;
                    // We could not propagate the result value to the
                    // listener.
                    return false;
                }
            }));
    // skip the "post" part if requestPump is default-constructed
    if (requestPumpP)
    {
        LLEventPump& requestPump(requestPumpP.getPump());
        // If replyPumpNamePath is non-empty, store the replyPump name in the
        // request event.
        LLSD modevent(event);
        storeToLLSDPath(modevent, replyPumpNamePath, replyPump.getName());
        LL_DEBUGS("lleventcoro") << callerName << ": coroutine " << listenerName
                                 << " posting to " << requestPump.getName()
                                 << LL_ENDL;

        // *NOTE:Mani - Removed because modevent could contain user's hashed passwd.
        //                         << ": " << modevent << LL_ENDL;
        requestPump.post(modevent);
    }
    LL_DEBUGS("lleventcoro") << callerName << ": coroutine " << listenerName
                             << " about to wait on LLEventPump " << replyPump.getName()
                             << LL_ENDL;
    return connection;
}

} // anonymous

LLSD llcoro::postAndSuspend(const LLSD& event, const LLEventPumpOrPumpName& requestPump,
                 const LLEventPumpOrPumpName& replyPump, const LLSD& replyPumpNamePath)
{
    LLCoros::Promise<LLSD> promise;
    std::string listenerName(listenerNameForCoro());

    // Store connection into an LLTempBoundListener so we implicitly
    // disconnect on return from this function.
    LLTempBoundListener connection =
        postAndSuspendSetup("postAndSuspend()", listenerName, promise,
                            event, requestPump, replyPump, replyPumpNamePath);

    // declare the future
    LLCoros::Future<LLSD> future = LLCoros::getFuture(promise);
    // calling get() on the future makes us wait for it
    LLSD value(future.get());
    LL_DEBUGS("lleventcoro") << "postAndSuspend(): coroutine " << listenerName
                             << " resuming with " << value << LL_ENDL;
    // returning should disconnect the connection
    return value;
}

LLSD llcoro::postAndSuspendWithTimeout(const LLSD& event,
                                       const LLEventPumpOrPumpName& requestPump,
                                       const LLEventPumpOrPumpName& replyPump,
                                       const LLSD& replyPumpNamePath,
                                       F32 timeout, const LLSD& timeoutResult)
{
    LLCoros::Promise<LLSD> promise;
    std::string listenerName(listenerNameForCoro());

    // Store connection into an LLTempBoundListener so we implicitly
    // disconnect on return from this function.
    LLTempBoundListener connection =
        postAndSuspendSetup("postAndSuspendWithTimeout()", listenerName, promise,
                            event, requestPump, replyPump, replyPumpNamePath);

    // declare the future
    LLCoros::Future<LLSD> future = LLCoros::getFuture(promise);
    // wait for specified timeout
    boost::fibers::future_status status =
        future.wait_for(std::chrono::milliseconds(long(timeout * 1000)));
    // if the future is NOT yet ready, return timeoutResult instead
    if (status == boost::fibers::future_status::timeout)
    {
        LL_DEBUGS("lleventcoro") << "postAndSuspendWithTimeout(): coroutine " << listenerName
                                 << " timed out after " << timeout << " seconds,"
                                 << " resuming with " << timeoutResult << LL_ENDL;
        return timeoutResult;
    }
    else
    {
        llassert_always(status == boost::fibers::future_status::ready);

        // future is now ready, no more waiting
        LLSD value(future.get());
        LL_DEBUGS("lleventcoro") << "postAndSuspendWithTimeout(): coroutine " << listenerName
                                 << " resuming with " << value << LL_ENDL;
        // returning should disconnect the connection
        return value;
    }
}
