/** 
 * @file lldeadmantimer_test.cpp
 * @brief Tests for the LLDeadmanTimer class.
 *
 * $LicenseInfo:firstyear=2013&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2013, Linden Research, Inc.
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

#include "linden_common.h"

#include "../lldeadmantimer.h"
#include "../llsd.h"

#include "../test/lltut.h"

// Convert between floating point time deltas and U64 time deltas.
// Reflects an implementation detail inside lldeadmantimer.cpp

static U64 float_time_to_u64(F64 delta)
{
	return U64(delta * gClockFrequency);
}

static F64 u64_time_to_float(U64 delta)
{
	return delta * gClockFrequencyInv;
}


namespace tut
{

struct deadmantimer_test
{
	deadmantimer_test()
		{
			// LLTimer internals updating
			update_clock_frequencies();
		}
};

typedef test_group<deadmantimer_test> deadmantimer_group_t;
typedef deadmantimer_group_t::object deadmantimer_object_t;
tut::deadmantimer_group_t deadmantimer_instance("LLDeadmanTimer");

// Basic construction test and isExpired() call
template<> template<>
void deadmantimer_object_t::test<1>()
{
	F64 started(42.0), stopped(97.0);
	U64 count(U64L(8));
	LLDeadmanTimer timer(10.0);

	ensure_equals("isExpired() returns false after ctor()", timer.isExpired(started, stopped, count), false);
	ensure_approximately_equals("t1 - isExpired() does not modify started", started, F64(42.0), 2);
	ensure_approximately_equals("t1 - isExpired() does not modify stopped", stopped, F64(97.0), 2);
	ensure_equals("t1 - isExpired() does not modify count", count, U64L(8));
}


// Construct with zero horizon - not useful generally but will be useful in testing
template<> template<>
void deadmantimer_object_t::test<2>()
{
	F64 started(42.0), stopped(97.0);
	U64 count(U64L(8));
	LLDeadmanTimer timer(0.0);			// Zero is pre-expired

	ensure_equals("isExpired() still returns false with 0.0 time ctor()", timer.isExpired(started, stopped, count), false);
}


// "pre-expired" timer - starting a timer with a 0.0 horizon will result in
// expiration on first test.
template<> template<>
void deadmantimer_object_t::test<3>()
{
	F64 started(42.0), stopped(97.0);
	U64 count(U64L(8));
	LLDeadmanTimer timer(0.0);

	timer.start();
	ensure_equals("isExpired() returns true with 0.0 horizon time", timer.isExpired(started, stopped, count), true);
	ensure_approximately_equals("expired timer with no bell ringing has stopped == started", started, stopped, 8);
}


// "pre-expired" timer - bell rings are ignored as we're already expired.
template<> template<>
void deadmantimer_object_t::test<4>()
{
	F64 started(42.0), stopped(97.0);
	U64 count(U64L(8));
	LLDeadmanTimer timer(0.0);
	
	timer.start();
	timer.ringBell(LLTimer::getCurrentClockCount() + float_time_to_u64(1000.0));
	ensure_equals("isExpired() returns true with 0.0 horizon time after bell ring", timer.isExpired(started, stopped, count), true);
	ensure_approximately_equals("ringBell has no impact on expired timer leaving stopped == started", started, stopped, 8);
}


// start() test - unexpired timer reports unexpired
template<> template<>
void deadmantimer_object_t::test<5>()
{
	F64 started(42.0), stopped(97.0);
	U64 count(U64L(8));
	LLDeadmanTimer timer(10.0);
	
	timer.start();
	ensure_equals("isExpired() returns false after starting with 10.0 horizon time", timer.isExpired(started, stopped, count), false);
	ensure_approximately_equals("t5 - isExpired() does not modify started", started, F64(42.0), 2);
	ensure_approximately_equals("t5 - isExpired() does not modify stopped", stopped, F64(97.0), 2);
	ensure_equals("t5 - isExpired() does not modify count", count, U64L(8));
}


// start() test - start in the past but not beyond 1 horizon
template<> template<>
void deadmantimer_object_t::test<6>()
{
	F64 started(42.0), stopped(97.0);
	U64 count(U64L(8));
	LLDeadmanTimer timer(10.0);

	// Would like to do subtraction on current time but can't because
	// the implementation on Windows is zero-based.  We wrap around
	// the backside resulting in a large U64 number.
	
	U64 the_past(LLTimer::getCurrentClockCount());
	U64 now(the_past + float_time_to_u64(5.0));
	timer.start(the_past);
	ensure_equals("isExpired() returns false with 10.0 horizon time starting 5.0 in past", timer.isExpired(started, stopped, count, now), false);
	ensure_approximately_equals("t6 - isExpired() does not modify started", started, F64(42.0), 2);
	ensure_approximately_equals("t6 - isExpired() does not modify stopped", stopped, F64(97.0), 2);
	ensure_equals("t6 - isExpired() does not modify count", count, U64L(8));
}


// start() test - start in the past but well beyond 1 horizon
template<> template<>
void deadmantimer_object_t::test<7>()
{
	F64 started(42.0), stopped(97.0);
	U64 count(U64L(8));
	LLDeadmanTimer timer(10.0);

	// Would like to do subtraction on current time but can't because
	// the implementation on Windows is zero-based.  We wrap around
	// the backside resulting in a large U64 number.
	
	U64 the_past(LLTimer::getCurrentClockCount());
	U64 now(the_past + float_time_to_u64(20.0));
	timer.start(the_past);
	ensure_equals("isExpired() returns true with 10.0 horizon time starting 20.0 in past", timer.isExpired(started, stopped, count, now), true);
	ensure_approximately_equals("starting before horizon still gives equal started / stopped", started, stopped, 8);
}


// isExpired() test - results are read-once.  Probes after first true are false.
template<> template<>
void deadmantimer_object_t::test<8>()
{
	F64 started(42.0), stopped(97.0);
	U64 count(U64L(8));
	LLDeadmanTimer timer(10.0);

	// Would like to do subtraction on current time but can't because
	// the implementation on Windows is zero-based.  We wrap around
	// the backside resulting in a large U64 number.
	
	U64 the_past(LLTimer::getCurrentClockCount());
	U64 now(the_past + float_time_to_u64(20.0));
	timer.start(the_past);
	ensure_equals("t8 - isExpired() returns true with 10.0 horizon time starting 20.0 in past", timer.isExpired(started, stopped, count, now), true);

	started = 42.0;
	stopped = 97.0;
	count = U64L(8);
	ensure_equals("t8 - second isExpired() returns false after true", timer.isExpired(started, stopped, count, now), false);
	ensure_approximately_equals("t8 - 2nd isExpired() does not modify started", started, F64(42.0), 2);
	ensure_approximately_equals("t8 - 2nd isExpired() does not modify stopped", stopped, F64(97.0), 2);
	ensure_equals("t8 - 2nd isExpired() does not modify count", count, U64L(8));
}


// ringBell() test - see that we can keep a timer from expiring
template<> template<>
void deadmantimer_object_t::test<9>()
{
	F64 started(42.0), stopped(97.0);
	U64 count(U64L(8));
	LLDeadmanTimer timer(5.0);

	U64 now(LLTimer::getCurrentClockCount());
	F64 real_start(u64_time_to_float(now));
	timer.start();

	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	ensure_equals("t9 - 5.0 horizon timer has not timed out after 10 1-second bell rings", timer.isExpired(started, stopped, count, now), false);
	F64 last_good_ring(u64_time_to_float(now));

	// Jump forward and expire
	now += float_time_to_u64(10.0);
	ensure_equals("t9 - 5.0 horizon timer expires on 10-second jump", timer.isExpired(started, stopped, count, now), true);
	ensure_approximately_equals("t9 - started matches start() time", started, real_start, 4);
	ensure_approximately_equals("t9 - stopped matches last ringBell() time", stopped, last_good_ring, 4);
	ensure_equals("t9 - 10 good ringBell()s", count, U64L(10));
	ensure_equals("t9 - single read only", timer.isExpired(started, stopped, count, now), false);
}


// restart after expiration test - verify that restarts behave well
template<> template<>
void deadmantimer_object_t::test<10>()
{
	F64 started(42.0), stopped(97.0);
	U64 count(U64L(8));
	LLDeadmanTimer timer(5.0);

	U64 now(LLTimer::getCurrentClockCount());
	F64 real_start(u64_time_to_float(now));
	timer.start();

	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	ensure_equals("t10 - 5.0 horizon timer has not timed out after 10 1-second bell rings", timer.isExpired(started, stopped, count, now), false);
	F64 last_good_ring(u64_time_to_float(now));

	// Jump forward and expire
	now += float_time_to_u64(10.0);
	ensure_equals("t10 - 5.0 horizon timer expires on 10-second jump", timer.isExpired(started, stopped, count, now), true);
	ensure_approximately_equals("t10 - started matches start() time", started, real_start, 4);
	ensure_approximately_equals("t10 - stopped matches last ringBell() time", stopped, last_good_ring, 4);
	ensure_equals("t10 - 10 good ringBell()s", count, U64L(10));
	ensure_equals("t10 - single read only", timer.isExpired(started, stopped, count, now), false);

	// Jump forward and restart
	now += float_time_to_u64(1.0);
	real_start = u64_time_to_float(now);
	timer.start(now);

	// Run a modified bell ring sequence
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	now += float_time_to_u64(1.0);
	timer.ringBell(now);
	ensure_equals("t10 - 5.0 horizon timer has not timed out after 8 1-second bell rings", timer.isExpired(started, stopped, count, now), false);
	last_good_ring = u64_time_to_float(now);

	// Jump forward and expire
	now += float_time_to_u64(10.0);
	ensure_equals("t10 - 5.0 horizon timer expires on 8-second jump", timer.isExpired(started, stopped, count, now), true);
	ensure_approximately_equals("t10 - 2nd started matches start() time", started, real_start, 4);
	ensure_approximately_equals("t10 - 2nd stopped matches last ringBell() time", stopped, last_good_ring, 4);
	ensure_equals("t10 - 8 good ringBell()s", count, U64L(8));
	ensure_equals("t10 - single read only - 2nd start", timer.isExpired(started, stopped, count, now), false);
}


} // end namespace tut
