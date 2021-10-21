/** 
 * @file fsperfstats.cpp
 * @brief Stats collection to support perf floater and auto tune
 *
 * $LicenseInfo:firstyear=2021&license=fsviewerlgpl$
 * Phoenix Firestorm Viewer Source Code
 * Copyright (C) 2021, The Phoenix Firestorm Project, Inc.
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
#include "fsperfstats.h"
namespace FSPerfStats
{
    std::atomic<int> 	StatsRecorder::writeBuffer{0};
    bool 	            StatsRecorder::collectionEnabled{true};
	std::array<StatsRecorder::StatsTypeMatrix,2>  StatsRecorder::statsDoubleBuffer{ {} };
    std::array<StatsRecorder::StatsSummaryArray,2> StatsRecorder::max{ {} };
    std::array<StatsRecorder::StatsSummaryArray,2> StatsRecorder::sum{ {} };

}