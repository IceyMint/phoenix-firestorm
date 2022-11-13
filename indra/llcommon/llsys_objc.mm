/**
 * @file llsys_objc.mm
 * @brief obj-c implementation of the system information functions
 *
 * $LicenseInfo:firstyear=2022&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2022, Linden Research, Inc.
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

#import "llsys_objc.h"
#import <AppKit/AppKit.h>

static int intAtStringIndex(NSArray *array, int index)
{
    return int([(NSString *)[array objectAtIndex:index] integerValue]);
}

bool LLGetDarwinOSInfo(int &major, int &minor, int &patch)
{
    if (NSAppKitVersionNumber > NSAppKitVersionNumber10_8)
    {
        NSOperatingSystemVersion osVersion = [[NSProcessInfo processInfo] operatingSystemVersion];
        major = int(osVersion.majorVersion);
        minor = int(osVersion.minorVersion);
        patch = int(osVersion.patchVersion);
    }
    else
    {
        NSString* versionString = [[NSDictionary dictionaryWithContentsOfFile:
                                    @"/System/Library/CoreServices/SystemVersion.plist"] objectForKey:@"ProductVersion"];
        NSArray* versions = [versionString componentsSeparatedByString:@"."];
        NSUInteger count = [versions count];
        if (count > 0)
        {
            major = intAtStringIndex(versions, 0);
            if (count > 1)
            {
                minor = intAtStringIndex(versions, 1);
                if (count > 2)
                {
                    patch = intAtStringIndex(versions, 2);
                }
            }
        }
    }
    return true;
}
