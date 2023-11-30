/**
 * @file llprofiler_ategories.h
 * @brief Profiling categories to minimize Tracy memory usage when viewing captures.
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

#ifndef LL_PROFILER_CATEGORIES_H
#define LL_PROFILER_CATEGORIES_H

// A Tracy capture can quickly consume memory.  Use these defines to selectively turn on/off Tracy profiling for these categories.
// The biggest memory usage ones are:
//
//    LL_PROFILER_CATEGORY_ENABLE_DRAWPOOL
//    LL_PROFILER_CATEGORY_ENABLE_LLSD
//    LL_PROFILER_CATEGORY_ENABLE_MEMORY
//    LL_PROFILER_CATEGORY_ENABLE_SHADERS
//
// NOTE: You can still manually use:
//     LL_PROFILE_ZONE_SCOPED();
//     LL_PROFILE_ZONE_NAMED("name");
// but just be aware that those will ALWAYS show up in a Tracy capture
//  a) using more memory, and
//  b) adding visual clutter.
#define LL_PROFILER_CATEGORY_ENABLE_APP         1
#define LL_PROFILER_CATEGORY_ENABLE_AVATAR      1
#define LL_PROFILER_CATEGORY_ENABLE_DISPLAY     1
#define LL_PROFILER_CATEGORY_ENABLE_DRAWABLE    1
#define LL_PROFILER_CATEGORY_ENABLE_DRAWPOOL    1
#define LL_PROFILER_CATEGORY_ENABLE_ENVIRONMENT 1
#define LL_PROFILER_CATEGORY_ENABLE_FACE        1
#define LL_PROFILER_CATEGORY_ENABLE_LLSD        1
#define LL_PROFILER_CATEGORY_ENABLE_LOGGING     1
#define LL_PROFILER_CATEGORY_ENABLE_MATERIAL    1
#define LL_PROFILER_CATEGORY_ENABLE_MEDIA       1
#define LL_PROFILER_CATEGORY_ENABLE_MEMORY      0
#define LL_PROFILER_CATEGORY_ENABLE_NETWORK     1
#define LL_PROFILER_CATEGORY_ENABLE_OCTREE      1
#define LL_PROFILER_CATEGORY_ENABLE_PIPELINE    1
#define LL_PROFILER_CATEGORY_ENABLE_SHADER      1
#define LL_PROFILER_CATEGORY_ENABLE_SPATIAL     1
#define LL_PROFILER_CATEGORY_ENABLE_STATS       1
#define LL_PROFILER_CATEGORY_ENABLE_STRING      1
#define LL_PROFILER_CATEGORY_ENABLE_TEXTURE     1
#define LL_PROFILER_CATEGORY_ENABLE_THREAD      1
#define LL_PROFILER_CATEGORY_ENABLE_UI          1
#define LL_PROFILER_CATEGORY_ENABLE_VIEWER      1
#define LL_PROFILER_CATEGORY_ENABLE_VERTEX      1
#define LL_PROFILER_CATEGORY_ENABLE_VOLUME      1
#define LL_PROFILER_CATEGORY_ENABLE_WIN32       1

#if LL_PROFILER_CATEGORY_ENABLE_APP
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_APP  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_APP LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_APP(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_APP
#endif

#if LL_PROFILER_CATEGORY_ENABLE_AVATAR
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_AVATAR  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_AVATAR LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_AVATAR(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_AVATAR
#endif

#if LL_PROFILER_CATEGORY_ENABLE_DISPLAY
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_DISPLAY  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_DISPLAY LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_DISPLAY(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_DISPLAY
#endif

#if LL_PROFILER_CATEGORY_ENABLE_DRAWABLE
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_DRAWABLE  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_DRAWABLE LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_DRAWABLE(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_DRAWABLE
#endif

#if LL_PROFILER_CATEGORY_ENABLE_DRAWPOOL
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_DRAWPOOL  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_DRAWPOOL LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_DRAWPOOL(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_DRAWPOOL
#endif

#if LL_PROFILER_CATEGORY_ENABLE_ENVIRONMENT
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_ENVIRONMENT  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_ENVIRONMENT LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_ENVIRONMENT(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_ENVIRONMENT
#endif

#if LL_PROFILER_CATEGORY_ENABLE_FACE
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_FACE  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_FACE LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_FACE(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_FACE
#endif

#if LL_PROFILER_CATEGORY_ENABLE_LLSD
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_LLSD  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_LLSD LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_LLSD(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_LLSD
#endif

#if LL_PROFILER_CATEGORY_ENABLE_LOGGING
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_LOGGING  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_LOGGING LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_LOGGING(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_LOGGING
#endif

#if LL_PROFILER_CATEGORY_ENABLE_MATERIAL
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_MATERIAL  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_MATERIAL LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_MATERIAL(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_MATERIAL
#endif

#if LL_PROFILER_CATEGORY_ENABLE_MEDIA
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_MEDIA  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_MEDIA LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_MEDIA(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_MEDIA
#endif

#if LL_PROFILER_CATEGORY_ENABLE_MEMORY
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_MEMORY  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_MEMORY LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_MEMORY(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_MEMORY
#endif

#if LL_PROFILER_CATEGORY_ENABLE_NETWORK
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_NETWORK  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_NETWORK LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_NETWORK(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_NETWORK
#endif

#if LL_PROFILER_CATEGORY_ENABLE_OCTREE
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_OCTREE  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_OCTREE LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_OCTREE(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_OCTREE
#endif

#if LL_PROFILER_CATEGORY_ENABLE_PIPELINE
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_PIPELINE  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_PIPELINE LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_PIPELINE(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_PIPELINE
#endif

#if LL_PROFILER_CATEGORY_ENABLE_SHADER
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_SHADER  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_SHADER LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_SHADER(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_SHADER
#endif

#if LL_PROFILER_CATEGORY_ENABLE_SPATIAL
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_SPATIAL  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_SPATIAL LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_SPATIAL(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_SPATIAL
#endif

#if LL_PROFILER_CATEGORY_ENABLE_STATS
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_STATS  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_STATS LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_STATS(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_STATS
#endif

#if LL_PROFILER_CATEGORY_ENABLE_STRING
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_STRING  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_STRING LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_STRING(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_STRING
#endif

#if LL_PROFILER_CATEGORY_ENABLE_TEXTURE
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_TEXTURE  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_TEXTURE LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_TEXTURE(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_TEXTURE
#endif

#if LL_PROFILER_CATEGORY_ENABLE_THREAD
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_THREAD  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_THREAD LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_THREAD(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_THREAD
#endif

#if LL_PROFILER_CATEGORY_ENABLE_UI
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_UI  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_UI LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_UI(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_UI
#endif

#if LL_PROFILER_CATEGORY_ENABLE_VERTEX
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_VERTEX  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_VERTEX LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_VERTEX(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_VERTEX
#endif

#if LL_PROFILER_CATEGORY_ENABLE_VIEWER
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_VIEWER  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_VIEWER LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_VIEWER(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_VIEWER
#endif

#if LL_PROFILER_CATEGORY_ENABLE_VOLUME
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_VOLUME  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_VOLUME LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_VOLUME(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_VOLUME
#endif

#if LL_PROFILER_CATEGORY_ENABLE_WIN32
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_WIN32  LL_PROFILE_ZONE_NAMED
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_WIN32 LL_PROFILE_ZONE_SCOPED
#else
    #define LL_PROFILE_ZONE_NAMED_CATEGORY_WIN32(name)
    #define LL_PROFILE_ZONE_SCOPED_CATEGORY_WIN32
#endif

#endif // LL_PROFILER_CATEGORIES_H

