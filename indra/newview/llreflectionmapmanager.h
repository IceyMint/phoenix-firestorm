/**
 * @file llreflectionmapmanager.h
 * @brief LLReflectionMapManager class declaration
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

#pragma once

#include "llreflectionmap.h"
#include "llrendertarget.h"
#include "llcubemaparray.h"

class LLSpatialGroup;

// number of reflection probes to keep in vram
#define LL_REFLECTION_PROBE_COUNT 256

// reflection probe resolution
#define LL_REFLECTION_PROBE_RESOLUTION 256

class alignas(16) LLReflectionMapManager
{
    LL_ALIGN_NEW
public:
    // allocate an environment map of the given resolution 
    LLReflectionMapManager();
    
    // maintain reflection probes
    void update();

    // drop a reflection probe at the specified position in agent space
    void addProbe(const LLVector3& pos);

    // add a probe for the given spatial group
    LLReflectionMap* addProbe(LLSpatialGroup* group);
    
    // Populate "maps" with the N most relevant Reflection Maps where N is no more than maps.size()
    // If less than maps.size() ReflectionMaps are available, will assign trailing elements to nullptr.
    //  maps -- presized array of Reflection Map pointers
    void getReflectionMaps(std::vector<LLReflectionMap*>& maps);

    // called by LLSpatialGroup constructor
    // If spatial group should receive a Reflection Probe, will create one for the specified spatial group
    LLReflectionMap* registerSpatialGroup(LLSpatialGroup* group);

    // force an update of all probes
    void rebuild();

    // called on region crossing to "shift" probes into new coordinate frame
    void shift(const LLVector4a& offset);

private:
    friend class LLPipeline;

    // delete the probe with the given index in mProbes
    void deleteProbe(U32 i);

    // get a free cube index
    // if no cube indices are free, free one starting from the back of the probe list
    S32 allocateCubeIndex();

    // update the neighbors of the given probe 
    void updateNeighbors(LLReflectionMap* probe);

    void setUniforms();

    // render target for cube snapshots
    // used to generate mipmaps without doing a copy-to-texture
    LLRenderTarget mRenderTarget;

    std::vector<LLRenderTarget> mMipChain;

    // storage for reflection probes
    LLPointer<LLCubeMapArray> mTexture;

    // array indicating if a particular cubemap is free
    bool mCubeFree[LL_REFLECTION_PROBE_COUNT];

    // start tracking the given spatial group
    void trackGroup(LLSpatialGroup* group);
    
    // perform an update on the currently updating Probe
    void doProbeUpdate();
    
    // list of active reflection maps
    std::vector<LLPointer<LLReflectionMap> > mProbes;

    // list of reflection maps to kill
    std::vector<LLPointer<LLReflectionMap> > mKillList;

    // list of reflection maps to create
    std::vector<LLPointer<LLReflectionMap> > mCreateList;

    // handle to UBO
    U32 mUBO = 0;

    // list of maps being used for rendering
    std::vector<LLReflectionMap*> mReflectionMaps;

    LLReflectionMap* mUpdatingProbe = nullptr;
    U32 mUpdatingFace = 0;

};

