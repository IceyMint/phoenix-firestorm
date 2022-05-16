/**
 * @file llreflectionmap.cpp
 * @brief LLReflectionMap class implementation
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

#include "llviewerprecompiledheaders.h"

#include "llreflectionmap.h"
#include "pipeline.h"
#include "llviewerwindow.h"
#include "llviewerregion.h"

extern F32SecondsImplicit gFrameTimeSeconds;

LLReflectionMap::LLReflectionMap()
{
    mLastUpdateTime = gFrameTimeSeconds;
}

void LLReflectionMap::update(U32 resolution, U32 face)
{
    LL_PROFILE_ZONE_SCOPED_CATEGORY_DISPLAY;
    mLastUpdateTime = gFrameTimeSeconds;
    llassert(mCubeArray.notNull());
    llassert(mCubeIndex != -1);
    llassert(LLPipeline::sRenderDeferred);
    
    // make sure we don't walk off the edge of the render target
    while (resolution > gPipeline.mDeferredScreen.getWidth() ||
        resolution > gPipeline.mDeferredScreen.getHeight())
    {
        resolution /= 2;
    }
    gViewerWindow->cubeSnapshot(LLVector3(mOrigin), mCubeArray, mCubeIndex, face);
}

bool LLReflectionMap::shouldUpdate()
{
    const F32 UPDATE_INTERVAL = 10.f; // update no more than this often
    const F32 TIMEOUT_INTERVAL = 30.f; // update no less than this often
    const F32 RENDER_TIMEOUT = 1.f; // don't update if hasn't been used for rendering for this long
    
    if (mLastBindTime > gFrameTimeSeconds - RENDER_TIMEOUT)
    {   
        if (mDirty && mLastUpdateTime < gFrameTimeSeconds - UPDATE_INTERVAL)
        {
            return true;
        }

        if (mLastUpdateTime < gFrameTimeSeconds - TIMEOUT_INTERVAL)
        {
            return true;
        }
    }

    return false;
}

void LLReflectionMap::dirty()
{
    mDirty = true;
    mLastUpdateTime = gFrameTimeSeconds;
}

void LLReflectionMap::autoAdjustOrigin()
{
    LL_PROFILE_ZONE_SCOPED_CATEGORY_DISPLAY;

    if (mGroup)
    {
        const LLVector4a* bounds = mGroup->getBounds();
        auto* node = mGroup->getOctreeNode();

        if (mGroup->getSpatialPartition()->mPartitionType == LLViewerRegion::PARTITION_TERRAIN)
        {
            // for terrain, make probes float a couple meters above the highest point in the surface patch
            mOrigin = bounds[0];
            mOrigin.getF32ptr()[2] += bounds[1].getF32ptr()[2] + 3.f;

            // update radius to encompass bounding box
            LLVector4a d;
            d.setAdd(bounds[0], bounds[1]);
            d.sub(mOrigin);
            mRadius = d.getLength3().getF32();
        }
        else if (mGroup->getSpatialPartition()->mPartitionType == LLViewerRegion::PARTITION_VOLUME)
        {
            // cast a ray towards 8 corners of bounding box
            // nudge origin towards center of empty space

            if (node->isLeaf() || node->getChildCount() > 1 || node->getData().size() > 0)
            { // use center of object bounding box for leaf nodes or nodes with multiple child nodes
                mOrigin = bounds[0];

                LLVector4a start;
                LLVector4a end;

                LLVector4a size = bounds[1];

                LLVector4a corners[] =
                {
                    { 1, 1, 1 },
                    { -1, 1, 1 },
                    { 1, -1, 1 },
                    { -1, -1, 1 },
                    { 1, 1, -1 },
                    { -1, 1, -1 },
                    { 1, -1, -1 },
                    { -1, -1, -1 }
                };

                for (int i = 0; i < 8; ++i)
                {
                    corners[i].mul(size);
                    corners[i].add(bounds[0]);
                }

                LLVector4a extents[2];
                extents[0].setAdd(bounds[0], bounds[1]);
                extents[1].setSub(bounds[0], bounds[1]);

                bool hit = false;
                for (int i = 0; i < 8; ++i)
                {
                    int face = -1;
                    LLVector4a intersection;
                    LLDrawable* drawable = mGroup->lineSegmentIntersect(bounds[0], corners[i], true, false, &face, &intersection);
                    if (drawable != nullptr)
                    {
                        hit = true;
                        update_min_max(extents[0], extents[1], intersection);
                    }
                    else
                    {
                        update_min_max(extents[0], extents[1], corners[i]);
                    }
                }

                if (hit)
                {
                    mOrigin.setAdd(extents[0], extents[1]);
                    mOrigin.mul(0.5f);
                }

                // make sure radius encompasses all objects
                LLSimdScalar r2 = 0.0;
                for (int i = 0; i < 8; ++i)
                {
                    LLVector4a v;
                    v.setSub(corners[i], mOrigin);

                    LLSimdScalar d = v.dot3(v);

                    if (d > r2)
                    {
                        r2 = d;
                    }
                }

                mRadius = llmax(sqrtf(r2.getF32()), 8.f);
            }
            else
            {
                // use center of octree node volume for nodes that are just branches without data
                mOrigin = node->getCenter();

                // update radius to encompass entire octree node volume
                mRadius = node->getSize().getLength3().getF32();

                //mOrigin = bounds[0];
                //mRadius = bounds[1].getLength3().getF32();

            }
        }
    }
}

bool LLReflectionMap::intersects(LLReflectionMap* other)
{
    LLVector4a delta;
    delta.setSub(other->mOrigin, mOrigin);

    F32 dist = delta.dot3(delta).getF32();

    F32 r2 = mRadius + other->mRadius;

    r2 *= r2;

    return dist < r2;
}
