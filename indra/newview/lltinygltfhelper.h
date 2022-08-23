/**
 * @file   lltinygltfhelper.h
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

#include "llgltfmaterial.h"
#include "llpointer.h"
#include "tinygltf/tiny_gltf.h"

class LLImageRaw;
class LLViewerFetchedTexture;

namespace LLTinyGLTFHelper
{
    void setFromModel(LLGLTFMaterial* mat, tinygltf::Model& model);
    LLColor4 getColor(const std::vector<double>& in);
    const tinygltf::Image* getImageFromTextureIndex(const tinygltf::Model& model, S32 texture_index);
    LLImageRaw* getTexture(const std::string& folder, const tinygltf::Model& model, S32 texture_index, std::string& name);
    LLImageRaw* getTexture(const std::string& folder, const tinygltf::Model& model, S32 texture_index);

    void initFetchedTextures(tinygltf::Material& material,
        LLPointer<LLImageRaw>& albedo_img,
        LLPointer<LLImageRaw>& normal_img,
        LLPointer<LLImageRaw>& mr_img,
        LLPointer<LLImageRaw>& emissive_img,
        LLPointer<LLImageRaw>& occlusion_img,
        LLPointer<LLViewerFetchedTexture>& albedo_tex,
        LLPointer<LLViewerFetchedTexture>& normal_tex,
        LLPointer<LLViewerFetchedTexture>& mr_tex,
        LLPointer<LLViewerFetchedTexture>& emissive_tex);
}

