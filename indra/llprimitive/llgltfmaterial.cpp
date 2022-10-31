/**
 * @file llgltfmaterial.cpp
 * @brief Material definition
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

#include "linden_common.h"

#include "llgltfmaterial.h"

#include "tinygltf/tiny_gltf.h"

const char* GLTF_FILE_EXTENSION_TRANSFORM = "KHR_texture_transform";
const char* GLTF_FILE_EXTENSION_TRANSFORM_SCALE = "scale";
const char* GLTF_FILE_EXTENSION_TRANSFORM_OFFSET = "offset";
const char* GLTF_FILE_EXTENSION_TRANSFORM_ROTATION = "rotation";

LLGLTFMaterial::LLGLTFMaterial(const LLGLTFMaterial& rhs)
{
    *this = rhs;
}

LLGLTFMaterial& LLGLTFMaterial::operator=(const LLGLTFMaterial& rhs)
{
    //have to do a manual operator= because of LLRefCount
    mBaseColorId = rhs.mBaseColorId;
    mNormalId = rhs.mNormalId;
    mMetallicRoughnessId = rhs.mMetallicRoughnessId;
    mEmissiveId = rhs.mEmissiveId;

    mBaseColor = rhs.mBaseColor;
    mEmissiveColor = rhs.mEmissiveColor;

    mMetallicFactor = rhs.mMetallicFactor;
    mRoughnessFactor = rhs.mRoughnessFactor;
    mAlphaCutoff = rhs.mAlphaCutoff;

    mDoubleSided = rhs.mDoubleSided;
    mAlphaMode = rhs.mAlphaMode;

    mTextureTransform = rhs.mTextureTransform;

    return *this;
}

bool LLGLTFMaterial::fromJSON(const std::string& json, std::string& warn_msg, std::string& error_msg)
{
#if 1
    tinygltf::TinyGLTF gltf;

    tinygltf::Model model_in;

    if (gltf.LoadASCIIFromString(&model_in, &error_msg, &warn_msg, json.c_str(), json.length(), ""))
    {
        setFromModel(model_in, 0);

        //DEBUG generate json and print
        LL_INFOS() << asJSON(true) << LL_ENDL;

        return true;
    }
#endif
    return false;
}

std::string LLGLTFMaterial::asJSON(bool prettyprint) const
{
#if 1
    tinygltf::TinyGLTF gltf;

    tinygltf::Model model_out;

    std::ostringstream str;

    writeToModel(model_out, 0);

    gltf.WriteGltfSceneToStream(&model_out, str, prettyprint, false);

    return str.str();
#else
    return "";
#endif
}

void LLGLTFMaterial::setFromModel(const tinygltf::Model& model, S32 mat_index)
{
    if (model.materials.size() <= mat_index)
    {
        return;
    }

    const tinygltf::Material& material_in = model.materials[mat_index];

    // Apply base color texture
    setFromTexture(model, material_in.pbrMetallicRoughness.baseColorTexture, GLTF_TEXTURE_INFO_BASE_COLOR, mBaseColorId);
    // Apply normal map
    setFromTexture(model, material_in.normalTexture, GLTF_TEXTURE_INFO_NORMAL, mNormalId);
    // Apply metallic-roughness texture
    setFromTexture(model, material_in.pbrMetallicRoughness.metallicRoughnessTexture, GLTF_TEXTURE_INFO_METALLIC_ROUGHNESS, mMetallicRoughnessId);
    // Apply emissive texture
    setFromTexture(model, material_in.emissiveTexture, GLTF_TEXTURE_INFO_EMISSIVE, mEmissiveId);

    setAlphaMode(material_in.alphaMode);
    mAlphaCutoff = llclamp((F32)material_in.alphaCutoff, 0.f, 1.f);

    mBaseColor.set(material_in.pbrMetallicRoughness.baseColorFactor);
    mEmissiveColor.set(material_in.emissiveFactor);

    mMetallicFactor = llclamp((F32)material_in.pbrMetallicRoughness.metallicFactor, 0.f, 1.f);
    mRoughnessFactor = llclamp((F32)material_in.pbrMetallicRoughness.roughnessFactor, 0.f, 1.f);

    mDoubleSided = material_in.doubleSided;
}

LLVector2 vec2_from_json(const tinygltf::Value::Object& object, const char* key, const LLVector2& default_value)
{
    const auto it = object.find(key);
    if (it == object.end())
    {
        return default_value;
    }
    const tinygltf::Value& vec2_json = std::get<1>(*it);
    if (!vec2_json.IsArray() || vec2_json.ArrayLen() < LENGTHOFVECTOR2)
    {
        return default_value;
    }
    LLVector2 value;
    for (U32 i = 0; i < LENGTHOFVECTOR2; ++i)
    {
        const tinygltf::Value& real_json = vec2_json.Get(i);
        if (!real_json.IsReal())
        {
            return default_value;
        }
        value.mV[i] = (F32)real_json.Get<double>();
    }
    return value;
}

F32 float_from_json(const tinygltf::Value::Object& object, const char* key, const F32 default_value)
{
    const auto it = object.find(key);
    if (it == object.end())
    {
        return default_value;
    }
    const tinygltf::Value& real_json = std::get<1>(*it);
    if (!real_json.IsReal())
    {
        return default_value;
    }
    return (F32)real_json.GetNumberAsDouble();
}

template<typename T>
std::string gltf_get_texture_image(const tinygltf::Model& model, const T& texture_info)
{
    const S32 texture_idx = texture_info.index;
    if (texture_idx < 0 || texture_idx >= model.textures.size())
    {
        return "";
    }
    const tinygltf::Texture& texture = model.textures[texture_idx];

    // Ignore texture.sampler for now

    const S32 image_idx = texture.source;
    if (image_idx < 0 || image_idx >= model.images.size())
    {
        return "";
    }
    const tinygltf::Image& image = model.images[image_idx];

    return image.uri;
}

// *NOTE: Use template here as workaround for the different similar texture info classes
template<typename T>
void LLGLTFMaterial::setFromTexture(const tinygltf::Model& model, const T& texture_info, TextureInfo texture_info_id, LLUUID& texture_id_out)
{
    const std::string uri = gltf_get_texture_image(model, texture_info);
    texture_id_out.set(uri);

    const tinygltf::Value::Object& extensions_object = texture_info.extensions;
    const auto transform_it = extensions_object.find(GLTF_FILE_EXTENSION_TRANSFORM);
    if (transform_it != extensions_object.end())
    {
        const tinygltf::Value& transform_json = std::get<1>(*transform_it);
        if (transform_json.IsObject())
        {
            const tinygltf::Value::Object& transform_object = transform_json.Get<tinygltf::Value::Object>();
            TextureTransform& transform = mTextureTransform[texture_info_id];
            transform.mOffset = vec2_from_json(transform_object, GLTF_FILE_EXTENSION_TRANSFORM_OFFSET, getDefaultTextureOffset());
            transform.mScale = vec2_from_json(transform_object, GLTF_FILE_EXTENSION_TRANSFORM_SCALE, getDefaultTextureScale());
            transform.mRotation = float_from_json(transform_object, GLTF_FILE_EXTENSION_TRANSFORM_ROTATION, getDefaultTextureRotation());
        }
    }
}

void LLGLTFMaterial::writeToModel(tinygltf::Model& model, S32 mat_index) const
{
    if (model.materials.size() < mat_index+1)
    {
        model.materials.resize(mat_index + 1);
    }

    tinygltf::Material& material_out = model.materials[mat_index];

    constexpr bool is_override = false;

    // set base color texture
    writeToTexture(model, material_out.pbrMetallicRoughness.baseColorTexture, GLTF_TEXTURE_INFO_BASE_COLOR, mBaseColorId, is_override, LLUUID());
    // set normal texture
    writeToTexture(model, material_out.normalTexture, GLTF_TEXTURE_INFO_NORMAL, mNormalId, is_override, LLUUID());
    // set metallic-roughness texture
    writeToTexture(model, material_out.pbrMetallicRoughness.metallicRoughnessTexture, GLTF_TEXTURE_INFO_METALLIC_ROUGHNESS, mMetallicRoughnessId, is_override, LLUUID());
    // set emissive texture
    writeToTexture(model, material_out.emissiveTexture, GLTF_TEXTURE_INFO_EMISSIVE, mEmissiveId, is_override, LLUUID());

    material_out.alphaMode = getAlphaMode();
    material_out.alphaCutoff = mAlphaCutoff;

    mBaseColor.write(material_out.pbrMetallicRoughness.baseColorFactor);
    material_out.emissiveFactor.resize(3); // 0 size by default
    mEmissiveColor.write(material_out.emissiveFactor);

    material_out.pbrMetallicRoughness.metallicFactor = mMetallicFactor;
    material_out.pbrMetallicRoughness.roughnessFactor = mRoughnessFactor;

    material_out.doubleSided = mDoubleSided;

    model.asset.version = "2.0";
}

template<typename T>
void gltf_allocate_texture_image(tinygltf::Model& model, T& texture_info, const std::string& uri)
{
    const S32 image_idx = model.images.size();
    model.images.emplace_back();
    model.images[image_idx].uri = uri;

    // The texture, not to be confused with the texture info
    const S32 texture_idx = model.textures.size();
    model.textures.emplace_back();
    tinygltf::Texture& texture = model.textures[texture_idx];
    texture.source = image_idx;

    texture_info.index = texture_idx;
}

template<typename T>
void LLGLTFMaterial::writeToTexture(tinygltf::Model& model, T& texture_info, TextureInfo texture_info_id, const LLUUID& texture_id, bool is_override, const LLUUID& base_texture_id) const
{
    if (texture_id.isNull() || (is_override && texture_id == base_texture_id))
    {
        return;
    }

    gltf_allocate_texture_image(model, texture_info, texture_id.asString());

    tinygltf::Value::Object transform_map;
    const TextureTransform& transform = mTextureTransform[texture_info_id];
    transform_map[GLTF_FILE_EXTENSION_TRANSFORM_OFFSET] = tinygltf::Value(tinygltf::Value::Array({
        tinygltf::Value(transform.mOffset.mV[VX]),
        tinygltf::Value(transform.mOffset.mV[VY])
    }));
    transform_map[GLTF_FILE_EXTENSION_TRANSFORM_SCALE] = tinygltf::Value(tinygltf::Value::Array({
        tinygltf::Value(transform.mScale.mV[VX]),
        tinygltf::Value(transform.mScale.mV[VY])
    }));
    transform_map[GLTF_FILE_EXTENSION_TRANSFORM_ROTATION] = tinygltf::Value(transform.mRotation);
    texture_info.extensions[GLTF_FILE_EXTENSION_TRANSFORM] = tinygltf::Value(transform_map);
}


void LLGLTFMaterial::setBaseColorId(const LLUUID& id)
{
    mBaseColorId = id;
}

void LLGLTFMaterial::setNormalId(const LLUUID& id)
{
    mNormalId = id;
}

void LLGLTFMaterial::setMetallicRoughnessId(const LLUUID& id)
{
    mMetallicRoughnessId = id;
}

void LLGLTFMaterial::setEmissiveId(const LLUUID& id)
{
    mEmissiveId = id;
}

void LLGLTFMaterial::setBaseColorFactor(const LLColor3& baseColor, F32 transparency)
{
    mBaseColor.set(baseColor, transparency);
    mBaseColor.clamp();
}

void LLGLTFMaterial::setAlphaCutoff(F32 cutoff)
{
    mAlphaCutoff = llclamp(cutoff, 0.f, 1.f);
}

void LLGLTFMaterial::setEmissiveColorFactor(const LLColor3& emissiveColor)
{
    mEmissiveColor = emissiveColor;
    mEmissiveColor.clamp();
}

void LLGLTFMaterial::setMetallicFactor(F32 metallic)
{
    mMetallicFactor = llclamp(metallic, 0.f, 1.f);
}

void LLGLTFMaterial::setRoughnessFactor(F32 roughness)
{
    mRoughnessFactor = llclamp(roughness, 0.f, 1.f);
}

void LLGLTFMaterial::setAlphaMode(S32 mode)
{
    mAlphaMode = (AlphaMode) llclamp(mode, (S32) ALPHA_MODE_OPAQUE, (S32) ALPHA_MODE_MASK);
}

void LLGLTFMaterial::setDoubleSided(bool double_sided)
{
    // sure, no clamping will ever be needed for a bool, but include the
    // setter for consistency with the clamping API
    mDoubleSided = double_sided;
}

void LLGLTFMaterial::setTextureOffset(TextureInfo texture_info, const LLVector2& offset)
{
    mTextureTransform[texture_info].mOffset = offset;
}

void LLGLTFMaterial::setTextureScale(TextureInfo texture_info, const LLVector2& scale)
{
    mTextureTransform[texture_info].mScale = scale;
}

void LLGLTFMaterial::setTextureRotation(TextureInfo texture_info, float rotation)
{
    mTextureTransform[texture_info].mRotation = rotation;
}

// Default value accessors

LLUUID LLGLTFMaterial::getDefaultBaseColorId()
{
    return LLUUID::null;
}

LLUUID LLGLTFMaterial::getDefaultNormalId()
{
    return LLUUID::null;
}

LLUUID LLGLTFMaterial::getDefaultEmissiveId()
{
    return LLUUID::null;
}

LLUUID LLGLTFMaterial::getDefaultMetallicRoughnessId()
{
    return LLUUID::null;
}

F32 LLGLTFMaterial::getDefaultAlphaCutoff()
{
    return 0.f;
}

S32 LLGLTFMaterial::getDefaultAlphaMode()
{
    return (S32) ALPHA_MODE_OPAQUE;
}

F32 LLGLTFMaterial::getDefaultMetallicFactor()
{
    return 0.f;
}

F32 LLGLTFMaterial::getDefaultRoughnessFactor()
{
    return 0.f;
}

LLColor4 LLGLTFMaterial::getDefaultBaseColor()
{
    return LLColor4::white;
}

LLColor3 LLGLTFMaterial::getDefaultEmissiveColor()
{
    return LLColor3::black;
}

bool LLGLTFMaterial::getDefaultDoubleSided()
{
    return false;
}

LLVector2 LLGLTFMaterial::getDefaultTextureOffset()
{
    return LLVector2(0.f, 0.f);
}

LLVector2 LLGLTFMaterial::getDefaultTextureScale()
{
    return LLVector2(1.f, 1.f);
}

F32 LLGLTFMaterial::getDefaultTextureRotation()
{
    return 0.f;
}

void LLGLTFMaterial::writeOverridesToModel(tinygltf::Model& model, S32 mat_index, LLGLTFMaterial const* base_material) const
{
    if (model.materials.size() < mat_index + 1)
    {
        model.materials.resize(mat_index + 1);
    }

    tinygltf::Material& material_out = model.materials[mat_index];

    // TODO - fix handling of resetting to null/default values

    constexpr bool is_override = true;

    // set base color texture
    writeToTexture(model, material_out.pbrMetallicRoughness.baseColorTexture, GLTF_TEXTURE_INFO_BASE_COLOR, mBaseColorId, is_override, base_material->mBaseColorId);
    // set normal texture
    writeToTexture(model, material_out.normalTexture, GLTF_TEXTURE_INFO_NORMAL, mNormalId, is_override, base_material->mNormalId);
    // set metallic-roughness texture
    writeToTexture(model, material_out.pbrMetallicRoughness.metallicRoughnessTexture, GLTF_TEXTURE_INFO_METALLIC_ROUGHNESS, mMetallicRoughnessId, is_override, base_material->mMetallicRoughnessId);
    // set emissive texture
    writeToTexture(model, material_out.emissiveTexture, GLTF_TEXTURE_INFO_EMISSIVE, mEmissiveId, is_override, base_material->mEmissiveId);

    if (mAlphaMode != base_material->mAlphaMode)
    {
        material_out.alphaMode = getAlphaMode();
    }

    if (mAlphaCutoff != base_material->mAlphaCutoff)
    {
        material_out.alphaCutoff = mAlphaCutoff;
    }

    if (mBaseColor != base_material->mBaseColor)
    {
        mBaseColor.write(material_out.pbrMetallicRoughness.baseColorFactor);
    }

    if (mEmissiveColor != base_material->mEmissiveColor)
    {
        material_out.emissiveFactor.resize(3); // 0 size by default
        mEmissiveColor.write(material_out.emissiveFactor);
    }

    if (mMetallicFactor != base_material->mMetallicFactor)
    {
        material_out.pbrMetallicRoughness.metallicFactor = mMetallicFactor;
    }

    if (mRoughnessFactor != base_material->mRoughnessFactor)
    {
        material_out.pbrMetallicRoughness.roughnessFactor = mRoughnessFactor;
    }

    if (mDoubleSided != base_material->mDoubleSided)
    {
        material_out.doubleSided = mDoubleSided;
    }
}

void LLGLTFMaterial::applyOverride(const LLGLTFMaterial& override_mat)
{
    // TODO: potentially reimplement this with a more general purpose JSON merge

    if (override_mat.mBaseColorId != getDefaultBaseColorId())
    {
        mBaseColorId = override_mat.mBaseColorId;
    }

    if (override_mat.mNormalId != getDefaultNormalId())
    {
        mNormalId = override_mat.mNormalId;
    }

    if (override_mat.mMetallicRoughnessId != getDefaultMetallicRoughnessId())
    {
        mMetallicRoughnessId = override_mat.mMetallicRoughnessId;
    }

    if (override_mat.mEmissiveId != getDefaultEmissiveId())
    {
        mEmissiveId = override_mat.mEmissiveId;
    }

    if (override_mat.mBaseColor != getDefaultBaseColor())
    {
        mBaseColor = override_mat.mBaseColor;
    }

    if (override_mat.mEmissiveColor != getDefaultEmissiveColor())
    {
        mEmissiveColor = override_mat.mEmissiveColor;
    }

    if (override_mat.mMetallicFactor != getDefaultMetallicFactor())
    {
        mMetallicFactor = override_mat.mMetallicFactor;
    }

    if (override_mat.mRoughnessFactor != getDefaultRoughnessFactor())
    {
        mRoughnessFactor = override_mat.mRoughnessFactor;
    }

    if (override_mat.mAlphaMode != getDefaultAlphaMode())
    {
        mAlphaMode = override_mat.mAlphaMode;
    }
    if (override_mat.mAlphaCutoff != getDefaultAlphaCutoff())
    {
        mAlphaCutoff = override_mat.mAlphaCutoff;
    }

    if (override_mat.mDoubleSided != getDefaultDoubleSided())
    {
        mDoubleSided = override_mat.mDoubleSided;
    }

    for (int i = 0; i < GLTF_TEXTURE_INFO_COUNT; ++i)
    {
        if (override_mat.mTextureTransform[i].mOffset != getDefaultTextureOffset())
        {
            mTextureTransform[i].mOffset = override_mat.mTextureTransform[i].mOffset;
        }

        if (override_mat.mTextureTransform[i].mScale != getDefaultTextureScale())
        {
            mTextureTransform[i].mScale = override_mat.mTextureTransform[i].mScale;
        }

        if (override_mat.mTextureTransform[i].mScale != getDefaultTextureScale())
        {
            mTextureTransform[i].mScale = override_mat.mTextureTransform[i].mScale;
        }

        if (override_mat.mTextureTransform[i].mRotation != getDefaultTextureRotation())
        {
            mTextureTransform[i].mRotation = override_mat.mTextureTransform[i].mRotation;
        }
    }
}
