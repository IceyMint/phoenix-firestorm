/**
 * @file llvrtrackingsettings.cpp
 * @brief P2-07a Stage 1: VR tracking settings parser and snapshots.
 */

#include "llviewerprecompiledheaders.h"

#include "llvrtrackingsettings.h"

#include "llcontrol.h"
#include "llviewercontrol.h" // gSavedSettings
#include "llvrboneregistry.h"

const char* const FSVRTrackingSettings::SETTING_DISABLED_BONES = "FSVRTrackingDisabledBones";
const char* const FSVRTrackingSettings::SETTING_TRACK_HEAD_PITCH = "FSVRTrackHeadPitch";
const char* const FSVRTrackingSettings::SETTING_TRACK_HEAD_ROLL = "FSVRTrackHeadRoll";
const char* const FSVRTrackingSettings::SETTING_BODY_FOLLOWS_HEADSET = "FVRBodyFollowsHeadset";

// static
FSVRTrackingDisabledBoneSet FSVRTrackingSettings::parseDisabledBones(
    const LLSD& document, const std::set<std::string>& known_ids)
{
    FSVRTrackingDisabledBoneSet result;
    result.original_document = document;
    result.unrecognized_fields = LLSD::emptyMap();

    // Absent setting: synthesize a valid empty version-1 set.
    if (document.isUndefined())
    {
        result.schema_version = CURRENT_SCHEMA_VERSION;
        result.parse_status = FSVRTrackingDisabledBoneSet::ParseStatus::VALID;
        result.writable = true;
        return result;
    }

    if (!document.isMap())
    {
        result.parse_status = FSVRTrackingDisabledBoneSet::ParseStatus::MALFORMED;
        result.writable = false;
        return result;
    }

    // Classify the schema version BEFORE enforcing version-1 shape:
    // a future version may lay its fields out differently and must
    // still be recognized as an unsupported future version (never
    // silently treated as malformed version 1).
    if (!document.has("version") || !document["version"].isInteger())
    {
        result.parse_status = FSVRTrackingDisabledBoneSet::ParseStatus::MALFORMED;
        result.writable = false;
        return result;
    }
    const S32 version = document["version"].asInteger();
    if (version < 1)
    {
        result.parse_status = FSVRTrackingDisabledBoneSet::ParseStatus::MALFORMED;
        result.writable = false;
        return result;
    }
    result.schema_version = static_cast<U32>(version);

    if (version > static_cast<S32>(CURRENT_SCHEMA_VERSION))
    {
        // Preserve the original document raw; never guess at a future
        // schema and never let the version-1 writer overwrite it.
        result.parse_status =
            FSVRTrackingDisabledBoneSet::ParseStatus::UNSUPPORTED_FUTURE_VERSION;
        result.writable = false;
        return result;
    }

    if (!document.has("disabled") || !document["disabled"].isArray())
    {
        result.parse_status = FSVRTrackingDisabledBoneSet::ParseStatus::MALFORMED;
        result.writable = false;
        return result;
    }

    // Version 1: collect membership. Duplicates collapse in the set;
    // non-string entries make the document malformed rather than
    // being silently dropped.
    for (LLSD::array_const_iterator it = document["disabled"].beginArray();
         it != document["disabled"].endArray(); ++it)
    {
        if (!it->isString())
        {
            result.parse_status = FSVRTrackingDisabledBoneSet::ParseStatus::MALFORMED;
            result.writable = false;
            result.disabled_ids.clear();
            return result;
        }
        // Every string entry is membership, including an empty
        // string: it is an unknown identifier, preserved exactly
        // like any other, never silently dropped.
        result.disabled_ids.insert(it->asString());
    }

    // Preserve unknown top-level fields for compatible round trips.
    for (LLSD::map_const_iterator it = document.beginMap();
         it != document.endMap(); ++it)
    {
        if (it->first != "version" && it->first != "disabled")
        {
            result.unrecognized_fields[it->first] = it->second;
        }
    }

    for (const std::string& id : result.disabled_ids)
    {
        if (known_ids.find(id) != known_ids.end())
        {
            result.known_disabled_ids.insert(id);
        }
        else
        {
            result.unknown_disabled_ids.insert(id);
        }
    }

    result.parse_status = FSVRTrackingDisabledBoneSet::ParseStatus::VALID;
    result.writable = true;
    return result;
}

// static
LLSD FSVRTrackingSettings::serializeDisabledBones(
    const FSVRTrackingDisabledBoneSet& parsed)
{
    if (!parsed.writable ||
        parsed.parse_status != FSVRTrackingDisabledBoneSet::ParseStatus::VALID)
    {
        return LLSD();
    }

    LLSD document = parsed.unrecognized_fields.isMap()
                        ? parsed.unrecognized_fields
                        : LLSD::emptyMap();
    document["version"] = static_cast<S32>(CURRENT_SCHEMA_VERSION);

    // std::set iteration order is sorted, so the writer is
    // deterministic for a given membership (spec section 2.1).
    LLSD disabled = LLSD::emptyArray();
    for (const std::string& id : parsed.disabled_ids)
    {
        disabled.append(id);
    }
    document["disabled"] = disabled;
    return document;
}

namespace
{
// Deterministic canonical rendering of a complete LLSD document,
// used for change detection (LLSD has no operator==). The canonical
// form covers the WHOLE document -- unknown-field values included --
// so a change anywhere in the raw setting is observed and the
// snapshot's preserved document can never go stale relative to the
// saved one (review finding P207A-S1-03). LLSD maps iterate in key
// order, so the rendering is deterministic.
void appendCanonicalLLSD(const LLSD& value, std::string& out)
{
    switch (value.type())
    {
    case LLSD::TypeUndefined:
        out += "u;";
        break;
    case LLSD::TypeBoolean:
        out += value.asBoolean() ? "b:1;" : "b:0;";
        break;
    case LLSD::TypeInteger:
    {
        out += "i:";
        out += std::to_string(value.asInteger());
        out += ";";
        break;
    }
    case LLSD::TypeReal:
    {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%.17g", value.asReal());
        out += "r:";
        out += buffer;
        out += ";";
        break;
    }
    case LLSD::TypeString:
    {
        const std::string text = value.asString();
        out += "s:";
        out += std::to_string(text.size());
        out += ":";
        out += text;
        out += ";";
        break;
    }
    case LLSD::TypeUUID:
        out += "uuid:";
        out += value.asString();
        out += ";";
        break;
    case LLSD::TypeDate:
        out += "date:";
        out += value.asString();
        out += ";";
        break;
    case LLSD::TypeBinary:
    {
        const std::vector<U8> data = value.asBinary();
        out += "bin:";
        out += std::to_string(data.size());
        out += ":";
        static const char* const hex = "0123456789abcdef";
        for (const U8 byte : data)
        {
            out += hex[byte >> 4];
            out += hex[byte & 0x0f];
        }
        out += ";";
        break;
    }
    case LLSD::TypeArray:
        out += "a[";
        for (LLSD::array_const_iterator it = value.beginArray();
             it != value.endArray(); ++it)
        {
            appendCanonicalLLSD(*it, out);
        }
        out += "]";
        break;
    case LLSD::TypeMap:
        out += "m{";
        for (LLSD::map_const_iterator it = value.beginMap();
             it != value.endMap(); ++it)
        {
            out += it->first;
            out += "=";
            appendCanonicalLLSD(it->second, out);
        }
        out += "}";
        break;
    default:
        out += "?;";
        break;
    }
}

std::string canonicalLLSD(const LLSD& value)
{
    std::string out;
    appendCanonicalLLSD(value, out);
    return out;
}

std::string s_last_canonical;
bool s_last_canonical_valid = false;
bool s_last_pitch = true;
bool s_last_roll = true;
bool s_last_body_follow = true;
FSVRTrackingSettingsSnapshot s_snapshot;
bool s_snapshot_initialized = false;
} // namespace

// static
const FSVRTrackingSettingsSnapshot& FSVRTrackingSettings::captureForFrame()
{
    const LLSD raw_disabled = gSavedSettings.getLLSD(SETTING_DISABLED_BONES);
    const bool pitch = gSavedSettings.getBOOL(SETTING_TRACK_HEAD_PITCH);
    const bool roll = gSavedSettings.getBOOL(SETTING_TRACK_HEAD_ROLL);
    const bool body_follow = gSavedSettings.getBOOL(SETTING_BODY_FOLLOWS_HEADSET);
    const std::string canonical = canonicalLLSD(raw_disabled);

    const bool changed = !s_snapshot_initialized || !s_last_canonical_valid ||
                         canonical != s_last_canonical ||
                         pitch != s_last_pitch || roll != s_last_roll ||
                         body_follow != s_last_body_follow;
    if (!changed)
    {
        return s_snapshot;
    }

    s_last_canonical = canonical;
    s_last_canonical_valid = true;
    s_last_pitch = pitch;
    s_last_roll = roll;
    s_last_body_follow = body_follow;

    FSVRTrackingSettingsSnapshot next;
    next.generation = s_snapshot_initialized ? s_snapshot.generation + 1 : 1;
    next.disabled_bones =
        parseDisabledBones(raw_disabled, FSVRBoneRegistry::knownIds());
    next.head_pitch_enabled = pitch;
    next.head_roll_enabled = roll;
    next.body_follows_headset = body_follow;
    next.schema_valid =
        next.disabled_bones.parse_status ==
        FSVRTrackingDisabledBoneSet::ParseStatus::VALID;
    // Body-follow does not affect legacy equivalence in P2-07a: it is
    // not implemented yet (spec section 2.2).
    next.legacy_equivalent =
        next.schema_valid && !next.disabled_bones.isBoneDisabled("mNeck") &&
        !next.disabled_bones.isBoneDisabled("mHead") && pitch && roll;
    next.selective_requested = !next.legacy_equivalent;
    next.selective_verified = false; // Stage 1: hard gate.

    s_snapshot = next;
    s_snapshot_initialized = true;
    return s_snapshot;
}
