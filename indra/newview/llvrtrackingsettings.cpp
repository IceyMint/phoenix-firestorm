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

    if (!document.isMap() || !document.has("version") ||
        !document["version"].isInteger() || !document.has("disabled") ||
        !document["disabled"].isArray())
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
        const std::string id = it->asString();
        if (!id.empty())
        {
            result.disabled_ids.insert(id);
        }
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
// Effective-settings fingerprint of the raw saved value, for change
// detection (LLSD has no operator==). Only the parts that can affect
// the effective snapshot are fingerprinted: schema version, disabled
// membership and the presence of unknown top-level fields. A change
// confined to an unknown field's value cannot change any effective
// setting, so it correctly does not advance the generation.
struct RawFingerprint
{
    bool is_map = false;
    bool has_version = false;
    S32 version = 0;
    bool disabled_is_array = false;
    bool disabled_has_nonstring = false;
    std::set<std::string> disabled_ids;
    std::set<std::string> extra_keys;

    bool operator==(const RawFingerprint& other) const
    {
        return is_map == other.is_map && has_version == other.has_version &&
               version == other.version &&
               disabled_is_array == other.disabled_is_array &&
               disabled_has_nonstring == other.disabled_has_nonstring &&
               disabled_ids == other.disabled_ids &&
               extra_keys == other.extra_keys;
    }
    bool operator!=(const RawFingerprint& other) const
    {
        return !(*this == other);
    }
};

RawFingerprint fingerprintRaw(const LLSD& document)
{
    RawFingerprint fp;
    if (document.isUndefined() || !document.isMap())
    {
        return fp;
    }
    fp.is_map = true;
    if (document.has("version") && document["version"].isInteger())
    {
        fp.has_version = true;
        fp.version = document["version"].asInteger();
    }
    if (document.has("disabled") && document["disabled"].isArray())
    {
        fp.disabled_is_array = true;
        for (LLSD::array_const_iterator it = document["disabled"].beginArray();
             it != document["disabled"].endArray(); ++it)
        {
            if (it->isString())
            {
                fp.disabled_ids.insert(it->asString());
            }
            else
            {
                fp.disabled_has_nonstring = true;
            }
        }
    }
    for (LLSD::map_const_iterator it = document.beginMap();
         it != document.endMap(); ++it)
    {
        if (it->first != "version" && it->first != "disabled")
        {
            fp.extra_keys.insert(it->first);
        }
    }
    return fp;
}

RawFingerprint s_last_fingerprint;
bool s_last_fingerprint_valid = false;
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
    const RawFingerprint fingerprint = fingerprintRaw(raw_disabled);

    const bool changed = !s_snapshot_initialized || !s_last_fingerprint_valid ||
                         fingerprint != s_last_fingerprint ||
                         pitch != s_last_pitch || roll != s_last_roll ||
                         body_follow != s_last_body_follow;
    if (!changed)
    {
        return s_snapshot;
    }

    s_last_fingerprint = fingerprint;
    s_last_fingerprint_valid = true;
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
