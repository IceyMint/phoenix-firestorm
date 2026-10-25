/**
 * @file llvrtrackingsettings.h
 * @brief P2-07a Stage 1: persistent VR tracking settings model.
 *
 * Implements the versioned disabled-bone set (design section 1.2),
 * the parsed-settings structure and the immutable per-frame settings
 * snapshot (implementation spec sections 2.1 and 2.2). Stage 1 is
 * shadow-only: these settings are parsed, snapshotted and traced,
 * but they do not drive any joint.
 */

#ifndef LL_LLVRTRACKINGSETTINGS_H
#define LL_LLVRTRACKINGSETTINGS_H

#include "llsd.h"

#include <set>
#include <string>

// Parsed form of the FSVRTrackingDisabledBones LLSD document.
// Unknown identifiers are preserved verbatim and ignored for driving;
// unknown top-level fields of a valid version-1 document are kept for
// compatible round trips. A future schema version is preserved raw
// and marked not writable; the version-1 writer must never silently
// overwrite it.
struct FSVRTrackingDisabledBoneSet
{
    enum class ParseStatus
    {
        VALID,
        MALFORMED,
        UNSUPPORTED_FUTURE_VERSION
    };

    U32 schema_version = 0;
    ParseStatus parse_status = ParseStatus::MALFORMED;
    // Complete persisted membership, including unknown identifiers.
    std::set<std::string> disabled_ids;
    // Intersection with the current bone registry.
    std::set<std::string> known_disabled_ids;
    // Preserved but ignored for driving.
    std::set<std::string> unknown_disabled_ids;
    // Extra version-1 top-level fields, preserved on round trip.
    LLSD unrecognized_fields;
    // Unmodified input document (failure / future-version preservation).
    LLSD original_document;
    bool writable = false;

    bool isBoneDisabled(const std::string& canonical_id) const
    {
        return known_disabled_ids.find(canonical_id) != known_disabled_ids.end();
    }
};

// Immutable effective settings for one frame. Captured exactly once
// per frame by the coordinator; a settings change takes effect on the
// next frame boundary, never mid-blend.
struct FSVRTrackingSettingsSnapshot
{
    // Observed settings-state revision: advances on any accepted raw
    // change, including transitions to malformed or unsupported
    // documents (it is not a count of valid configurations only).
    U64 generation = 0;
    FSVRTrackingDisabledBoneSet disabled_bones;
    bool head_pitch_enabled = true;
    bool head_roll_enabled = true;
    // Stored in P2-07a; body-follow itself arrives in P2-07b.
    bool body_follows_headset = true;
    bool schema_valid = false;
    // Effective head/neck selections equal the legacy behavior:
    // mNeck and mHead enabled, pitch and roll enabled, schema valid.
    bool legacy_equivalent = true;
    // At least one effective head/neck or component selection differs
    // from the legacy behavior.
    bool selective_requested = false;
    // Always false in Stage 1; becomes a runtime gate after the
    // Stage-2 audit proofs and build verification.
    bool selective_verified = false;
};

class FSVRTrackingSettings
{
public:
    // Pure parser (unit-testable, no globals). known_ids is the
    // registry's canonical identifier set.
    static FSVRTrackingDisabledBoneSet parseDisabledBones(
        const LLSD& document, const std::set<std::string>& known_ids);

    // Deterministic version-1 writer. Returns undefined LLSD when the
    // set is not writable (malformed input is never "repaired" into a
    // document that loses information; a future version is never
    // overwritten).
    static LLSD serializeDisabledBones(const FSVRTrackingDisabledBoneSet& parsed);

    // Reads the four saved settings, re-parses only when the raw
    // values changed, and returns the immutable snapshot for the
    // current frame. Generation increments on each accepted change.
    static const FSVRTrackingSettingsSnapshot& captureForFrame();

    static constexpr U32 CURRENT_SCHEMA_VERSION = 1;
    static const char* const SETTING_DISABLED_BONES;
    static const char* const SETTING_TRACK_HEAD_PITCH;
    static const char* const SETTING_TRACK_HEAD_ROLL;
    static const char* const SETTING_BODY_FOLLOWS_HEADSET;
};

#endif // LL_LLVRTRACKINGSETTINGS_H
