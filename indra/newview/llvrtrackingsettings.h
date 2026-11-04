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
    // P2-07B C4 (spec section 3.1): positional-solve enablement.
    bool track_head_position = false;   // FSVRTrackHeadPosition
    bool track_hand_position = false;   // FSVRTrackHandPosition
    // P2-07B C4 (spec sections 3.1/3.2): the L4 toggle. The VALUE
    // is per-account (gSavedPerAccountSettings); the snapshot
    // carries effective state only, like every other field here.
    bool solver_bend_disabled_bones = false; // FSVRSolverBendDisabledBones
    // P2-07B C4 (spec section 3.4): diagnostic toggles, read
    // through this snapshot like the tracking settings.
    bool diag_bend_limits = true;       // FSVRDiagBendLimits
    bool diag_soft_preference = true;   // FSVRDiagSoftPreference
    bool diag_force_unit_scale = false; // FSVRDiagForceUnitScale
    bool diag_positional_solve = true;  // FSVRDiagPositionalSolve
    bool diag_full_chain_solve = true;  // FSVRDiagFullChainSolve
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

// P2-07B C4 (spec section 3.3): the load validator's result over a
// raw FSVRCalibrationRecord value. The parsed fields are meaningful
// only for the status that admits them: head fields for
// USABLE_HEAD_ONLY and USABLE_FULL, arm fields for USABLE_FULL only.
// hand_calibration_present is the section 3.3 guard made explicit:
// a head-only record MUST NOT be read as a hand calibration.
struct FSVRCalibrationRecordValidation
{
    // Named RecordStatus, not Status: Xlib defines Status as a
    // macro on this platform.
    enum class RecordStatus
    {
        NO_RECORD,
        USABLE_HEAD_ONLY,
        USABLE_FULL
    };

    RecordStatus status = RecordStatus::NO_RECORD;
    bool hand_calibration_present = false;
    S64 captured_utc = 0;
    F64 head_ratio = 0.0;
    F64 avatar_cornea_height = 0.0;
    F64 player_hmd_height = 0.0;
    F64 arm_ratio_left = 0.0;
    F64 arm_ratio_right = 0.0;
    F64 avatar_reach_left = 0.0;
    F64 avatar_reach_right = 0.0;
    F64 player_reach_left = 0.0;
    F64 player_reach_right = 0.0;
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

    // Reads the saved settings (Stage-1 four plus the C4 eight),
    // re-parses only when the raw values changed, and returns the
    // immutable snapshot for the current frame. Generation
    // increments on each accepted change. Also installs the CP-4
    // coupling listener on first call.
    static const FSVRTrackingSettingsSnapshot& captureForFrame();

    // P2-07B C4 (CP-4): the coupling's pure decision over one
    // commit-signal (previous, new) document pair. True iff both
    // documents parse VALID and the UPPER_BODY rollup's disabled
    // count transitions >0 -> 0 (the group-enable transition).
    // Exposed so harness H4 tests the production decision
    // directly, like parseDisabledBones.
    static bool couplingWouldFire(const LLSD& previous_document,
                                  const LLSD& new_document);

    // P2-07B C4 (CP-5): the per-account store surface. All thin
    // over gSavedPerAccountSettings; no persistence code lives
    // here (save-on-quit is the store's native behavior).
    // The L4 toggle's raw read -- the same read captureForFrame
    // snapshots (one formulation, two call sites).
    static bool solverBendDisabledBones();
    // The calibration record's current raw LLSD (empty map when
    // no record). Content validity is validateCalibrationRecord's.
    static LLSD calibrationRecordRaw();
    // THE record commit point (spec section 3.3 atomic commit):
    // a single setLLSD of a complete map. C5 builds the map; this
    // surface performs the one write. No other write path exists.
    static void writeCalibrationRecord(const LLSD& record);
    // P2-07B C4 (CP-5, spec section 3.3 load validation): the
    // pure validator over a raw record value. Reads nothing,
    // writes nothing; which account the record belongs to is
    // guaranteed by the per-account store's file selection, not
    // by a validator clause.
    static FSVRCalibrationRecordValidation validateCalibrationRecord(
        const LLSD& raw_record);

    static constexpr U32 CURRENT_SCHEMA_VERSION = 1;
    static constexpr S32 CALIBRATION_RECORD_SCHEMA_VERSION = 1;
    static const char* const SETTING_DISABLED_BONES;
    static const char* const SETTING_TRACK_HEAD_PITCH;
    static const char* const SETTING_TRACK_HEAD_ROLL;
    static const char* const SETTING_BODY_FOLLOWS_HEADSET;
    static const char* const SETTING_TRACK_HEAD_POSITION;
    static const char* const SETTING_TRACK_HAND_POSITION;
    static const char* const SETTING_SOLVER_BEND_DISABLED_BONES;
    static const char* const SETTING_CALIBRATION_RECORD;
    static const char* const SETTING_DIAG_BEND_LIMITS;
    static const char* const SETTING_DIAG_SOFT_PREFERENCE;
    static const char* const SETTING_DIAG_FORCE_UNIT_SCALE;
    static const char* const SETTING_DIAG_POSITIONAL_SOLVE;
    static const char* const SETTING_DIAG_FULL_CHAIN_SOLVE;
};

#endif // LL_LLVRTRACKINGSETTINGS_H
