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

// P2-07B C6-E (CP-5 part 1a): the Gate-1 release authorization
// stage of this BUILD. The representation is the tree itself: the
// value is produced by exactly one source constant in
// llvrtrackingsettings.cpp, advanced only by a dedicated
// authorization commit citing its evidence (Package B spec
// section 7.3 / brief CP-5 part 4). No settings control, debug
// toggle, environment variable, or file input exists for it.
enum class FSVRGate1Authorization : U8
{
    NONE = 0,       // ordinary build: no authorization
    CANDIDATE = 1,  // Stage A: candidate verification build
    PRODUCTION = 2  // Stage B: production activation
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
    // P2-07B Stage 1 (CP-S1-3): the provisional lean gain
    // (redesign section 4). lean_gain is the raw user value
    // when lean_gain_valid, and 1.0 otherwise; consumers
    // use the value only under the validity flag (E5
    // carries it, CP-S1-4). A global gSavedSettings F32,
    // deliberately NOT per-account: it is a feel
    // preference, not a profile (redesign sections 5/11).
    F32  lean_gain       = 1.f;  // FSVRLeanResponseGain
    bool lean_gain_valid = true;
    // P2-07B Fix 3 (brief section 2.6): the posture-model
    // settings, read beside lean_gain in the same style.
    // posture_swap exchanges the two posture behaviours
    // (the effective behaviour is a pure function of the
    // neutral's stamped posture and this flag). root_gain
    // is the root channel's k, with the shipped gain
    // semantics: root_gain is the raw user value when
    // root_gain_valid, and 1.0 otherwise; consumers use
    // the value only under the validity flag (the root
    // channel fails closed when it is false, never
    // clamped). standing_eye_height_cm is the typed
    // physical reference (floor to eye, plain cm):
    // standing_eye_height_set is false when the user has
    // not entered one (the declared 0 -- a first-class
    // provisional state, not an error);
    // standing_eye_height_invalid names a present but
    // unusable value (negative or non-finite). The eye
    // height is carried as typed state only: neither
    // channel's factor consumes it while OQ-7's
    // usable-length references are unresolved, so its
    // invalidity fails NO channel and is reported
    // against none [FIX3-CODE-R1-04]. The value is
    // pinned to 0 whenever it is not set, so the
    // snapshot never carries a non-finite or negative
    // F32 downstream.
    bool posture_swap = false;            // FSVRPostureSwap
    // P2-07B Fix 3 R5 [FIX3-R5-CODE-02]: the teleport
    // detector's measured-band readiness. False until
    // the legitimate/teleport bands are measured and
    // proven separable; while false, live root-follow
    // stays off on the R5 path (fail closed).
    bool teleport_detector_proven = false; // FSVRTeleportDetectorProven
    // P2-07B Fix 3 R5 (brief 8.2): the persistent
    // in-world posture indicator toggle.
    bool show_posture_indicator = true;    // FSVRShowPostureIndicator
    F32  root_gain = 1.f;                 // FSVRRootResponseGain
    bool root_gain_valid = true;
    F32  standing_eye_height_cm = 0.f;    // FSVRStandingEyeHeightCm
    bool standing_eye_height_set = false;
    bool standing_eye_height_invalid = false;
    // P2-07B Fix 3 [FIX3-CODE-R1-01]: the manual limb
    // ratios (brief section 2.4's manual-ratio dials),
    // one per channel, each with its own independent
    // state in the eye height's (value, set, invalid)
    // form. The declared 0 is UNSET (manual_*_set
    // false): the channel's derived ratio or provisional
    // baseline rules. A present value inside the valid
    // domain (finite, 0.05..3.0 -- the domain is the
    // main chat's design specification, recorded in
    // P2-07B-FIX3-CODE-REVISION-R2.md for the R2
    // verdict to rule on) is carried in manual_*_ratio
    // and engages (the glue maps it to the composition
    // seam's VALID). A present value outside the domain
    // (or non-finite) is INVALID: manual_*_invalid is
    // true and the ratio is pinned to 0, so the snapshot
    // never carries a non-finite or out-of-domain F32
    // downstream; the seam fails that channel closed,
    // never clamped, never quietly replaced by the
    // derived value. The two channels' states are
    // derived and carried independently.
    F32  manual_lean_ratio = 0.f;         // FSVRManualLeanRatio
    bool manual_lean_set = false;
    bool manual_lean_invalid = false;
    F32  manual_root_ratio = 0.f;         // FSVRManualRootRatio
    bool manual_root_set = false;
    bool manual_root_invalid = false;
    // P2-07B Fix 3 [FIX3-CODE-R3]: the root-follow bound
    // (OQ-2 as ruled by Leah 2026-11-07: STOP AT THE
    // BOUND). FSVRRootFollowBoundMeters is the radius in
    // PHYSICAL metres, measured on the pre-scale
    // displacement from the sync anchor; the consumed
    // displacement is radially clamped to it before
    // per-channel scaling (the main chat's design call,
    // recorded in P2-07B-FIX3-CODE-REVISION-R3.md for
    // the R3 ruling; declared default 2.5). The
    // derivation's valid domain is finite AND
    // 0 < v <= 10: zero is not a radius (and is the
    // setting's absent form), a negative or non-finite
    // value is not a radius, and a value beyond 10 m is
    // beyond any physical playspace -- it would make
    // the bound meaningless and, in the common case, is
    // a unit error (centimetres typed as metres) that
    // must fail loudly rather than silently become
    // effectively unbounded following. root_follow_bound_m
    // carries the raw value when valid and 0 otherwise,
    // so the snapshot never carries a non-finite or
    // out-of-domain F32 downstream. When the bound is
    // invalid the policy is NOT in force: live root
    // movement stays gated off (never unbounded, never
    // silent) and the standing composition keeps the
    // shipped lean behaviour [FIX3-CODE-R2-01].
    F32  root_follow_bound_m = 0.f;       // FSVRRootFollowBoundMeters
    bool root_follow_bound_valid = false;
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
    // P2-07B C6-E (CP-5 part 1a): the Gate-1 release
    // authorization (Package B spec section 7.3). Derived in
    // captureForFrame from the authorization constant in
    // llvrtrackingsettings.cpp; false in every build whose tree
    // does not contain an authorization commit. Not a setting:
    // no control, file, or environment input exists. Beside
    // selective_verified and independent of it: that field's
    // capture line and its one consumer are untouched.
    bool stage2_constraint_authorized = false;
    // The authorization stage of this build, recorded for
    // traces and evidence labeling: 0 = none, 1 = Stage A
    // candidate verification, 2 = Stage B production (the
    // FSVRGate1Authorization value). Derived with the field
    // above, from the same constant.
    U8 gate1_authorization_stage = 0;
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
    static const char* const SETTING_LEAN_RESPONSE_GAIN;
    static const char* const SETTING_POSTURE_SWAP;
    static const char* const SETTING_ROOT_RESPONSE_GAIN;
    static const char* const SETTING_STANDING_EYE_HEIGHT_CM;
    static const char* const SETTING_MANUAL_LEAN_RATIO;
    static const char* const SETTING_MANUAL_ROOT_RATIO;
    static const char* const SETTING_ROOT_FOLLOW_BOUND_METERS;
    static const char* const SETTING_TELEPORT_DETECTOR_PROVEN;
    static const char* const SETTING_SHOW_POSTURE_INDICATOR;
};

#endif // LL_LLVRTRACKINGSETTINGS_H
