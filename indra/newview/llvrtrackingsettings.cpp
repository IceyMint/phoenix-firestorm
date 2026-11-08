/**
 * @file llvrtrackingsettings.cpp
 * @brief P2-07a Stage 1: VR tracking settings parser and snapshots.
 */

#include "llviewerprecompiledheaders.h"

#include "llvrtrackingsettings.h"

#include "llcontrol.h"
#include "llviewercontrol.h" // gSavedSettings, gSavedPerAccountSettings
#include "llvrboneregistry.h"

#include <cmath>

const char* const FSVRTrackingSettings::SETTING_DISABLED_BONES = "FSVRTrackingDisabledBones";
const char* const FSVRTrackingSettings::SETTING_TRACK_HEAD_PITCH = "FSVRTrackHeadPitch";
const char* const FSVRTrackingSettings::SETTING_TRACK_HEAD_ROLL = "FSVRTrackHeadRoll";
const char* const FSVRTrackingSettings::SETTING_BODY_FOLLOWS_HEADSET = "FVRBodyFollowsHeadset";
const char* const FSVRTrackingSettings::SETTING_TRACK_HEAD_POSITION = "FSVRTrackHeadPosition";
const char* const FSVRTrackingSettings::SETTING_TRACK_HAND_POSITION = "FSVRTrackHandPosition";
const char* const FSVRTrackingSettings::SETTING_SOLVER_BEND_DISABLED_BONES = "FSVRSolverBendDisabledBones";
const char* const FSVRTrackingSettings::SETTING_CALIBRATION_RECORD = "FSVRCalibrationRecord";
const char* const FSVRTrackingSettings::SETTING_DIAG_BEND_LIMITS = "FSVRDiagBendLimits";
const char* const FSVRTrackingSettings::SETTING_DIAG_SOFT_PREFERENCE = "FSVRDiagSoftPreference";
const char* const FSVRTrackingSettings::SETTING_DIAG_FORCE_UNIT_SCALE = "FSVRDiagForceUnitScale";
const char* const FSVRTrackingSettings::SETTING_DIAG_POSITIONAL_SOLVE = "FSVRDiagPositionalSolve";
const char* const FSVRTrackingSettings::SETTING_DIAG_FULL_CHAIN_SOLVE = "FSVRDiagFullChainSolve";
const char* const FSVRTrackingSettings::SETTING_LEAN_RESPONSE_GAIN = "FSVRLeanResponseGain";
const char* const FSVRTrackingSettings::SETTING_POSTURE_SWAP = "FSVRPostureSwap";
const char* const FSVRTrackingSettings::SETTING_ROOT_RESPONSE_GAIN = "FSVRRootResponseGain";
const char* const FSVRTrackingSettings::SETTING_STANDING_EYE_HEIGHT_CM = "FSVRStandingEyeHeightCm";
const char* const FSVRTrackingSettings::SETTING_MANUAL_LEAN_RATIO = "FSVRManualLeanRatio";
const char* const FSVRTrackingSettings::SETTING_MANUAL_ROOT_RATIO = "FSVRManualRootRatio";
const char* const FSVRTrackingSettings::SETTING_ROOT_FOLLOW_BOUND_METERS = "FSVRRootFollowBoundMeters";

// P2-07B C6-E Gate-1 authorization (CP-5 part 1a). NONE in every
// ordinary build. Advanced ONLY by a dedicated authorization
// commit that changes this constant and nothing else, citing its
// evidence (CP-5 part 4: Stage A candidate verification, then
// Stage B production activation). captureForFrame derives the
// snapshot's stage2_constraint_authorized and
// gate1_authorization_stage fields from this value and from
// nothing else. There is no other producer: no settings control,
// debug toggle, environment variable, or file input exists.
static constexpr FSVRGate1Authorization kGate1Authorization =
    FSVRGate1Authorization::NONE;

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
    case LLSD::TypeURI:
    {
        const std::string text = value.asString();
        out += "uri:";
        out += std::to_string(text.size());
        out += ":";
        out += text;
        out += ";";
        break;
    }
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
            // Keys are length-prefixed: without the length, two
            // different maps can render identically ({"a"=1,"b"=2}
            // vs {"a=i:1;b"=2}) and a real change would be missed
            // (review finding P207A-S1-03, second round).
            out += "k:";
            out += std::to_string(it->first.size());
            out += ":";
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
// P2-07B C4 (CP-3): last-observed raw values for the eight new
// snapshot fields, initialized to the spec defaults so a fresh
// settings file's first capture is not a spurious "change" beyond
// the initialization capture itself.
bool s_last_track_head_position = false;
bool s_last_track_hand_position = false;
bool s_last_solver_bend_disabled_bones = false;
bool s_last_diag_bend_limits = true;
bool s_last_diag_soft_preference = true;
bool s_last_diag_force_unit_scale = false;
bool s_last_diag_positional_solve = true;
bool s_last_diag_full_chain_solve = true;
// P2-07B Stage 1 (CP-S1-3): last-observed lean gain, in
// the snapshot's derived form -- the (validity, sanitized
// value) pair -- initialized to the declared default's
// derived form so a fresh settings file's first capture is
// not a spurious "change" beyond the initialization
// capture itself. Change detection compares this derived
// pair, never the raw F32 [S1-CODE-R1-01]: a NaN raw
// value never equals itself, so the raw comparison
// registered a persistent NaN as a change on every
// capture and churned the snapshot generation.
F32 s_last_lean_gain = 1.f;
bool s_last_lean_gain_valid = true;
// P2-07B Fix 3 (brief section 2.6): last-observed Fix 3
// values, in the snapshot's derived forms like the lean
// gain's pair above -- the posture swap flag, the root
// gain's (validity, sanitized value) pair, and the eye
// height's (set, invalid, sanitized value) triple --
// initialized to the declared defaults' derived forms.
bool s_last_posture_swap = false;
F32 s_last_root_gain = 1.f;
bool s_last_root_gain_valid = true;
F32 s_last_standing_eye_height_cm = 0.f;
bool s_last_standing_eye_height_set = false;
bool s_last_standing_eye_height_invalid = false;
// P2-07B Fix 3 [FIX3-CODE-R1-01]: the manual ratios'
// derived triples, one per channel, in the same form.
F32 s_last_manual_lean_ratio = 0.f;
bool s_last_manual_lean_set = false;
bool s_last_manual_lean_invalid = false;
F32 s_last_manual_root_ratio = 0.f;
bool s_last_manual_root_set = false;
bool s_last_manual_root_invalid = false;
// P2-07B Fix 3 [FIX3-CODE-R3]: the root-follow bound's
// derived pair (sanitized value, validity), initialized
// to the declared default's derived form (2.5, valid).
F32 s_last_root_follow_bound_m = 2.5f;
bool s_last_root_follow_bound_valid = true;
FSVRTrackingSettingsSnapshot s_snapshot;
bool s_snapshot_initialized = false;

// P2-07B C4 (CP-4): coupling listener state. The connection is
// held for process lifetime (the control outlives the process).
// s_coupling_baseline is bookkeeping only [R1]: transitions are
// measured from each signal's own (previous, new) pair, never
// from recorded state.
bool s_coupling_installed = false;
boost::signals2::scoped_connection s_coupling_connection;
LLSD s_coupling_baseline;

void onDisabledBonesCommitted(LLControlVariable* control,
                              const LLSD& new_value,
                              const LLSD& previous_value)
{
    (void)control;
    s_coupling_baseline = new_value; // bookkeeping only [R1]
    if (FSVRTrackingSettings::couplingWouldFire(previous_value, new_value))
    {
        // The one coupling write: one-way, Head position only.
        // Never writes false; never touches FSVRTrackHandPosition.
        gSavedSettings.setBOOL(FSVRTrackingSettings::SETTING_TRACK_HEAD_POSITION,
                               true);
    }
}

// Installs the coupling subscription once, lazily, on the first
// captureForFrame() call (both control groups are fully loaded by
// then). [R1 -- C4-01] Installation includes the seeding read: the
// control's CURRENT value is recorded as the baseline, performing
// no coupling. The baseline is never established by a commit
// event; every post-installation event, the first included, is a
// genuine transition on its own (previous, new) pair.
void installCouplingListener()
{
    if (s_coupling_installed)
    {
        return;
    }
    LLControlVariable* control =
        gSavedSettings.getControl(FSVRTrackingSettings::SETTING_DISABLED_BONES);
    if (!control)
    {
        // Not declared yet: retry on a later capture. (Production
        // declares the control in settings.xml, so this is a
        // harness-ordering guard, not an expected path.)
        return;
    }
    s_coupling_baseline = control->getValue(); // [R1] seeding read
    s_coupling_connection =
        control->getCommitSignal()->connect(&onDisabledBonesCommitted);
    s_coupling_installed = true;
}
} // namespace

// static
const FSVRTrackingSettingsSnapshot& FSVRTrackingSettings::captureForFrame()
{
    // P2-07B C4 (CP-4): the coupling listener installs lazily on
    // the first capture; see installCouplingListener for the
    // [R1] seeding contract.
    installCouplingListener();

    const LLSD raw_disabled = gSavedSettings.getLLSD(SETTING_DISABLED_BONES);
    const bool pitch = gSavedSettings.getBOOL(SETTING_TRACK_HEAD_PITCH);
    const bool roll = gSavedSettings.getBOOL(SETTING_TRACK_HEAD_ROLL);
    const bool body_follow = gSavedSettings.getBOOL(SETTING_BODY_FOLLOWS_HEADSET);
    // P2-07B C4 (CP-3): the eight new raw values. Seven are read
    // from gSavedSettings; the L4 toggle is the single cross-group
    // read -- its value lives in gSavedPerAccountSettings.
    const bool track_head_position =
        gSavedSettings.getBOOL(SETTING_TRACK_HEAD_POSITION);
    const bool track_hand_position =
        gSavedSettings.getBOOL(SETTING_TRACK_HAND_POSITION);
    const bool solver_bend_disabled_bones = solverBendDisabledBones();
    const bool diag_bend_limits = gSavedSettings.getBOOL(SETTING_DIAG_BEND_LIMITS);
    const bool diag_soft_preference =
        gSavedSettings.getBOOL(SETTING_DIAG_SOFT_PREFERENCE);
    const bool diag_force_unit_scale =
        gSavedSettings.getBOOL(SETTING_DIAG_FORCE_UNIT_SCALE);
    const bool diag_positional_solve =
        gSavedSettings.getBOOL(SETTING_DIAG_POSITIONAL_SOLVE);
    const bool diag_full_chain_solve =
        gSavedSettings.getBOOL(SETTING_DIAG_FULL_CHAIN_SOLVE);
    // P2-07B Stage 1 (CP-S1-3): the lean gain's raw value,
    // read beside the diag reads, and the snapshot
    // derivation, computed here rather than in the snapshot
    // fill below so change detection can compare the
    // derived pair (see the tuple below and
    // [S1-CODE-R1-01]). Validation is finite AND within
    // the bounded domain 0.05..3.0 (DP-S1-2 /
    // S1-BRIEF-R1-02 -- control limits, not body
    // measurements): an out-of-domain value is rejected
    // outright, never clamped to a value the user did
    // not set.
    const F32 lean_gain_raw =
        gSavedSettings.getF32(SETTING_LEAN_RESPONSE_GAIN);
    const bool lean_gain_valid = std::isfinite(lean_gain_raw) &&
                                 lean_gain_raw >= 0.05f &&
                                 lean_gain_raw <= 3.0f;
    const F32 lean_gain = lean_gain_valid ? lean_gain_raw : 1.f;
    // P2-07B Fix 3 (brief section 2.6): the Fix 3 raw
    // values, read beside the lean gain, with their
    // derivations computed here in the same style. The
    // root gain takes the lean gain's exact contract
    // (finite AND within 0.05..3.0; rejected outright,
    // never clamped). The eye height's declared 0 is
    // the unset state -- a first-class provisional
    // state, not an error; a finite positive value is
    // set; anything else (negative, NaN, infinite) is
    // present-but-invalid, carried as typed state only
    // -- neither channel consumes the value while
    // OQ-7's references are unresolved, so its
    // invalidity fails no channel and is reported
    // against none [FIX3-CODE-R1-04]. Its sanitized
    // value is 0 whenever it is not set.
    const bool posture_swap =
        gSavedSettings.getBOOL(SETTING_POSTURE_SWAP);
    const F32 root_gain_raw =
        gSavedSettings.getF32(SETTING_ROOT_RESPONSE_GAIN);
    const bool root_gain_valid = std::isfinite(root_gain_raw) &&
                                 root_gain_raw >= 0.05f &&
                                 root_gain_raw <= 3.0f;
    const F32 root_gain = root_gain_valid ? root_gain_raw : 1.f;
    const F32 eye_height_raw =
        gSavedSettings.getF32(SETTING_STANDING_EYE_HEIGHT_CM);
    const bool eye_height_set = std::isfinite(eye_height_raw) &&
                                eye_height_raw > 0.f;
    const bool eye_height_invalid = !eye_height_set &&
                                    eye_height_raw != 0.f;
    const F32 eye_height_cm = eye_height_set ? eye_height_raw : 0.f;
    // P2-07B Fix 3 [FIX3-CODE-R1-01]: the manual limb
    // ratios, one derivation per channel in the eye
    // height's style. The declared 0 is UNSET (the
    // setting absent and a deliberate 0 are the same
    // state by specification): the channel's derived
    // ratio or provisional baseline rules, silently.
    // A present value engages only inside the valid
    // domain (finite AND within 0.05..3.0 -- the domain
    // is the main chat's design specification, recorded
    // in P2-07B-FIX3-CODE-REVISION-R2.md for the R2
    // verdict to rule on; it mirrors the gain domain
    // deliberately, a limb-length ratio outside that
    // span not being a body proportion); a present
    // value outside it (out-of-domain, negative, NaN,
    // infinite) is INVALID and fails its channel closed
    // at the composition seam -- never clamped, never
    // quietly replaced by the derived value (brief
    // section 2.4). The carried value is the raw value
    // when valid and 0 otherwise, so the snapshot never
    // carries a non-finite or out-of-domain F32
    // downstream; a NaN raw reads as present (NaN != 0)
    // and lands INVALID, never as a per-capture change
    // (the derived triple, not the raw F32, rides the
    // change tuple below -- [S1-CODE-R1-01]'s rule).
    const F32 manual_lean_raw =
        gSavedSettings.getF32(SETTING_MANUAL_LEAN_RATIO);
    const bool manual_lean_set = manual_lean_raw != 0.f;
    const bool manual_lean_valid = std::isfinite(manual_lean_raw) &&
                                   manual_lean_raw >= 0.05f &&
                                   manual_lean_raw <= 3.0f;
    const bool manual_lean_invalid = manual_lean_set &&
                                     !manual_lean_valid;
    const F32 manual_lean_ratio = manual_lean_valid ? manual_lean_raw
                                                    : 0.f;
    const F32 manual_root_raw =
        gSavedSettings.getF32(SETTING_MANUAL_ROOT_RATIO);
    const bool manual_root_set = manual_root_raw != 0.f;
    const bool manual_root_valid = std::isfinite(manual_root_raw) &&
                                   manual_root_raw >= 0.05f &&
                                   manual_root_raw <= 3.0f;
    const bool manual_root_invalid = manual_root_set &&
                                     !manual_root_valid;
    const F32 manual_root_ratio = manual_root_valid ? manual_root_raw
                                                    : 0.f;
    // P2-07B Fix 3 [FIX3-CODE-R3]: the root-follow bound
    // (OQ-2 as ruled by Leah 2026-11-07: STOP AT THE
    // BOUND). The declared default is 2.5 physical
    // metres (the main chat's design call, recorded in
    // P2-07B-FIX3-CODE-REVISION-R3.md for the R3
    // ruling). The derivation's valid domain is finite
    // AND 0 < v <= 10: zero is the setting's absent
    // form and not a radius; a negative or non-finite
    // value is not a radius; a value beyond 10 m is
    // beyond any physical playspace and, in the common
    // case, a unit error (centimetres typed as metres)
    // that must fail loudly rather than silently become
    // effectively unbounded following. The carried value
    // is the raw value when valid and 0 otherwise, so
    // the snapshot never carries a non-finite or
    // out-of-domain F32 downstream. When the bound is
    // invalid the policy is not in force: the adapter
    // keeps live root movement gated off (never
    // unbounded) and preserves the shipped standing
    // lean composition [FIX3-CODE-R2-01], announcing
    // the cause in the record. The derived pair, never
    // the raw F32, rides the change tuple below
    // ([S1-CODE-R1-01]'s rule, so a NaN raw cannot
    // re-fire a generation change every capture).
    const F32 root_follow_bound_raw =
        gSavedSettings.getF32(SETTING_ROOT_FOLLOW_BOUND_METERS);
    const bool root_follow_bound_valid =
        std::isfinite(root_follow_bound_raw) &&
        root_follow_bound_raw > 0.f &&
        root_follow_bound_raw <= 10.f;
    const F32 root_follow_bound_m = root_follow_bound_valid
                                        ? root_follow_bound_raw
                                        : 0.f;
    const std::string canonical = canonicalLLSD(raw_disabled);

    const bool changed = !s_snapshot_initialized || !s_last_canonical_valid ||
                         canonical != s_last_canonical ||
                         pitch != s_last_pitch || roll != s_last_roll ||
                         body_follow != s_last_body_follow ||
                         track_head_position != s_last_track_head_position ||
                         track_hand_position != s_last_track_hand_position ||
                         solver_bend_disabled_bones !=
                             s_last_solver_bend_disabled_bones ||
                         diag_bend_limits != s_last_diag_bend_limits ||
                         diag_soft_preference != s_last_diag_soft_preference ||
                         diag_force_unit_scale != s_last_diag_force_unit_scale ||
                         diag_positional_solve != s_last_diag_positional_solve ||
                         diag_full_chain_solve != s_last_diag_full_chain_solve ||
                         lean_gain != s_last_lean_gain ||
                         lean_gain_valid != s_last_lean_gain_valid ||
                         posture_swap != s_last_posture_swap ||
                         root_gain != s_last_root_gain ||
                         root_gain_valid != s_last_root_gain_valid ||
                         eye_height_cm != s_last_standing_eye_height_cm ||
                         eye_height_set != s_last_standing_eye_height_set ||
                         eye_height_invalid !=
                             s_last_standing_eye_height_invalid ||
                         manual_lean_ratio != s_last_manual_lean_ratio ||
                         manual_lean_set != s_last_manual_lean_set ||
                         manual_lean_invalid !=
                             s_last_manual_lean_invalid ||
                         manual_root_ratio != s_last_manual_root_ratio ||
                         manual_root_set != s_last_manual_root_set ||
                         manual_root_invalid !=
                             s_last_manual_root_invalid ||
                         root_follow_bound_m !=
                             s_last_root_follow_bound_m ||
                         root_follow_bound_valid !=
                             s_last_root_follow_bound_valid;
    if (!changed)
    {
        return s_snapshot;
    }

    s_last_canonical = canonical;
    s_last_canonical_valid = true;
    s_last_pitch = pitch;
    s_last_roll = roll;
    s_last_body_follow = body_follow;
    s_last_track_head_position = track_head_position;
    s_last_track_hand_position = track_hand_position;
    s_last_solver_bend_disabled_bones = solver_bend_disabled_bones;
    s_last_diag_bend_limits = diag_bend_limits;
    s_last_diag_soft_preference = diag_soft_preference;
    s_last_diag_force_unit_scale = diag_force_unit_scale;
    s_last_diag_positional_solve = diag_positional_solve;
    s_last_diag_full_chain_solve = diag_full_chain_solve;
    s_last_lean_gain = lean_gain;
    s_last_lean_gain_valid = lean_gain_valid;
    s_last_posture_swap = posture_swap;
    s_last_root_gain = root_gain;
    s_last_root_gain_valid = root_gain_valid;
    s_last_standing_eye_height_cm = eye_height_cm;
    s_last_standing_eye_height_set = eye_height_set;
    s_last_standing_eye_height_invalid = eye_height_invalid;
    s_last_manual_lean_ratio = manual_lean_ratio;
    s_last_manual_lean_set = manual_lean_set;
    s_last_manual_lean_invalid = manual_lean_invalid;
    s_last_manual_root_ratio = manual_root_ratio;
    s_last_manual_root_set = manual_root_set;
    s_last_manual_root_invalid = manual_root_invalid;
    s_last_root_follow_bound_m = root_follow_bound_m;
    s_last_root_follow_bound_valid = root_follow_bound_valid;

    FSVRTrackingSettingsSnapshot next;
    next.generation = s_snapshot_initialized ? s_snapshot.generation + 1 : 1;
    next.disabled_bones =
        parseDisabledBones(raw_disabled, FSVRBoneRegistry::knownIds());
    next.head_pitch_enabled = pitch;
    next.head_roll_enabled = roll;
    next.body_follows_headset = body_follow;
    // P2-07B C4 (CP-3): the new fields are carried, not
    // interpreted, by the snapshot. No coupling is derived here
    // (the coupling is CP-4's write-side rule, never a capture
    // derivation), and the calibration record is NOT read here
    // (record consumption is a separate channel, DP-4).
    next.track_head_position = track_head_position;
    next.track_hand_position = track_hand_position;
    next.solver_bend_disabled_bones = solver_bend_disabled_bones;
    next.diag_bend_limits = diag_bend_limits;
    next.diag_soft_preference = diag_soft_preference;
    next.diag_force_unit_scale = diag_force_unit_scale;
    next.diag_positional_solve = diag_positional_solve;
    next.diag_full_chain_solve = diag_full_chain_solve;
    // P2-07B Stage 1 (CP-S1-3): the lean gain's snapshot
    // fields, from the derivation beside the raw read
    // above. E5 fails closed on the validity flag
    // (CP-S1-4). When invalid, lean_gain is pinned to
    // 1.0 -- inert under the failed E5 -- so the snapshot
    // never carries a non-finite or out-of-domain F32
    // downstream. LL controls do not clamp F32 values;
    // this derivation is the enforcement. A missing or
    // mistyped control degrades through getF32 to the
    // declared 1.0, valid: the control is declared in
    // settings.xml in the same change, so that path is
    // belt-and-braces, in the C4 idiom. The derived pair
    // rides the change tuple above, so a change in the
    // snapshot's gain state advances the generation and
    // retires continuity history through the existing
    // term -- a solved pose produced under one gain is
    // never a soft-preference witness under another
    // (OPEN-S1-5).
    next.lean_gain_valid = lean_gain_valid;
    next.lean_gain = lean_gain;
    // P2-07B Fix 3 (brief section 2.6): the Fix 3 fields,
    // from the derivations beside the raw reads above.
    // The derived forms ride the change tuple, so a
    // change in any of them advances the generation and
    // retires continuity history through the existing
    // term, exactly as a gain change does.
    next.posture_swap = posture_swap;
    next.root_gain_valid = root_gain_valid;
    next.root_gain = root_gain;
    next.standing_eye_height_cm = eye_height_cm;
    next.standing_eye_height_set = eye_height_set;
    next.standing_eye_height_invalid = eye_height_invalid;
    // P2-07B Fix 3 [FIX3-CODE-R1-01]: the manual
    // ratios' derived triples, from the derivations
    // beside the raw reads above. They ride the change
    // tuple, so a change in either channel's manual
    // state advances the generation and retires
    // continuity history through the existing term,
    // exactly as a gain change does.
    next.manual_lean_ratio = manual_lean_ratio;
    next.manual_lean_set = manual_lean_set;
    next.manual_lean_invalid = manual_lean_invalid;
    next.manual_root_ratio = manual_root_ratio;
    next.manual_root_set = manual_root_set;
    next.manual_root_invalid = manual_root_invalid;
    // P2-07B Fix 3 [FIX3-CODE-R3]: the root-follow
    // bound's derived pair, from the derivation beside
    // the raw read above. It rides the change tuple, so
    // a change in the bound state advances the
    // generation and retires continuity history through
    // the existing term, exactly as a gain change does.
    next.root_follow_bound_m = root_follow_bound_m;
    next.root_follow_bound_valid = root_follow_bound_valid;
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
    // P2-07B C6-E (CP-5 part 1a): the Gate-1 authorization
    // derivation, beside the selective_verified hard-false and
    // independent of it. The constant is build-time, so the
    // derivation rides the snapshot build like every other
    // field here; it is not part of the change detection above
    // because it cannot change within a build.
    next.stage2_constraint_authorized =
        (kGate1Authorization != FSVRGate1Authorization::NONE);
    next.gate1_authorization_stage =
        static_cast<U8>(kGate1Authorization);

    s_snapshot = next;
    s_snapshot_initialized = true;
    return s_snapshot;
}

// ---------------------------------------------------------------------------
// P2-07B C4 (CP-4): the Upper-body -> Head-position coupling.
//
// A one-way, write-side rule observed through the disabled-bones
// control's commit signal. The handler (installCouplingListener,
// above) evaluates each event with this pure decision over the
// signal's own (previous, new) pair.
//
// OQ-2 finding (verified at C4 coding time, 2026-11-03): YES --
// LLControlGroup::loadFromFile DOES fire commit signals when a
// loaded value differs from the control's current value.
// LLControlVariable::setValue fires firePropertyChanged on any
// comparable change (indra/llxml/llcontrol.cpp:225, fire at :271;
// the signal invocation is indra/llxml/llcontrol.h:167-170), and
// loadFromFile applies persisted values through setValue
// (llcontrol.cpp:1180) and defaults through setDefaultValue,
// which also fires on change (llcontrol.cpp setDefaultValue);
// the legacy fallback loadFromFileLegacy applies values through
// the typed set() wrappers, which are setValue (llcontrol.h:158).
// CP-4 stays correct under this answer by construction: in the
// real session flow the settings files load before the first
// captureForFrame installs the listener, so load events are
// simply unobserved; any load after installation arrives as an
// ordinary (previous, new) pair and is evaluated like any other
// write -- and the [R1] seeding contract guarantees the state a
// load leaves behind can never, by itself, fire the coupling.
// ---------------------------------------------------------------------------

// static
bool FSVRTrackingSettings::couplingWouldFire(const LLSD& previous_document,
                                             const LLSD& new_document)
{
    const std::set<std::string> known_ids = FSVRBoneRegistry::knownIds();
    const FSVRTrackingDisabledBoneSet previous =
        parseDisabledBones(previous_document, known_ids);
    const FSVRTrackingDisabledBoneSet next =
        parseDisabledBones(new_document, known_ids);

    // A document the viewer itself treats as not-writable
    // (malformed or future-version, in either position) computes
    // no transition and never fires: a settings side effect must
    // not be produced by a document that cannot drive anything.
    if (previous.parse_status != FSVRTrackingDisabledBoneSet::ParseStatus::VALID ||
        next.parse_status != FSVRTrackingDisabledBoneSet::ParseStatus::VALID)
    {
        return false;
    }

    S32 previous_upper_disabled = -1;
    S32 next_upper_disabled = -1;
    for (const FSVRRollupState& state :
         FSVRBoneRegistry::computeRollups(previous.known_disabled_ids))
    {
        if (state.group == FSVRRollup::UPPER_BODY)
        {
            previous_upper_disabled = state.disabled_count;
        }
    }
    for (const FSVRRollupState& state :
         FSVRBoneRegistry::computeRollups(next.known_disabled_ids))
    {
        if (state.group == FSVRRollup::UPPER_BODY)
        {
            next_upper_disabled = state.disabled_count;
        }
    }
    // The group-enable transition (DP-1): Upper body's bones
    // leave the disabled set entirely.
    return previous_upper_disabled > 0 && next_upper_disabled == 0;
}

// ---------------------------------------------------------------------------
// P2-07B C4 (CP-5): the per-account store surface.
// ---------------------------------------------------------------------------

// static
bool FSVRTrackingSettings::solverBendDisabledBones()
{
    return gSavedPerAccountSettings.getBOOL(SETTING_SOLVER_BEND_DISABLED_BONES);
}

// static
LLSD FSVRTrackingSettings::calibrationRecordRaw()
{
    return gSavedPerAccountSettings.getLLSD(SETTING_CALIBRATION_RECORD);
}

// static
void FSVRTrackingSettings::writeCalibrationRecord(const LLSD& record)
{
    gSavedPerAccountSettings.setLLSD(SETTING_CALIBRATION_RECORD, record);
}

namespace
{
// Numeric field acceptance for the section 3.3 validator: a real
// or integer LLSD value (an XML round trip can render a whole
// value as an integer) whose value is finite.
bool finiteRecordNumber(const LLSD& value, F64& out)
{
    if (value.isReal())
    {
        out = value.asReal();
    }
    else if (value.isInteger())
    {
        out = static_cast<F64>(value.asInteger());
    }
    else
    {
        return false;
    }
    return std::isfinite(out);
}

bool validRecordRatio(const LLSD& value, F64& out)
{
    return finiteRecordNumber(value, out) && out > 0.0;
}
} // namespace

// static
FSVRCalibrationRecordValidation
FSVRTrackingSettings::validateCalibrationRecord(const LLSD& raw_record)
{
    // C4-CODE-01: parse into a local candidate only. Every
    // rejection path returns a fresh default
    // FSVRCalibrationRecordValidation{} -- a rejected record is
    // never partially read, so no parsed field (head fields
    // included) may survive on a NO_RECORD result. Parsed values
    // are returned only for USABLE_HEAD_ONLY or USABLE_FULL.
    FSVRCalibrationRecordValidation parsed;

    // Section 3.3 load validation. Any failed clause is
    // no-record: a rejected record is never partially read.
    if (!raw_record.isMap() || raw_record.size() == 0)
    {
        return FSVRCalibrationRecordValidation{};
    }
    if (!raw_record.has("version") || !raw_record["version"].isInteger() ||
        raw_record["version"].asInteger() != CALIBRATION_RECORD_SCHEMA_VERSION)
    {
        return FSVRCalibrationRecordValidation{};
    }
    // floor_verified records that the capture's anchor was
    // verified through the section 4.2 gates (Package B spec
    // section 4.3 / DP-B1: the field's meaning is the
    // calibration-anchored verification, not a physical-floor
    // check); a record without it MUST NOT be used (and per
    // section 3.3 must never have been committed).
    if (!raw_record.has("floor_verified") ||
        !raw_record["floor_verified"].isBoolean() ||
        !raw_record["floor_verified"].asBoolean())
    {
        return FSVRCalibrationRecordValidation{};
    }

    // Head fields: present, finite; the ratio finite and positive.
    if (!raw_record.has("head_ratio") ||
        !validRecordRatio(raw_record["head_ratio"], parsed.head_ratio) ||
        !raw_record.has("avatar_cornea_height") ||
        !finiteRecordNumber(raw_record["avatar_cornea_height"],
                            parsed.avatar_cornea_height) ||
        !raw_record.has("player_hmd_height") ||
        !finiteRecordNumber(raw_record["player_hmd_height"],
                            parsed.player_hmd_height))
    {
        return FSVRCalibrationRecordValidation{};
    }
    if (raw_record.has("captured_utc") &&
        raw_record["captured_utc"].isInteger())
    {
        parsed.captured_utc = raw_record["captured_utc"].asInteger();
    }

    // Arm fields: either all present and valid, or all absent
    // (head-only). Any partial presence or invalid value rejects
    // the whole record.
    static const char* const arm_fields[] = {"arm_ratio_left",
                                             "arm_ratio_right",
                                             "avatar_reach_left",
                                             "avatar_reach_right",
                                             "player_reach_left",
                                             "player_reach_right"};
    bool any_arm_present = false;
    bool all_arm_present = true;
    for (const char* field : arm_fields)
    {
        if (raw_record.has(field))
        {
            any_arm_present = true;
        }
        else
        {
            all_arm_present = false;
        }
    }
    if (!any_arm_present)
    {
        parsed.status = FSVRCalibrationRecordValidation::RecordStatus::USABLE_HEAD_ONLY;
        parsed.hand_calibration_present = false;
        return parsed;
    }
    if (!all_arm_present ||
        !validRecordRatio(raw_record["arm_ratio_left"], parsed.arm_ratio_left) ||
        !validRecordRatio(raw_record["arm_ratio_right"],
                          parsed.arm_ratio_right) ||
        !finiteRecordNumber(raw_record["avatar_reach_left"],
                            parsed.avatar_reach_left) ||
        !finiteRecordNumber(raw_record["avatar_reach_right"],
                            parsed.avatar_reach_right) ||
        !finiteRecordNumber(raw_record["player_reach_left"],
                            parsed.player_reach_left) ||
        !finiteRecordNumber(raw_record["player_reach_right"],
                            parsed.player_reach_right))
    {
        // Rejection is total (C4-CODE-01): return a fresh default
        // so neither the arm values read so far nor the head
        // values parsed above survive on a NO_RECORD result.
        return FSVRCalibrationRecordValidation{};
    }

    parsed.status = FSVRCalibrationRecordValidation::RecordStatus::USABLE_FULL;
    parsed.hand_calibration_present = true;
    return parsed;
}
