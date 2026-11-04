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
                         diag_full_chain_solve != s_last_diag_full_chain_solve;
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
    // Capture-completion is implied by floor_verified == true; a
    // record whose floor was not verified MUST NOT be used (and
    // per section 3.3 must never have been committed).
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
