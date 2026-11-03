/**
 * @file llvrtrackingframe.cpp
 * @brief P2-07a Stage 1: frame coordination, immutable decision
 *        publication and vr_tracking trace records (spec 3.2, 6.2).
 *
 * Stage 1 contract: this code observes the legacy path and publishes
 * shadow decisions. It does not modify joint states, does not add an
 * XR locate, and does not add an animation evaluation.
 */

#include "llviewerprecompiledheaders.h"

#include "llvrtrackingframe.h"

#include "llopenxrmanager.h"
#include "llopenxrtrace.h"
#include "llvoavatarself.h"
#include "llvrtrackingarbitrator.h"
#include "llvrboneregistry.h"

#include <sstream>

namespace
{
// XR_HAND_JOINT_COUNT_EXT, restated (this TU carries no OpenXR
// headers; see llvrtrackingframe.h's mask-helper contract).
constexpr U32 kHandJointCount = 26;

FSVRTrackingSettingsSnapshot s_pending_snapshot;
U64 s_pending_token = 0;
bool s_pending_valid = false;

FSVRTrackingFrameDecisions s_current;
bool s_current_valid = false;

U64 s_last_traced_generation = 0;

void emitTrace(const FSVRTrackingFrameDecisions& decisions,
               const FSVRTrackingFrameInput& input,
               const FSVRTrackingSettingsSnapshot& snapshot)
{
    if (!LLOpenXRTrace::enabled())
    {
        return;
    }

    if (snapshot.generation != s_last_traced_generation)
    {
        s_last_traced_generation = snapshot.generation;
        // LLOpenXRTrace::writeRecord wraps the payload in the
        // record's own braces: pass bare comma-separated fields,
        // never an enclosing object (review finding P207A-S1-01).
        std::ostringstream settings_fields;
        settings_fields << "\"settings_generation\":" << snapshot.generation
                        << ",\"schema_version\":"
                        << snapshot.disabled_bones.schema_version
                        << ",\"schema_valid\":"
                        << (snapshot.schema_valid ? "true" : "false")
                        << ",\"selective_requested\":"
                        << (snapshot.selective_requested ? "true" : "false");
        LLOpenXRTrace::writeRecord("vr_tracking_settings",
                                   settings_fields.str());
    }

    const std::vector<FSVRRollupState> rollups =
        FSVRBoneRegistry::computeRollups(
            snapshot.disabled_bones.known_disabled_ids);
    std::string head_rollup = "not_implemented";
    std::string upper_body_rollup = "not_implemented";
    std::string arms_rollup = "not_implemented";
    std::string lower_body_rollup = "not_implemented";
    for (const FSVRRollupState& rollup : rollups)
    {
        if (rollup.group == FSVRRollup::HEAD)
        {
            head_rollup = rollup.derived_state;
        }
        else if (rollup.group == FSVRRollup::UPPER_BODY)
        {
            upper_body_rollup = rollup.derived_state;
        }
        else if (rollup.group == FSVRRollup::ARMS_INCLUDING_HANDS)
        {
            arms_rollup = rollup.derived_state;
        }
        else if (rollup.group == FSVRRollup::LOWER_BODY)
        {
            lower_body_rollup = rollup.derived_state;
        }
    }

    std::ostringstream fields;
    fields << "\"settings_generation\":" << decisions.settings_generation
           << ",\"schema_version\":" << snapshot.disabled_bones.schema_version
           << ",\"schema_valid\":"
           << (snapshot.schema_valid ? "true" : "false")
           << ",\"legacy_fast_path\":true"
           << ",\"selective_requested\":"
           << (decisions.selective_requested ? "true" : "false")
           << ",\"selective_applied\":false"
           << ",\"applied_basis\":\"legacy_policy_estimate\""
           << ",\"app_frame_token\":" << decisions.app_frame_token
           << ",\"hmd_published_locate_generation\":";
    if (input.has_published_generation)
    {
        fields << input.published_locate_generation;
    }
    else
    {
        fields << "null";
    }
    fields << ",\"hmd_consumed_generation\":";
    if (input.has_sample)
    {
        fields << input.locate_generation;
    }
    else
    {
        fields << "null";
    }
    fields << ",\"source_active\":"
           << ((input.override_owned && input.has_sample) ? "true" : "false")
           << ",\"source_pose_valid\":"
           << ((input.has_sample && input.sample_valid && input.sample_tracked)
                   ? "true"
                   : "false")
           << ",\"override_hold\":"
           << (input.interruption_hold ? "true" : "false")
           << ",\"head_rollup\":\"" << head_rollup << "\""
           << ",\"upper_body_rollup\":\"" << upper_body_rollup << "\""
           << ",\"arms_rollup\":\"" << arms_rollup << "\""
           << ",\"lower_body_rollup\":\"" << lower_body_rollup << "\""
           // AMEND-02 (code cross-check R1): other_rollups is a
           // frozen legacy field -- CP-8 is additive-only, so it
           // keeps its original value; LOWER_BODY's real state is
           // the distinct lower_body_rollup field above.
           << ",\"other_rollups\":\"not_implemented\""
           << ",\"hand_tracking_available\":"
           << (input.hand_tracking_available ? "true" : "false")
           << ",\"hmd_position_valid\":"
           << (input.hmd_position_valid ? "true" : "false")
           << ",\"hand_sample_present\":["
           << (input.hand_sample_present[0] ? "true" : "false") << ","
           << (input.hand_sample_present[1] ? "true" : "false") << "]"
           << ",\"bone_decisions\":[";
    bool first = true;
    for (const FSVRBoneDecision& bone : decisions.bones)
    {
        if (!first)
        {
            fields << ",";
        }
        first = false;
        fields << "{\"bone\":\"" << bone.bone_id << "\""
               << ",\"desired\":\""
               << FSVRTrackingCoordinator::decisionName(bone.desired_decision)
               << "\""
               << ",\"desired_source\":\"" << bone.desired_source << "\""
               << ",\"desired_position\":\""
               << FSVRTrackingCoordinator::decisionName(
                      bone.desired_position_decision)
               << "\""
               << ",\"desired_position_source\":\""
               << bone.desired_position_source << "\""
               << ",\"desired_orientation\":\""
               << FSVRTrackingCoordinator::decisionName(
                      bone.desired_orientation_decision)
               << "\""
               << ",\"desired_orientation_source\":\""
               << bone.desired_orientation_source << "\""
               << ",\"applied\":\""
               << FSVRTrackingCoordinator::decisionName(bone.applied_decision)
               << "\""
               << ",\"applied_source\":\"" << bone.applied_source << "\""
               << ",\"fallback_reason\":\"" << bone.fallback_reason << "\""
               << ",\"pitch_participates\":"
               << (bone.pitch_participates ? "true" : "false")
               << ",\"roll_participates\":"
               << (bone.roll_participates ? "true" : "false")
               << ",\"capability\":\""
               << FSVRTrackingCoordinator::stateName(
                      bone.source_states.capability)
               << "\""
               << ",\"user_enabled\":\""
               << FSVRTrackingCoordinator::stateName(
                      bone.source_states.user_enabled)
               << "\""
               << ",\"input_active\":\""
               << FSVRTrackingCoordinator::stateName(
                      bone.source_states.input_active)
               << "\""
               << ",\"pose_valid\":\""
               << FSVRTrackingCoordinator::stateName(
                      bone.source_states.pose_valid)
               << "\""
               << ",\"confidence\":\""
               << FSVRTrackingCoordinator::stateName(
                      bone.source_states.confidence)
               << "\""
               << ",\"shadow_only\":true}";
    }
    fields << "]";
    LLOpenXRTrace::writeRecord("vr_tracking", fields.str());
}

void fillPublishedGeneration(FSVRTrackingFrameInput& input)
{
    // The published generation comes from the frame context frozen
    // by beginPacedFrame() BEFORE idle, read through the manager's
    // token-validated accessor (review finding P207A-S1-04, second
    // round: the avatar eye frame is only frozen after idle, so it
    // cannot serve as the pre-idle published source).
    unsigned long long published = 0;
    if (LLOpenXRManager::instance().getPublishedLocateGeneration(
            input.app_frame_token, published))
    {
        input.published_locate_generation = published;
        input.has_published_generation = true;
    }
}

void fillHandFields(FSVRTrackingFrameInput& input)
{
    // P2-07B C3 (CP-8): fill the frame input's C3 fields from
    // the manager's adopted sample of the SAME locate the head
    // observation describes -- the paced adoption precedes idle
    // and the accessors serve only the current adopted sample,
    // so the generation binding is inherited from the manager's
    // adopted-sample discipline, not re-checked field-by-field.
    // On the legacy loop the inherited one-iteration lag applies
    // (OQ-2). Absent hands keep the struct's zero defaults: the
    // masks are written ONLY under the presence gate, so an
    // absent hand can never carry stale bits [R1].
    LLOpenXRManager& manager = LLOpenXRManager::instance();
    input.hand_tracking_available = manager.getHandTrackingAvailable();
    LLVector3 hmd_position;
    if (manager.getHmdPosition(hmd_position))
    {
        input.hmd_position = hmd_position;
        input.hmd_position_valid = true;
    }
    for (int side = 0; side < 2; ++side)
    {
        const bool right_hand = (side == 1);
        LLVector3 joint_position;
        LLQuaternion joint_orientation;
        U32 joint_flags = 0;
        // Presence is the hand-active gate: getHandJoint applies
        // the sample/active gates (the wrist, joint 1, is the
        // probe read; its values are not carried -- the wrist
        // fields below come from the wrist accessors).
        if (!manager.getHandJoint(right_hand, 1, joint_position,
                                  joint_orientation, joint_flags))
        {
            continue;
        }
        input.hand_sample_present[side] = true;
        LLVector3 wrist_position;
        if (manager.getWristPosition(right_hand, wrist_position))
        {
            input.wrist_position[side] = wrist_position;
            input.wrist_position_valid[side] = true;
        }
        LLQuaternion wrist_orientation;
        if (manager.getWristOrientation(right_hand, wrist_orientation))
        {
            input.wrist_orientation[side] = wrist_orientation;
            input.wrist_orientation_valid[side] = true;
        }
        // [R2 -- C3-R2-02]: mask construction is the production
        // helper's, called once for this hand; no inline mask
        // assembly anywhere.
        U32 joint_location_flags[kHandJointCount] = {};
        for (U32 j = 0; j < kHandJointCount; ++j)
        {
            LLVector3 p;
            LLQuaternion q;
            U32 flags = 0;
            if (manager.getHandJoint(right_hand, j, p, q, flags))
            {
                joint_location_flags[j] = flags;
            }
        }
        const FSVRHandValidityMasks masks =
            buildHandValidityMasks(joint_location_flags);
        input.hand_position_tracked_mask[side] = masks.position_mask;
        input.hand_orientation_tracked_mask[side] = masks.orientation_mask;
    }
}

void publishDecisions(const FSVRTrackingFrameInput& input,
                      const FSVRTrackingSettingsSnapshot& snapshot)
{
    FSVRTrackingFrameDecisions decisions;
    decisions.app_frame_token = input.app_frame_token;
    decisions.settings_generation = snapshot.generation;
    decisions.legacy_fast_path = true;
    decisions.selective_requested = snapshot.selective_requested;
    decisions.selective_applied = false;

    for (const FSVRTrackingBoneEntry& entry : FSVRBoneRegistry::entries())
    {
        // P2-07a decided HEAD-rollup bones only. P2-07B C3
        // (CP-9): UPPER_BODY and ARMS_INCLUDING_HANDS bones are
        // decided too (shadow decisions; selective_applied stays
        // false). mSkull (NONE) is audit-only and never decided
        // for driving; LOWER_BODY has no entries.
        if (entry.group != FSVRRollup::HEAD &&
            entry.group != FSVRRollup::UPPER_BODY &&
            entry.group != FSVRRollup::ARMS_INCLUDING_HANDS)
        {
            continue;
        }
        decisions.bones.push_back(
            FSVRTrackingArbitrator::evaluateBone(entry, snapshot, input));
    }

    s_current = decisions;
    s_current_valid = true;
    emitTrace(decisions, input, snapshot);
}
} // namespace

// P2-07B C3 [R2 -- C3-R2-02]: the contract is stated in
// llvrtrackingframe.h (beside the frame-input code, so H3 links
// this production function directly). The flag bit values are
// XrSpaceLocationFlags (openxr.h), restated because this
// translation unit carries no OpenXR headers.
FSVRHandValidityMasks buildHandValidityMasks(const U32 joint_location_flags[26])
{
    const U32 kOrientationValid = 0x00000001;
    const U32 kPositionValid = 0x00000002;
    const U32 kOrientationTracked = 0x00000004;
    const U32 kPositionTracked = 0x00000008;
    FSVRHandValidityMasks masks;
    for (U32 j = 0; j < 26; ++j)
    {
        const U32 flags = joint_location_flags[j];
        if ((flags & kPositionValid) != 0 && (flags & kPositionTracked) != 0)
        {
            masks.position_mask |= (1u << j);
        }
        if ((flags & kOrientationValid) != 0 && (flags & kOrientationTracked) != 0)
        {
            masks.orientation_mask |= (1u << j);
        }
    }
    return masks;
}

// static
void FSVRTrackingCoordinator::beginFrame()
{
    s_pending_snapshot = FSVRTrackingSettings::captureForFrame();
    s_pending_token = LLOpenXRTrace::currentAppFrameToken();
    s_pending_valid = true;
}

// static
void FSVRTrackingCoordinator::finalizeAfterLookAt(
    const LLAgentCamera::ViewpointOverrideHeadObservation& observation)
{
    const FSVRTrackingSettingsSnapshot snapshot =
        s_pending_valid ? s_pending_snapshot
                        : FSVRTrackingSettings::captureForFrame();

    FSVRTrackingFrameInput input;
    input.app_frame_token =
        s_pending_valid ? s_pending_token
                        : LLOpenXRTrace::currentAppFrameToken();
    input.settings_generation = snapshot.generation;
    input.locate_generation = observation.sample_generation;
    input.sample_time_ns = observation.sample_time_ns;
    input.sample_tracked = observation.sample_tracked;
    input.sample_valid = observation.sample_consumed;
    input.has_sample = observation.sample_consumed;
    input.vr_active = LLOpenXRManager::instance().isVRModeActive();
    input.override_owned = observation.override_owned;
    input.override_present = observation.override_present;
    input.interruption_hold = observation.interruption_hold;
    input.calibrated_yaw = observation.target_yaw_rad;
    input.hmd_pitch = observation.target_pitch_rad;
    input.hmd_roll = observation.target_roll_rad;
    input.look_direction = observation.look_direction;
    input.avatar_valid = !gAgentAvatarp.isNull();
    // P2-07B C3 (DP-6): the decisions payload's bone set grew
    // (3 decided bones -> up to 46) and hand-branch records
    // carry two channel decisions; the single 1 -> 2 bump
    // covers both changes ([R1] reconciliation -- no second
    // bump).
    input.registry_generation = 2;
    fillPublishedGeneration(input);
    fillHandFields(input);

    s_pending_valid = false;
    publishDecisions(input, snapshot);
}

// static
void FSVRTrackingCoordinator::finalizeWithoutObservation()
{
    const FSVRTrackingSettingsSnapshot snapshot =
        s_pending_valid ? s_pending_snapshot
                        : FSVRTrackingSettings::captureForFrame();

    FSVRTrackingFrameInput input;
    input.app_frame_token =
        s_pending_valid ? s_pending_token
                        : LLOpenXRTrace::currentAppFrameToken();
    input.settings_generation = snapshot.generation;
    input.vr_active = LLOpenXRManager::instance().isVRModeActive();
    input.avatar_valid = !gAgentAvatarp.isNull();
    input.registry_generation = 2; // P2-07B C3 (DP-6), as above.
    fillPublishedGeneration(input);

    s_pending_valid = false;
    publishDecisions(input, snapshot);
}

// static
const FSVRTrackingFrameDecisions* FSVRTrackingCoordinator::currentDecisions()
{
    return s_current_valid ? &s_current : nullptr;
}

// static
const char* FSVRTrackingCoordinator::decisionName(FSVRDecisionKind kind)
{
    switch (kind)
    {
    case FSVRDecisionKind::TRACKED:
        return "tracked";
    case FSVRDecisionKind::ANIMATED:
        return "animated";
    case FSVRDecisionKind::LEGACY_HOLD:
        return "legacy_hold";
    case FSVRDecisionKind::DEFERRED:
        return "deferred";
    case FSVRDecisionKind::UNAVAILABLE:
    default:
        return "unavailable";
    }
}

// static
const char* FSVRTrackingCoordinator::stateName(FSVRStateValue value)
{
    switch (value)
    {
    case FSVRStateValue::PASS:
        return "pass";
    case FSVRStateValue::FAIL:
        return "fail";
    case FSVRStateValue::NOT_APPLICABLE:
    default:
        return "not_applicable";
    }
}
