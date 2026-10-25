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
    for (const FSVRRollupState& rollup : rollups)
    {
        if (rollup.group == FSVRRollup::HEAD)
        {
            head_rollup = rollup.derived_state;
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
           << ",\"other_rollups\":\"not_implemented\""
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
    // The published generation comes from the frame context's frozen
    // snapshot, independently of the head-consumption record
    // (review finding P207A-S1-04).
    LLXRAvatarEyeFrame eye_frame;
    if (LLOpenXRManager::instance().getAvatarEyeFrame(eye_frame) &&
        eye_frame.valid)
    {
        input.published_locate_generation = eye_frame.locate_generation;
        input.has_published_generation = true;
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
        // Only HEAD-rollup bones get per-frame decisions in P2-07a;
        // mSkull is audit-only and never decided for driving.
        if (entry.group != FSVRRollup::HEAD)
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
    input.registry_generation = 1;
    fillPublishedGeneration(input);

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
    input.registry_generation = 1;
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
