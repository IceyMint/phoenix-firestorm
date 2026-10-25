/**
 * @file llvrtrackingarbitrator.cpp
 * @brief P2-07a Stage 1: five-state HMD arbitration (shadow only).
 */

#include "llviewerprecompiledheaders.h"

#include "llvrtrackingarbitrator.h"

// static
FSVRDecisionKind FSVRTrackingArbitrator::legacyAppliedDecision(
    const FSVRTrackingFrameInput& input)
{
    if (input.interruption_hold)
    {
        return FSVRDecisionKind::LEGACY_HOLD;
    }
    if (input.override_owned && input.has_sample)
    {
        return FSVRDecisionKind::TRACKED;
    }
    return FSVRDecisionKind::ANIMATED;
}

// static
FSVRBoneDecision FSVRTrackingArbitrator::evaluateBone(
    const FSVRTrackingBoneEntry& entry,
    const FSVRTrackingSettingsSnapshot& snapshot,
    const FSVRTrackingFrameInput& input)
{
    FSVRBoneDecision decision;
    decision.bone_id = entry.canonical_id;
    decision.frame_token = input.app_frame_token;
    decision.settings_generation = snapshot.generation;
    decision.shadow_only = true; // Stage 1 never drives joints.
    decision.audit_verified =
        entry.audit_status == FSVRTrackingBoneEntry::AuditStatus::VERIFIED;
    decision.direct_tracking_eligible =
        entry.direct_tracking_eligible && decision.audit_verified &&
        snapshot.selective_verified;
    decision.pitch_participates = snapshot.head_pitch_enabled;
    decision.roll_participates = snapshot.head_roll_enabled;

    FSVRSourceStates& states = decision.source_states;
    states.capability =
        input.vr_active ? FSVRStateValue::PASS : FSVRStateValue::FAIL;
    const bool user_enabled = !snapshot.disabled_bones.isBoneDisabled(
        entry.canonical_id);
    states.user_enabled =
        user_enabled ? FSVRStateValue::PASS : FSVRStateValue::FAIL;
    states.input_active = (input.override_owned && input.has_sample)
                              ? FSVRStateValue::PASS
                              : FSVRStateValue::FAIL;
    states.pose_valid = (input.has_sample && input.sample_valid &&
                         input.sample_tracked)
                            ? FSVRStateValue::PASS
                            : FSVRStateValue::FAIL;
    states.confidence = FSVRStateValue::NOT_APPLICABLE; // HMD: none.

    // Desired owner, with adjudication-B precedence.
    if (input.interruption_hold)
    {
        decision.desired_decision = FSVRDecisionKind::LEGACY_HOLD;
        decision.desired_source = "HMD";
        decision.fallback_reason = "interruption";
    }
    else if (!user_enabled)
    {
        decision.desired_decision = FSVRDecisionKind::ANIMATED;
        decision.desired_source = "animation";
        decision.fallback_reason = "disabled";
    }
    else if (states.capability != FSVRStateValue::PASS)
    {
        decision.desired_decision = FSVRDecisionKind::UNAVAILABLE;
        decision.desired_source = "none";
        decision.fallback_reason = "capability";
    }
    else if (!input.avatar_valid)
    {
        decision.desired_decision = FSVRDecisionKind::UNAVAILABLE;
        decision.desired_source = "none";
        decision.fallback_reason = "missing_joint";
    }
    else if (states.input_active != FSVRStateValue::PASS)
    {
        decision.desired_decision = FSVRDecisionKind::ANIMATED;
        decision.desired_source = "animation";
        decision.fallback_reason = "inactive";
    }
    else if (states.pose_valid != FSVRStateValue::PASS)
    {
        decision.desired_decision = FSVRDecisionKind::ANIMATED;
        decision.desired_source = "animation";
        decision.fallback_reason = "invalid";
    }
    else
    {
        decision.desired_decision = FSVRDecisionKind::TRACKED;
        decision.desired_source = "HMD";
        decision.fallback_reason = "";
    }

    // Applied owner: the unchanged legacy path.
    decision.applied_decision = legacyAppliedDecision(input);
    decision.applied_source =
        decision.applied_decision == FSVRDecisionKind::ANIMATED
            ? "animation"
            : "HMD";
    return decision;
}
