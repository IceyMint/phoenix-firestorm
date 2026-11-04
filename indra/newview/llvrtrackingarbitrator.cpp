/**
 * @file llvrtrackingarbitrator.cpp
 * @brief P2-07a Stage 1: five-state HMD arbitration (shadow only).
 *        P2-07B C3 (CP-9): source-aware dispatch on the registry
 *        entry's ownership_priority -- the HMD branch is Stage 1's
 *        logic unchanged; the body-estimation and hand-tracking
 *        branches and the empty-priority case are C3's.
 */

#include "llviewerprecompiledheaders.h"

#include "llvrtrackingarbitrator.h"

namespace
{
// P2-07B C3 (CP-9): the side of an arm entry, from the canonical
// id's Left/Right suffix (the registry has no side field).
// Returns 0 (left), 1 (right), or -1 when the id carries no side.
int handSideForBoneId(const std::string& canonical_id)
{
    const std::string left_suffix = "Left";
    const std::string right_suffix = "Right";
    if (canonical_id.size() > left_suffix.size() &&
        canonical_id.compare(canonical_id.size() - left_suffix.size(),
                             left_suffix.size(), left_suffix) == 0)
    {
        return 0;
    }
    if (canonical_id.size() > right_suffix.size() &&
        canonical_id.compare(canonical_id.size() - right_suffix.size(),
                             right_suffix.size(), right_suffix) == 0)
    {
        return 1;
    }
    return -1;
}

// One channel's outcome in the hand branch (CP-9 [R1]): the two
// channels share the capability and input-active gates and then
// consult ONLY their own channel's predicate. The ladders
// differ: a wrist channel degrades hand -> animation; a finger
// channel has no animation rung (hand-tracking-or-none).
struct FSVRHandChannelOutcome
{
    FSVRDecisionKind decision = FSVRDecisionKind::UNAVAILABLE;
    std::string source = "none";
    std::string reason = "capability";
};

FSVRHandChannelOutcome evaluateHandChannel(bool capability_pass,
                                           bool input_active_pass,
                                           bool channel_pass,
                                           bool finger_ladder)
{
    FSVRHandChannelOutcome outcome;
    if (!capability_pass)
    {
        // AMEND-01 (code cross-check R1): the capability gate
        // follows the same per-bone ladder as the gates below --
        // a wrist channel degrades to animation; only a finger
        // channel bottoms out at unavailable/none.
        outcome.decision = finger_ladder ? FSVRDecisionKind::UNAVAILABLE
                                         : FSVRDecisionKind::ANIMATED;
        outcome.source = finger_ladder ? "none" : "animation";
        outcome.reason = "capability";
    }
    else if (!input_active_pass)
    {
        outcome.decision = finger_ladder ? FSVRDecisionKind::UNAVAILABLE
                                         : FSVRDecisionKind::ANIMATED;
        outcome.source = finger_ladder ? "none" : "animation";
        outcome.reason = "inactive";
    }
    else if (!channel_pass)
    {
        outcome.decision = finger_ladder ? FSVRDecisionKind::UNAVAILABLE
                                         : FSVRDecisionKind::ANIMATED;
        outcome.source = finger_ladder ? "none" : "animation";
        outcome.reason = "invalid";
    }
    else
    {
        outcome.decision = FSVRDecisionKind::TRACKED;
        outcome.source = "hand_tracking";
        outcome.reason = "";
    }
    return outcome;
}
} // namespace

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
    decision.audit_verified =
        entry.audit_status == FSVRTrackingBoneEntry::AuditStatus::VERIFIED;
    // P2-07B C6-E (Package B spec section 7.3, PB-R1-05 /
    // DP-B5 conditional (a)): runtime-effective form is
    // emitted ONLY for explicitly authorized head/spine
    // constraint decisions -- the spine-chain bones under
    // C6's authority -- ONLY for a bone whose Stage-2
    // audit proof stands, and ONLY in a build carrying
    // the release authorization (CP-5 part 1a). Every other
    // decision family remains shadow-only regardless of
    // the head path's activation state; Stage-1 mHead/
    // mNeck rotation ownership is not created, altered, or
    // re-derived here.
    decision.shadow_only =
        !(snapshot.stage2_constraint_authorized &&
          decision.audit_verified &&
          entry.constraint_adapter_id == "spine_chain");
    decision.direct_tracking_eligible =
        entry.direct_tracking_eligible && decision.audit_verified &&
        snapshot.selective_verified;
    decision.pitch_participates = snapshot.head_pitch_enabled;
    decision.roll_participates = snapshot.head_roll_enabled;

    const bool user_enabled = !snapshot.disabled_bones.isBoneDisabled(
        entry.canonical_id);

    // CP-9 dispatch: on the entry's FIRST ownership priority
    // (the ranking's head -- the data Stage 1 recorded but never
    // consumed).
    const std::string owner = entry.ownership_priority.empty()
                                  ? std::string()
                                  : entry.ownership_priority.front();

    if (owner == "HMD")
    {
        // The Stage-1 HMD branch, unchanged in every term
        // (H3 clause 5 regression: mNeck/mHead decisions are
        // byte-identical to the pre-C3 arbitrator).
        FSVRSourceStates& states = decision.source_states;
        states.capability =
            input.vr_active ? FSVRStateValue::PASS : FSVRStateValue::FAIL;
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

        // The channel fields mirror the single decision.
        decision.desired_position_source = decision.desired_source;
        decision.desired_position_decision = decision.desired_decision;
        decision.desired_orientation_source = decision.desired_source;
        decision.desired_orientation_decision = decision.desired_decision;

        // Applied owner: the unchanged legacy path.
        decision.applied_decision = legacyAppliedDecision(input);
        decision.applied_source =
            decision.applied_decision == FSVRDecisionKind::ANIMATED
                ? "animation"
                : "HMD";
        return decision;
    }

    // ---- Non-HMD branches (P2-07B C3, CP-9) ----
    // Head settings must not leak into non-HEAD decisions.
    decision.pitch_participates = false;
    decision.roll_participates = false;
    // The applied side for non-HEAD entries is unconditional:
    // the legacy path never drives arms/hands/spine from
    // tracking (section 2.10), and legacyAppliedDecision's HMD
    // reading must not leak into them.
    decision.applied_decision = FSVRDecisionKind::ANIMATED;
    decision.applied_source = "animation";

    FSVRSourceStates& states = decision.source_states;
    states.user_enabled =
        user_enabled ? FSVRStateValue::PASS : FSVRStateValue::FAIL;
    states.confidence = FSVRStateValue::NOT_APPLICABLE;

    // [R2 -- C3-R2-01] step 1, ELIGIBILITY, first in every
    // branch: a per-bone-disabled bone, or any bone while the
    // avatar is invalid, exits through today's fallbacks before
    // ownership identity is consulted. (Ownership identity /
    // source availability / applied decision are the branch
    // publications below, for eligible bones only.)
    if (!user_enabled)
    {
        states.capability = FSVRStateValue::FAIL;
        states.input_active = FSVRStateValue::FAIL;
        states.pose_valid = FSVRStateValue::FAIL;
        decision.desired_decision = FSVRDecisionKind::ANIMATED;
        decision.desired_source = "animation";
        decision.fallback_reason = "disabled";
        decision.desired_position_source = decision.desired_source;
        decision.desired_position_decision = decision.desired_decision;
        decision.desired_orientation_source = decision.desired_source;
        decision.desired_orientation_decision = decision.desired_decision;
        return decision;
    }
    if (!input.avatar_valid)
    {
        states.capability = FSVRStateValue::FAIL;
        states.input_active = FSVRStateValue::FAIL;
        states.pose_valid = FSVRStateValue::FAIL;
        decision.desired_decision = FSVRDecisionKind::UNAVAILABLE;
        decision.desired_source = "none";
        decision.fallback_reason = "missing_joint";
        decision.desired_position_source = decision.desired_source;
        decision.desired_position_decision = decision.desired_decision;
        decision.desired_orientation_source = decision.desired_source;
        decision.desired_orientation_decision = decision.desired_decision;
        return decision;
    }

    if (owner == "body_estimation")
    {
        // C3-01: the recorded owner, published in its
        // source-unavailable form. C3 plumbs no body-estimation
        // sample, so availability never passes for an eligible
        // bone: desired names the owner, applied stays
        // animation, and the reason is recorded. These bones
        // never publish "hand_tracking" or "HMD".
        states.capability =
            input.vr_active ? FSVRStateValue::PASS : FSVRStateValue::FAIL;
        states.input_active = FSVRStateValue::FAIL;
        states.pose_valid = FSVRStateValue::FAIL;
        decision.desired_decision = FSVRDecisionKind::UNAVAILABLE;
        decision.desired_source = "body_estimation";
        decision.fallback_reason = "source_unavailable";
        decision.desired_position_source = decision.desired_source;
        decision.desired_position_decision = decision.desired_decision;
        decision.desired_orientation_source = decision.desired_source;
        decision.desired_orientation_decision = decision.desired_decision;
        return decision;
    }

    if (owner == "hand_tracking")
    {
        // The hand branch, PER CHANNEL [R1 -- C3-03]: shared
        // gates (capability, user -- already passed above -- and
        // hand-active), then each channel consults ONLY its own
        // predicate. For a wrist those are CP-4's two validity
        // fields as carried by CP-8; for a finger, the bone's
        // OWN joint bit in that channel's validity mask [R1 --
        // C3-02] -- an active hand alone claims nothing for any
        // finger.
        const int side = handSideForBoneId(entry.canonical_id);
        const bool capability_pass =
            input.hand_tracking_available && side >= 0;
        states.capability =
            capability_pass ? FSVRStateValue::PASS : FSVRStateValue::FAIL;
        const bool input_active_pass =
            capability_pass && input.hand_sample_present[side];
        states.input_active =
            input_active_pass ? FSVRStateValue::PASS : FSVRStateValue::FAIL;

        const bool is_finger = entry.xr_hand_joint_index >= 0;
        bool position_pass = false;
        bool orientation_pass = false;
        if (input_active_pass)
        {
            if (is_finger)
            {
                const U32 bit = 1u << entry.xr_hand_joint_index;
                position_pass =
                    (input.hand_position_tracked_mask[side] & bit) != 0;
                orientation_pass =
                    (input.hand_orientation_tracked_mask[side] & bit) != 0;
            }
            else
            {
                position_pass = input.wrist_position_valid[side];
                orientation_pass = input.wrist_orientation_valid[side];
            }
        }
        // The record's single pose_valid state mirrors the
        // position channel (the record's primary channel).
        states.pose_valid =
            position_pass ? FSVRStateValue::PASS : FSVRStateValue::FAIL;

        const FSVRHandChannelOutcome position = evaluateHandChannel(
            capability_pass, input_active_pass, position_pass, is_finger);
        const FSVRHandChannelOutcome orientation = evaluateHandChannel(
            capability_pass, input_active_pass, orientation_pass, is_finger);
        decision.desired_position_decision = position.decision;
        decision.desired_position_source = position.source;
        decision.desired_orientation_decision = orientation.decision;
        decision.desired_orientation_source = orientation.source;
        // The pre-existing single fields mirror the POSITION
        // channel (the channel wrist-target consumers read).
        decision.desired_decision = position.decision;
        decision.desired_source = position.source;
        decision.fallback_reason = position.reason;
        return decision;
    }

    // Empty priority -- after C3-01, collars/shoulders ONLY: no
    // tracking source by design (HAND-Q1). Desired is animation
    // whenever the bone is eligible (the exits above); NEVER a
    // tracking source. input_active/pose_valid stay FAIL: no
    // tracking input exists for these bones to be active with.
    states.capability =
        input.vr_active ? FSVRStateValue::PASS : FSVRStateValue::FAIL;
    states.input_active = FSVRStateValue::FAIL;
    states.pose_valid = FSVRStateValue::FAIL;
    decision.desired_decision = FSVRDecisionKind::ANIMATED;
    decision.desired_source = "animation";
    decision.fallback_reason = "";
    decision.desired_position_source = decision.desired_source;
    decision.desired_position_decision = decision.desired_decision;
    decision.desired_orientation_source = decision.desired_source;
    decision.desired_orientation_decision = decision.desired_decision;
    return decision;
}

namespace
{
// P2-07B C6-E (CP-5 part 3): one domain's runtime-effective
// fact over a published set. The domain is named by its
// registry constraint_adapter_id values (one for the spine
// chain, two for the arms). True iff at least one published
// decision belongs to the domain and no member is
// shadow-only; see the declarations in the header for the
// full contract.
bool domainDecisionsRuntimeEffective(
    const FSVRTrackingFrameDecisions& decisions,
    const char* adapter_id_a, const char* adapter_id_b)
{
    bool any_member = false;
    for (const FSVRBoneDecision& bone : decisions.bones)
    {
        const FSVRTrackingBoneEntry* entry =
            FSVRBoneRegistry::find(bone.bone_id);
        if (!entry)
        {
            continue;
        }
        if (entry->constraint_adapter_id != adapter_id_a &&
            (!adapter_id_b ||
             entry->constraint_adapter_id != adapter_id_b))
        {
            continue;
        }
        any_member = true;
        if (bone.shadow_only)
        {
            return false;
        }
    }
    return any_member;
}
} // namespace

bool spineDecisionsRuntimeEffective(
    const FSVRTrackingFrameDecisions& decisions)
{
    return domainDecisionsRuntimeEffective(decisions, "spine_chain",
                                           nullptr);
}

bool armDecisionsRuntimeEffective(
    const FSVRTrackingFrameDecisions& decisions)
{
    return domainDecisionsRuntimeEffective(decisions, "arm_left",
                                           "arm_right");
}
