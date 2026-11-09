/**
 * @file llvrtrackingframe.h
 * @brief P2-07a Stage 1: per-frame tracking inputs, decisions and
 *        the frame coordinator (implementation spec 2.5-2.8, 3.2).
 *
 * The coordinator captures the settings snapshot and frame identity
 * immediately before LLAgentCamera::updateLookAt() and finalizes
 * shadow arbitration immediately after it. It never changes the
 * legacy head path: Stage 1 decisions are diagnostic only.
 */

#ifndef LL_LLVRTRACKINGFRAME_H
#define LL_LLVRTRACKINGFRAME_H

#include "llvrtrackingsettings.h"
#include "llagentcamera.h"

#include <string>
#include <vector>

// One frame's worth of source observations (spec 2.5). Fields are
// explicitly absent-able: a missing sample is recorded as missing,
// never filled from a newer or unrelated XR locate.
struct FSVRTrackingFrameInput
{
    U64 app_frame_token = 0;
    U64 settings_generation = 0;
    // The locate generation of the head sample actually consumed by
    // updateLookAt() (from the manager's head-consumption record).
    U64 locate_generation = 0;
    // The locate generation of the currently published XR snapshot,
    // sourced independently from the frame context (review finding
    // P207A-S1-04). has_published_generation is false when no
    // published snapshot is available; the trace then emits null
    // rather than copying the consumed value.
    U64 published_locate_generation = 0;
    bool has_published_generation = false;
    S64 sample_time_ns = 0;
    bool sample_tracked = false;
    bool sample_valid = false;
    bool has_sample = false;
    bool vr_active = false;
    bool override_owned = false;
    bool override_present = false;
    bool interruption_hold = false;
    F32 calibrated_yaw = 0.f;
    F32 hmd_pitch = 0.f;
    F32 hmd_roll = 0.f;
    LLVector3 look_direction = LLVector3::zero;
    bool avatar_valid = false;
    U64 registry_generation = 0;
    // P2-07B C3 (CP-7): HMD position of the consumed sample
    // (the C2 carriage, assigned to C3 by C2's OQ-1).
    LLVector3 hmd_position = LLVector3::zero;
    bool hmd_position_valid = false;
    // P2-07B C3 (CP-7): per-hand wrist sample of the same
    // locate (index 0 = left, 1 = right). Position and
    // orientation are represented independently (spec section
    // 6.2, mixed sourcing; DP-4 carries both).
    LLVector3 wrist_position[2] = {LLVector3::zero, LLVector3::zero};
    LLQuaternion wrist_orientation[2];
    bool wrist_position_valid[2] = {false, false};
    bool wrist_orientation_valid[2] = {false, false};
    // Hand located & active this frame (the shared gate both
    // validity masks hang from).
    bool hand_sample_present[2] = {false, false};
    // CP-2's capability fact (the four-term conjunction).
    bool hand_tracking_available = false;
    // P2-07B C3 [R1 -- C3-02]: per-joint VALIDITY for each hand
    // (validity only -- no per-joint pose data; DP-5 stands).
    // Bit j (0..25) = XR hand joint j, per the registry's index
    // mapping. A set bit means that joint's channel was
    // VALID && TRACKED in the adopted locate (the
    // genuinely-tracked predicate). Both masks are zero
    // whenever hand_sample_present[side] is false; their
    // generation is this struct's locate_generation (the hand
    // state IS the HMD sample's locate -- no second stamp).
    U32 hand_position_tracked_mask[2] = {0, 0};
    U32 hand_orientation_tracked_mask[2] = {0, 0};
};

// P2-07B C3 [R2 -- C3-R2-02]: per-joint validity-mask
// construction (CP-8). Pure: plain flag words in, masks out; no
// manager state, no OpenXR calls. joint_location_flags[j] is
// CP-4 getHandJoint's out_location_flags for joint j of ONE
// hand; the joint count is XR_HAND_JOINT_COUNT_EXT (26),
// restated here because this header carries no OpenXR types.
// Bit j of a mask is set iff word j carries BOTH that channel's
// VALID and TRACKED bits (XrSpaceLocationFlags values:
// orientation 0x01/0x04, position 0x02/0x08); each channel is
// computed from its own bit pair only -- the channels are
// independent by construction. An all-zero flag array (the
// inactive/absent hand's shape) yields {0, 0} by construction,
// not by a special case at the call site. Bits for joints no
// bone maps to (PALM 0, the METACARPALs 2/6/11/16/21, the
// finger TIPs 10/15/20/25) are computed and retained verbatim;
// the helper assigns bits to no bone (bone assignment happens
// only in the arbitrator, via the registry's index table). The
// coordinator fill calls this once per present hand; no inline
// mask assembly exists anywhere.
struct FSVRHandValidityMasks
{
    U32 position_mask = 0;
    U32 orientation_mask = 0;
};
FSVRHandValidityMasks buildHandValidityMasks(const U32 joint_location_flags[26]);

enum class FSVRStateValue
{
    PASS,
    FAIL,
    NOT_APPLICABLE
};

// The five independently represented eligibility states (brief v2).
struct FSVRSourceStates
{
    FSVRStateValue capability = FSVRStateValue::FAIL;
    FSVRStateValue user_enabled = FSVRStateValue::FAIL;
    FSVRStateValue input_active = FSVRStateValue::FAIL;
    FSVRStateValue pose_valid = FSVRStateValue::FAIL;
    // The HMD supplies no per-joint confidence: NOT_APPLICABLE.
    FSVRStateValue confidence = FSVRStateValue::NOT_APPLICABLE;
};

enum class FSVRDecisionKind
{
    TRACKED,
    ANIMATED,
    LEGACY_HOLD,
    DEFERRED,
    UNAVAILABLE
};

// Per-bone arbitration decision (spec 2.6). Desired and applied are
// recorded separately: in Stage 1 the desired owner is a shadow
// evaluation while the applied owner is a legacy-path POLICY
// ESTIMATE (inferred from ownership/sample state), not a measured
// final joint contribution. The Stage-2 audits supply measured
// applied evidence.
struct FSVRBoneDecision
{
    std::string bone_id;
    FSVRSourceStates source_states;
    std::string desired_source; // "HMD", "hand_tracking",
    // "body_estimation", "animation" or "none"
    FSVRDecisionKind desired_decision = FSVRDecisionKind::UNAVAILABLE;
    // P2-07B C3 [R1 -- C3-03]: for hand-branch bones the desired
    // side is decided PER CHANNEL (position and orientation
    // degrade independently -- spec section 6.2 mixed sourcing
    // at the decision layer); desired_source / desired_decision
    // above then mirror the POSITION channel (the channel
    // wrist-target consumers read). For every other branch both
    // channel fields mirror the single decision.
    std::string desired_position_source;
    FSVRDecisionKind desired_position_decision = FSVRDecisionKind::UNAVAILABLE;
    std::string desired_orientation_source;
    FSVRDecisionKind desired_orientation_decision = FSVRDecisionKind::UNAVAILABLE;
    std::string applied_source;
    FSVRDecisionKind applied_decision = FSVRDecisionKind::UNAVAILABLE;
    std::string fallback_reason;
    bool pitch_participates = false;
    bool roll_participates = false;
    bool direct_tracking_eligible = false;
    bool audit_verified = false;
    bool shadow_only = true;
    U64 frame_token = 0;
    U64 settings_generation = 0;
};

struct FSVRTrackingFrameDecisions
{
    U64 app_frame_token = 0;
    U64 settings_generation = 0;
    bool legacy_fast_path = true;
    bool selective_requested = false;
    bool selective_applied = false; // always false in Stage 1
    std::vector<FSVRBoneDecision> bones;
};

class FSVRTrackingCoordinator
{
public:
    // Immediately before updateLookAt(): capture the settings
    // snapshot exactly once and register the frame identity.
    static void beginFrame();

    // Immediately after updateLookAt(): bind the observation from
    // that call, evaluate shadow decisions, publish immutably.
    static void finalizeAfterLookAt(
        const LLAgentCamera::ViewpointOverrideHeadObservation& observation);

    // updateLookAt() produced no observation (e.g. no agent camera
    // state): publish decisions from an explicitly empty input.
    static void finalizeWithoutObservation();

    static const FSVRTrackingFrameDecisions* currentDecisions();

    static const char* decisionName(FSVRDecisionKind kind);
    static const char* stateName(FSVRStateValue value);
};

#endif // LL_LLVRTRACKINGFRAME_H
