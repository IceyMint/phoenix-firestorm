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
};

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
    std::string desired_source; // "HMD", "animation" or "none"
    FSVRDecisionKind desired_decision = FSVRDecisionKind::UNAVAILABLE;
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
