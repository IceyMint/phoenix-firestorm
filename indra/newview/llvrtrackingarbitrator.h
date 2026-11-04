/**
 * @file llvrtrackingarbitrator.h
 * @brief P2-07a Stage 1: pure five-state arbitration evaluation
 *        (implementation spec 2.4, 2.6).
 *
 * The evaluation is a pure function of (registry entry, settings
 * snapshot, frame input) so it can be unit-tested without an avatar.
 * Precedence (adjudication B): a P2-04 interruption hold wins inside
 * interruption states; ordinary disabled/unavailable ownership falls
 * back to animation.
 */

#ifndef LL_LLVRTRACKINGARBITRATOR_H
#define LL_LLVRTRACKINGARBITRATOR_H

#include "llvrboneregistry.h"
#include "llvrtrackingframe.h"
#include "llvrtrackingsettings.h"

class FSVRTrackingArbitrator
{
public:
    static FSVRBoneDecision evaluateBone(
        const FSVRTrackingBoneEntry& entry,
        const FSVRTrackingSettingsSnapshot& snapshot,
        const FSVRTrackingFrameInput& input);

    // What the unchanged legacy path actually did with this bone this
    // frame, derived from the same frame input.
    static FSVRDecisionKind legacyAppliedDecision(
        const FSVRTrackingFrameInput& input);
};

// P2-07B C6-E (CP-5 part 3): the glue's per-domain gate-1
// derivations over a published decision set, reading the form the
// arbitrator emitted above -- the glue never reinterprets a shadow
// decision into an effective one. A domain's fact holds iff the
// published set CONTAINS that domain's decisions and none of them
// is shadow-only. Domain membership is the registry's
// constraint_adapter_id for the decision's bone: "spine_chain" for
// the spine fact, "arm_left"/"arm_right" for the arm fact. A
// decision whose bone resolves to no registry entry belongs to no
// domain and bears on neither fact (the coordinator publishes
// registry bones only, so this does not arise in production).
// Pure functions over the published set; H10 fixtures them against
// fixture decision sets and the real arbitrator's emitted sets.
bool spineDecisionsRuntimeEffective(
    const FSVRTrackingFrameDecisions& decisions);
bool armDecisionsRuntimeEffective(
    const FSVRTrackingFrameDecisions& decisions);

#endif // LL_LLVRTRACKINGARBITRATOR_H
