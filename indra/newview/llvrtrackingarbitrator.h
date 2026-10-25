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

#endif // LL_LLVRTRACKINGARBITRATOR_H
