/**
 * @file llvrarmsolver.h
 * @brief P2-07B C7: the production write-through arm solver and
 *        arm post-clamp (implementation spec section 6.2; brief
 *        P2-07B-C6C7-IMPLEMENTATION-BRIEF, SHIP round 3,
 *        CP-3/CP-5/CP-6). See llvrarmsolver.cpp for the procedure
 *        notes. Like C6, these classes consume plan inputs only.
 */

#ifndef LL_LLVRARMSOLVER_H
#define LL_LLVRARMSOLVER_H

#include "llmath.h"

#include "llvradapterapply.h"

#include <map>

class FSVRArmSolver : public FSVRAdapterWriteThroughArmSolver
{
public:
    // CP-5. Both-eligible sides run the scratch-chain RP3
    // procedure; a side with an ineligible elbow runs
    // shoulder-only aiming against the preserved elbow
    // configuration's radius [R1 -- C6C7-01]; a side with an
    // ineligible shoulder runs elbow-only flexion, whose 145
    // degree anatomical cap applies only while
    // FSVRDiagBendLimits is on [R1 -- C6C7-02]. Pole history is
    // caller-owned input/output state only (context in, outcome
    // out); this class stores none of it [R1/R2 -- C6C7-03].
    FSVRAdapterWriteThroughOutcome solveArmWriteThrough(
        const FSVRAdapterSolverContext& context,
        FSVRAdapterSide side,
        FSVRAdapterWriteThroughHost& host) override;

    // CP-3 target-composition seam, OPEN-4-PROVISIONAL (spec
    // section 6.2): the target is the SOLVED shoulder's world
    // position plus the player's hand displacement from session
    // neutral, scaled by the per-bone arm ratio (forced to 1.0
    // by FSVRDiagForceUnitScale). OPEN-4 (shoulder-anchored
    // versus root-anchored runtime composition) is unresolved;
    // when it resolves, this function is where the resolution
    // lands. Pure; nothing in Package A calls it from production
    // code.
    static LLVector3 composeWristTargetProvisional(
        const LLVector3& solved_shoulder_world,
        const LLVector3& player_displacement_from_neutral,
        F32 arm_ratio, bool force_unit_scale);
};

// CP-6: the arm post-clamp. Applies CP-1 to the write-through
// readback (shoulder against the chest frame, elbow segment
// cap) by editing the readback locals the transaction then
// recomposes from. Mandatory but inert while
// FSVRDiagBendLimits is false: it returns without edits, and
// the transaction still runs -- the clamp existing but inert is
// what keeps the side from failing closed under the diagnostic
// (spec section 6.1).
class FSVRArmPostClamp : public FSVRAdapterArmPostClamp
{
public:
    void clampArm(
        const FSVRAdapterSolverContext& context,
        FSVRAdapterSide side,
        const std::map<S32, LLQuaternion>& readback_world_rotations,
        const std::map<S32, LLVector3>& readback_world_positions,
        std::map<S32, LLQuaternion>& in_out_readback_locals) override;
};

#endif // LL_LLVRARMSOLVER_H
