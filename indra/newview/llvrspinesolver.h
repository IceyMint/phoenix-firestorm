/**
 * @file llvrspinesolver.h
 * @brief P2-07B C6: the production spine solver (implementation
 *        spec section 6.1; brief P2-07B-C6C7-IMPLEMENTATION-BRIEF,
 *        SHIP round 3, CP-3/CP-4). Implements the C1 transaction's
 *        FSVRAdapterSpineSolver interface against the plan inputs
 *        only: no settings, no records, no frame input, no
 *        OpenXR state enter this class (spec sections 6.1/8).
 */

#ifndef LL_LLVRSPINESOLVER_H
#define LL_LLVRSPINESOLVER_H

#include "llmath.h"

#include "llvradapterapply.h"

#include <map>

class FSVRSpineSolver : public FSVRAdapterSpineSolver
{
public:
    // CP-4. An invalid head target is "no constraint": the out
    // map is left empty. Otherwise bounded damped Gauss-Newton
    // (Levenberg) iterations in swing space over the eligible
    // working domain, clamping through CP-1 after each accepted
    // step, minimizing the cornea-midpoint objective under the
    // final head orientation (cross-check R2-04) plus the D3
    // soft-preference and continuity terms.
    void solveSpine(const FSVRAdapterSolverContext& context,
                    std::map<S32, LLQuaternion>& out_target_worlds)
        override;

    // CP-3 target-composition seam (spec section 5, verbatim):
    // the session-neutral cornea midpoint plus the tracked HMD
    // displacement from session neutral, scaled isotropically
    // by the per-bone head ratio; FSVRDiagForceUnitScale forces
    // the ratio to 1.0 here and nowhere else (brief OQ-6). Pure;
    // harness-exercised in Package A, glue-called in Package B.
    static LLVector3 composeHeadTarget(
        const LLVector3& neutral_cornea_world,
        const LLVector3& hmd_displacement,
        F32 head_ratio, bool force_unit_scale);
};

#endif // LL_LLVRSPINESOLVER_H
