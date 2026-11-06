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
    // final head FRAME -- orientation and position (cross-check
    // R2-04, by-construction reading per P2-07B Fix 1 CP-F1-2:
    // the objective's cornea is evaluated at the
    // post-reconciliation frame the adapter will commit for the
    // same chain state, via evaluatePostReconciliationCornea)
    // plus the D3 soft-preference and continuity terms.
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

    // P2-07B Fix 1 (CP-F1-2): the post-reconciliation cornea
    // model. Composes the plan chain from the given
    // domain-locals map (exactly the map corneaOfSwings builds
    // from its swings), reconstructs the head frame CP-F1-1
    // will commit for that chain state -- the neck and head
    // stage worlds restored to the base composition under the
    // same limit cascade and the same combined-bound
    // selection, refusal included -- and evaluates the cornea
    // under that frame: orientation AND position, so the
    // cornea the solver optimizes is the cornea that will
    // exist (INV-F1-06). Pure over (plan, base world
    // rotations, domain locals); harness-exercised through
    // this seam (F1S-3 finite-differences it). Returns the
    // model cornea; the out-parameters (any may be null)
    // receive the model head world rotation, the model head
    // world position, and the model neck world position --
    // the composed chain's neck origin, the reconstructed
    // cornea's only swing-dependent point, consumed by the
    // iteration's Jacobian (FIX1-R2-01). If either head-group
    // name is absent from the plan, the evaluation falls back
    // to the pre-Fix-1 chain evaluation (the absence rule
    // shared with CP-F1-1).
    static LLVector3 evaluatePostReconciliationCornea(
        const FSVRAdapterPlanInput& plan,
        const std::vector<LLQuaternion>& base_world_rotations,
        const std::map<S32, LLQuaternion>& domain_local,
        LLQuaternion* out_model_head_world,
        LLVector3* out_model_head_pos,
        LLVector3* out_model_neck_pos);
};

#endif // LL_LLVRSPINESOLVER_H
