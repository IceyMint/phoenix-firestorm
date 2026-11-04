/**
 * @file llvrspinesolver.cpp
 * @brief P2-07B C6 implementation: see llvrspinesolver.h.
 *
 * Method: bounded damped Gauss-Newton (Levenberg) iterations
 * in swing space on the brief CP-4 objective -- a documented
 * deviation from DP-1's named Jacobian-transpose scheme, taken
 * on harness evidence (see the iteration comment in solveSpine
 * and the C6/C7 work log). The variables are the per-joint
 * swings; each joint's base twist is carried through untouched
 * (design decision D2, zero added twist by construction) and
 * the composed local is clamped through CP-1 after every
 * accepted step (spec section 6.1 step 6); the initial
 * candidate is the base pose's CP-1 projection, so no iterate
 * the solver can emit leaves CP-1's feasible region. The solver
 * keeps no cross-cycle state: continuity enters only as a
 * preference term over the context's previous_targets (DP-4).
 */

#include "llviewerprecompiledheaders.h"

#include "llvrspinesolver.h"

#include "llvrjointlimits.h"

#include <cmath>
#include <set>
#include <vector>

namespace
{

// DP-9 numeric budgets: starting values for Leah's headset
// feel, named so a later tuning pass changes exactly one place.
constexpr S32 kMaxIterations = 16;
constexpr F32 kStepCapRad = 5.f * F_PI / 180.f;
constexpr F32 kConvergenceM = 0.0001f; // 0.1 mm at the cornea
constexpr F32 kWeightSoft = 0.05f;     // W_SOFT (D3 preference)
constexpr F32 kWeightPrevious = 0.02f; // W_PREV (continuity)
constexpr F32 kStepGain = 1.2f;

// The named provisional restricted domain (brief CP-4): a
// configuration consumed only while FSVRDiagFullChainSolve is
// false; NOT a settled rule (OPEN-7 remains open).
const std::set<std::string>& restrictedDomain()
{
    static const std::set<std::string> kRestricted = {"mSpine4",
                                                      "mChest"};
    return kRestricted;
}

// Gate-5 eligibility, evaluated with the transaction's own
// predicate and the gate-4 authority values the transaction
// itself derives from the plan (computePlan), so the solver's
// working domain and the commit's admitted set cannot disagree.
bool solverEligible(const FSVRAdapterPlanInput& plan,
                    const FSVRAdapterJointInput& joint)
{
    const bool spine_authority =
        plan.head_position_enabled && plan.spine_sample_valid;
    const bool arm_authority =
        plan.hand_position_enabled && plan.arm_sample_valid;
    return FSVRAdapterApply::isJointEligible(plan, joint,
                                             spine_authority,
                                             arm_authority);
}

struct ChainState
{
    std::vector<LLQuaternion> world_rot;
    std::vector<LLVector3> world_pos;
};

// Compose the whole plan tree parent-first over the plan root
// frame. domain_local, when given, overrides the local rotation
// of SPINE-domain joints (indexed by plan index; entries for
// other joints are ignored).
ChainState composeChain(const FSVRAdapterPlanInput& plan,
                        const std::map<S32, LLQuaternion>* domain_local)
{
    const S32 count = static_cast<S32>(plan.joints.size());
    ChainState state;
    state.world_rot.assign(count, LLQuaternion());
    state.world_pos.assign(count, LLVector3::zero);
    for (S32 i = 0; i < count; ++i)
    {
        const FSVRAdapterJointInput& joint = plan.joints[i];
        LLQuaternion local = joint.base_local;
        if (domain_local && joint.domain == FSVRAdapterDomain::SPINE)
        {
            auto it = domain_local->find(i);
            if (it != domain_local->end())
            {
                local = it->second;
            }
        }
        local.normalize();
        if (joint.parent_index < 0)
        {
            state.world_rot[i] = local * plan.root_world_rotation;
            state.world_pos[i] =
                joint.rest_offset * plan.root_world_rotation +
                plan.root_world_position;
        }
        else
        {
            state.world_rot[i] =
                local * state.world_rot[joint.parent_index];
            state.world_pos[i] =
                joint.rest_offset * state.world_rot[joint.parent_index] +
                state.world_pos[joint.parent_index];
        }
        state.world_rot[i].normalize();
    }
    return state;
}

S32 findJoint(const FSVRAdapterPlanInput& plan, const char* name)
{
    for (S32 i = 0; i < static_cast<S32>(plan.joints.size()); ++i)
    {
        if (plan.joints[i].name == name)
        {
            return i;
        }
    }
    return -1;
}

} // namespace

LLVector3 FSVRSpineSolver::composeHeadTarget(
    const LLVector3& neutral_cornea_world,
    const LLVector3& hmd_displacement, F32 head_ratio,
    bool force_unit_scale)
{
    const F32 ratio = force_unit_scale ? 1.f : head_ratio;
    return neutral_cornea_world + hmd_displacement * ratio;
}

void FSVRSpineSolver::solveSpine(
    const FSVRAdapterSolverContext& context,
    std::map<S32, LLQuaternion>& out_target_worlds)
{
    if (!context.plan || !context.plan->head_target_valid)
    {
        return;
    }
    const FSVRAdapterPlanInput& plan = *context.plan;
    const S32 head_index = findJoint(plan, "mHead");
    if (head_index < 0)
    {
        return;
    }

    // Working domain: SPINE joints eligible under the
    // transaction's own predicate, so solver and commit can
    // never disagree (brief CP-4); intersected with the named
    // restricted configuration while the full-chain diagnostic
    // is off (OPEN-7).
    std::vector<S32> working;
    std::map<S32, LLQuaternion> swing;
    std::map<S32, LLQuaternion> twist;
    std::map<S32, LLQuaternion> base_swing;
    std::map<S32, LLQuaternion> prev_swing;
    for (S32 i = 0; i < static_cast<S32>(plan.joints.size()); ++i)
    {
        const FSVRAdapterJointInput& joint = plan.joints[i];
        if (joint.domain != FSVRAdapterDomain::SPINE)
        {
            continue;
        }
        if (!solverEligible(plan, joint))
        {
            continue;
        }
        if (!plan.diag_full_chain_solve &&
            restrictedDomain().count(joint.name) == 0)
        {
            continue;
        }
        working.push_back(i);
        LLQuaternion joint_swing;
        LLQuaternion joint_twist;
        FSVRJointLimits::decomposeSwingTwistZ(joint.base_local,
                                              joint_swing, joint_twist);
        swing[i] = joint_swing;
        twist[i] = joint_twist;
        base_swing[i] = joint_swing;

        // Continuity term source: the previous cycle's solved
        // local, formed from the previous world orientations
        // exactly as the transaction forms a local (world
        // against parent world). Absent entries contribute no
        // term (brief CP-4).
        auto target_it = context.previous_targets.find(i);
        if (target_it != context.previous_targets.end())
        {
            LLQuaternion parent_prev_world;
            bool have_parent = false;
            if (joint.parent_index < 0)
            {
                parent_prev_world = plan.root_world_rotation;
                have_parent = true;
            }
            else
            {
                auto parent_it =
                    context.previous_targets.find(joint.parent_index);
                if (parent_it != context.previous_targets.end())
                {
                    parent_prev_world = parent_it->second;
                    have_parent = true;
                }
            }
            if (have_parent)
            {
                LLQuaternion prev_local =
                    target_it->second * ~parent_prev_world;
                prev_local.normalize();
                LLQuaternion prev_joint_swing;
                LLQuaternion prev_joint_twist;
                FSVRJointLimits::decomposeSwingTwistZ(
                    prev_local, prev_joint_swing, prev_joint_twist);
                prev_swing[i] = prev_joint_swing;
            }
        }
    }
    if (working.empty())
    {
        return;
    }

    const LLVector3 target = plan.head_target_world;
    const LLVector3 cornea_offset = plan.cornea_offset_head_local;

    auto composedLocal = [&](S32 joint_index) {
        return FSVRJointLimits::composeSwingTwistZ(swing[joint_index],
                                                   twist[joint_index]);
    };

    // Objective of record (brief CP-4): squared cornea error
    // plus the D3 soft-preference and continuity priors over
    // swing angles. The iteration below minimizes exactly this
    // objective; the priors enter as residual rows whose
    // Jacobian in the swing DOF coordinates is the identity
    // (the standard prior treatment; the rotations involved
    // are within the authored caps, so the frame error of that
    // identification is second-order and absorbed by the
    // damping).
    auto swingAngleVec = [](const LLQuaternion& rel_in) {
        LLQuaternion rel = rel_in;
        rel.normalize();
        if (rel.mQ[VS] < 0.f)
        {
            rel = LLQuaternion(-rel.mQ[VX], -rel.mQ[VY],
                               -rel.mQ[VZ], -rel.mQ[VS]);
        }
        const F32 vlen = std::sqrt(rel.mQ[VX] * rel.mQ[VX] +
                                   rel.mQ[VY] * rel.mQ[VY] +
                                   rel.mQ[VZ] * rel.mQ[VZ]);
        if (vlen < 1e-9f)
        {
            return LLVector3::zero;
        }
        const F32 angle = 2.f * std::atan2(vlen, rel.mQ[VS]);
        const F32 scale = angle / vlen;
        return LLVector3(rel.mQ[VX] * scale, rel.mQ[VY] * scale,
                         rel.mQ[VZ] * scale);
    };
    auto objectiveOf = [&](const std::map<S32, LLQuaternion>& sw,
                           const LLVector3& cornea) {
        F32 obj = (target - cornea).lengthSquared();
        for (S32 i : working)
        {
            if (plan.diag_soft_preference)
            {
                const LLVector3 a =
                    swingAngleVec(sw.at(i) * ~base_swing[i]);
                obj += kWeightSoft * (a.mV[0] * a.mV[0] +
                                      a.mV[1] * a.mV[1]);
            }
            auto prev_it = prev_swing.find(i);
            if (prev_it != prev_swing.end())
            {
                const LLVector3 a =
                    swingAngleVec(sw.at(i) * ~prev_it->second);
                obj += kWeightPrevious * (a.mV[0] * a.mV[0] +
                                          a.mV[1] * a.mV[1]);
            }
        }
        return obj;
    };
    auto corneaOfSwings = [&](const std::map<S32, LLQuaternion>& sw) {
        std::map<S32, LLQuaternion> locals;
        for (S32 i : working)
        {
            locals[i] = FSVRJointLimits::composeSwingTwistZ(
                sw.at(i), twist[i]);
        }
        const ChainState state = composeChain(plan, &locals);
        return cornea_offset * state.world_rot[head_index] +
               state.world_pos[head_index];
    };
    // After a step, clamp every working joint's composed local
    // through CP-1 (unless the diagnostic suspends it) and
    // re-derive the swing, so the carried twist is exactly the
    // base twist and the swing stays the only variable.
    auto clampAndRederive = [&](std::map<S32, LLQuaternion>& sw) {
        for (S32 i : working)
        {
            LLQuaternion local =
                FSVRJointLimits::composeSwingTwistZ(sw[i], twist[i]);
            if (plan.diag_bend_limits)
            {
                FSVRSpineJointCaps caps;
                if (FSVRJointLimits::spineCapsForJoint(
                        plan.joints[i].name, caps))
                {
                    local = FSVRJointLimits::clampSpineLocal(local,
                                                             caps);
                }
            }
            LLQuaternion new_swing;
            LLQuaternion new_twist;
            FSVRJointLimits::decomposeSwingTwistZ(local, new_swing,
                                                  new_twist);
            sw[i] = new_swing;
        }
    };

    // Iteration: damped Gauss-Newton (Levenberg) steps on the
    // objective above. DEVIATION from brief DP-1's named
    // Jacobian-transpose scheme, taken on harness evidence and
    // reported with this component: fixed-gain JT on this chain
    // is ill-conditioned (the cornea's vertical mode's singular
    // value is ~25x smaller than the flexion mode's), so within
    // the DP-9 budget of 16 iterations JT oscillated in the
    // stiff mode and crawled in the soft one, ending 13.7 cm
    // from a reachable target (traces in the C6/C7 work log).
    // The objective, weights, per-iteration step cap, clamp
    // discipline and iteration budget are unchanged; only the
    // step computation differs. The best iterate seen (by the
    // objective) is the one emitted, so the solver can never
    // emit a pose worse than the base pose's objective.
    //
    // Feasible initial candidate [R2 -- C6C7-CODE-02]: project
    // every working joint's base swing through the CP-1 clamp
    // BEFORE the convergence test and the best-iterate
    // initialization, so the candidate the loop evaluates,
    // improves and emits is inside the feasible region from
    // the start. (Previously best_swing initialized from the
    // raw animated pose; a base pose already at its
    // out-of-caps cornea target exited on the convergence test
    // and emitted the unclamped swing.) base_swing keeps the
    // original animated pose separately -- it is the D3
    // soft-preference reference, and the projection must not
    // contaminate it. With the bend-limits diagnostic off the
    // clamp is suspended and this projection is a no-op: no
    // feasible region is defined then.
    clampAndRederive(swing);

    const S32 ndof = static_cast<S32>(working.size()) * 2;
    std::map<S32, LLQuaternion> best_swing = swing;
    F32 best_obj = objectiveOf(swing, corneaOfSwings(swing));
    F32 lambda = 1e-3f;
    for (S32 iteration = 0; iteration < kMaxIterations; ++iteration)
    {
        std::map<S32, LLQuaternion> locals;
        for (S32 i : working)
        {
            locals[i] = composedLocal(i);
        }
        const ChainState state = composeChain(plan, &locals);
        const LLVector3 cornea =
            cornea_offset * state.world_rot[head_index] +
            state.world_pos[head_index];
        const LLVector3 error_vec = target - cornea;
        if (error_vec.length() < kConvergenceM)
        {
            break;
        }

        // Jacobian columns: cornea velocity per unit swing
        // step about the parent frame's X / Y axes.
        std::vector<LLVector3> jac(ndof);
        for (S32 k = 0; k < static_cast<S32>(working.size()); ++k)
        {
            const S32 i = working[k];
            const FSVRAdapterJointInput& joint = plan.joints[i];
            const LLQuaternion parent_world =
                joint.parent_index < 0
                    ? plan.root_world_rotation
                    : state.world_rot[joint.parent_index];
            const LLVector3 axis_x =
                LLVector3(1.f, 0.f, 0.f) * parent_world;
            const LLVector3 axis_y =
                LLVector3(0.f, 1.f, 0.f) * parent_world;
            const LLVector3 r = cornea - state.world_pos[i];
            jac[2 * k] = axis_x % r;
            jac[2 * k + 1] = axis_y % r;
        }
        // Gradient of the objective: -J^T e + priors.
        std::vector<F32> grad(ndof, 0.f);
        for (S32 d = 0; d < ndof; ++d)
        {
            grad[d] = -(jac[d] * error_vec);
        }
        for (S32 k = 0; k < static_cast<S32>(working.size()); ++k)
        {
            const S32 i = working[k];
            if (plan.diag_soft_preference)
            {
                const LLVector3 a =
                    swingAngleVec(swing[i] * ~base_swing[i]);
                grad[2 * k] += kWeightSoft * a.mV[0];
                grad[2 * k + 1] += kWeightSoft * a.mV[1];
            }
            auto prev_it = prev_swing.find(i);
            if (prev_it != prev_swing.end())
            {
                const LLVector3 a =
                    swingAngleVec(swing[i] * ~prev_it->second);
                grad[2 * k] += kWeightPrevious * a.mV[0];
                grad[2 * k + 1] += kWeightPrevious * a.mV[1];
            }
        }
        // Normal matrix J^T J (+ prior weights on the diagonal).
        std::vector<F32> normal(ndof * ndof, 0.f);
        for (S32 a = 0; a < ndof; ++a)
        {
            for (S32 b = 0; b < ndof; ++b)
            {
                normal[a * ndof + b] = jac[a] * jac[b];
            }
        }
        for (S32 k = 0; k < static_cast<S32>(working.size()); ++k)
        {
            F32 prior = 0.f;
            if (plan.diag_soft_preference)
            {
                prior += kWeightSoft;
            }
            if (prev_swing.count(working[k]))
            {
                prior += kWeightPrevious;
            }
            normal[(2 * k) * ndof + 2 * k] += prior;
            normal[(2 * k + 1) * ndof + 2 * k + 1] += prior;
        }

        // Damped solve with retry: a step is accepted only if
        // the (post-clamp) objective improves. The step is
        // scaled UNIFORMLY (direction preserved) to the DP-9
        // per-joint cap, then backtracked; per-joint clamping
        // of the step vector would distort the Gauss-Newton
        // direction and stall the iteration (harness evidence:
        // the first implementation stalled at 19 mm on a
        // reachable target).
        bool accepted = false;
        for (S32 attempt = 0; attempt < 4 && !accepted; ++attempt)
        {
            std::vector<F32> mat = normal;
            for (S32 d = 0; d < ndof; ++d)
            {
                mat[d * ndof + d] += lambda;
            }
            std::vector<F32> rhs(ndof);
            for (S32 d = 0; d < ndof; ++d)
            {
                rhs[d] = -grad[d];
            }
            // Gaussian elimination with partial pivoting.
            bool singular = false;
            for (S32 col = 0; col < ndof && !singular; ++col)
            {
                S32 pivot = col;
                for (S32 row = col + 1; row < ndof; ++row)
                {
                    if (std::fabs(mat[row * ndof + col]) >
                        std::fabs(mat[pivot * ndof + col]))
                    {
                        pivot = row;
                    }
                }
                if (std::fabs(mat[pivot * ndof + col]) < 1e-12f)
                {
                    singular = true;
                    break;
                }
                if (pivot != col)
                {
                    for (S32 cc = 0; cc < ndof; ++cc)
                    {
                        std::swap(mat[col * ndof + cc],
                                  mat[pivot * ndof + cc]);
                    }
                    std::swap(rhs[col], rhs[pivot]);
                }
                for (S32 row = col + 1; row < ndof; ++row)
                {
                    const F32 factor =
                        mat[row * ndof + col] / mat[col * ndof + col];
                    for (S32 cc = col; cc < ndof; ++cc)
                    {
                        mat[row * ndof + cc] -=
                            factor * mat[col * ndof + cc];
                    }
                    rhs[row] -= factor * rhs[col];
                }
            }
            if (singular)
            {
                lambda *= 10.f;
                continue;
            }
            std::vector<F32> step(ndof, 0.f);
            for (S32 row = ndof - 1; row >= 0; --row)
            {
                F32 sum = rhs[row];
                for (S32 cc = row + 1; cc < ndof; ++cc)
                {
                    sum -= mat[row * ndof + cc] * step[cc];
                }
                step[row] = sum / mat[row * ndof + row];
            }
            F32 max_joint_step = 0.f;
            for (S32 k = 0; k < static_cast<S32>(working.size());
                 ++k)
            {
                const F32 len = std::sqrt(step[2 * k] * step[2 * k] +
                                          step[2 * k + 1] *
                                              step[2 * k + 1]);
                max_joint_step = std::max(max_joint_step, len);
            }
            F32 scale = 1.f;
            if (max_joint_step > kStepCapRad)
            {
                scale = kStepCapRad / max_joint_step;
            }
            for (F32 backtrack = 1.f; backtrack >= 0.0625f && !accepted;
                 backtrack *= 0.5f)
            {
                std::map<S32, LLQuaternion> trial = swing;
                for (S32 k = 0; k < static_cast<S32>(working.size());
                     ++k)
                {
                    const F32 step_x =
                        step[2 * k] * scale * backtrack;
                    const F32 step_y =
                        step[2 * k + 1] * scale * backtrack;
                    if (std::fabs(step_x) < 1e-12f &&
                        std::fabs(step_y) < 1e-12f)
                    {
                        continue;
                    }
                    const LLQuaternion dq =
                        LLQuaternion(step_x,
                                     LLVector3(1.f, 0.f, 0.f)) *
                        LLQuaternion(step_y,
                                     LLVector3(0.f, 1.f, 0.f));
                    trial[working[k]] = trial[working[k]] * dq;
                    trial[working[k]].normalize();
                }
                clampAndRederive(trial);
                const F32 trial_obj =
                    objectiveOf(trial, corneaOfSwings(trial));
                if (trial_obj < best_obj)
                {
                    swing = trial;
                    best_swing = trial;
                    best_obj = trial_obj;
                    lambda = std::max(lambda / 3.f, 1e-6f);
                    accepted = true;
                }
            }
            if (!accepted)
            {
                lambda *= 10.f;
            }
        }
        if (!accepted)
        {
            break;
        }
    }
    swing = best_swing;

    // Emission: compose the final chain parent-first and emit
    // the working domain's world orientations; nothing else
    // (pelvis/root/neck/head are never emitted, D1/D7).
    std::map<S32, LLQuaternion> locals;
    for (S32 i : working)
    {
        locals[i] = composedLocal(i);
    }
    const ChainState state = composeChain(plan, &locals);
    for (S32 i : working)
    {
        out_target_worlds[i] = state.world_rot[i];
    }
}
