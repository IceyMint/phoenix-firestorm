/**
 * @file llvradapterapply.cpp
 * @brief P2-07B C1: Stage-2 adapter apply point. See
 *        llvradapterapply.h for the contract.
 *
 * Composition algebra (C0 section 4.2.1, proven from
 * llquaternion.cpp:534-546): in LL's operator*, the LEFT operand
 * applies first, so a joint's world rotation is
 *     world = local * parent_world
 * and the committed local for a desired world target is
 *     local = target_world * ~parent_world
 * with world quaternions normalized before operator~ (operator*
 * does not renormalize). Parent worlds are always composed from
 * the just-committed locals of this transaction (same-cycle by
 * construction); LLJoint::setWorldRotation is never used, because
 * it derives locals from the parent's CACHED world matrix, which
 * inside updateMotions is still the previous cycle's (brief
 * section 2.6).
 */

#include "llviewerprecompiledheaders.h"

#include "llvradapterapply.h"

#include <set>

namespace
{

// Compose the base (arbitrated) pose over the root frame, parents
// first. The input list is topologically ordered.
void composeBaseWorlds(const FSVRAdapterPlanInput& input,
                       std::vector<LLQuaternion>& out_rotations,
                       std::vector<LLVector3>& out_positions)
{
    const S32 count = static_cast<S32>(input.joints.size());
    out_rotations.resize(count);
    out_positions.resize(count);
    for (S32 i = 0; i < count; ++i)
    {
        const FSVRAdapterJointInput& joint = input.joints[i];
        LLQuaternion parent_rot = input.root_world_rotation;
        LLVector3 parent_pos = input.root_world_position;
        if (joint.parent_index >= 0)
        {
            parent_rot = out_rotations[joint.parent_index];
            parent_pos = out_positions[joint.parent_index];
        }
        LLQuaternion world = joint.base_local * parent_rot;
        world.normalize();
        out_rotations[i] = world;
        out_positions[i] = parent_pos + (joint.rest_offset * parent_rot);
    }
}

bool domainAuthority(const FSVRAdapterJointInput& joint,
                     bool spine_authority, bool arm_authority)
{
    switch (joint.domain)
    {
    case FSVRAdapterDomain::SPINE:
        return spine_authority;
    case FSVRAdapterDomain::ARM_LEFT:
    case FSVRAdapterDomain::ARM_RIGHT:
        return arm_authority;
    default:
        return false;
    }
}

// The plan-side write-through host (boundary contract in the
// header). It holds the transaction's provisional state for one
// side's arm phase: a copy of the locals the transaction has
// resolved so far (post-spine), which the solver edits through
// the host. The transaction reads the result back only if the
// solver completes; on abort the host is simply destroyed with
// the transaction's committed state untouched (contract point
// (c)). Worlds are composed from the provisional locals on every
// read -- the host keeps no world cache, so a parent-world read
// after a provisional parent-changing write necessarily reflects
// it (contract point (b)).
class PlanWriteThroughHost final : public FSVRAdapterWriteThroughHost
{
public:
    PlanWriteThroughHost(const FSVRAdapterPlanInput& input,
                         FSVRAdapterDomain side_domain,
                         const std::vector<LLQuaternion>& seed_locals)
        : mInput(input), mSideDomain(side_domain), mLocals(seed_locals)
    {
    }

    bool writeProvisionalLocal(S32 joint_index,
                               const LLQuaternion& local) override
    {
        if (!inRange(joint_index) ||
            mInput.joints[joint_index].domain != mSideDomain)
        {
            // Only this side's arm chain may be written through
            // the host (contract point (a)).
            return false;
        }
        LLQuaternion q = local;
        q.normalize();
        mLocals[joint_index] = q;
        mWritten.insert(joint_index);
        return true;
    }

    bool readProvisionalLocal(S32 joint_index,
                              LLQuaternion& out_local) const override
    {
        if (!inRange(joint_index))
        {
            return false;
        }
        out_local = mLocals[joint_index];
        return true;
    }

    bool readWorldRotation(S32 joint_index,
                           LLQuaternion& out_rotation) const override
    {
        LLVector3 pos;
        return composeWorld(joint_index, out_rotation, pos);
    }

    bool readWorldPosition(S32 joint_index,
                           LLVector3& out_position) const override
    {
        LLQuaternion rot;
        return composeWorld(joint_index, rot, out_position);
    }

    bool wasWritten(S32 joint_index) const
    {
        return mWritten.find(joint_index) != mWritten.end();
    }

private:
    bool inRange(S32 index) const
    {
        return index >= 0 &&
               index < static_cast<S32>(mInput.joints.size());
    }

    bool composeWorld(S32 joint_index, LLQuaternion& out_rot,
                      LLVector3& out_pos) const
    {
        if (!inRange(joint_index))
        {
            return false;
        }
        // Collect the chain up to the plan root, then compose
        // downward from the root frame.
        std::vector<S32> chain;
        for (S32 cursor = joint_index; cursor >= 0;
             cursor = mInput.joints[cursor].parent_index)
        {
            chain.push_back(cursor);
        }
        LLQuaternion rot = mInput.root_world_rotation;
        LLVector3 pos = mInput.root_world_position;
        for (auto it = chain.rbegin(); it != chain.rend(); ++it)
        {
            const FSVRAdapterJointInput& joint = mInput.joints[*it];
            LLQuaternion world = mLocals[*it] * rot;
            world.normalize();
            pos = pos + (joint.rest_offset * rot);
            rot = world;
        }
        out_rot = rot;
        out_pos = pos;
        return true;
    }

    const FSVRAdapterPlanInput& mInput;
    FSVRAdapterDomain mSideDomain;
    std::vector<LLQuaternion> mLocals;
    std::set<S32> mWritten;
};

} // namespace

// static
bool FSVRAdapterApply::isJointEligible(const FSVRAdapterPlanInput& input,
                                       const FSVRAdapterJointInput& joint,
                                       bool spine_authority,
                                       bool arm_authority)
{
    if (!joint.in_registry)
    {
        return false;
    }
    if (joint.domain == FSVRAdapterDomain::NONE)
    {
        // Head, neck, collars, structural ancestors: never written
        // (brief section 3.5 [R1], spec section 6.2).
        return false;
    }
    if (!joint.decision_grants_base &&
        !domainAuthority(joint, spine_authority, arm_authority))
    {
        return false;
    }
    if (joint.disabled)
    {
        // L4 exception is spine-only (spec section 6.1; D4: it
        // never extends to arms).
        if (!(input.bend_disabled_bones &&
              joint.domain == FSVRAdapterDomain::SPINE))
        {
            return false;
        }
    }
    return true;
}

// static
FSVRAdapterCycleKind FSVRAdapterApply::classifyCycle(
    bool preview_cycle, bool controller_paused)
{
    if (preview_cycle)
    {
        return FSVRAdapterCycleKind::PREVIEW;
    }
    if (controller_paused)
    {
        // A paused controller reaches the post-blend hook only
        // through FORCE_UPDATE (brief section 2.3).
        return FSVRAdapterCycleKind::PAUSED_FORCE;
    }
    return FSVRAdapterCycleKind::NORMAL;
}

// static
FSVRAdapterPlanResult FSVRAdapterApply::computePlan(
    const FSVRAdapterPlanInput& input,
    FSVRAdapterSpineSolver& spine_solver,
    FSVRAdapterArmSolver& arm_solver)
{
    return computePlan(input, spine_solver, arm_solver, nullptr, nullptr);
}

// static
FSVRAdapterPlanResult FSVRAdapterApply::computePlan(
    const FSVRAdapterPlanInput& input,
    FSVRAdapterSpineSolver& spine_solver,
    FSVRAdapterArmSolver& arm_solver,
    FSVRAdapterWriteThroughArmSolver* write_through_solver,
    FSVRAdapterArmPostClamp* post_clamp)
{
    FSVRAdapterPlanResult result;
    const S32 count = static_cast<S32>(input.joints.size());

    // The base composition is computed in every outcome, so a
    // gate-failed result still reports the arbitrated pose as the
    // resolved pose (no-write = arbitrated pose, brief section
    // 3.6).
    std::vector<LLQuaternion> base_world_rotations;
    std::vector<LLVector3> base_world_positions;
    composeBaseWorlds(input, base_world_rotations, base_world_positions);
    result.resolved_locals.resize(count);
    for (S32 i = 0; i < count; ++i)
    {
        result.resolved_locals[i] = input.joints[i].base_local;
    }
    result.resolved_world_rotations = base_world_rotations;
    result.resolved_world_positions = base_world_positions;

    auto fail = [&result](FSVRAdapterGate gate) {
        result.first_failing_gate = gate;
        return result;
    };

    // Cycle gate: production gate inputs (brief section 4 [R1]
    // cases). A paused controller's ordinary update never blends;
    // a FORCE_UPDATE on a paused controller does reach the blend
    // (section 2.3), and an Animation Preview cycle publishes live
    // decisions -- the gates, not the hook placement, make both
    // no-ops (OQ-5, discharged by test).
    if (!input.blend_ran || input.cycle != FSVRAdapterCycleKind::NORMAL)
    {
        return fail(FSVRAdapterGate::CYCLE);
    }
    // Non-self context (the bridge exists only on LLVOAvatarSelf;
    // this check is what the harness exercises).
    if (!input.context_armed)
    {
        return fail(FSVRAdapterGate::CONTEXT);
    }
    // Gate 1: frame identity. Consuming Stage-1 shadow decisions
    // as if they were grants is the failure mode this gate exists
    // to prevent.
    if (!input.decisions_published || !input.decisions_current ||
        !input.decisions_runtime_effective)
    {
        return fail(FSVRAdapterGate::FRAME_IDENTITY);
    }
    // Gate 2 [R1]: the headset/session gate, full stop.
    if (!input.vr_active)
    {
        return fail(FSVRAdapterGate::VR_SESSION);
    }
    // Gate 3: positional master gate (spec section 3.4).
    if (!input.positional_solve_enabled)
    {
        return fail(FSVRAdapterGate::POSITIONAL_MASTER);
    }
    // Gate 4 [R1]: per-solver enablement and per-solver positional
    // sample validity, evaluated independently.
    const bool spine_authority =
        input.head_position_enabled && input.spine_sample_valid;
    const bool arm_authority =
        input.hand_position_enabled && input.arm_sample_valid;
    if (!spine_authority && !arm_authority)
    {
        return fail(FSVRAdapterGate::NO_SOLVER);
    }

    result.applied = true;

    FSVRAdapterSolverContext context;
    context.plan = &input;
    context.base_world_rotations = base_world_rotations;
    context.base_world_positions = base_world_positions;
    context.previous_targets = input.previous_targets;

    // Committed state for the transaction: starts as the base
    // pose and is updated joint by joint, parents first.
    std::vector<LLQuaternion> committed_rotations = base_world_rotations;
    std::vector<LLVector3> committed_positions = base_world_positions;
    std::vector<LLQuaternion> committed_locals = result.resolved_locals;

    auto parentFrame = [&](S32 index, LLQuaternion& out_rot, LLVector3& out_pos) {
        const S32 parent = input.joints[index].parent_index;
        if (parent >= 0)
        {
            out_rot = committed_rotations[parent];
            out_pos = committed_positions[parent];
        }
        else
        {
            out_rot = input.root_world_rotation;
            out_pos = input.root_world_position;
        }
    };

    // One joint of a commit pass: honor the pass's target map for
    // eligible joints of the pass's domains, otherwise carry the
    // base local over the just-committed parents.
    auto commitJoint = [&](S32 i,
                           const std::map<S32, LLQuaternion>& targets,
                           bool honor_targets) {
        const FSVRAdapterJointInput& joint = input.joints[i];
        LLQuaternion parent_rot;
        LLVector3 parent_pos;
        parentFrame(i, parent_rot, parent_pos);

        bool wrote = false;
        auto target_it = targets.find(i);
        if (honor_targets && target_it != targets.end() &&
            isJointEligible(input, joint, spine_authority, arm_authority))
        {
            LLQuaternion target = target_it->second;
            target.normalize();
            LLQuaternion parent_norm = parent_rot;
            parent_norm.normalize();
            // local = target_world * ~parent_world (C0 4.2.1).
            LLQuaternion local = target * ~parent_norm;
            committed_locals[i] = local;
            committed_rotations[i] = target;
            result.writes.push_back({i, local});
            wrote = true;
        }
        if (!wrote)
        {
            LLQuaternion world = joint.base_local * parent_rot;
            world.normalize();
            committed_rotations[i] = world;
            committed_locals[i] = joint.base_local;
        }
        // A joint's world position depends on its parent's world
        // rotation, never on its own rotation.
        committed_positions[i] = parent_pos + (joint.rest_offset * parent_rot);
    };

    // D6 order, step 1: run C6 against the base pose (brief
    // section 3.4 step 2). C6 returns target worlds; it writes
    // nothing.
    std::map<S32, LLQuaternion> spine_targets;
    if (spine_authority)
    {
        spine_solver.solveSpine(context, spine_targets);
        result.spine_solver_invoked = true;
    }

    // D6 order, step 2 (brief section 3.4 steps 3-4): commit the
    // spine parent-first. Non-arm joints carry their base locals
    // over the committed parents; arm joints are composed the same
    // way for now, which is what resolves the shoulders from the
    // solved spine.
    for (S32 i = 0; i < count; ++i)
    {
        const FSVRAdapterDomain domain = input.joints[i].domain;
        const bool is_arm = (domain == FSVRAdapterDomain::ARM_LEFT ||
                             domain == FSVRAdapterDomain::ARM_RIGHT);
        commitJoint(i, spine_targets, !is_arm &&
                        domain == FSVRAdapterDomain::SPINE);
    }

    // The write-through arm phase for one side (boundary contract
    // in the header): run the solver against a provisional host
    // seeded from the transaction's committed state, then
    // readback -> post-clamp -> commit, all inside the
    // transaction. Returns true iff the write-through solver was
    // actually invoked for the side -- a side that fails closed
    // on the mandatory post-clamp (R2-01) or has no chain returns
    // false, so the caller's invoked flags never claim a run
    // that did not happen.
    auto runWriteThroughSide = [&](FSVRAdapterSide side) {
        const FSVRAdapterDomain side_domain =
            (side == FSVRAdapterSide::LEFT) ? FSVRAdapterDomain::ARM_LEFT
                                            : FSVRAdapterDomain::ARM_RIGHT;
        FSVRAdapterArmOutcome& outcome =
            (side == FSVRAdapterSide::LEFT) ? result.arm_outcome_left
                                            : result.arm_outcome_right;

        // The side's chain in list order (shoulder first, elbow
        // last), and the wrist: the structural joint whose parent
        // is the chain's last joint.
        std::vector<S32> chain;
        for (S32 i = 0; i < count; ++i)
        {
            if (input.joints[i].domain == side_domain)
            {
                chain.push_back(i);
            }
        }
        if (chain.empty())
        {
            return false;
        }
        S32 wrist_index = -1;
        for (S32 i = 0; i < count; ++i)
        {
            if (input.joints[i].parent_index == chain.back() &&
                input.joints[i].domain != side_domain)
            {
                wrist_index = i;
                break;
            }
        }

        outcome.write_through_ran = true;
        if (!post_clamp)
        {
            // R2-01 fail-closed: the post-clamp is mandatory for
            // a write-through transaction (contract point (d)).
            // Without it the side's write-through result is
            // aborted before the solver runs -- nothing provisional
            // exists, nothing reaches the commit, and the side
            // keeps the pose step 2 already resolved for it. The
            // outcome names the missing clamp; that outcome, on
            // the plan result, is the diagnostic.
            outcome.write_through_aborted = true;
            outcome.abort_reason =
                FSVRAdapterArmAbortReason::MISSING_POST_CLAMP;
            return false;
        }
        PlanWriteThroughHost host(input, side_domain, committed_locals);
        const FSVRAdapterWriteThroughOutcome wt =
            write_through_solver->solveArmWriteThrough(context, side, host);
        if (!wt.completed)
        {
            // Contract point (c): the provisional state is
            // discarded entire. The side keeps the composition
            // step 2 already resolved for it; nothing provisional
            // reaches the commit or the resolved pose.
            outcome.write_through_aborted = true;
            outcome.abort_reason =
                FSVRAdapterArmAbortReason::SOLVER_INCOMPLETE;
            return true;
        }

        // Readback (contract point (d)): the provisional locals
        // for the chain, and the worlds composed from them over
        // the committed parents.
        std::map<S32, LLQuaternion> readback_locals;
        std::map<S32, LLQuaternion> readback_rotations;
        std::map<S32, LLVector3> readback_positions;
        {
            std::vector<LLQuaternion> rb_rot = committed_rotations;
            std::vector<LLVector3> rb_pos = committed_positions;
            auto readbackCompose = [&](S32 i) {
                const FSVRAdapterJointInput& joint = input.joints[i];
                LLQuaternion parent_rot = input.root_world_rotation;
                LLVector3 parent_pos = input.root_world_position;
                if (joint.parent_index >= 0)
                {
                    parent_rot = rb_rot[joint.parent_index];
                    parent_pos = rb_pos[joint.parent_index];
                }
                LLQuaternion local = joint.base_local;
                auto it = readback_locals.find(i);
                if (it != readback_locals.end())
                {
                    local = it->second;
                }
                LLQuaternion world = local * parent_rot;
                world.normalize();
                rb_rot[i] = world;
                rb_pos[i] = parent_pos + (joint.rest_offset * parent_rot);
                readback_rotations[i] = rb_rot[i];
                readback_positions[i] = rb_pos[i];
            };
            for (S32 i : chain)
            {
                LLQuaternion local;
                host.readProvisionalLocal(i, local);
                readback_locals[i] = local;
            }
            for (S32 i : chain)
            {
                readbackCompose(i);
            }
            if (wrist_index >= 0)
            {
                readbackCompose(wrist_index);
            }
        }

        // The post-RP3 clamp (C0 sections 5.2/5.3), invoked by the
        // transaction on the readback, before commit -- the
        // ordering is the transaction's guarantee (brief section
        // 3.4). The clamp is guaranteed present here: a missing
        // clamp failed the side closed above (R2-01).
        post_clamp->clampArm(context, side, readback_rotations,
                              readback_positions, readback_locals);

        // Gate-5 rejection set for this pass, with the R2-02
        // cascade: a joint the solver provisionally wrote is
        // rejected when gate 5 does not admit it, and every
        // provisionally written descendant of a rejected joint is
        // rejected with it -- its readback local was computed
        // against the provisional parent the commit did not take,
        // so committing it would produce a world the solver never
        // produced and the clamp never evaluated. The chain is in
        // list order (parents before children), so a single pass
        // propagates the cascade transitively. (The plain
        // target-world path needs no cascade: commitJoint composes
        // each local from the target against the parent's ACTUAL
        // committed world, so a rejected parent cannot corrupt a
        // descendant there.)
        std::set<S32> rejected;
        for (S32 i : chain)
        {
            if (!host.wasWritten(i))
            {
                continue;
            }
            bool reject =
                !isJointEligible(input, input.joints[i],
                                 spine_authority, arm_authority);
            for (S32 p = input.joints[i].parent_index; !reject && p >= 0;
                 p = input.joints[p].parent_index)
            {
                reject = rejected.find(p) != rejected.end();
            }
            if (reject)
            {
                rejected.insert(i);
            }
        }

        // Commit the chain parent-first: joints the solver wrote
        // and the rejection set admits take the (possibly clamped)
        // readback local; everything else carries its base local
        // over the just-committed parents, exactly as the target
        // path does.
        for (S32 i : chain)
        {
            const FSVRAdapterJointInput& joint = input.joints[i];
            LLQuaternion parent_rot;
            LLVector3 parent_pos;
            parentFrame(i, parent_rot, parent_pos);
            if (host.wasWritten(i) && rejected.find(i) == rejected.end())
            {
                LLQuaternion local = readback_locals[i];
                local.normalize();
                committed_locals[i] = local;
                LLQuaternion world = local * parent_rot;
                world.normalize();
                committed_rotations[i] = world;
                result.writes.push_back({i, local});
            }
            else
            {
                LLQuaternion world = joint.base_local * parent_rot;
                world.normalize();
                committed_rotations[i] = world;
                committed_locals[i] = joint.base_local;
            }
            committed_positions[i] =
                parent_pos + (joint.rest_offset * parent_rot);
        }

        // The wrist rides on the committed elbow: recompose it so
        // the resolved pose -- and the wrist error below -- reflect
        // the final clamped rotations, not the pre-solve ones.
        if (wrist_index >= 0)
        {
            const FSVRAdapterJointInput& wrist = input.joints[wrist_index];
            LLQuaternion parent_rot;
            LLVector3 parent_pos;
            parentFrame(wrist_index, parent_rot, parent_pos);
            LLQuaternion world = wrist.base_local * parent_rot;
            world.normalize();
            committed_rotations[wrist_index] = world;
            committed_positions[wrist_index] =
                parent_pos + (wrist.rest_offset * parent_rot);
            if (wt.has_wrist_target)
            {
                outcome.wrist_error =
                    (committed_positions[wrist_index] - wt.wrist_target_world)
                        .length();
                outcome.wrist_error_valid = true;
            }
        }
        return true;
    };

    // D6 order, step 3 (brief section 3.4 step 5): run C7 per side
    // on the solved shoulders, then commit the arms parent-first.
    // Hand targets never re-open the spine: the spine pass is
    // already complete.
    if (arm_authority)
    {
        const FSVRAdapterSide sides[2] = {FSVRAdapterSide::LEFT,
                                          FSVRAdapterSide::RIGHT};
        for (FSVRAdapterSide side : sides)
        {
            // The shoulder is the first joint of the side's arm
            // domain in list order (its parent is the collar).
            S32 shoulder_index = -1;
            for (S32 i = 0; i < count; ++i)
            {
                const FSVRAdapterDomain domain = input.joints[i].domain;
                if ((side == FSVRAdapterSide::LEFT &&
                     domain == FSVRAdapterDomain::ARM_LEFT) ||
                    (side == FSVRAdapterSide::RIGHT &&
                     domain == FSVRAdapterDomain::ARM_RIGHT))
                {
                    shoulder_index = i;
                    break;
                }
            }
            if (shoulder_index < 0)
            {
                continue;
            }

            if (write_through_solver)
            {
                // The C7/RP3 boundary takes this side's arm phase;
                // the plain target-world solver is not invoked.
                // The invoked flag is set only when the solver
                // actually ran for this side: a fail-closed abort
                // (R2-01) returns before invocation and must not
                // report itself invoked.
                if (runWriteThroughSide(side))
                {
                    if (side == FSVRAdapterSide::LEFT)
                    {
                        result.arm_solver_invoked_left = true;
                    }
                    else
                    {
                        result.arm_solver_invoked_right = true;
                    }
                }
                continue;
            }

            std::map<S32, LLQuaternion> arm_targets;
            arm_solver.solveArm(context, side,
                                committed_rotations[shoulder_index],
                                committed_positions[shoulder_index],
                                arm_targets);
            if (side == FSVRAdapterSide::LEFT)
            {
                result.arm_solver_invoked_left = true;
            }
            else
            {
                result.arm_solver_invoked_right = true;
            }

            for (S32 i = 0; i < count; ++i)
            {
                const FSVRAdapterDomain domain = input.joints[i].domain;
                const bool in_side =
                    (side == FSVRAdapterSide::LEFT &&
                     domain == FSVRAdapterDomain::ARM_LEFT) ||
                    (side == FSVRAdapterSide::RIGHT &&
                     domain == FSVRAdapterDomain::ARM_RIGHT);
                if (in_side)
                {
                    commitJoint(i, arm_targets, true);
                }
            }
        }
    }

    result.resolved_locals = committed_locals;
    result.resolved_world_rotations = committed_rotations;
    result.resolved_world_positions = committed_positions;
    return result;
}

// static
const char* FSVRAdapterApply::gateName(FSVRAdapterGate gate)
{
    switch (gate)
    {
    case FSVRAdapterGate::NONE:
        return "none";
    case FSVRAdapterGate::CYCLE:
        return "cycle";
    case FSVRAdapterGate::CONTEXT:
        return "context";
    case FSVRAdapterGate::FRAME_IDENTITY:
        return "frame_identity";
    case FSVRAdapterGate::VR_SESSION:
        return "vr_session";
    case FSVRAdapterGate::POSITIONAL_MASTER:
        return "positional_master";
    case FSVRAdapterGate::NO_SOLVER:
        return "no_solver";
    case FSVRAdapterGate::TOKEN:
        return "token";
    }
    return "unknown";
}

// static
const char* FSVRAdapterApply::armAbortReasonName(
    FSVRAdapterArmAbortReason reason)
{
    switch (reason)
    {
    case FSVRAdapterArmAbortReason::NONE:
        return "none";
    case FSVRAdapterArmAbortReason::SOLVER_INCOMPLETE:
        return "solver_incomplete";
    case FSVRAdapterArmAbortReason::MISSING_POST_CLAMP:
        return "missing_post_clamp";
    }
    return "unknown";
}

FSVRAdapterPlanResult FSVRAdapterApplyOnce::apply(
    const FSVRAdapterPlanInput& input,
    FSVRAdapterSpineSolver& spine_solver,
    FSVRAdapterArmSolver& arm_solver)
{
    return apply(input, spine_solver, arm_solver, nullptr, nullptr);
}

FSVRAdapterPlanResult FSVRAdapterApplyOnce::apply(
    const FSVRAdapterPlanInput& input,
    FSVRAdapterSpineSolver& spine_solver,
    FSVRAdapterArmSolver& arm_solver,
    FSVRAdapterWriteThroughArmSolver* write_through_solver,
    FSVRAdapterArmPostClamp* post_clamp)
{
    if (mHasToken && input.frame_token == mLastToken)
    {
        FSVRAdapterPlanResult result;
        result.first_failing_gate = FSVRAdapterGate::TOKEN;
        return result;
    }
    mHasToken = true;
    mLastToken = input.frame_token;

    FSVRAdapterPlanResult result = FSVRAdapterApply::computePlan(
        input, spine_solver, arm_solver, write_through_solver, post_clamp);
    if (result.applied)
    {
        ++mApplyCount;
    }
    return result;
}

#ifndef FSVR_ADAPTER_PLAN_ONLY

// ---------------------------------------------------------------------------
// Viewer glue. Everything below assembles the plan inputs from the
// live viewer and performs the committed writes. It is excluded
// from the standalone H1 harness (FSVR_ADAPTER_PLAN_ONLY), exactly
// as the Stage-1 harness excludes the frame coordinator module.
// ---------------------------------------------------------------------------

#include "llcharacter.h"
#include "lljoint.h"
#include "llvrboneregistry.h"
#include "llvrtrackingframe.h"
#include "llvrtrackingsettings.h"

#include <algorithm>
#include <map>

namespace
{

// C3 owns the constraint_adapter_id vocabulary (spec section
// 1/C3). P2-07B C3 (CP-6) landed it: "spine_chain" (the spine
// chain), "arm_left" / "arm_right" (shoulder/elbow/wrist per
// side -- the mapper receives only the id string, so the side
// lives in the string, DP-2). Empty and unrecognized ids still
// map to NONE: the stub's conservative contract, preserved
// verbatim (no domain, no writes). This fulfils the code
// cross-check round 1 recorded clarification: registering bones
// in the registry alone changed nothing here until this mapper
// gained the real entries.
FSVRAdapterDomain domainForAdapterId(const std::string& adapter_id)
{
    if (adapter_id == "spine_chain")
    {
        return FSVRAdapterDomain::SPINE;
    }
    if (adapter_id == "arm_left")
    {
        return FSVRAdapterDomain::ARM_LEFT;
    }
    if (adapter_id == "arm_right")
    {
        return FSVRAdapterDomain::ARM_RIGHT;
    }
    return FSVRAdapterDomain::NONE;
}

} // namespace

void FSVRAdapterApplyForCharacter(LLCharacter* character, bool preview_cycle)
{
    if (!character)
    {
        return;
    }

    const FSVRTrackingFrameDecisions* decisions =
        FSVRTrackingCoordinator::currentDecisions();
    if (!decisions)
    {
        // Nothing published for this frame: nothing to process.
        return;
    }
    const FSVRTrackingSettingsSnapshot& snapshot =
        FSVRTrackingSettings::captureForFrame();

    FSVRAdapterPlanInput input;
    input.frame_token = decisions->app_frame_token;
    // The hook is post-blend by construction (brief section 3.2);
    // the cycle classification comes from production state: the
    // Animation Preview flag the override read from the avatar,
    // and the motion controller's paused flag (a paused controller
    // reaches the hook only through FORCE_UPDATE).
    input.blend_ran = true;
    input.cycle = FSVRAdapterApply::classifyCycle(
        preview_cycle, character->getMotionController().isPaused());
    // The bridge exists only on LLVOAvatarSelf (DP-1), so reaching
    // this function is the armed context.
    input.context_armed = true;

    // Gate 1 inputs. Decisions are current when the snapshot they
    // were finalized against is still the captured snapshot
    // (brief section 2.3: arbitration is finalized earlier in the
    // same frame than the motion update).
    input.decisions_published = true;
    input.decisions_current =
        (snapshot.generation == decisions->settings_generation);
    // Runtime-effective means the C4 decision form; Stage-1
    // decisions are shadow-only, so this is false until C4 lands.
    input.decisions_runtime_effective = !decisions->bones.empty();
    for (const FSVRBoneDecision& bone : decisions->bones)
    {
        if (bone.shadow_only)
        {
            input.decisions_runtime_effective = false;
            break;
        }
    }

    // Gate 2 and the Stage-2 snapshot fields (gates 3/4) are C3/C4
    // inputs: vr_active is carried by FSVRTrackingFrameInput,
    // which the coordinator does not publish, and the snapshot has
    // no FSVRTrackHeadPosition / FSVRTrackHandPosition /
    // FSVRDiag* fields yet (brief OQ-6). The glue passes the
    // conservative values; gate 1 already renders the glue inert
    // at baseline, and no gate is bypassed -- each will read its
    // real source when C3/C4 land.
    input.vr_active = false;
    input.positional_solve_enabled = false;
    input.head_position_enabled = false;
    input.hand_position_enabled = false;
    input.spine_sample_valid = false;
    input.arm_sample_valid = false;
    input.bend_disabled_bones = false;

    // Build the plan joint list from the registry (OQ-7: C1
    // iterates registry/decisions; it does not hard-code a bone
    // list). Unregistered ancestors are pulled in as structural
    // joints so the tree is complete under the pelvis root frame;
    // composition always goes through real LLJoint parents, never
    // registry parent strings.
    LLJoint* root_joint = character->getRootJoint();
    std::vector<LLJoint*> ordered;
    std::map<LLJoint*, const FSVRTrackingBoneEntry*> entry_by_joint;
    for (const FSVRTrackingBoneEntry& entry : FSVRBoneRegistry::entries())
    {
        if (entry.expected_joint_name.empty())
        {
            continue;
        }
        LLJoint* joint = character->getJoint(entry.expected_joint_name);
        if (!joint || joint == root_joint)
        {
            continue;
        }
        entry_by_joint[joint] = &entry;
        // Collect the chain from this joint up to (excluding) the
        // root joint, then append any missing joints top-down so
        // parents always precede children.
        std::vector<LLJoint*> chain;
        for (LLJoint* cursor = joint; cursor && cursor != root_joint;
             cursor = cursor->getParent())
        {
            chain.push_back(cursor);
        }
        for (auto it = chain.rbegin(); it != chain.rend(); ++it)
        {
            if (std::find(ordered.begin(), ordered.end(), *it) == ordered.end())
            {
                ordered.push_back(*it);
            }
        }
    }

    std::map<LLJoint*, S32> index_by_joint;
    for (S32 i = 0; i < static_cast<S32>(ordered.size()); ++i)
    {
        index_by_joint[ordered[i]] = i;
    }

    std::map<std::string, const FSVRBoneDecision*> decision_by_bone;
    for (const FSVRBoneDecision& bone : decisions->bones)
    {
        decision_by_bone[bone.bone_id] = &bone;
    }

    input.joints.reserve(ordered.size());
    for (LLJoint* joint : ordered)
    {
        FSVRAdapterJointInput joint_input;
        joint_input.name = joint->getName();
        LLJoint* parent = joint->getParent();
        auto parent_it = (parent && parent != root_joint)
            ? index_by_joint.find(parent)
            : index_by_joint.end();
        joint_input.parent_index =
            (parent_it != index_by_joint.end()) ? parent_it->second : -1;
        joint_input.rest_offset = joint->getPosition();
        joint_input.base_local = joint->getRotation();

        auto entry_it = entry_by_joint.find(joint);
        if (entry_it != entry_by_joint.end())
        {
            const FSVRTrackingBoneEntry& entry = *entry_it->second;
            joint_input.in_registry = true;
            joint_input.domain = domainForAdapterId(entry.constraint_adapter_id);
            joint_input.disabled =
                snapshot.disabled_bones.isBoneDisabled(entry.canonical_id);
            auto decision_it = decision_by_bone.find(entry.canonical_id);
            if (decision_it != decision_by_bone.end())
            {
                const FSVRBoneDecision& decision = *decision_it->second;
                joint_input.decision_grants_base =
                    decision.desired_decision == FSVRDecisionKind::TRACKED &&
                    decision.desired_source == "HMD";
            }
        }
        input.joints.push_back(joint_input);
    }

    // The transaction root frame is the pelvis world. These are
    // the joint's cached world values at apply time; solver target
    // worlds are expressed in this same frame, which is the frame
    // the coordinator/solver side works in. (Freshness of the root
    // frame itself is a C6/C7 integration property; Package A
    // produces no writes regardless. The pre-runtime-enable
    // verification obligation for this frame is recorded at
    // FSVRAdapterPlanInput::root_world_rotation in the header.)
    if (root_joint)
    {
        input.root_world_rotation = root_joint->getWorldRotation();
        input.root_world_position = root_joint->getWorldPosition();
    }

    // One apply-once instance and the null solvers for the process
    // (the self avatar is unique). Real solvers arrive with C6/C7.
    static FSVRAdapterApplyOnce s_apply_once;
    static FSVRAdapterNullSpineSolver s_spine_solver;
    static FSVRAdapterNullArmSolver s_arm_solver;

    const FSVRAdapterPlanResult result =
        s_apply_once.apply(input, s_spine_solver, s_arm_solver);

    // The committed writes: C1 is the sole owner of the final
    // committed output. Locals only, via setRotation -- never
    // setWorldRotation (brief section 2.6).
    for (const FSVRAdapterWrite& write : result.writes)
    {
        if (write.joint_index >= 0 &&
            write.joint_index < static_cast<S32>(ordered.size()))
        {
            ordered[write.joint_index]->setRotation(write.local);
        }
    }
}

#endif // FSVR_ADAPTER_PLAN_ONLY
