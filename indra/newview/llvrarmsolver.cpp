/**
 * @file llvrarmsolver.cpp
 * @brief P2-07B C7 implementation: see llvrarmsolver.h.
 *
 * RP3 procedure (brief DP-2): a scratch LLJoint chain
 * (collar_s <- shoulder_s <- elbow_s <- wrist_s, plus a
 * parentless goal_s) is seeded from the transaction's current
 * state read through the host; RP3 runs against it in up to
 * four passes, re-capturing base rotations from the scratch
 * joints' current locals at each pass (setupJoints), so each
 * pass starts from the composed state, not the cycle base. The
 * goal's world position is established explicitly from the
 * wrist target before the first pass -- a parentless joint has
 * no host state to inherit it from. LLJointSolverRP3 itself is
 * unmodified (CP-9); this file adapts around it.
 *
 * C7 does not clamp its RP3 output: the transaction applies
 * CP-6 to the readback (brief CP-5 step 5).
 */

#include "llviewerprecompiledheaders.h"

#include "llvrarmsolver.h"

#include "lljoint.h"
#include "lljointsolverrp3.h"
#include "llvrjointlimits.h"

#include <cmath>

namespace
{

constexpr F32 kDegToRad = F_PI / 180.f;
constexpr F32 kRadToDeg = 180.f / F_PI;
// DP-9 budgets (tuning constants, see llvrspinesolver.cpp).
constexpr S32 kMaxRp3Passes = 4;
constexpr F32 kRp3ConvergenceM = 0.0005f; // 0.5 mm at the wrist
// DP-3 pole constants (implementation choices, not spec
// values): the rig-default pole direction in the root frame --
// backward (-X) and outward (side-signed Y) at 45 degrees --
// and the per-cycle maximum applied-pole step.
constexpr F32 kPoleStepMaxDeg = 15.f;

S32 findJoint(const FSVRAdapterPlanInput& plan, const std::string& name)
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

bool sideIsLeft(FSVRAdapterSide side)
{
    return side == FSVRAdapterSide::LEFT;
}

S32 sideIndex(FSVRAdapterSide side)
{
    return sideIsLeft(side) ? 0 : 1;
}

std::string sideSuffix(FSVRAdapterSide side)
{
    return sideIsLeft(side) ? "Left" : "Right";
}

// Gate-5 eligibility with the transaction's own predicate and
// authority values (see llvrspinesolver.cpp).
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

// Host reads, unwrapped: the host answers every in-range joint
// (it composes from the provisional state), and every index
// used here was resolved by name from the same plan.
LLVector3 hostWorldPosition(const FSVRAdapterWriteThroughHost& host,
                            S32 joint_index)
{
    LLVector3 pos = LLVector3::zero;
    host.readWorldPosition(joint_index, pos);
    return pos;
}

LLQuaternion hostWorldRotation(const FSVRAdapterWriteThroughHost& host,
                               S32 joint_index)
{
    LLQuaternion rot;
    host.readWorldRotation(joint_index, rot);
    return rot;
}

LLQuaternion hostProvisionalLocal(const FSVRAdapterWriteThroughHost& host,
                                  S32 joint_index)
{
    LLQuaternion local;
    host.readProvisionalLocal(joint_index, local);
    return local;
}

// DP-3: the rig-default pole direction, expressed in the root
// frame: backward and outward at 45 degrees, mirror-correct.
LLVector3 rigDefaultPoleLocal(FSVRAdapterSide side)
{
    LLVector3 dir(-1.f, sideIsLeft(side) ? 1.f : -1.f, 0.f);
    dir.normalize();
    return dir;
}

// Project a candidate direction perpendicular to the
// shoulder->wrist axis and normalize. Returns false when the
// candidate is (near-)parallel to the axis.
bool projectPerpendicular(const LLVector3& candidate,
                          const LLVector3& axis_hat,
                          LLVector3& out_dir)
{
    LLVector3 projected = candidate - axis_hat * (candidate * axis_hat);
    if (projected.length() < 1e-5f)
    {
        return false;
    }
    projected.normalize();
    out_dir = projected;
    return true;
}

// The pole fallback chain (brief CP-5 step 6), shared by the
// RP3 procedure and (as a bend reference only) elbow-only
// flexion: the animated elbow direction projected against the
// current shoulder->wrist axis; the caller-supplied previous
// pole, re-projected; the rig default re-expressed from the
// root frame. All candidates are expressed against the CURRENT
// axis (the shoulder's committed position toward the target).
LLVector3 selectPoleDirection(const FSVRAdapterSolverContext& context,
                              FSVRAdapterSide side,
                              S32 shoulder_index, S32 elbow_index,
                              const LLVector3& axis_hat)
{
    LLVector3 dir;
    const LLVector3 animated =
        context.base_world_positions[elbow_index] -
        context.base_world_positions[shoulder_index];
    if (projectPerpendicular(animated, axis_hat, dir))
    {
        return dir;
    }
    const S32 sidx = sideIndex(side);
    if (context.previous_pole_valid[sidx] &&
        projectPerpendicular(context.previous_pole_world[sidx],
                             axis_hat, dir))
    {
        return dir;
    }
    const LLVector3 rig_default = rigDefaultPoleLocal(side) *
        context.plan->root_world_rotation;
    if (projectPerpendicular(rig_default, axis_hat, dir))
    {
        return dir;
    }
    // The axis is aligned with every candidate (the axis IS the
    // rig default's plane): any perpendicular direction serves;
    // the pole only orients the bend plane of a degenerate arm.
    LLVector3 any = std::fabs(axis_hat.mV[VZ]) < 0.9f
        ? LLVector3(0.f, 0.f, 1.f)
        : LLVector3(1.f, 0.f, 0.f);
    projectPerpendicular(any, axis_hat, dir);
    return dir;
}

// Blend the applied pole toward the newly computed pole with
// the per-cycle maximum angular step (DP-3): continuity across
// cycles, no discontinuous flips.
LLVector3 stepLimitPole(const LLVector3& previous_dir,
                        const LLVector3& candidate_dir)
{
    const F32 dot = previous_dir * candidate_dir;
    if (dot > 0.99999f)
    {
        return candidate_dir;
    }
    LLVector3 axis = previous_dir % candidate_dir;
    if (axis.length() < 1e-6f)
    {
        // Opposite directions: rotate about any perpendicular
        // axis by the step bound.
        axis = std::fabs(previous_dir.mV[VZ]) < 0.9f
            ? previous_dir % LLVector3(0.f, 0.f, 1.f)
            : previous_dir % LLVector3(1.f, 0.f, 0.f);
        if (axis.length() < 1e-6f)
        {
            return candidate_dir;
        }
    }
    axis.normalize();
    const LLQuaternion step(kPoleStepMaxDeg * kDegToRad, axis);
    LLVector3 stepped = previous_dir * step;
    stepped.normalize();
    return stepped;
}

LLQuaternion normalizedQuat(LLQuaternion q)
{
    q.normalize();
    return q;
}

LLVector3 unitVec(LLVector3 v)
{
    v.normalize();
    return v;
}

} // namespace

LLVector3 FSVRArmSolver::composeWristTargetProvisional(
    const LLVector3& solved_shoulder_world,
    const LLVector3& player_displacement_from_neutral, F32 arm_ratio,
    bool force_unit_scale)
{
    // OPEN-4-PROVISIONAL: see the header. The shoulder-anchored
    // composition is the spec section 6.2 equation as written;
    // it is not a ruling on OPEN-4.
    const F32 ratio = force_unit_scale ? 1.f : arm_ratio;
    return solved_shoulder_world +
           player_displacement_from_neutral * ratio;
}

FSVRAdapterWriteThroughOutcome FSVRArmSolver::solveArmWriteThrough(
    const FSVRAdapterSolverContext& context, FSVRAdapterSide side,
    FSVRAdapterWriteThroughHost& host)
{
    FSVRAdapterWriteThroughOutcome outcome;
    if (!context.plan)
    {
        outcome.completed = false;
        return outcome;
    }
    const FSVRAdapterPlanInput& plan = *context.plan;
    const std::string suffix = sideSuffix(side);
    const S32 collar_index = findJoint(plan, "mCollar" + suffix);
    const S32 shoulder_index = findJoint(plan, "mShoulder" + suffix);
    const S32 elbow_index = findJoint(plan, "mElbow" + suffix);
    const S32 wrist_index = findJoint(plan, "mWrist" + suffix);
    if (collar_index < 0 || shoulder_index < 0 || elbow_index < 0 ||
        wrist_index < 0)
    {
        outcome.completed = false;
        return outcome;
    }
    // Every path from here completes unless a host write is
    // rejected (a procedure failure): the no-target return, the
    // aim/flex outcomes and the neither-eligible report are all
    // completions (brief CP-5 steps 1-2).
    outcome.completed = true;

    const S32 sidx = sideIndex(side);
    if (!plan.wrist_target_valid[sidx])
    {
        // No constraint from this solver (brief CP-5 step 1):
        // completed, nothing written, no target reported.
        outcome.has_wrist_target = false;
        return outcome;
    }
    const LLVector3 target = plan.wrist_target_world[sidx];
    outcome.has_wrist_target = true;
    outcome.wrist_target_world = target;

    const bool shoulder_ok =
        solverEligible(plan, plan.joints[shoulder_index]);
    const bool elbow_ok = solverEligible(plan, plan.joints[elbow_index]);

    if (!shoulder_ok && !elbow_ok)
    {
        // No writes; the target is still reported (brief CP-5
        // step 2, neither-eligible outcome).
        return outcome;
    }

    if (shoulder_ok && !elbow_ok)
    {
        // ------------------------------------------------------
        // Shoulder-only aiming [R1 -- C6C7-01]: rotate the
        // shoulder so the base-elbow chain's A->C direction aims
        // at the target. The elbow local is never written, so
        // the wrist lands on the sphere about A whose radius is
        // the PRESERVED configuration's radius (the base pose's
        // A->C distance), in the target's direction; the
        // distance is unmet by construction. An OFF elbow is
        // never straightened.
        // ------------------------------------------------------
        const LLVector3 a_pos = hostWorldPosition(host, shoulder_index);
        const LLVector3 c_pos = hostWorldPosition(host, wrist_index);
        const LLVector3 current_dir = c_pos - a_pos;
        const LLVector3 target_dir = target - a_pos;
        if (current_dir.length() < 1e-6f || target_dir.length() < 1e-6f)
        {
            return outcome;
        }
        LLQuaternion aim;
        aim.shortestArc(unitVec(current_dir), unitVec(target_dir));
        const LLQuaternion a_world =
            normalizedQuat(hostWorldRotation(host, shoulder_index));
        const LLQuaternion collar_world =
            normalizedQuat(hostWorldRotation(host, collar_index));
        const LLQuaternion new_world = normalizedQuat(a_world * aim);
        const LLQuaternion new_local =
            normalizedQuat(new_world * ~collar_world);
        if (!host.writeProvisionalLocal(shoulder_index, new_local))
        {
            outcome.completed = false;
        }
        return outcome;
    }

    if (!shoulder_ok && elbow_ok)
    {
        // ------------------------------------------------------
        // Elbow-only flexion [R1 -- C6C7-02]: the 1-DOF segment
        // angle from the law of cosines against |target - A|,
        // clamped to the unconditional geometric range
        // [0, 180] degrees (through the distance clamp) and --
        // only while diag_bend_limits holds -- to the section
        // 6.3 anatomical cap of 145 degrees. The rotation is
        // about the current bend axis, pole-referenced when the
        // base arm is straight.
        // ------------------------------------------------------
        const LLVector3 a_pos = hostWorldPosition(host, shoulder_index);
        const LLVector3 b_pos = hostWorldPosition(host, elbow_index);
        const LLVector3 c_pos = hostWorldPosition(host, wrist_index);
        const F32 len_ab = plan.joints[elbow_index].rest_offset.length();
        const F32 len_bc = plan.joints[wrist_index].rest_offset.length();
        if (len_ab < 1e-9f || len_bc < 1e-9f)
        {
            outcome.completed = false;
            return outcome;
        }
        F32 distance = (target - a_pos).length();
        const F32 min_distance = std::fabs(len_ab - len_bc);
        const F32 max_distance = len_ab + len_bc;
        distance = distance < min_distance ? min_distance
                     : (distance > max_distance ? max_distance
                                                : distance);
        F32 cos_beta = (len_ab * len_ab + len_bc * len_bc -
                        distance * distance) /
                       (2.f * len_ab * len_bc);
        cos_beta = cos_beta < -1.f ? -1.f : (cos_beta > 1.f ? 1.f : cos_beta);
        F32 delta_target_deg = 180.f - std::acos(cos_beta) * kRadToDeg;
        if (plan.diag_bend_limits &&
            delta_target_deg > FSVR_ELBOW_MAX_SEGMENT_DEG)
        {
            delta_target_deg = FSVR_ELBOW_MAX_SEGMENT_DEG;
        }
        const F32 delta_current_deg =
            FSVRJointLimits::elbowSegmentAngleDeg(a_pos, b_pos, c_pos);
        const F32 delta_deg = delta_target_deg - delta_current_deg;
        if (std::fabs(delta_deg) < 1e-4f)
        {
            return outcome;
        }
        // Bend axis: the current bend plane's normal when the
        // arm is bent; the target plane's normal when it is
        // straight; the pole-referenced axis when even that is
        // degenerate (target on the arm line).
        LLVector3 axis = (b_pos - a_pos) % (c_pos - b_pos);
        if (axis.length() < 1e-5f)
        {
            axis = (b_pos - a_pos) % (target - a_pos);
        }
        if (axis.length() < 1e-5f)
        {
            LLVector3 axis_hat = c_pos - a_pos;
            if (axis_hat.length() > 1e-6f)
            {
                axis_hat.normalize();
                const LLVector3 pole = selectPoleDirection(
                    context, side, shoulder_index, elbow_index, axis_hat);
                axis = axis_hat % pole;
            }
        }
        if (axis.length() < 1e-5f)
        {
            axis = LLVector3(0.f, 0.f, 1.f);
        }
        axis.normalize();
        // Choose the rotation sign whose result lands the wrist
        // closer to the target: the unsigned segment delta does
        // not by itself say which way around the axis flexes
        // toward the target.
        const LLVector3 bc_vec = c_pos - b_pos;
        F32 best_error = -1.f;
        F32 best_angle = 0.f;
        for (S32 sign = -1; sign <= 1; sign += 2)
        {
            const LLQuaternion candidate_rot(delta_deg * kDegToRad *
                                                 static_cast<F32>(sign),
                                             axis);
            const LLVector3 candidate_c =
                b_pos + bc_vec * candidate_rot;
            const F32 error = (candidate_c - target).length();
            if (best_error < 0.f || error < best_error)
            {
                best_error = error;
                best_angle = delta_deg * kDegToRad *
                             static_cast<F32>(sign);
            }
        }
        const LLQuaternion bend_rot(best_angle, axis);
        const LLQuaternion b_world =
            normalizedQuat(hostWorldRotation(host, elbow_index));
        const LLQuaternion a_world =
            normalizedQuat(hostWorldRotation(host, shoulder_index));
        const LLQuaternion new_world = normalizedQuat(b_world * bend_rot);
        const LLQuaternion new_local =
            normalizedQuat(new_world * ~a_world);
        if (!host.writeProvisionalLocal(elbow_index, new_local))
        {
            outcome.completed = false;
        }
        return outcome;
    }

    // ----------------------------------------------------------
    // Both eligible: the RP3 procedure (brief CP-5 steps 3-6).
    // ----------------------------------------------------------
    const LLVector3 a_pos = hostWorldPosition(host, shoulder_index);
    const LLVector3 collar_pos = hostWorldPosition(host, collar_index);
    const LLQuaternion collar_world =
        normalizedQuat(hostWorldRotation(host, collar_index));

    // Pole (step 6): fallback chain against the current
    // shoulder->target axis; continuity step against the
    // caller-supplied previous pole when one is carried.
    LLVector3 axis_hat = target - a_pos;
    const bool axis_ok = axis_hat.length() > 1e-6f;
    if (axis_ok)
    {
        axis_hat.normalize();
    }
    else
    {
        axis_hat = LLVector3(0.f, sideIsLeft(side) ? 1.f : -1.f, 0.f);
    }
    LLVector3 pole_dir = selectPoleDirection(context, side, shoulder_index,
                                             elbow_index, axis_hat);
    if (context.previous_pole_valid[sidx])
    {
        LLVector3 previous = context.previous_pole_world[sidx];
        if (previous.length() > 1e-6f)
        {
            previous.normalize();
            pole_dir = stepLimitPole(previous, pole_dir);
        }
    }
    outcome.has_pole = true;
    outcome.pole_world = pole_dir;
    // RP3 takes the pole in the frame of A's parent (the
    // collar): pole_collar = pole_world composed with the
    // collar world inverse.
    const LLVector3 pole_collar = pole_dir * ~collar_world;

    // Scratch chain (step 3). The collar scratch joint is
    // parentless: its local PRS IS its world PRS, seeded from
    // the host's committed collar state, so the scratch chain
    // composes in the plan's world frame.
    LLJoint collar_s("mCollar" + suffix, nullptr);
    LLJoint shoulder_s("mShoulder" + suffix, &collar_s);
    LLJoint elbow_s("mElbow" + suffix, &shoulder_s);
    LLJoint wrist_s("mWrist" + suffix, &elbow_s);
    LLJoint goal_s("fsVrGoal", nullptr);
    collar_s.setPosition(collar_pos);
    collar_s.setRotation(collar_world);
    shoulder_s.setPosition(plan.joints[shoulder_index].rest_offset);
    elbow_s.setPosition(plan.joints[elbow_index].rest_offset);
    wrist_s.setPosition(plan.joints[wrist_index].rest_offset);
    wrist_s.setRotation(hostProvisionalLocal(host, wrist_index));
    // Step 3's explicit goal seeding: the parentless goal's
    // world position is established from the wrist target
    // before the first pass.
    goal_s.setPosition(target);
    goal_s.setRotation(LLQuaternion());

    LLJointSolverRP3 solver;
    for (S32 pass = 0; pass < kMaxRp3Passes; ++pass)
    {
        if (pass == 0)
        {
            shoulder_s.setRotation(
                hostProvisionalLocal(host, shoulder_index));
            elbow_s.setRotation(hostProvisionalLocal(host, elbow_index));
        }
        // Refresh the scratch world matrices from the chain
        // root: RP3 reads world matrices (its pole transform
        // and setWorldRotation) that nothing inside solve()
        // refreshes, so each pass starts from a consistent
        // composed state (brief section 2.3).
        (void)wrist_s.getWorldPosition();
        collar_s.updateWorldMatrix();
        shoulder_s.updateWorldMatrix();
        elbow_s.updateWorldMatrix();
        wrist_s.updateWorldMatrix();
        goal_s.updateWorldMatrix();
        // Re-capture base rotations from the scratch joints'
        // CURRENT locals and lengths from their positions, so
        // the pass starts from the composed state, not the
        // cycle base.
        solver.setupJoints(&shoulder_s, &elbow_s, &wrist_s, &goal_s);
        solver.setPoleVector(pole_collar);
        solver.setTwist(0.f);
        solver.solve();
        const LLVector3 wrist_world = wrist_s.getWorldPosition();
        if ((wrist_world - target).length() < kRp3ConvergenceM)
        {
            break;
        }
    }

    if (!host.writeProvisionalLocal(shoulder_index,
                                    shoulder_s.getRotation()) ||
        !host.writeProvisionalLocal(elbow_index, elbow_s.getRotation()))
    {
        outcome.completed = false;
        return outcome;
    }
    // completed stays true; the outcome reports the target
    // (already set) and the pole selected this call.
    return outcome;
}

void FSVRArmPostClamp::clampArm(
    const FSVRAdapterSolverContext& context, FSVRAdapterSide side,
    const std::map<S32, LLQuaternion>& readback_world_rotations,
    const std::map<S32, LLVector3>& readback_world_positions,
    std::map<S32, LLQuaternion>& in_out_readback_locals)
{
    if (!context.plan || !context.plan->diag_bend_limits)
    {
        // The diagnostic suspends the clamp; the clamp existing
        // but inert is what keeps the side from failing closed
        // (brief CP-6).
        return;
    }
    const FSVRAdapterPlanInput& plan = *context.plan;
    const std::string suffix = sideSuffix(side);
    const bool left = sideIsLeft(side);
    const S32 collar_index = findJoint(plan, "mCollar" + suffix);
    const S32 shoulder_index = findJoint(plan, "mShoulder" + suffix);
    const S32 elbow_index = findJoint(plan, "mElbow" + suffix);
    const S32 wrist_index = findJoint(plan, "mWrist" + suffix);
    if (shoulder_index < 0 || elbow_index < 0 || wrist_index < 0)
    {
        return;
    }

    // Shoulder: build q_rel from the readback worlds. The
    // chest is an ancestor of the chain; its committed world is
    // recovered exactly from the readback shoulder world
    // through the two locals between them -- the shoulder's
    // readback local and the collar's local (the collar is
    // never written, so its committed local is its base local)
    // -- using the transaction's own composition identity
    // (world = local * parent world, hence parent world =
    // ~local * world). C0 section 5.2's rule then applies in
    // the chest frame; the raw shoulder local is never clamped
    // against the shoulder numbers.
    LLQuaternion shoulder_world;
    bool have_shoulder_world = false;
    auto rot_it = readback_world_rotations.find(shoulder_index);
    auto loc_it = in_out_readback_locals.find(shoulder_index);
    if (rot_it != readback_world_rotations.end() &&
        loc_it != in_out_readback_locals.end() && collar_index >= 0)
    {
        const LLQuaternion shoulder_local = normalizedQuat(loc_it->second);
        const LLQuaternion collar_local =
            normalizedQuat(plan.joints[collar_index].base_local);
        const LLQuaternion collar_world = normalizedQuat(
            ~shoulder_local * normalizedQuat(rot_it->second));
        const LLQuaternion chest_world =
            normalizedQuat(~collar_local * collar_world);
        const LLQuaternion q_rel = normalizedQuat(
            normalizedQuat(rot_it->second) * ~chest_world);
        bool clamped = false;
        const LLQuaternion q_rel_clamped =
            FSVRJointLimits::clampShoulderRelative(q_rel, left, &clamped);
        if (clamped)
        {
            const LLQuaternion new_world =
                normalizedQuat(q_rel_clamped * chest_world);
            const LLQuaternion new_local =
                normalizedQuat(new_world * ~collar_world);
            in_out_readback_locals[shoulder_index] = new_local;
            shoulder_world = new_world;
            have_shoulder_world = true;
        }
        else
        {
            shoulder_world = rot_it->second;
            have_shoulder_world = true;
        }
    }
    if (!have_shoulder_world)
    {
        // Without the shoulder world the elbow recomposition
        // below cannot be made consistent; the transaction's
        // own error recompute still reports honestly.
        return;
    }

    // Elbow: the segment angle from the readback positions,
    // recomposed consistently after the shoulder edit (the
    // positions map predates the edit, so B and C are recomposed
    // from the edited shoulder world and the readback locals).
    auto pos_it = readback_world_positions.find(shoulder_index);
    if (pos_it == readback_world_positions.end())
    {
        return;
    }
    const LLVector3 a_pos = pos_it->second;
    auto elbow_loc_it = in_out_readback_locals.find(elbow_index);
    const LLQuaternion elbow_local =
        elbow_loc_it != in_out_readback_locals.end()
            ? normalizedQuat(elbow_loc_it->second)
            : normalizedQuat(plan.joints[elbow_index].base_local);
    const LLQuaternion elbow_world =
        normalizedQuat(elbow_local * shoulder_world);
    const LLVector3 b_pos =
        plan.joints[elbow_index].rest_offset * shoulder_world + a_pos;
    const LLVector3 c_pos =
        plan.joints[wrist_index].rest_offset * elbow_world + b_pos;
    const F32 delta_deg =
        FSVRJointLimits::elbowSegmentAngleDeg(a_pos, b_pos, c_pos);

    // Hyperextension side: the bend normal against the
    // reference normal of the pole the solver actually
    // selected and applied this cycle [R2 -- C6C7-CODE-04].
    // The CP-5 selection (animated -> transported history ->
    // rig default, then the DP-3 continuity step against the
    // carried pole) is a deterministic function of the context
    // and the wrist target, so it is reconstructed here from
    // exactly the inputs solveArmWriteThrough used: the axis
    // is the readback shoulder position toward the target (the
    // shoulder's position is ancestor-determined, so the
    // readback value is the value the solver read), the
    // animated candidate and history come from the context,
    // and the same selectPoleDirection / stepLimitPole helpers
    // run. (Previously the reference was the rig-default pole
    // unconditionally, so a legitimate bend toward a valid
    // non-default pole could be misjudged as hyperextension
    // against the default plane and wrongly straightened.)
    // When the side carried no wrist target the solver
    // selected no pole; the rig default remains the reference.
    const LLVector3 bend_normal = (b_pos - a_pos) % (c_pos - b_pos);
    LLVector3 pole_world =
        rigDefaultPoleLocal(side) * plan.root_world_rotation;
    const S32 sidx = sideIndex(side);
    if (plan.wrist_target_valid[sidx])
    {
        LLVector3 axis_hat = plan.wrist_target_world[sidx] - a_pos;
        if (axis_hat.length() > 1e-6f)
        {
            axis_hat.normalize();
        }
        else
        {
            axis_hat = LLVector3(0.f, left ? 1.f : -1.f, 0.f);
        }
        LLVector3 selected = selectPoleDirection(
            context, side, shoulder_index, elbow_index, axis_hat);
        if (context.previous_pole_valid[sidx])
        {
            LLVector3 previous = context.previous_pole_world[sidx];
            if (previous.length() > 1e-6f)
            {
                previous.normalize();
                selected = stepLimitPole(previous, selected);
            }
        }
        pole_world = selected;
    }
    // Reference normal: pole x (C - A). (Verified against the
    // harness's elbow-only fixtures: with the opposite cross
    // order an anatomical flexion reads as hyperextension and
    // the clamp straightens a correctly bent elbow.)
    LLVector3 ref_normal = pole_world % (c_pos - a_pos);
    F32 side_sign = 1.f;
    if (bend_normal.length() > 1e-5f && ref_normal.length() > 1e-5f)
    {
        ref_normal.normalize();
        LLVector3 n = bend_normal;
        n.normalize();
        side_sign = (n * ref_normal) < 0.f ? -1.f : 1.f;
    }
    const F32 delta_signed = side_sign * delta_deg;
    if (delta_signed <= FSVR_ELBOW_MAX_SEGMENT_DEG &&
        delta_signed >= 0.f)
    {
        return;
    }
    const F32 delta_target =
        delta_signed < 0.f ? 0.f : FSVR_ELBOW_MAX_SEGMENT_DEG;
    const F32 adjust_deg = delta_target - delta_signed;

    LLVector3 axis;
    if (bend_normal.length() > 1e-5f)
    {
        axis = bend_normal;
        axis.normalize();
    }
    else if (ref_normal.length() > 1e-5f)
    {
        axis = ref_normal;
        axis.normalize();
    }
    else
    {
        return;
    }
    // Reduce B's rotation about its own bend axis until delta
    // is at the boundary (C0 section 5.3): try both rotation
    // senses and keep the one whose recomposed segment angle
    // lands on the target.
    F32 best_score = -1.f;
    LLQuaternion best_local = elbow_local;
    for (S32 sign = -1; sign <= 1; sign += 2)
    {
        const LLQuaternion rot(adjust_deg * kDegToRad *
                                   static_cast<F32>(sign),
                               axis);
        const LLQuaternion candidate_world =
            normalizedQuat(elbow_world * rot);
        const LLVector3 candidate_c =
            plan.joints[wrist_index].rest_offset * candidate_world +
            b_pos;
        const F32 candidate_delta =
            FSVRJointLimits::elbowSegmentAngleDeg(a_pos, b_pos,
                                                  candidate_c);
        const LLVector3 candidate_normal =
            (b_pos - a_pos) % (candidate_c - b_pos);
        F32 candidate_sign = 1.f;
        if (candidate_normal.length() > 1e-5f &&
            ref_normal.length() > 1e-5f)
        {
            LLVector3 n = candidate_normal;
            n.normalize();
            candidate_sign = (n * ref_normal) < 0.f ? -1.f : 1.f;
        }
        const F32 score =
            std::fabs(candidate_sign * candidate_delta - delta_target);
        if (best_score < 0.f || score < best_score)
        {
            best_score = score;
            best_local =
                normalizedQuat(candidate_world * ~shoulder_world);
        }
    }
    in_out_readback_locals[elbow_index] = best_local;
}
