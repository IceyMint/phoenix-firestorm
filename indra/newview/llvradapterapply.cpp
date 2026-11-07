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

#include "llvrcalibration.h"
#include "llvrbonelog.h"
#include "llvrspinesolver.h"

#include <cmath>
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

// ------------------------------------------------------------------
// P2-07B Fix 1 (CP-F1-1): head-group reconciliation helpers. All
// operations are total-angle / quaternion operations; no Euler
// decomposition appears anywhere in the new code (brief
// section 7 R-6).
// ------------------------------------------------------------------

// The head-group local cap: HEAD_ROTATION_CONSTRAINT's value,
// F_PI_BY_TWO * 0.8f (72 deg), duplicated by value per the
// llagentcamera.cpp:149 precedent ("match LLHeadRotMotion";
// the constant itself is at llheadrotmotion.cpp:91). The
// native 72 deg is a COMBINED bound -- the head's total
// rotation relative to its basis frame, constrained once at
// llheadrotmotion.cpp:323 before the lag-share distribution
// writes the two locals -- and no per-joint local cap exists
// natively (CP-F1-0 Q4; brief INV-F1-03 [R2]). This constant
// is the adapter's per-local backstop value for the cascade
// (the neck stage's cap, and the head stage's constrain under
// the native operation and value); the combined bound itself
// is enforced by the selection in CP-F1-1 step 5.
constexpr F32 kHeadGroupLocalCapRad = F_PI_BY_TWO * 0.8f;

// F32 tolerance on the combined-bound pass test (CP-F1-1
// step 5): the candidate at or below the bound plus this
// tolerance is the cascade pair, selected unchanged.
constexpr F32 kHeadGroupBoundTolRad = 1e-5f;

// Shortest-arc, sign-invariant total angle of a rotation
// (q and -q measure identically): 2*atan2(|vec|, |w|) over
// the normalized quaternion. The atan2 form keeps full F32
// resolution at small angles, where the acos(|w|) form
// quantizes (CP-F1-1 step 4 [R2] (i)).
F32 quaternionAngleRad(const LLQuaternion& q_in)
{
    LLQuaternion q = q_in;
    q.normalize();
    const F32 vlen = std::sqrt(q.mQ[VX] * q.mQ[VX] +
                               q.mQ[VY] * q.mQ[VY] +
                               q.mQ[VZ] * q.mQ[VZ]);
    return 2.f * std::atan2(vlen, std::fabs(q.mQ[VW]));
}

// The rotation q_in rescaled to the given total angle about
// its own shortest-arc axis (scale_angle, CP-F1-1 step 5).
LLQuaternion scaleQuaternionAngle(const LLQuaternion& q_in,
                                  F32 target_angle_rad)
{
    LLQuaternion q = q_in;
    q.normalize();
    if (q.mQ[VW] < 0.f)
    {
        q = LLQuaternion(-q.mQ[VX], -q.mQ[VY], -q.mQ[VZ],
                         -q.mQ[VW]);
    }
    const F32 vlen = std::sqrt(q.mQ[VX] * q.mQ[VX] +
                               q.mQ[VY] * q.mQ[VY] +
                               q.mQ[VZ] * q.mQ[VZ]);
    if (vlen < 1e-9f)
    {
        // No axis to scale about: the rotation is the
        // identity at F32 resolution, and the only target
        // the selection forms for it is the identity's own.
        return LLQuaternion();
    }
    const LLVector3 axis(q.mQ[VX] / vlen, q.mQ[VY] / vlen,
                         q.mQ[VZ] / vlen);
    return LLQuaternion(target_angle_rad, axis);
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
const std::vector<std::string>&
FSVRAdapterApply::calibrationHoldJointNames()
{
    // Spec section 3.1's in-scope set, by skeleton joint
    // name (brief section 2.7, confirmed against
    // llvrboneregistry.cpp): the D7 spine chain, mNeck,
    // mHead, and per side collar/shoulder/elbow/wrist.
    // Fingers are not held and not read.
    static const std::vector<std::string> names = {
        "mSpine1",      "mSpine2",      "mTorso",
        "mSpine3",      "mSpine4",      "mChest",
        "mNeck",        "mHead",
        "mCollarLeft",  "mShoulderLeft", "mElbowLeft", "mWristLeft",
        "mCollarRight", "mShoulderRight", "mElbowRight", "mWristRight"};
    return names;
}

// static
bool FSVRAdapterApply::continuityCarries(
    bool have_previous, U64 frame_token, U64 previous_token,
    U64 settings_generation, U64 previous_settings_generation,
    U64 continuity_epoch, U64 previous_continuity_epoch,
    U64 install_generation, U64 previous_install_generation)
{
    // P2-07B C8 (CP-3): the glue's retirement predicate, with
    // the continuity-epoch conjunct added to the committed
    // token/generation terms. See the header for the contract;
    // the glue calls this function, so what H8 fixtures is
    // what production evaluates.
    // P2-07B Stage 1 (CP-S1-2): the installation-generation
    // conjunct (S1-BRIEF-R1-01) -- carried history was
    // produced relative to the neutral installed under
    // previous_install_generation; a resync (any producer)
    // advances the view's generation and retires it.
    return have_previous &&
           frame_token == previous_token + 1 &&
           settings_generation == previous_settings_generation &&
           continuity_epoch == previous_continuity_epoch &&
           install_generation == previous_install_generation;
}

// ---------------------------------------------------------------------------
// P2-07B Fix 4 (CP-F4-1): the base-provenance seam (brief
// sections 2.2/2.3). Pure over the store and the cycle's
// named inputs, in the DIAG-1 house pattern: the glue owns
// the store, the lifecycle detection and the bonelog
// emission; these functions own the classification rules,
// so the fixtures exercise exactly what production runs.
// ---------------------------------------------------------------------------

namespace
{

// The brief's closeness tolerance: seed vs lineage, and
// live vs lineage on a no-write cycle, are "the same"
// below 0.01 degrees (section 2.2).
constexpr F32 kProvenanceToleranceRad = 0.01f * DEG_TO_RAD;

F32 provenanceAngleDeg(const LLQuaternion& a,
                       const LLQuaternion& b)
{
    return quaternionAngleRad(a * ~b) * RAD_TO_DEG;
}

bool provenanceClose(const LLQuaternion& a,
                     const LLQuaternion& b)
{
    return quaternionAngleRad(a * ~b) <= kProvenanceToleranceRad;
}

} // namespace

// static
void FSVRAdapterApply::provenanceReset(
    FSVRBaseProvenanceStore& store,
    const std::vector<FSVRAdapterJointInput>& joints,
    const char* reason,
    std::vector<FSVRBaseProvenanceEvent>& events)
{
    store.entries.clear();
    store.dormant.clear();
    for (const FSVRAdapterJointInput& joint : joints)
    {
        FSVRBaseProvenanceEntry entry;
        entry.has_commit = false;
        entry.anim_local = joint.base_local;
        entry.expected_live_local = joint.base_local;
        store.entries[joint.name] = entry;
    }
    FSVRBaseProvenanceEvent event;
    event.kind = FSVRBaseProvenanceEventKind::RESET;
    event.reason = reason ? reason : "";
    events.push_back(event);
}

// static
void FSVRAdapterApply::provenanceAssembly(
    FSVRBaseProvenanceStore& store,
    std::vector<FSVRAdapterJointInput>& joints,
    const std::vector<FSVRBaseBlendFact>& facts,
    bool record_valid,
    std::vector<FSVRBaseProvenanceEvent>& events)
{
    // Membership reconciliation (brief section 2.5): a
    // stored joint no longer in the presented set leaves
    // the active set, loudly -- but its provenance is NOT
    // dropped [FIX4-CODE-R1-02]: it moves to the dormant
    // map under the same ownership and is restored
    // verbatim if the joint returns, because the joint's
    // live local may still be an adapter commit from
    // before its departure, and re-seeding from it would
    // adopt the adapter's own bend as animation.
    {
        std::set<std::string> present_names;
        for (const FSVRAdapterJointInput& joint : joints)
        {
            present_names.insert(joint.name);
        }
        for (auto it = store.entries.begin();
             it != store.entries.end();)
        {
            if (present_names.count(it->first) == 0)
            {
                FSVRBaseProvenanceEvent event;
                event.kind = FSVRBaseProvenanceEventKind::RESET;
                event.joint_name = it->first;
                event.reason = "membership";
                events.push_back(event);
                store.dormant[it->first] = it->second;
                it = store.entries.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    for (size_t i = 0; i < joints.size(); ++i)
    {
        FSVRAdapterJointInput& joint = joints[i];
        const LLQuaternion live_local = joint.base_local;
        const FSVRBaseBlendFact fact =
            (i < facts.size()) ? facts[i] : FSVRBaseBlendFact();
        auto it = store.entries.find(joint.name);
        if (it == store.entries.end())
        {
            // Re-entry after a membership absence under
            // unchanged ownership [FIX4-CODE-R1-02]: the
            // dormant provenance is restored verbatim and
            // this cycle classifies normally below. The
            // joint's live local is NOT consulted as a
            // seed: it may still be the adapter's commit
            // from before the absence.
            auto dormant_it = store.dormant.find(joint.name);
            if (dormant_it != store.dormant.end())
            {
                store.entries[joint.name] = dormant_it->second;
                store.dormant.erase(dormant_it);
                it = store.entries.find(joint.name);
            }
            else
            {
                // First sight (store creation, or a joint
                // that newly joined the set under this
                // ownership): seeded from the joint's
                // current local. The caller guarantees
                // this runs before any adapter write
                // reaches the joint under the current
                // ownership (F4-S6), so the seed is the
                // joint's animation/rest local, never a
                // commit.
                FSVRBaseProvenanceEntry entry;
                entry.has_commit = false;
                entry.anim_local = live_local;
                entry.expected_live_local = live_local;
                store.entries[joint.name] = entry;
                continue;
            }
        }
        FSVRBaseProvenanceEntry& entry = it->second;
        if (!entry.has_commit)
        {
            // Never adapter-written: the live local IS the
            // animation stack's value, tracked verbatim.
            entry.anim_local = live_local;
            entry.expected_live_local = live_local;
            continue;
        }
        if (!record_valid || !fact.present)
        {
            // Fail closed [R3 verdict safeguard +
            // FIX4-CODE-R1-01]: a tracked joint with no
            // trustworthy classification this cycle is NOT
            // read as NO_WRITE (which would carry
            // contaminated state silently, and a stale
            // written entry must never be classified at
            // all) -- but its provenance is NOT dropped
            // either. A missing record is a gap in the
            // observation, not evidence about authorship:
            // the event names the gap, the presented base
            // stays the joint's trusted animation local,
            // and the commit history and live lineage are
            // retained untouched, so the next classified
            // cycle resumes exactly where the last
            // trustworthy one left off. Re-seeding from
            // the live local here would adopt a value that
            // may be the adapter's own commit -- the
            // ratchet this fix exists to break.
            FSVRBaseProvenanceEvent event;
            event.kind = FSVRBaseProvenanceEventKind::RESET;
            event.joint_name = joint.name;
            event.reason = record_valid
                               ? "record_inconsistent"
                               : "record_unavailable";
            events.push_back(event);
            joint.base_local = entry.anim_local;
            continue;
        }
        switch (fact.blend_class)
        {
        case FSVRBaseBlendClass::SEED_DISCARDED:
            // The blend discarded its seed: the result is
            // animation output entire, whatever its value
            // -- including a value equal to the stored
            // commit [R1 case (i)]. Adopt unconditionally.
            entry.anim_local = fact.result_local;
            entry.expected_live_local = fact.result_local;
            break;
        case FSVRBaseBlendClass::SEED_PRESERVED:
            // Additive-only: the result factors as
            // added_rot * seed by the blend's own
            // construction. The gate judges the SEED
            // against the lineage (never the last commit
            // [R2]): a seed the lineage cannot explain is
            // a discontinuity the rules do not cover --
            // carry the tracked local, drop the additive
            // contribution, say so, and re-base the
            // lineage to the blend's result so the next
            // cycle is judged from where the skeleton
            // actually is. No silent fallback.
            if (provenanceClose(fact.seed_local,
                                entry.expected_live_local))
            {
                entry.anim_local =
                    fact.added_rot * entry.anim_local;
            }
            else
            {
                FSVRBaseProvenanceEvent event;
                event.kind =
                    FSVRBaseProvenanceEventKind::UNCLASSIFIED;
                event.joint_name = joint.name;
                event.divergence_deg =
                    provenanceAngleDeg(
                        fact.seed_local,
                        entry.expected_live_local);
                events.push_back(event);
            }
            entry.expected_live_local = fact.result_local;
            break;
        case FSVRBaseBlendClass::NO_WRITE:
        default:
            // The blend did not write the joint. The live
            // local is either the lineage value (our own
            // commit, or an animation-attributed local, or
            // their composition) or it moved without the
            // blend and without us: a foreign write. The
            // foreign value is adopted (the skeleton holds
            // it and the next seed is built on it) and the
            // lineage re-bases to it. Inside tolerance the
            // cycle is an ordinary carry: the presented
            // base is the tracked animation local and the
            // lineage does not revert [R2].
            if (!provenanceClose(live_local,
                                 entry.expected_live_local))
            {
                entry.anim_local = live_local;
                entry.expected_live_local = live_local;
            }
            break;
        }
        joint.base_local = entry.anim_local;
    }
}

// static
void FSVRAdapterApply::provenanceCommitWrites(
    FSVRBaseProvenanceStore& store,
    const std::vector<FSVRAdapterJointInput>& joints,
    const std::vector<FSVRAdapterWrite>& writes)
{
    for (const FSVRAdapterWrite& write : writes)
    {
        if (write.joint_index < 0 ||
            write.joint_index >=
                static_cast<S32>(joints.size()))
        {
            continue;
        }
        auto it =
            store.entries.find(joints[write.joint_index].name);
        if (it == store.entries.end())
        {
            continue;
        }
        it->second.has_commit = true;
        it->second.commit_local = write.local;
        it->second.expected_live_local = write.local;
    }
}

bool decideSpineSampleValid(const FSVRSpineSampleInputs& in)
{
    // P2-07B C6-E (CP-2): the E5-E9 conjunction, verbatim.
    // Extracted so H10 tests the production composition
    // (Package B spec section 9); the glue supplies the
    // cycle's values, this function only composes them.
    // P2-07B Stage 1 (CP-S1-4): E5's input is the renamed
    // scale_input_valid; the conjunction's shape is
    // unchanged.
    return in.scale_input_valid && in.neutral_established &&
           in.hmd_position_valid && in.sitting_signals_clear &&
           !in.calibration_in_progress;
}

bool composeHeadTargetBlock(
    const FSVRSessionNeutralView& session,
    const LLVector3& hmd_position_local, F32 session_base_yaw_rad,
    const LLQuaternion& root_world_rotation,
    const LLVector3& root_world_position, bool diag_force_unit_scale,
    F32 lean_gain,
    LLVector3& out_head_target_world,
    LLVector3& out_cornea_offset_head_local)
{
    // P2-07B C6-E (CP-4; Package B spec section 7.3). Pure
    // over its named inputs; the glue calls it only when the
    // CP-2 conjunction holds for the cycle, and the
    // established check below is the composition's own
    // presence condition (a session without a neutral has
    // nothing to compose from).
    if (!session.established)
    {
        return false;
    }
    // Step 1: the displacement from the session anchor, both
    // in the current LOCAL space (the anchor makes no floor
    // claim). hmd_position_local is the cycle's ONE manager
    // read -- the same sample E7 judged (DP-C6E-3).
    const LLVector3 d_local =
        hmd_position_local - session.anchor_hmd_local;
    // Step 2: the rotation path's placement pipeline in its
    // vector form (brief section 2.13): P re-expresses the
    // raw (right, up, back) components as (forward, left,
    // up) components -- the correspondence stage 2's scalar
    // reading implies -- q_yaw is the session's yaw-only
    // frame alignment (theta_B, ICR-1), and the plan's root
    // frame is the avatar-side carrier, substituted for the
    // camera's chest basis by the spec's own ruling
    // (PB-R1-03 bars the arbitrated pose here).
    const LLVector3 p_mapped(-d_local.mV[VZ], -d_local.mV[VX],
                             d_local.mV[VY]);
    const LLQuaternion q_yaw(session_base_yaw_rad, LLVector3::z_axis);
    LLVector3 d_world =
        (p_mapped * q_yaw) * root_world_rotation;
    // P2-07B Stage 1 (CP-S1-3): the provisional mapping. For
    // a sync-established neutral, the user's lean gain
    // scales the target displacement BEFORE the solver sees
    // it -- applied to d_world, not to the seam ratio, so
    // FSVRDiagForceUnitScale keeps its exact shipped
    // meaning (it forces the *calibration* ratio to 1.0;
    // the gain is a user preference and survives the
    // diagnostic). At gain 1.0 the cornea target moves
    // exactly as far as the headset moved, in world units:
    // a baseline response, not an anatomical proportion
    // (REDESIGN-R1-01).
    // The post-multiplication finite guard (S1-BRIEF-R1-02)
    // lives in this branch only: this is the one path where
    // a user-supplied multiplier meets an unbounded
    // displacement, and a nonfinite target must never reach
    // the solver from it. The measured path introduces no
    // new multiplication and keeps its shipped behaviour
    // exactly (its inputs remain bounded by the record
    // validator); guarding it too would break the measured
    // path's pinned bit-identity for no Stage 1 need.
    const bool provisional =
        session.scale_source == FSVRScaleSource::PROVISIONAL_SYNC;
    if (provisional)
    {
        d_world *= lean_gain;
        if (!std::isfinite(d_world.mV[VX]) ||
            !std::isfinite(d_world.mV[VY]) ||
            !std::isfinite(d_world.mV[VZ]))
        {
            return false;
        }
    }
    // Step 3: the neutral cornea, fixed root-relative at
    // establishment (PB-R1-03), re-expressed in the CURRENT
    // root frame -- the composition's only per-cycle
    // pose-dependent input. The cycle's arbitrated pose is
    // never read here.
    const LLVector3 neutral_cornea_world =
        session.neutral_cornea_root * root_world_rotation +
        root_world_position;
    // Step 4: C6's shipped composition seam. P2-07B Stage 1
    // (CP-S1-3): the ratio is selected by the view's scale
    // source -- the record's head_ratio for a measured
    // neutral (bit-identical to the shipped line), the
    // provisional baseline constant for a sync-established
    // one (whose record is a default NO_RECORD basis whose
    // head_ratio is never read). For NONE the block is
    // unreachable in production (E5 has already failed the
    // cycle, CP-S1-4) and the baseline is the safe value
    // regardless. The diagnostic is consumed here, at
    // composition, per spec section 3.4 -- the record is
    // not touched.
    const F32 scale_ratio =
        (session.scale_source == FSVRScaleSource::MEASURED_RECORD)
            ? static_cast<F32>(session.record.head_ratio)
            : static_cast<F32>(
                  FSVRCalibration::PROVISIONAL_LEAN_BASELINE);
    out_head_target_world = FSVRSpineSolver::composeHeadTarget(
        neutral_cornea_world, d_world, scale_ratio,
        diag_force_unit_scale);
    if (provisional &&
        (!std::isfinite(out_head_target_world.mV[VX]) ||
         !std::isfinite(out_head_target_world.mV[VY]) ||
         !std::isfinite(out_head_target_world.mV[VZ])))
    {
        // S1-BRIEF-R1-02: no target is published from a
        // nonfinite provisional composition.
        out_head_target_world = LLVector3::zero;
        return false;
    }
    // Step 5: the section 4.6 cornea offset, verbatim.
    out_cornea_offset_head_local = session.cornea_offset_head_local;
    return true;
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
    // P2-07B DIAG-1 (D-1, Route A): retain the base
    // composition on the result, copied here -- before any
    // committed-state modification -- so gate failures and
    // whole-transaction refusal leave it unmodified. One
    // composition, computed once; no second implementation
    // exists to drift.
    result.base_world_rotations = base_world_rotations;
    result.base_world_positions = base_world_positions;
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
    // P2-07B C5 (CP-5): the calibration-hold branch (Package B
    // spec section 3.1's authorization contract). Entered iff
    // the plan input carries an active C5 calibration
    // transaction; its authority derives from that transaction
    // alone and is independent of gates 1-5 by design, so it
    // sits after the cycle/context checks that precede gate 1
    // and before every gate exit. In a hold cycle the solver
    // phases do not run and no solver is invoked: the result
    // is the held pose -- bind locals (identity, C0 section 1)
    // for exactly the section 3.1 hold set, base locals
    // carried over their held parents for every other joint --
    // with exactly one write per hold-set joint present in
    // the input, and no other writes (no targets, no pole
    // state, no continuity bookkeeping).
    if (input.calibration_hold_active)
    {
        const std::vector<std::string>& hold_names =
            FSVRAdapterApply::calibrationHoldJointNames();
        std::vector<char> is_hold(count, 0);
        for (S32 i = 0; i < count; ++i)
        {
            for (const std::string& name : hold_names)
            {
                if (input.joints[i].name == name)
                {
                    is_hold[i] = 1;
                    break;
                }
            }
        }
        result.applied = true;
        for (S32 i = 0; i < count; ++i)
        {
            const FSVRAdapterJointInput& joint = input.joints[i];
            LLQuaternion parent_rot = input.root_world_rotation;
            LLVector3 parent_pos = input.root_world_position;
            if (joint.parent_index >= 0)
            {
                parent_rot = result.resolved_world_rotations[joint.parent_index];
                parent_pos = result.resolved_world_positions[joint.parent_index];
            }
            const LLQuaternion local =
                is_hold[i] ? LLQuaternion() : joint.base_local;
            LLQuaternion world = local * parent_rot;
            world.normalize();
            result.resolved_locals[i] = local;
            result.resolved_world_rotations[i] = world;
            result.resolved_world_positions[i] =
                parent_pos + (joint.rest_offset * parent_rot);
            if (is_hold[i])
            {
                result.writes.push_back({i, LLQuaternion()});
            }
        }
        return result;
    }
    // Gate 1: frame identity. Consuming Stage-1 shadow decisions
    // as if they were grants is the failure mode this gate exists
    // to prevent. P2-07B C6-E (CP-5 part 3): the test is the
    // scoped disjunction over the per-domain facts -- a domain's
    // shadow state never blocks the other domain, and the legacy
    // all-bones field no longer gates (recorded fact only).
    if (!input.decisions_published || !input.decisions_current ||
        !(input.spine_decisions_runtime_effective ||
          input.arm_decisions_runtime_effective))
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
    // sample validity, evaluated independently. P2-07B C6-E
    // (CP-5 part 3): each domain's authority additionally
    // requires that domain's scoped runtime-effective fact, so
    // no authority forms on shadow decisions for its domain.
    const bool spine_authority =
        input.head_position_enabled && input.spine_sample_valid &&
        input.spine_decisions_runtime_effective;
    const bool arm_authority =
        input.hand_position_enabled && input.arm_sample_valid &&
        input.arm_decisions_runtime_effective;
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
    // [R2 -- C6C7-03 transport] The caller-owned pole history
    // rides beside previous_targets by the same mechanical copy.
    context.previous_pole_valid[0] = input.previous_pole_valid[0];
    context.previous_pole_valid[1] = input.previous_pole_valid[1];
    context.previous_pole_world[0] = input.previous_pole_world[0];
    context.previous_pole_world[1] = input.previous_pole_world[1];

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
        // P2-07B Fix 4 (CP-F4-3): retain the solver's
        // returned target worlds verbatim on the result --
        // here, immediately after the solve, before any
        // clamping or commit-local conversion below.
        // Instrumentation only; nothing reads the field
        // back into the plan (F4-Z1).
        result.solver_target_worlds = spine_targets;
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
        const size_t writes_before_side = result.writes.size();
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

        // [R2 -- C6C7-03 transport] Forward the solver's pole to
        // the side's arm outcome only now: the side ran, was not
        // aborted, its post-clamp ran, and its commit processing
        // is complete, with at least one of its writes committed.
        // Assigning this beside the solver's return (before the
        // post-clamp and eligibility/commit processing) would
        // expose a provisional pole for a side the transaction
        // may still discard; the aborted paths above return
        // before this point and forward nothing.
        if (wt.has_pole && result.writes.size() > writes_before_side)
        {
            outcome.pole_valid = true;
            outcome.pole_world = wt.pole_world;
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

    // P2-07B Fix 1 (CP-F1-1): head-group orientation
    // reconciliation. Runs on every applied cycle, after the
    // arm phase and immediately before finalization, so the
    // committed write order is spine -> arms -> head-group.
    // Head-group joints are neither ancestors nor descendants
    // of any arm joint, so the placement cannot disturb arm
    // results, and no later phase can invalidate the
    // reconciliation. The positional solve may move the head
    // group's worlds through its spine ancestors; it may not
    // change the head group's orientations (INV-F1-01/02):
    // the neck and head stage worlds are restored to their
    // base-composed values by the commit pass's own
    // target->local formula, under the limit cascade and the
    // combined-bound selection below. When no spine commit
    // moved an ancestor of the neck, the correction is the
    // identity and the epsilon suppression emits nothing
    // (S1-G5). When the transaction does not apply at all,
    // computePlan has already returned -- whole-head
    // switching is preserved structurally, not by a
    // conditional (checklist 0.5).
    {
        S32 neck_index = -1;
        S32 head_index = -1;
        for (S32 i = 0; i < count; ++i)
        {
            if (input.joints[i].name == "mNeck")
            {
                neck_index = i;
            }
            else if (input.joints[i].name == "mHead")
            {
                head_index = i;
            }
        }
        if (neck_index >= 0 && head_index >= 0)
        {
            const LLQuaternion& base_neck_world =
                base_world_rotations[neck_index];
            const LLQuaternion& base_head_world =
                base_world_rotations[head_index];
            LLQuaternion parent_neck_rot;
            LLVector3 parent_neck_pos;
            parentFrame(neck_index, parent_neck_rot,
                        parent_neck_pos);
            LLQuaternion parent_neck_norm = parent_neck_rot;
            parent_neck_norm.normalize();

            // Step 1 (neck stage): the corrected neck local
            // by the commit pass's own formula (local =
            // target * ~parent_world) with the target set to
            // the neck's BASE world, capped at the head-group
            // local cap (step 3). Recompose the neck world.
            LLQuaternion cand_neck_local =
                base_neck_world * ~parent_neck_norm;
            cand_neck_local.normalize();
            cand_neck_local.constrain(kHeadGroupLocalCapRad);
            LLQuaternion cand_neck_world =
                cand_neck_local * parent_neck_rot;
            cand_neck_world.normalize();

            // Step 2 (head stage): the corrected head local
            // against the recomposed neck world, then the
            // native operation itself -- constrain(
            // HEAD_ROTATION_CONSTRAINT), the identical call
            // and value the native path applies
            // (llheadrotmotion.cpp:321; constant at :91 --
            // the value is kHeadGroupLocalCapRad). Recompose
            // the head world. In the uncapped case step 1
            // restored the neck world to base exactly, so
            // this returns the head local to its base value:
            // the correction concentrates at the neck, and
            // spill reaches the head local only when the
            // neck cap refuses part of the correction.
            LLQuaternion cand_head_local =
                base_head_world * ~cand_neck_world;
            cand_head_local.normalize();
            cand_head_local.constrain(kHeadGroupLocalCapRad);
            LLQuaternion cand_head_world =
                cand_head_local * cand_neck_world;
            cand_head_world.normalize();

            // Step 5 [R3/R4]: combined-bound selection. The
            // cascade bounds the two locals; it does not
            // bound the quantity Rule 1 states -- the head's
            // combined rotation relative to its basis. The
            // basis is located by CP-F1-0 Q4's rule (mTorso,
            // else mPelvis, else the plan root frame) and
            // measured against the basis joint's BASE world
            // -- the frame the native path composed this
            // cycle's base pose against. The guarantee is
            // the base-basis statement of INV-F1-03 [R4]
            // (Q-H): fidelity to the native pre-solve
            // constraint; no committed-torso bound is
            // claimed.
            S32 basis_index = -1;
            for (S32 i = 0; i < count; ++i)
            {
                if (input.joints[i].name == "mTorso")
                {
                    basis_index = i;
                    break;
                }
            }
            if (basis_index < 0)
            {
                for (S32 i = 0; i < count; ++i)
                {
                    if (input.joints[i].name == "mPelvis")
                    {
                        basis_index = i;
                        break;
                    }
                }
            }
            const LLQuaternion basis_base_world =
                basis_index >= 0
                    ? base_world_rotations[basis_index]
                    : input.root_world_rotation;
            LLQuaternion basis_norm = basis_base_world;
            basis_norm.normalize();
            const F32 theta_b = quaternionAngleRad(
                base_head_world * ~basis_norm);
            const F32 theta_c = quaternionAngleRad(
                cand_head_world * ~basis_norm);

            LLQuaternion final_neck_local = cand_neck_local;
            LLQuaternion final_head_local = cand_head_local;
            bool split_selected = false;
            if (theta_c >
                kHeadGroupLocalCapRad + kHeadGroupBoundTolRad)
            {
                // Bounded result: W*, the combined-legal
                // target closest to base on the cascade's
                // own residual arc, realized by the EVEN
                // SPLIT -- the native distribution's own
                // shape (NECK_LAG 0.5, llheadrotmotion.cpp:
                // 358-359). Under the production convention
                // (world = local * parent_world) the two
                // halves must compose the total relative
                // rotation T = W* * ~P [R4 -- FIX1-R3-01;
                // the superseded order ~P * W* is a conjugate
                // and does not recompose to W*].
                LLQuaternion d =
                    cand_head_world * ~base_head_world;
                d.normalize();
                const F32 rho_max = std::max(
                    0.f, kHeadGroupLocalCapRad - theta_b);
                LLQuaternion w_star =
                    scaleQuaternionAngle(d, rho_max) *
                    base_head_world;
                w_star.normalize();
                LLQuaternion t = w_star * ~parent_neck_norm;
                t.normalize();
                const F32 tau = quaternionAngleRad(t);
                if (tau > 2.f * kHeadGroupLocalCapRad)
                {
                    // Refusal [R4 -- FIX1-R3-02]:
                    // whole-transaction rejection. The
                    // result returns to its entry state --
                    // writes cleared entire (spine, arm and
                    // head-group), the finalization copy
                    // below does not run, so resolved_*
                    // report the base composition -- and the
                    // arm outcomes' committed claims are
                    // retracted (a pole or wrist error
                    // computed against a discarded commit
                    // must not be adopted); the write-through
                    // boundary's own abort records stand.
                    // applied stays true and
                    // first_failing_gate stays NONE: the
                    // transaction ran. The refusal is named
                    // by the result's head_group_refusal
                    // field.
                    result.writes.clear();
                    result.arm_outcome_left.pole_valid = false;
                    result.arm_outcome_left.wrist_error_valid =
                        false;
                    result.arm_outcome_right.pole_valid = false;
                    result.arm_outcome_right.wrist_error_valid =
                        false;
                    result.head_group_refusal = true;
                    return result;
                }
                const LLQuaternion half =
                    scaleQuaternionAngle(t, tau * 0.5f);
                final_neck_local = half;
                final_head_local = half;
                split_selected = true;
            }

            // Step 4 [R2 (ii)/R3]: epsilon/write evaluation
            // of the FINAL pair (the cascade pair, or the
            // selection's split pair), per stage, decided
            // BEFORE any recomposition. A stage whose
            // corrected local differs from its committed
            // local by at least 1e-4 rad appends one write
            // (neck before head) and updates the committed
            // arrays; stage 2 forms its candidate against
            // the post-decision neck world.
            constexpr F32 kHeadGroupEpsilonRad = 1e-4f;
            bool neck_emitted = false;
            {
                const F32 d_neck = quaternionAngleRad(
                    final_neck_local *
                    ~committed_locals[neck_index]);
                if (d_neck >= kHeadGroupEpsilonRad)
                {
                    result.writes.push_back(
                        {neck_index, final_neck_local});
                    committed_locals[neck_index] =
                        final_neck_local;
                    neck_emitted = true;
                }
            }
            if (!neck_emitted && !split_selected)
            {
                // Stage 1 was suppressed: the carried neck
                // world stands, so the head candidate is
                // re-formed against it [R2 (ii)] (the two
                // neck worlds differ by less than the
                // epsilon, so this is the same candidate at
                // F32 class; the selection's split pair, by
                // contrast, is the final pair as selected
                // [R3] and is never re-formed).
                LLQuaternion carried_neck_world =
                    committed_rotations[neck_index];
                carried_neck_world.normalize();
                final_head_local =
                    base_head_world * ~carried_neck_world;
                final_head_local.normalize();
                final_head_local.constrain(
                    kHeadGroupLocalCapRad);
            }
            bool head_emitted = false;
            {
                const F32 d_head = quaternionAngleRad(
                    final_head_local *
                    ~committed_locals[head_index]);
                if (d_head >= kHeadGroupEpsilonRad)
                {
                    result.writes.push_back(
                        {head_index, final_head_local});
                    committed_locals[head_index] =
                        final_head_local;
                    head_emitted = true;
                }
            }
            if (neck_emitted || head_emitted)
            {
                // Recompose the emitted stages and every
                // plan descendant of theirs, parent-first,
                // by the commit pass's carry rule (a
                // descendant's local is unchanged; its
                // world follows its parent). The neck's own
                // world position never changes -- its
                // parent's world is untouched by this step;
                // the head's world position changes only
                // through the neck->head rest offset rotating
                // with the corrected neck world.
                std::vector<char> recompose(count, 0);
                for (S32 i = 0; i < count; ++i)
                {
                    const bool self =
                        (i == neck_index && neck_emitted) ||
                        (i == head_index && head_emitted);
                    const S32 parent =
                        input.joints[i].parent_index;
                    if (!self &&
                        !(parent >= 0 && recompose[parent]))
                    {
                        continue;
                    }
                    recompose[i] = 1;
                    LLQuaternion parent_rot =
                        input.root_world_rotation;
                    LLVector3 parent_pos =
                        input.root_world_position;
                    if (parent >= 0)
                    {
                        parent_rot = committed_rotations[parent];
                        parent_pos = committed_positions[parent];
                    }
                    LLQuaternion world =
                        committed_locals[i] * parent_rot;
                    world.normalize();
                    committed_rotations[i] = world;
                    committed_positions[i] =
                        parent_pos +
                        (input.joints[i].rest_offset *
                         parent_rot);
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
#include "llheadrotmotion.h"
#include "lljoint.h"
#include "llopenxrmanager.h"
#include "llpose.h"
#include "llopenxrtrace.h"
#include "llvrarmsolver.h"
#include "llvrcontinuity.h"
#include "llvrboneregistry.h"
#include "llvrspinesolver.h"
#include "llvrtrackingarbitrator.h"
#include "llvrtrackingframe.h"
#include "llvrtrackingsettings.h"
#include "llvoavatarself.h"

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

// ------------------------------------------------------------------
// P2-07B DIAG-1 (brief sections 2.3 / 2.5 / 3 item 3):
// the recorder's per-cycle consumer state. The headrot
// capture and the blend record reach the glue through the
// character's animation-data slots; the consumers enforce
// the cycle-validity contract (current generation only,
// per publishing instance). On a character change both
// consumers reset, so no accepted generation can carry
// across characters. The identity comparison itself --
// consumed slot pointers vs the capture's published state
// identities, same cycle, both generation-checked --
// happens here at sample assembly; the raw pointers are
// consumed here and passed no further.
// ------------------------------------------------------------------
LLCharacter* s_diag_consumer_character = nullptr;
FSVRHeadRotConsumer s_diag_headrot_consumer;
FSVRBlendObsConsumer s_diag_blend_consumer;

void convertBlendJoint(const FSVRBlendObsJoint& in,
                       FSVRBoneLogBlendJoint& out)
{
    out.generation = in.generation;
    out.seed_local = in.seed_local;
    out.rot_weight_sum = in.rot_weight_sum;
    out.blended_rot = in.blended_rot;
    out.added_rot = in.added_rot;
    out.result_local = in.result_local;
    out.slots.clear();
    for (S32 i = 0; i < in.slot_count; ++i)
    {
        const FSVRBlendObsSlot& slot = in.slots[i];
        FSVRBoneLogBlendSlot converted;
        converted.additive = slot.additive;
        converted.priority = slot.priority;
        converted.weight = slot.weight;
        converted.rotation = slot.rotation;
        converted.has_rot_sum = slot.has_rot_sum;
        converted.rot_sum_after = slot.rot_sum_after;
        converted.delta = slot.delta;
        converted.blended_after = slot.blended_after;
        converted.added_after = slot.added_after;
        converted.source_headrot = slot.source_headrot;
        out.slots.push_back(converted);
    }
    out.source_integrity_failure = in.source_integrity_failure;
}

} // namespace

void FSVRAdapterApplyForCharacter(LLCharacter* character, bool preview_cycle)
{
    if (!character)
    {
        return;
    }

    // P2-07B C8 (CP-1/CP-4/CP-5): service this cycle's
    // continuity signals first, before anything reads them:
    // drain the manager's finalized LOCAL-transition batches
    // (routing each through the rebase decision; a Suspend
    // invalidates the session neutral through C5's entry
    // point) and sample the self avatar's skeleton rebuild
    // generation (a change invalidates likewise). Placement
    // is the contract: the plan input assembled below stamps
    // the post-service continuity epoch, and C5's step hook
    // at the tail reads C5's epoch advanced by the same
    // service, so no cycle can assemble or step on
    // pre-transition state. The service runs on published
    // and bypass cycles alike; idle cost is one empty drain
    // plus one integer load + compare.
    FSVRContinuity::serviceCycleSignals();

    const FSVRTrackingFrameDecisions* decisions =
        FSVRTrackingCoordinator::currentDecisions();
    if (!decisions && !FSVRCalibration::cycleWorkPending())
    {
        // Nothing published for this frame, and calibration
        // has no work: nothing to process.
        return;
    }

    // P2-07B C5 (CP-4 item 3): the pre-cycle staleness check
    // runs before input assembly on both the decisions path
    // and the bypass path, so a starved transaction aborts
    // here rather than producing a hold cycle after a gap.
    FSVRCalibration::preCycleCheck();

    // P2-07B C6-E (C6E-CODE-R1-01): refresh C5's ICR-2
    // sitting fact from THIS cycle's avatar signals before
    // the assembly below copies the session view. Without
    // this the view's sitting_signals_clear would be the
    // value the post-apply step hook wrote LAST cycle, and
    // E8 would judge the first sitting cycle by the previous
    // cycle's standing state. The refresh is C5's own entry
    // point and the fact's only writer remains the session
    // machine's noteSittingSignals; the step hook at the
    // tail of this function refreshes again from the same
    // character through the same signal read.
    FSVRCalibration::refreshSittingSignals(character);

    const FSVRTrackingSettingsSnapshot& snapshot =
        FSVRTrackingSettings::captureForFrame();

    FSVRAdapterPlanInput input;
    // On the no-decisions bypass path (CP-4 item 6) the
    // token comes from the trace's current app frame token:
    // the same fallback the coordinator itself uses when it
    // has no pending token (llvrtrackingframe.cpp).
    input.frame_token =
        decisions ? decisions->app_frame_token
                  : static_cast<U64>(LLOpenXRTrace::currentAppFrameToken());
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
    // same frame than the motion update). On the no-decisions
    // bypass path (C5 CP-4 item 6) the three decisions fields
    // are false and the per-bone decision loops below are
    // skipped (the decision map stays empty, so every joint
    // takes its no-decision defaults); the rest of the
    // assembly is unchanged.
    input.decisions_published = (decisions != nullptr);
    input.decisions_current =
        decisions && (snapshot.generation == decisions->settings_generation);
    // The legacy all-bones form, retained as a recorded fact
    // (C6-E CP-5 part 3): it no longer gates anything.
    input.decisions_runtime_effective = false;
    if (decisions)
    {
        input.decisions_runtime_effective = !decisions->bones.empty();
        for (const FSVRBoneDecision& bone : decisions->bones)
        {
            if (bone.shadow_only)
            {
                input.decisions_runtime_effective = false;
                break;
            }
        }
        // Gate 1, scoped (P2-07B C6-E, CP-5 part 3):
        // runtime-effective is tested per solver domain over
        // the decisions the arbitrator emitted -- this glue
        // reads the emitted shadow_only form through the
        // arbitrator's pure derivations; it never reinterprets
        // it. Each fact holds iff the published set contains
        // the domain's decisions and none of them is
        // shadow-only.
        input.spine_decisions_runtime_effective =
            spineDecisionsRuntimeEffective(*decisions);
        input.arm_decisions_runtime_effective =
            armDecisionsRuntimeEffective(*decisions);
    }

    // Gate 2 input (P2-07B C6-E, CP-1): the manager's own
    // VR-mode fact -- the identical call the coordinator
    // makes for the same fact (llvrtrackingframe.cpp). This
    // discharges C4 brief DP-5's deferred half. It lands
    // inert: gate 1 is evaluated before gate 2 and stays
    // closed in every build without an authorization commit
    // (CP-5), so the plan's output is unchanged in all
    // production states (H10-12).
    input.vr_active = LLOpenXRManager::instance().isVRModeActive();
    // The six enablement/diagnostic fields are P2-07B C4
    // (CP-6): they read the user's actual positional settings
    // and diagnostic values from the frame snapshot captured
    // above, so the adapter plans on them.
    input.positional_solve_enabled = snapshot.diag_positional_solve;
    input.head_position_enabled = snapshot.track_head_position;
    input.hand_position_enabled = snapshot.track_hand_position;
    // P2-07B C6-E (CP-2/CP-3): the spine sample-validity
    // conjunction over this cycle's facts. The session view
    // is read once, here, as a copy: E5/E6/E8 and the CP-4
    // composition below all consume this same cycle's view.
    // E9 is C5's transaction state, read once at population;
    // the same read populates calibration_hold_active below
    // (CP-3: one fact, one read, never latched).
    const FSVRSessionNeutralView session_view =
        FSVRCalibration::sessionNeutralView();
    const bool calibration_active = FSVRCalibration::transactionActive();
    // The cycle's ONE HMD position read (CP-4 step 1,
    // DP-C6E-3): its bool is E7, and the same vector feeds
    // the composition below, so the sample judged valid is
    // the sample composed from -- never two reads that
    // could straddle an adoption.
    LLVector3 hmd_position_local;
    const bool hmd_position_read =
        LLOpenXRManager::instance().getHmdPosition(hmd_position_local);
    FSVRSpineSampleInputs sample_inputs;
    // P2-07B Stage 1 (CP-S1-4): E5 is scale-input validity
    // -- the view carries a named scale source (a measured
    // record validated at its producers, or the provisional
    // baseline), and the snapshot's lean gain is inside its
    // bounded domain. An invalid gain fails the cycle
    // closed for EVERY scale source: the user who typed a
    // broken value gets no positional spine -- the
    // diagnosable state -- not a quietly different feel.
    // (Pre-Stage-1 this field read the record's USABLE_*
    // status; a sync-established neutral has no record to
    // read, which is the redesign's whole point.)
    sample_inputs.scale_input_valid =
        session_view.scale_source != FSVRScaleSource::NONE &&
        snapshot.lean_gain_valid;
    sample_inputs.neutral_established = session_view.established;
    sample_inputs.hmd_position_valid = hmd_position_read;
    sample_inputs.sitting_signals_clear = session_view.sitting_signals_clear;
    sample_inputs.calibration_in_progress = calibration_active;
    input.spine_sample_valid = decideSpineSampleValid(sample_inputs);
    // Section 7.4: a standing false, not a conjunction
    // awaiting inputs -- no arm sample source exists in
    // this component (Package C's fact).
    input.arm_sample_valid = false;
    input.bend_disabled_bones = snapshot.solver_bend_disabled_bones;
    input.diag_bend_limits = snapshot.diag_bend_limits;
    input.diag_full_chain_solve = snapshot.diag_full_chain_solve;
    // C6/C7 (CP-7): the soft-preference diagnostic and the
    // snapshot generation reach the plan from the same
    // snapshot.
    input.diag_soft_preference = snapshot.diag_soft_preference;
    input.settings_generation = snapshot.generation;
    // P2-07B C8 (CP-3): the continuity stamp, read once at
    // input assembly from C8's router (post-service, above).
    // The retirement predicate below compares it against the
    // previous apply's stamp.
    input.continuity_epoch = FSVRContinuity::continuityEpoch();

    // P2-07B C5 (CP-4 item 2): the calibration hold flag.
    // The apply point gains no second mode channel: the flag
    // rides the plan input like every other field. P2-07B
    // C6-E (CP-3): the value is the population block's single
    // transactionActive() read, shared with E9 -- the same
    // fact, read once.
    input.calibration_hold_active = calibration_active;

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
    if (decisions)
    {
        for (const FSVRBoneDecision& bone : decisions->bones)
        {
            decision_by_bone[bone.bone_id] = &bone;
        }
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

    // P2-07B Fix 4 (CP-F4-2): base provenance. The store is
    // created at assembly -- ahead of this cycle's write
    // pass below, so no adapter write of this ownership
    // can precede its seeding (F4-S6) -- and seeded from
    // the joints' current locals. Each cycle the
    // classification rewrites every tracked joint's
    // presented base to its animation-attributed local, so
    // the plan is never presented the adapter's own prior
    // commits as animation (INV-F4-01), while the write
    // pass below still lands the commits on the live
    // joints for rendering. Provenance is behaviour, not
    // recording: it runs whether or not the bonelog is
    // enabled. Lifecycle (brief section 2.5 [R3]): RESET
    // only on character change and on skeleton reinit
    // evidenced by the rebuild generation itself; KEEP
    // across resync, settings change, frame-token gap,
    // gate failure, head-group refusal and headset-off
    // intervals -- none of those touch the store.
    static FSVRBaseProvenanceStore s_provenance;
    static LLCharacter* s_provenance_character = nullptr;
    static U64 s_provenance_skeleton_gen = 0;
    static bool s_provenance_have_skeleton_gen = false;
    static FSVRBlendObsConsumer s_provenance_blend_consumer;
    {
        std::vector<FSVRBaseProvenanceEvent> provenance_events;
        if (s_provenance_character != character)
        {
            // Character change: the incoming character's
            // provenance starts from its own current pose.
            // The FIRST bind is the store's creation --
            // adopted silently (nothing was superseded, so
            // no reset event); later changes reset loudly.
            const bool first_bind =
                (s_provenance_character == nullptr);
            s_provenance_character = character;
            s_provenance_blend_consumer = FSVRBlendObsConsumer();
            s_provenance_have_skeleton_gen = false;
            if (first_bind)
            {
                s_provenance.entries.clear();
                s_provenance.dormant.clear();
            }
            else
            {
                FSVRAdapterApply::provenanceReset(
                    s_provenance, input.joints,
                    "character_change", provenance_events);
            }
        }
        // Skeleton reinit, evidenced by the rebuild
        // generation itself -- the C8/C5-shell read
        // (llvrcalibration.cpp). The bonelog recorder's
        // skeleton serial is a separate, recorder-owned
        // signal and is never consulted for provenance.
        const U64 prov_skeleton_gen =
            gAgentAvatarp
                ? gAgentAvatarp->getSkeletonRebuildGeneration()
                : 0;
        if (s_provenance_have_skeleton_gen &&
            prov_skeleton_gen != s_provenance_skeleton_gen)
        {
            FSVRAdapterApply::provenanceReset(
                s_provenance, input.joints, "skeleton_reinit",
                provenance_events);
        }
        s_provenance_skeleton_gen = prov_skeleton_gen;
        s_provenance_have_skeleton_gen = true;

        // This cycle's blend classification, read from the
        // same generation-validated record the DIAG-1 tail
        // consumes (each consumer tracks its own accepted
        // generation). A joint the record's current pass
        // did not write is NO_WRITE; a joint outside the
        // observed set carries no fact; a record that did
        // not advance leaves every tracked joint's fact
        // unavailable, which the seam fails closed on --
        // never a silent NO_WRITE [R3 verdict safeguard].
        FSVRBlendObsRecord* prov_blend_record =
            static_cast<FSVRBlendObsRecord*>(
                character->getAnimationData("FSVRBlendObs"));
        const bool prov_record_valid =
            prov_blend_record &&
            s_provenance_blend_consumer.consumePass(
                prov_blend_record);
        std::vector<FSVRBaseBlendFact> prov_facts(
            input.joints.size());
        if (prov_record_valid)
        {
            for (size_t i = 0; i < input.joints.size(); ++i)
            {
                const FSVRBlendObsJoint* entry =
                    fsvrBlendObsEntry(*prov_blend_record,
                                      input.joints[i].name);
                if (!entry)
                {
                    continue;
                }
                FSVRBaseBlendFact& fact = prov_facts[i];
                fact.present = true;
                if (!entry->written ||
                    entry->generation !=
                        prov_blend_record->generation)
                {
                    fact.blend_class =
                        FSVRBaseBlendClass::NO_WRITE;
                    continue;
                }
                fact.seed_local = entry->seed_local;
                fact.added_rot = entry->added_rot;
                fact.result_local = entry->result_local;
                bool seed_discarded = false;
                for (S32 s = 0; s < entry->slot_count; ++s)
                {
                    seed_discarded = seed_discarded ||
                                     entry->slots[s].has_rot_sum;
                }
                fact.blend_class =
                    seed_discarded
                        ? FSVRBaseBlendClass::SEED_DISCARDED
                        : FSVRBaseBlendClass::SEED_PRESERVED;
            }
        }
        FSVRAdapterApply::provenanceAssembly(
            s_provenance, input.joints, prov_facts,
            prov_record_valid, provenance_events);
        for (const FSVRBaseProvenanceEvent& prov_event :
             provenance_events)
        {
            if (prov_event.kind ==
                FSVRBaseProvenanceEventKind::RESET)
            {
                FSVRBoneLog::eventBaseProvenanceReset(
                    prov_event.joint_name.empty()
                        ? nullptr
                        : prov_event.joint_name.c_str(),
                    prov_event.reason.c_str());
            }
            else
            {
                FSVRBoneLog::eventBaseProvenanceUnclassified(
                    prov_event.joint_name.c_str(),
                    prov_event.divergence_deg);
            }
        }
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

    // P2-07B C6-E (CP-4): the per-cycle head-target block.
    // Composition is gated on the E5-E9 conjunction only
    // (DP-C6E-8): the block is composed whenever the
    // conjunction holds, whether or not gates 1-4 will pass
    // this cycle -- the transaction's gates decide use;
    // validity describes the inputs. It rides the root
    // frame populated immediately above, so displacement
    // and neutral are never composed against different
    // poses. head_target_valid is assigned every cycle,
    // never latched: a lapsed cycle leaves it false and the
    // target fields at their defaults, unread (solveSpine
    // guards on the flag). The wrist block is written false
    // explicitly, every cycle (section 7.4).
    input.head_target_valid =
        input.spine_sample_valid &&
        composeHeadTargetBlock(session_view, hmd_position_local,
                               session_view.anchor_forward_yaw_rad,
                               input.root_world_rotation,
                               input.root_world_position,
                               snapshot.diag_force_unit_scale,
                               snapshot.lean_gain,
                               input.head_target_world,
                               input.cornea_offset_head_local);
    input.wrist_target_valid[0] = false;
    input.wrist_target_valid[1] = false;

    // One apply-once instance and the solvers for the process
    // (the self avatar is unique). C6/C7 (CP-7): the real
    // solvers sit in the slots -- C6 in the spine slot, C7 in
    // the write-through slot with its mandatory post-clamp; the
    // plain target-world arm slot stays the null solver (with a
    // write-through solver supplied it is never invoked).
    static FSVRAdapterApplyOnce s_apply_once;
    static FSVRSpineSolver s_spine_solver;
    static FSVRAdapterNullArmSolver s_plain_arm_solver;
    static FSVRArmSolver s_arm_solver;
    static FSVRArmPostClamp s_arm_post_clamp;

    // Continuity bookkeeping (CP-7/DP-4): the glue owns the
    // cross-cycle history the pure plan cannot hold -- the last
    // committed target map and, per side, the adopted pole --
    // each stamped with the frame token and settings generation
    // it was produced under. The solvers stay pure: history
    // reaches them only as plan inputs.
    static std::map<S32, LLQuaternion> s_previous_targets;
    static bool s_have_previous = false;
    static U64 s_previous_token = 0;
    static U64 s_previous_settings_generation = 0;
    // P2-07B C8 (CP-3): the continuity epoch the previous
    // apply's inputs were stamped with, refreshed on the
    // same paths that refresh the token/generation stamps.
    static U64 s_previous_continuity_epoch = 0;
    // P2-07B Stage 1 (CP-S1-2, S1-BRIEF-R1-01): the session
    // view's installation generation the stored history was
    // produced under, refreshed on the same paths. A
    // resync advances the view's generation, the predicate
    // below sees the mismatch, and history earned under the
    // old anchor retires before the next solve.
    static U64 s_previous_install_generation = 0;
    static bool s_pole_valid[2] = {false, false};
    static LLVector3 s_pole_world[2] = {LLVector3::zero,
                                        LLVector3::zero};

    // Retirement is evaluated before each apply, whether or not
    // a side will be invoked: history carries only when the
    // previous apply's token is the immediate predecessor (a gap
    // is a tracking loss, an authority lapse, or a
    // non-invocation) and the settings generation is unchanged.
    // Anything else retires BOTH histories entire here, at the
    // glue, with no invocation [R2 -- C6C7-CODE-03: retirement
    // is applied BEFORE the transaction, separate from the
    // aborted-side preservation on the success path below --
    // previously the stored target map survived a discontinuity
    // and the success path rebuilt from it, resurrecting stale
    // membership with non-solved poses].
    // P2-07B C8 (CP-3): the predicate itself is
    // FSVRAdapterApply::continuityCarries (plan TU, H8-
    // fixtured); the epoch term retires both histories here
    // when a LOCAL transition or a skeleton reinit landed
    // between the two applies -- events that produce neither
    // a token gap nor a settings-generation change.
    const bool continuity_carries = FSVRAdapterApply::continuityCarries(
        s_have_previous, input.frame_token, s_previous_token,
        input.settings_generation, s_previous_settings_generation,
        input.continuity_epoch, s_previous_continuity_epoch,
        session_view.install_generation, s_previous_install_generation);
    if (!continuity_carries)
    {
        s_previous_targets.clear();
        s_pole_valid[0] = false;
        s_pole_valid[1] = false;
    }
    // Per-side retirement [R2 -- C6C7-CODE-03]: a side that
    // loses its authority this cycle -- no wrist target, or the
    // arm authority off -- retires its pole and its share of
    // the target map here, even when the other side stays
    // active and the cycle's continuity as a whole carries; a
    // dead side's pole no longer persists indefinitely beside
    // a live one. (Package A production never populates wrist
    // targets, so this is inert until Packages B/C supply
    // them.)
    {
        const bool arm_authority =
            input.hand_position_enabled && input.arm_sample_valid;
        const FSVRAdapterDomain side_domains[2] = {
            FSVRAdapterDomain::ARM_LEFT, FSVRAdapterDomain::ARM_RIGHT};
        for (S32 s = 0; s < 2; ++s)
        {
            if (arm_authority && input.wrist_target_valid[s])
            {
                continue;
            }
            s_pole_valid[s] = false;
            for (auto it = s_previous_targets.begin();
                 it != s_previous_targets.end();)
            {
                if (it->first >= 0 &&
                    it->first < static_cast<S32>(input.joints.size()) &&
                    input.joints[it->first].domain == side_domains[s])
                {
                    it = s_previous_targets.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }
    }
    if (continuity_carries)
    {
        input.previous_targets = s_previous_targets;
        for (S32 s = 0; s < 2; ++s)
        {
            if (s_pole_valid[s])
            {
                input.previous_pole_valid[s] = true;
                input.previous_pole_world[s] = s_pole_world[s];
            }
        }
    }

    const FSVRAdapterPlanResult result = s_apply_once.apply(
        input, s_spine_solver, s_plain_arm_solver, &s_arm_solver,
        &s_arm_post_clamp);

    if (result.first_failing_gate == FSVRAdapterGate::TOKEN)
    {
        // A duplicate token: the apply did not happen; the
        // bookkeeping stands exactly as it was.
    }
    else if (result.first_failing_gate == FSVRAdapterGate::NONE)
    {
        // Store the committed-domain target worlds as the new
        // history, and adopt each side's pole only where the
        // side's outcome carries one: by construction that means
        // the side ran, was not aborted, its post-clamp ran, and
        // its result was committed. An aborted or discarded side
        // updates no history: neither its pole state nor its
        // share of previous_targets changes on account of this
        // call ([R2] lifecycle), so the new map starts from the
        // (post-retirement) stored one and refreshes ONLY the
        // joints this cycle actually committed [R2 --
        // C6C7-CODE-03: entries for joints not written this
        // cycle keep their stored values -- refreshing them
        // from the resolved pose would replace a solved history
        // with the joint's base composition, a non-solved pose].
        std::map<S32, LLQuaternion> new_targets = s_previous_targets;
        auto side_aborted = [&](FSVRAdapterDomain domain) {
            if (domain == FSVRAdapterDomain::ARM_LEFT)
            {
                return result.arm_outcome_left.write_through_aborted;
            }
            if (domain == FSVRAdapterDomain::ARM_RIGHT)
            {
                return result.arm_outcome_right.write_through_aborted;
            }
            return false;
        };
        auto refresh = [&](S32 joint_index) {
            if (joint_index >= 0 &&
                joint_index <
                    static_cast<S32>(
                        result.resolved_world_rotations.size()) &&
                !side_aborted(input.joints[joint_index].domain))
            {
                new_targets[joint_index] =
                    result.resolved_world_rotations[joint_index];
            }
        };
        for (const FSVRAdapterWrite& write : result.writes)
        {
            refresh(write.joint_index);
        }
        s_previous_targets = new_targets;
        s_previous_token = input.frame_token;
        s_previous_settings_generation = input.settings_generation;
        s_previous_continuity_epoch = input.continuity_epoch;
        s_previous_install_generation = session_view.install_generation;
        s_have_previous = true;
        if (result.arm_outcome_left.pole_valid)
        {
            s_pole_valid[0] = true;
            s_pole_world[0] = result.arm_outcome_left.pole_world;
        }
        if (result.arm_outcome_right.pole_valid)
        {
            s_pole_valid[1] = true;
            s_pole_world[1] = result.arm_outcome_right.pole_world;
        }
    }
    else
    {
        // A pre-solver gate failed: no solver ran, so the stored
        // targets are stale and both poles retire (the sides'
        // authority lapsed this cycle). The token stamp still
        // advances, so the next cycle's contiguity is judged
        // against this apply and finds an empty map.
        s_previous_targets.clear();
        s_previous_token = input.frame_token;
        s_previous_settings_generation = input.settings_generation;
        s_previous_continuity_epoch = input.continuity_epoch;
        s_previous_install_generation = session_view.install_generation;
        s_have_previous = true;
        s_pole_valid[0] = false;
        s_pole_valid[1] = false;
    }

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

    // P2-07B Fix 4 (CP-F4-2): the writes are on the
    // skeleton now; refresh the provenance store's commit
    // and lineage state for exactly the writes applied
    // (the result's write list verbatim -- gate-failed,
    // token-suppressed and refused results carry empty
    // write lists and refresh nothing, and a hold cycle's
    // identity writes DO refresh, as section 2.5 rules).
    FSVRAdapterApply::provenanceCommitWrites(
        s_provenance, input.joints, result.writes);

    // P2-07B C5 (CP-4 item 4): the post-apply step hook. The
    // transaction and session machines advance on this
    // cycle's plan input and plan result plus the shell's
    // fresh reads (manager samples, avatar signals, eye
    // state, read at step time, once, inside the module's
    // shell). When neither machine has work the hook is two
    // flag reads.
    FSVRCalibration::stepFromAdapterCycle(character, input, result);

    // P2-07B DIAG-1 (brief section 3 item 3): assemble the
    // recorder's headrot + blend inputs for this cycle.
    // Both arrive through the character's animation-data
    // slots (D-2 R-a publication; the motion controller's
    // blend-record publication) and are consumed under the
    // cycle-validity contract: current generation only, per
    // publishing instance. The identity comparison --
    // consumed slot pointers vs the capture's published
    // state identities -- is performed here, on stamped
    // COPIES of the blend entries; the raw pointers are
    // consumed here and passed no further (only the
    // semantic source marker reaches the recorder). No
    // identity, no attribution: without a valid
    // current-cycle capture the stamp identity is null and
    // no slot is marked. All of it is skipped when the
    // recorder is disabled.
    const FSVRHeadRotCapture* diag_capture = nullptr;
    FSVRBoneLogHeadRotSample diag_headrot;
    FSVRBoneLogBlendSample diag_blend;
    bool diag_have_headrot = false;
    bool diag_have_blend = false;
    if (FSVRBoneLog::enabled())
    {
        if (s_diag_consumer_character != character)
        {
            s_diag_consumer_character = character;
            s_diag_headrot_consumer = FSVRHeadRotConsumer();
            s_diag_blend_consumer = FSVRBlendObsConsumer();
        }
        const FSVRHeadRotCapture* published_capture =
            static_cast<const FSVRHeadRotCapture*>(
                character->getAnimationData("FSVRHeadRotCapture"));
        diag_capture =
            s_diag_headrot_consumer.accept(published_capture);
        if (diag_capture)
        {
            diag_headrot.neck_state = diag_capture->neck_state;
            diag_headrot.head_state = diag_capture->head_state;
            diag_headrot.last_head_rot_before =
                diag_capture->last_head_rot_before;
            diag_headrot.last_head_rot_after =
                diag_capture->last_head_rot_after;
            diag_headrot.instant = diag_capture->instant;
            diag_headrot.generation = diag_capture->generation;
            diag_have_headrot = true;
        }
        FSVRBlendObsRecord* blend_record =
            static_cast<FSVRBlendObsRecord*>(
                character->getAnimationData("FSVRBlendObs"));
        if (blend_record &&
            s_diag_blend_consumer.consumePass(blend_record))
        {
            for (S32 which = 0; which < 2; ++which)
            {
                const FSVRBlendObsJoint* entry =
                    s_diag_blend_consumer.entryFor(*blend_record,
                                                   which);
                if (!entry)
                {
                    continue;
                }
                FSVRBlendObsJoint stamped = *entry;
                fsvrStampBlendObsSource(
                    stamped,
                    which == 0
                        ? (diag_capture
                               ? diag_capture->neck_state_id
                               : nullptr)
                        : (diag_capture
                               ? diag_capture->head_state_id
                               : nullptr));
                // CODE-R1-01 (brief section 2.5):
                // independent consistency validation of
                // the identity-attributed slots against
                // this cycle's published ask and the
                // motion's effective mode captured at
                // publication. A mismatch flags the
                // entry's record-integrity failure in the
                // record; the identity attribution stands,
                // no slot is re-matched by value, and
                // nothing is written back.
                if (diag_capture)
                {
                    fsvrValidateBlendObsSource(
                        stamped,
                        which == 0
                            ? diag_capture->neck_state
                            : diag_capture->head_state,
                        diag_capture->priority,
                        diag_capture->additive);
                }
                if (which == 0)
                {
                    diag_blend.neck_present = true;
                    convertBlendJoint(stamped, diag_blend.neck);
                }
                else
                {
                    diag_blend.head_present = true;
                    convertBlendJoint(stamped, diag_blend.head);
                }
                diag_have_blend = true;
            }
        }
    }

    // P2-07B feel-regrounding diagnostic: the per-cycle
    // motion sample (FS_VR_BONELOG). Taken here, at the
    // tail, so the skeleton the recorder reads carries
    // this cycle's committed writes and the step hook --
    // including any sync it executed -- has already run.
    // Observation-only: the recorder early-outs on its
    // cached env bool when disabled and writes nothing
    // back, ever.
    FSVRBoneLog::sampleCycle(
        character, input, result,
        hmd_position_read, hmd_position_local,
        diag_have_headrot ? &diag_headrot : nullptr,
        diag_have_blend ? &diag_blend : nullptr);
}

#endif // FSVR_ADAPTER_PLAN_ONLY
