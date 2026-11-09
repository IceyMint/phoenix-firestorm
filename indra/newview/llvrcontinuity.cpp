/**
 * @file llvrcontinuity.cpp
 * @brief P2-07B C8: LOCAL-space continuity (Package B spec
 *        section 6; brief P2-07B-C8-IMPLEMENTATION-BRIEF
 *        [R2]).
 *
 * Above the FSVR_CONTINUITY_PURE_ONLY guard: the
 * finalization observer (CP-1), the rebase decision
 * (CP-2), and the epoch router (CP-4/CP-5 consumption) --
 * no globals, harness-compilable as production code (H8).
 * Below it: the viewer shell that drains the OpenXR
 * manager, samples the self avatar's rebuild generation,
 * and executes the C5 invalidation entry-point calls the
 * routing implies.
 */

#include "llviewerprecompiledheaders.h"

#include "llvrcontinuity.h"

#include <cmath>

// ---------------------------------------------------------------------------
// CP-1: the finalization observer.
// ---------------------------------------------------------------------------

void FSVRFinalizationObserver::noteFinalizedBatch(
    const FSVRFinalizedBatch& keys)
{
    // Interface-level dedup (CP-1 item 4): a key already
    // notified -- by any earlier presentation, including a
    // direct re-presentation -- is dropped. The manager's
    // structural dedup is the primary guarantee; this set
    // makes the observer's own contract hold standalone
    // (H8-12 fixtures it against this production code).
    FSVRFinalizedBatch fresh;
    for (const FSVRFinalizedTransitionKey& key : keys)
    {
        const std::pair<unsigned long long, long long> dedup_key(
            key.incarnation, key.change_time_ns);
        if (mNotifiedKeys.insert(dedup_key).second)
        {
            fresh.push_back(key);
        }
    }
    if (fresh.empty())
    {
        // A batch consisting solely of already-notified
        // keys yields no notification at all.
        return;
    }
    // One presentation stays one batch, unconditionally
    // (C8-CODE-R1-01): the epoch contract counts batches,
    // not keys, so no buffer bound may merge presentations
    // here -- an earlier defensive collapse did exactly
    // that and was removed on the cross-check's ruling.
    mPendingBatches.push_back(fresh);
}

std::vector<FSVRFinalizedBatch> FSVRFinalizationObserver::drainBatches()
{
    std::vector<FSVRFinalizedBatch> drained;
    drained.swap(mPendingBatches);
    return drained;
}

// ---------------------------------------------------------------------------
// CP-2: the rebase decision.
// ---------------------------------------------------------------------------

namespace
{

bool matrixElementsFinite(const LLMatrix4& matrix)
{
    for (S32 row = 0; row < 4; ++row)
    {
        for (S32 col = 0; col < 4; ++col)
        {
            if (!std::isfinite(matrix.mMatrix[row][col]))
            {
                return false;
            }
        }
    }
    return true;
}

// Proper-rigid validation of the upper 3x3 plus the
// homogeneous column, computed in F64 over the F32 elements
// (the tolerance is an absolute 1e-3 bound; F32 arithmetic
// would spend a visible fraction of it on rounding).
bool matrixIsRigid(const LLMatrix4& matrix)
{
    F64 basis[3][3];
    for (S32 row = 0; row < 3; ++row)
    {
        for (S32 col = 0; col < 3; ++col)
        {
            basis[row][col] = matrix.mMatrix[row][col];
        }
    }
    // Unit basis rows, pairwise orthogonal.
    for (S32 i = 0; i < 3; ++i)
    {
        const F64 length = std::sqrt(basis[i][0] * basis[i][0] +
                                     basis[i][1] * basis[i][1] +
                                     basis[i][2] * basis[i][2]);
        if (std::fabs(length - 1.0) > FSVR_RIGID_TOL)
        {
            return false;
        }
        for (S32 j = i + 1; j < 3; ++j)
        {
            const F64 dot = basis[i][0] * basis[j][0] +
                            basis[i][1] * basis[j][1] +
                            basis[i][2] * basis[j][2];
            if (std::fabs(dot) > FSVR_RIGID_TOL)
            {
                return false;
            }
        }
    }
    // Determinant +1 (a reflection is -1, a degeneracy ~0;
    // both fail here even when the rows above pass).
    const F64 cross[3] = {
        basis[1][1] * basis[2][2] - basis[1][2] * basis[2][1],
        basis[1][2] * basis[2][0] - basis[1][0] * basis[2][2],
        basis[1][0] * basis[2][1] - basis[1][1] * basis[2][0]};
    const F64 determinant = basis[0][0] * cross[0] +
                            basis[0][1] * cross[1] +
                            basis[0][2] * cross[2];
    if (std::fabs(determinant - 1.0) > FSVR_RIGID_TOL)
    {
        return false;
    }
    // Homogeneous column (0, 0, 0, 1): in LLMatrix4's row
    // convention the basis occupies rows 0..2 and the
    // translation row 3, so the affine constraint binds
    // mMatrix[0..2][3] to 0 and mMatrix[3][3] to 1. (The
    // translation itself, mMatrix[3][0..2], is
    // unconstrained -- a rebasing transform normally has
    // one.)
    if (std::fabs(matrix.mMatrix[0][3]) > FSVR_RIGID_TOL ||
        std::fabs(matrix.mMatrix[1][3]) > FSVR_RIGID_TOL ||
        std::fabs(matrix.mMatrix[2][3]) > FSVR_RIGID_TOL ||
        std::fabs(matrix.mMatrix[3][3] - 1.0) > FSVR_RIGID_TOL)
    {
        return false;
    }
    return true;
}

} // namespace

FSVRRebaseResult decideRebase(const FSVRTransitionEvidence& evidence,
                              const LLVector3& session_anchor_local)
{
    FSVRRebaseResult result; // Suspend until every test passes

    if (!evidence.has_origin_transform)
    {
        // The production case (DP-B6): no transform is not
        // evidence of continuity.
        return result;
    }
    // Authenticated (C8-R1-02): the transform must be bound
    // to THIS transition's identity. A correct transform
    // for a different transition is not evidence about
    // this one.
    if (evidence.transform_incarnation != evidence.incarnation ||
        evidence.transform_generation != evidence.generation ||
        evidence.transform_change_time_ns != evidence.change_time_ns)
    {
        return result;
    }
    if (!matrixElementsFinite(evidence.origin_transform))
    {
        return result;
    }
    if (!matrixIsRigid(evidence.origin_transform))
    {
        // Finiteness plus provenance do not substitute for
        // rigidity: scale, shear, reflection, and
        // degeneracy all suspend.
        return result;
    }

    result.decision = FSVRRebaseDecision::Rebase;
    result.rebased_anchor = session_anchor_local * evidence.origin_transform;
    return result;
}

// ---------------------------------------------------------------------------
// CP-4/CP-5: the epoch router.
// ---------------------------------------------------------------------------

FSVRRebaseResult FSVRContinuityRouter::routeTransitionEvidence(
    const FSVRTransitionEvidence& evidence,
    const LLVector3& session_anchor_local)
{
    const FSVRRebaseResult result = decideRebase(evidence,
                                                 session_anchor_local);
    if (result.decision == FSVRRebaseDecision::Suspend)
    {
        // CP-4: a non-rebased transition invalidates the
        // session neutral (the consumer notification is the
        // shell's, on this routing) and retires continuity
        // history at the glue (via the continuity stamp).
        ++mLocalTransitionEpoch;
        ++mContinuityEpoch;
    }
    // On Rebase neither epoch advances: the neutral
    // survives and neither history is retired (section
    // 6.3's negative).
    return result;
}

FSVRRebaseResult FSVRContinuityRouter::routeFinalizedBatch(
    const FSVRFinalizedBatch& batch)
{
    FSVRRebaseResult result; // Suspend
    if (batch.empty())
    {
        // Not an event (defensive; the production observer
        // never emits an empty batch): no epoch advance.
        return result;
    }
    // Representative identity: the greatest change_time_ns
    // in the batch (a take's consumed-newest key; a retire
    // batch's latest final). Ties resolve to the later key
    // in presentation order.
    const FSVRFinalizedTransitionKey* representative = &batch.front();
    for (const FSVRFinalizedTransitionKey& key : batch)
    {
        if (key.change_time_ns >= representative->change_time_ns)
        {
            representative = &key;
        }
    }
    // DP-C8-6 -- THE production evidence construction site:
    // has_origin_transform is false unconditionally. No
    // setting, diagnostic toggle, or environment variable
    // arms the rebase branch in v1; enabling it requires a
    // new ruling on a named evidence source (DP-B6).
    FSVRTransitionEvidence evidence;
    evidence.incarnation = representative->incarnation;
    evidence.generation = representative->generation;
    evidence.change_time_ns = representative->change_time_ns;
    evidence.has_origin_transform = false;
    return routeTransitionEvidence(evidence, LLVector3::zero);
}

bool FSVRContinuityRouter::noteSampledSkeletonGeneration(U64 generation)
{
    if (!mHaveGenerationSample)
    {
        // Seed-on-first-sample (CP-5): adopt the current
        // generation without firing. The initial loadAvatar
        // build predates any neutral and is never an event.
        mHaveGenerationSample = true;
        mLastSkeletonGeneration = generation;
        return false;
    }
    if (generation == mLastSkeletonGeneration)
    {
        return false;
    }
    // Exactly one event per generation value: the sample
    // updates with the event, so re-sampling the same new
    // generation cannot fire twice. (resetSkeleton's
    // deliberate double build advances the funnel by 2
    // inside one call stack; the intermediate value is
    // never sampled, and this still fires exactly once.)
    mLastSkeletonGeneration = generation;
    ++mSkeletonReinitEpoch;
    ++mContinuityEpoch;
    return true;
}

bool FSVRContinuityRouter::noteSessionIncarnation(
    unsigned long long incarnation)
{
    if (!mHaveSessionIncarnation)
    {
        // Seed-on-first-sample, mirroring CP-5's generation
        // seeding: the router is constructed with the
        // process, so whatever it first observes -- a live
        // session's incarnation, or 0 for no impl -- is its
        // baseline. No earlier session's state exists in
        // this router to retire.
        mHaveSessionIncarnation = true;
        mSessionIncarnation = incarnation;
        return false;
    }
    if (incarnation == mSessionIncarnation)
    {
        return false;
    }
    // Session boundary (C8-CODE-R1-02). The incarnation is
    // the manager's own process-static counter (P2-04b
    // D1'): assigned at initialize(), never reused within
    // the process, and 0 exactly when no impl exists -- so
    // any change is authoritative evidence that an impl
    // was destroyed and/or a new one created, whichever
    // edge (session end, session start, or both inside one
    // inter-service gap) this sample observes. Everything
    // the old session's state rested on is gone, whether or
    // not its queue entries were ever finalized -- the
    // STOPPING clear is deliberately unobserved (CP-1), so
    // this fence, not the queue, is what guarantees no
    // neutral crosses the boundary.
    mSessionIncarnation = incarnation;
    // Advance exactly as a Suspend routing advances: the
    // shell calls C5's local-transition invalidation entry
    // point once for this event, and that call advances
    // C5's own local-transition epoch in the same breath --
    // lockstep, and an interrupted calibration aborts on
    // C5's epoch comparison instead of resuming on the
    // dead session's observations. The continuity epoch
    // advance retires C1's positional history at the glue.
    ++mLocalTransitionEpoch;
    ++mContinuityEpoch;
    // Reseed the skeleton-generation baseline: the rebuild
    // counter is authoritative only within its avatar
    // instance, and a new instance can reproduce the dead
    // session's last value. The next sample seeds silently
    // for the current instance; a rebuild after that seeds
    // fires normally. (No skeleton_reinit_epoch advance:
    // no reinit was observed -- the baseline reset is this
    // event's whole effect on CP-5 consumption.)
    mHaveGenerationSample = false;
    return true;
}

#ifndef FSVR_CONTINUITY_PURE_ONLY

// ---------------------------------------------------------------------------
// Viewer shell. Everything below reads live viewer state
// (the manager's incarnation and drain, the self avatar's
// rebuild generation) and executes the consumer
// notifications the router implies (C5's invalidation
// entry points). The router instance is process-static,
// so its lifetime deliberately spans sessions; what keeps
// any epoch or baseline from crossing a session boundary
// is the incarnation fence in serviceCycleSignals()
// (C8-CODE-R1-02, discharging brief section 5.4's
// integration obligation in production code rather than
// presuming the invariant).
// ---------------------------------------------------------------------------

#include "llvrcalibration.h"
#include "llopenxrmanager.h"
#include "llvoavatarself.h"

namespace
{

FSVRContinuityRouter s_router;

} // namespace

// static
void FSVRContinuity::serviceCycleSignals()
{
    // Session-lifecycle fence (C8-CODE-R1-02): sample the
    // manager's incarnation FIRST -- before any batch is
    // routed and before this cycle's plan is assembled --
    // so a new session's first eligible positional target
    // can never be composed against the previous session's
    // neutral or history. Detecting the boundary here is
    // sufficient even though STOPPING itself may occur
    // while no adapter cycle runs: nothing composes a
    // target except through a cycle, and every cycle
    // services this fence before assembling its plan. The
    // invalidation goes through C5's existing entry point,
    // exactly as a Suspend routing delivers it.
    if (s_router.noteSessionIncarnation(
            LLOpenXRManager::instance().getIncarnation()))
    {
        FSVRCalibration::invalidateNeutralForLocalTransition();
    }

    // CP-1 item 5: pull-only delivery. The drain moves
    // batches only here, on the session machinery's own
    // stack (the adapter cycle), never on the camera's
    // finalization stack.
    const std::vector<FSVRFinalizedBatch> batches =
        LLOpenXRManager::instance()
            .drainFinalizedLocalTransitionBatches();
    for (const FSVRFinalizedBatch& batch : batches)
    {
        const FSVRRebaseResult result =
            s_router.routeFinalizedBatch(batch);
        if (result.decision == FSVRRebaseDecision::Suspend)
        {
            // The entry point advances C5's own
            // local-transition epoch in the same call, so
            // the router's epoch and C5's stay in lockstep:
            // this shell is the entry point's only
            // production caller, and it calls exactly once
            // per Suspend routing.
            FSVRCalibration::invalidateNeutralForLocalTransition();
        }
        // A Rebase routing owes no notification (and is
        // unreachable in production v1 -- DP-C8-6).
    }

    // CP-5 consumption: one integer load + compare per
    // cycle against the self avatar's rebuild generation.
    if (gAgentAvatarp)
    {
        if (s_router.noteSampledSkeletonGeneration(
                gAgentAvatarp->getSkeletonRebuildGeneration()))
        {
            FSVRCalibration::invalidateNeutralForSkeletonReinit();
        }
    }
}

// static
U64 FSVRContinuity::continuityEpoch()
{
    return s_router.continuityEpoch();
}

#endif // FSVR_CONTINUITY_PURE_ONLY
