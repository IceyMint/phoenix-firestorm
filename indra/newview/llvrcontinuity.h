/**
 * @file llvrcontinuity.h
 * @brief P2-07B C8: LOCAL-space continuity -- finalized
 *        transition observation, the rebase decision, and
 *        the continuity epoch routing (Package B spec
 *        section 6; brief P2-07B-C8-IMPLEMENTATION-BRIEF
 *        [R2], SHIP).
 *
 * C8 owns transitions and signals only. It observes the
 * OpenXR manager's LOCAL-transition finalizations without
 * participating in them (CP-1: a manager-owned observer
 * with a pull-only drain and camera body-turn annotation), decides
 * rebase-vs-suspend as a pure function over authenticated,
 * finite, rigid transform evidence (CP-2), detects skeleton
 * reinitialization through LLAvatarAppearance's rebuild
 * generation (CP-5; the counter itself is llappearance's
 * one-increment hook, AMEND-C8-R1-01), and routes the three
 * continuity epochs (CP-4): local_transition_epoch,
 * skeleton_reinit_epoch, and the dedicated continuity_epoch
 * the C1 glue stamps into the plan input (CP-3). The state
 * the signals invalidate belongs to the consumers: the
 * session neutral and the calibration transaction are C5's
 * (reached only through FSVRCalibration's invalidation
 * entry points, in the shell below), and the glue's
 * continuity history retires through its own predicate.
 *
 * Recovery milestone 1: authenticated OpenXR LOCAL events now supply
 * finite, gravity-aligned old-to-new transforms. Missing or invalid
 * evidence still suspends. Every transition retires positional history
 * and aborts acquisition windows; only an established neutral is rebased.
 *
 * Silence (DP-B7): no C8 path emits user-visible output.
 * Outcomes are available in the LOCAL calibration continuity trace
 * and C5 calibration reports.
 *
 * Factoring (house pattern, C4/C5/C6C7): the observer, the
 * decision helper, and the epoch router sit above the
 * FSVR_CONTINUITY_PURE_ONLY guard in the .cpp (no globals,
 * harness-compilable as production code); the viewer shell
 * (the manager drain, the self avatar's generation sample,
 * the calls into C5's invalidation entry points) sits below
 * it. H8 drives the production observer, decideRebase, the
 * router, and the glue's extracted continuity predicate.
 */

#ifndef LL_LLVRCONTINUITY_H
#define LL_LLVRCONTINUITY_H

#include "llmath.h"
#include "m4math.h"

#include <set>
#include <utility>
#include <vector>

// ---------------------------------------------------------------------------
// CP-1: finalized-transition evidence capture.
// ---------------------------------------------------------------------------

// Identity and transform evidence retained from an authenticated LOCAL event.
// LLLocalSpaceTransition extends this payload; finalization copies it intact.
struct FSVRFinalizedTransitionKey
{
    unsigned long long incarnation = 0;
    unsigned long long generation = 0;
    long long change_time_ns = 0;
    bool has_origin_transform = false;
    LLMatrix4 origin_transform; // old LOCAL -> new LOCAL, raw OpenXR axes
    F32 body_turn_rad = 0.f; // accepted LLAgent yaw; annotated by camera
};

// One take/retire call's finalized non-stale transitions:
// a take presents its superseded keys first and the
// consumed-newest key last; a retire presents its retired
// keys. One batch is ONE continuity event however many
// keys it carries (CP-1 item 3).
typedef std::vector<FSVRFinalizedTransitionKey> FSVRFinalizedBatch;

// The manager-owned finalization observer (CP-1 items 2-5).
// A member of LLOpenXRManagerImpl beside the queue it
// observes; the manager calls noteFinalizedBatch() from its
// two finalization sites and never calls out anywhere else.
// There is no callback, no registration, and no pointer to
// session state in this design: delivery is pull-only, via
// drainBatches(), on the consumer's own stack, so a
// callback into destroyed session state is structurally
// impossible. Batches undrained at session teardown are
// discarded with the impl.
//
// Exactly-once is two-layered (CP-1 item 4). The primary
// guarantee is the manager's own structural dedup: an
// identity is finalized at most once, by construction, so
// the fan-out sites cannot present it twice. The observer
// additionally dedups by finalized transition identity
// (the (incarnation, change_time_ns) key, exactly as the
// manager's finalized_local_transitions set is keyed):
// noteFinalizedBatch() drops any key already notified, and
// a batch left empty by dedup produces no notification.
// Stale-epoch drops are never presented (the manager does
// not call here for them), and the STOPPING queue clear is
// not a presentation either.
class FSVRFinalizationObserver
{
public:
    // Called by the manager once per take/retire call that
    // finalized at least one non-stale transition, with
    // that call's finalized keys. Keys already notified are
    // dropped; if no key survives, nothing is buffered.
    void noteFinalizedBatch(const FSVRFinalizedBatch& keys);

    // Annotate only an undrained, matching finalization. The camera calls
    // this on the same thread immediately after applying LLAgent::yaw.
    void noteBodyTurn(unsigned long long incarnation, long long time, F32 turn);

    // Pull-only drain: returns the accumulated batches and
    // clears the buffer. Empty when nothing finalized since
    // the last drain.
    std::vector<FSVRFinalizedBatch> drainBatches();

    // Undrained batch count (diagnostics and H8 fixtures).
    size_t pendingBatchCount() const { return mPendingBatches.size(); }

private:
    // No buffer bound and no merging (C8-CODE-R1-01): one
    // batch is one continuity event, and the epoch contract
    // counts events -- 65 independently finalized batches
    // are 65 epoch advances downstream, so collapsing them
    // into one batch would silently rewrite that count.
    // Batches are produced only by camera consumption
    // during an active session whose machinery drains per
    // cycle, so the buffer holds at most one batch in
    // practice; if that ever stops being true the buffer
    // simply grows until the next drain.
    std::set<std::pair<unsigned long long, long long>> mNotifiedKeys;
    std::vector<FSVRFinalizedBatch> mPendingBatches;
};

// ---------------------------------------------------------------------------
// CP-2: the pure rebase decision.
// ---------------------------------------------------------------------------

// Validated transform evidence bound to one transition identity.
struct FSVRTransitionEvidence
{
    // The transition's own identity (from CP-1's batch).
    unsigned long long incarnation = 0;
    unsigned long long generation = 0;
    long long change_time_ns = 0;
    bool has_origin_transform = false;
    LLMatrix4 origin_transform; // meaningful only when set
    // The transform's provenance: the identity of the
    // transition it claims to map old->new for. A transform
    // whose provenance does not match the transition above
    // is not evidence about this transition (C8-R1-02).
    unsigned long long transform_incarnation = 0;
    unsigned long long transform_generation = 0;
    long long transform_change_time_ns = 0;
};

enum class FSVRRebaseDecision
{
    Rebase,
    Suspend
};

struct FSVRRebaseResult
{
    FSVRRebaseDecision decision = FSVRRebaseDecision::Suspend;
    LLVector3 rebased_anchor; // valid only on Rebase
    LLMatrix4 origin_transform;
    F32 body_turn_rad = 0.f;
};

// The rigidity tolerance (CP-2 [R2]): an absolute bound on
// basis-row length deviations, pairwise dot products, and
// the determinant / homogeneous-column deviations.
constexpr F64 FSVR_RIGID_TOL = 1e-3;

// Rebase is returned iff ALL of the following hold; every
// other input returns Suspend:
//   1. has_origin_transform is true;
//   2. authenticated -- the transform's provenance triple
//      equals the transition's own identity;
//   3. finite -- every element of the matrix is finite;
//   4. rigid -- the upper 3x3's basis rows are unit length
//      and pairwise orthogonal, the determinant is +1, and
//      the homogeneous column is (0, 0, 0, 1), each within
//      FSVR_RIGID_TOL. Scale, shear, reflection (det -1),
//      and degeneracy (det ~ 0) all fail. The helper
//      validates the geometry itself; no caller-supplied
//      boolean stands in for it.
// On Rebase the anchor is re-expressed by the transform
// (the tree's own operator*(LLVector3, LLMatrix4)). The
// function is total and pure: no globals, no settings, no
// manager access.
// Checked conversion of the OpenXR new-origin pose to old->new coordinates.
// LOCAL is gravity aligned: reject pitch/roll, malformed quaternions and NaNs.
bool fsvrLocalOriginTransform(const LLVector3& position,
                              const LLQuaternion& orientation, LLMatrix4& out);
FSVRTransitionEvidence fsvrComposeLocalTransitions(const FSVRFinalizedBatch& batch);
F32 fsvrRebasedYaw(F32 yaw, const LLMatrix4& old_to_new);
F32 fsvrHeadYawDelta(F32 yaw, F32 base);

FSVRRebaseResult decideRebase(const FSVRTransitionEvidence& evidence,
                              const LLVector3& session_anchor_local);

// ---------------------------------------------------------------------------
// CP-4/CP-5: the epoch routing core.
// ---------------------------------------------------------------------------

// The routing half of C8's signal production, pure so H8
// fixtures the production routing (CP-6): finalized batches
// and sampled skeleton generations go in; the three
// continuity epochs advance per CP-4's table. The shell
// (below the guard) executes the consumer notifications the
// routing implies -- one FSVRCalibration invalidation entry
// point call per Suspend routing, one per generation-change
// event -- so the epochs here and C5's own counters advance
// in lockstep by construction (the shell is the entry
// points' only production caller).
class FSVRContinuityRouter
{
public:
    // Route one transition-evidence decision (CP-4): on
    // Suspend, local_transition_epoch and continuity_epoch
    // each advance once; on Rebase, neither advances (the
    // neutral survives, continuity history is retained --
    // spec section 6.3). Returns decideRebase's full result.
    // Legacy single-evidence seam for isolated fixtures. Production uses
    // routeFinalizedBatch, which additionally retires old-coordinate history.
    FSVRRebaseResult routeTransitionEvidence(
        const FSVRTransitionEvidence& evidence,
        const LLVector3& session_anchor_local);

    // Chronological composition; invalid/missing transforms suspend.
    // Every nonempty batch advances epochs to retire old-coordinate history.
    FSVRRebaseResult routeFinalizedBatch(
        const FSVRFinalizedBatch& batch,
        const LLVector3& anchor = LLVector3::zero);

    // CP-5 consumption predicate: feed one sampled rebuild
    // generation of the self avatar's LLAvatarAppearance.
    // The first sample after construction seeds silently
    // (adopts the current generation without firing -- the
    // initial loadAvatar build predates any neutral). A
    // sampled value differing from the last fires exactly
    // once per generation value: skeleton_reinit_epoch and
    // continuity_epoch advance, and the caller owes the
    // consumer notification. Returns true iff an event
    // fired on this sample.
    bool noteSampledSkeletonGeneration(U64 generation);

    // Session-lifecycle fence (C8-CODE-R1-02): feed the
    // OpenXR manager's current incarnation once per service
    // cycle, before any batch is routed. The incarnation is
    // the manager's own process-static counter (assigned at
    // initialize(), never reused; 0 exactly when no impl
    // exists), so a changed value is authoritative -- an
    // impl was destroyed and/or a new one created. The
    // first sample after construction seeds silently (the
    // router is born with the process; no earlier session's
    // state exists for it to retire). A change is a session
    // boundary: local_transition_epoch and continuity_epoch
    // advance exactly as on a Suspend routing (the caller
    // owes C5's local-transition invalidation entry point
    // exactly once, which retires the neutral and aborts an
    // interrupted calibration on C5's epoch comparison),
    // and the skeleton-generation baseline is reseeded --
    // the rebuild counter is authoritative only within its
    // avatar instance, so the next generation sample seeds
    // silently for the current instance instead of
    // comparing against the dead session's last value,
    // which a rebuilt instance can coincidentally
    // reproduce. Returns true iff a boundary fired on this
    // sample.
    bool noteSessionIncarnation(unsigned long long incarnation);

    U64 localTransitionEpoch() const { return mLocalTransitionEpoch; }
    U64 skeletonReinitEpoch() const { return mSkeletonReinitEpoch; }
    // The dedicated glue stamp (CP-4 [R2]): an independent
    // monotonic counter advanced at exactly the three note
    // sites above (transition routing, generation change,
    // session boundary) -- not a derived sum, so no
    // arithmetic identity or wraparound composition needs
    // defending. Consumers only ever test it for equality
    // against their own last-seen value.
    U64 continuityEpoch() const { return mContinuityEpoch; }

private:
    U64 mLocalTransitionEpoch = 0;
    U64 mSkeletonReinitEpoch = 0;
    U64 mContinuityEpoch = 0;
    bool mHaveGenerationSample = false;
    U64 mLastSkeletonGeneration = 0;
    bool mHaveSessionIncarnation = false;
    unsigned long long mSessionIncarnation = 0;
};

// ---------------------------------------------------------------------------
// Viewer shell (defined below the FSVR_CONTINUITY_PURE_ONLY
// guard in the .cpp).
// ---------------------------------------------------------------------------

class FSVRContinuity
{
public:
    // The per-cycle service, called by the C1 glue at the
    // head of every adapter cycle (before the plan input is
    // assembled, so the stamp this cycle's predicate reads
    // already reflects this cycle's finalizations): first
    // samples the manager's incarnation (a session boundary
    // calls C5's local-transition invalidation entry point
    // exactly once, before anything else is serviced --
    // C8-CODE-R1-02), then drains the manager's finalized
    // batches and routes each one (a Suspend routing calls
    // C5's local-transition invalidation entry point exactly
    // once), and samples the self avatar's skeleton rebuild
    // generation (a change calls C5's skeleton-reinit entry
    // point exactly once). Idle cost: one incarnation read,
    // one empty drain and one integer load + compare.
    static void serviceCycleSignals();

    // The router's current continuity epoch (CP-3's stamp
    // source, read by the glue at input assembly).
    static U64 continuityEpoch();
};

#endif // LL_LLVRCONTINUITY_H
