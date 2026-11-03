/**
 * @file llvradapterapply.h
 * @brief P2-07B C1: Stage-2 adapter apply point (implementation spec
 *        section 1/C1 and section 7/B3; brief
 *        P2-07B-C1-IMPLEMENTATION-BRIEF, SHIP round 3).
 *
 * C1 is the single post-blend write site for C6/C7 solver outputs.
 * A guarded hook inside LLMotionController::updateMotions,
 * immediately after mPoseBlender::blendAndApply(), calls
 * LLCharacter::applyVRAdapterPostBlend() (DP-1 [R3]: an
 * LLCharacter virtual with an empty default, overridden by
 * LLVOAvatarSelf). The override forwards to the glue at the bottom
 * of llvradapterapply.cpp, which assembles plan inputs from the
 * live coordinator decisions, settings snapshot and skeleton, runs
 * the plan, and performs the committed writes.
 *
 * Factoring (brief section 4): the gating and write-plan
 * computation is a pure function over plain inputs
 * (FSVRAdapterApply::computePlan), factored exactly as the Stage-1
 * arbitrator is pure, so the standalone H1 harness can exercise it
 * without an avatar. The glue is excluded from harness builds by
 * defining FSVR_ADAPTER_PLAN_ONLY.
 *
 * Package A state (brief OQ-6/OQ-7): C4 has not landed the Stage-2
 * snapshot fields or the runtime-effective decision form, and C3
 * has not registered the spine/arm bones, so the glue is inert by
 * construction: gate 1 rejects the shadow-only Stage-1 decisions,
 * and every registry entry's constraint_adapter_id is empty, which
 * maps to no solver domain. Non-VR and runtime behavior are
 * unchanged (INV-12).
 */

#ifndef LL_LLVRADAPTERAPPLY_H
#define LL_LLVRADAPTERAPPLY_H

#include "llmath.h"

#include <map>
#include <string>
#include <vector>

// Which solver domain a joint belongs to. NONE joints are never
// written by C1: the head and neck (brief section 3.5 [R1]: C1
// never writes mHead or mNeck in Package A), the collars (never
// written by the positional solve, spec section 6.2), the pelvis
// root, and structural ancestors pulled into the plan tree.
enum class FSVRAdapterDomain
{
    NONE,
    SPINE,
    ARM_LEFT,
    ARM_RIGHT
};

enum class FSVRAdapterSide
{
    LEFT,
    RIGHT
};

// Motion-cycle classification, derived by the glue from production
// state (motion-controller paused flag, Animation Preview render
// mode). The [R1] H1 cases feed exactly these values: the gate set
// consumes the classification, it does not assume it.
enum class FSVRAdapterCycleKind
{
    NORMAL,         // ordinary motion cycle; the blend ran
    PAUSED_NORMAL,  // paused controller, ordinary update (no blend;
                    // structurally unreachable from the hook)
    PAUSED_FORCE,   // paused controller, FORCE_UPDATE (blend ran)
    PREVIEW,        // Animation Preview cycle (blend ran)
    HIDDEN          // hidden update (no blend; never reaches the hook)
};

// First failing gate, in brief section 3.3 order (the cycle and
// context checks precede gate 1). C9 diagnostics trace this value.
enum class FSVRAdapterGate
{
    NONE = 0,
    CYCLE,              // no blend this cycle, or a paused / preview /
                        // hidden cycle kind
    CONTEXT,            // bridge not armed (non-self avatar)
    FRAME_IDENTITY,     // gate 1: decisions absent, stale, or not
                        // runtime-effective (shadow-only)
    VR_SESSION,         // gate 2: vr_active false (headset off)
    POSITIONAL_MASTER,  // gate 3: FSVRDiagPositionalSolve false
    NO_SOLVER,          // gate 4 outcome: neither solver holds
                        // constraint authority this cycle
    TOKEN               // apply-once wrapper: frame token already
                        // processed
};

// One joint of the plan tree. The list is topologically ordered
// (parents before children) and forms a single tree under the plan
// root frame, so composition always resolves a parent before its
// children (brief section 3.4 steps 3/6).
struct FSVRAdapterJointInput
{
    std::string name;
    // Index into the plan input's joint list; -1 means the joint's
    // parent is the plan root frame.
    S32 parent_index = -1;
    // Local position in the parent frame (the skeleton XML pos).
    // Composition goes through these real offsets, never a
    // stacked-lever approximation (C0 section 2 pivot zigzag).
    LLVector3 rest_offset = LLVector3::zero;
    // The post-blend local rotation: the arbitrated pose for this
    // cycle (brief section 3.4 step 1).
    LLQuaternion base_local;
    FSVRAdapterDomain domain = FSVRAdapterDomain::NONE;
    bool in_registry = false;
    // Arbitration grants this joint a tracking base (brief gate 5).
    bool decision_grants_base = false;
    // The joint is in the snapshot's disabled-bone set.
    bool disabled = false;
};

struct FSVRAdapterPlanInput
{
    U64 frame_token = 0;

    FSVRAdapterCycleKind cycle = FSVRAdapterCycleKind::NORMAL;
    bool blend_ran = false;
    // Self avatar with the bridge armed (DP-1). Non-self characters
    // never reach the glue; the plan still checks this so the
    // harness can exercise the non-self case.
    bool context_armed = false;

    // Gate 1 (frame identity).
    bool decisions_published = false;
    bool decisions_current = false;
    // C4's runtime-effective decision form. False while Stage-1
    // shadow-only decisions are what the coordinator publishes.
    bool decisions_runtime_effective = false;

    // Gate 2 (session gate only [R1]): the headset/session state.
    // Never derived from any bone's source decision (amendment 2).
    bool vr_active = false;

    // Gate 3: FSVRDiagPositionalSolve (spec section 3.4). False
    // suspends both positional solves at the apply point; the
    // orientation path is not C1's and is unaffected.
    bool positional_solve_enabled = false;

    // Gate 4 (per-solver [R1]): enablement and positional sample
    // validity are evaluated separately for each solver. Constraint
    // authority never depends on a bone holding a TRACKED source
    // decision (spec section 1, R2-01).
    bool head_position_enabled = false; // FSVRTrackHeadPosition
    bool hand_position_enabled = false; // FSVRTrackHandPosition
    bool spine_sample_valid = false;
    bool arm_sample_valid = false;

    // L4 (FSVRSolverBendDisabledBones, spec section 6.1): the
    // solver may bend disabled SPINE bones. Never extends to arms
    // (D4).
    bool bend_disabled_bones = false;

    // Diagnostic pass-throughs the solvers read from the context.
    // C1 never interprets them: FSVRDiagBendLimits=false suspends
    // the section 6.3 clamp INSIDE the solvers, and C1 must not
    // treat an unclamped result as limit-satisfying (spec section
    // 3.4 safety boundary).
    bool diag_bend_limits = true;
    bool diag_full_chain_solve = true;

    // The transaction root frame: the world rotation/position of
    // the parent of every parent_index == -1 joint (the pelvis
    // frame for the torso/arm chains). Solver target worlds are
    // expressed in this frame's world space.
    //
    // PRE-RUNTIME-ENABLE VERIFICATION OBLIGATION (recorded, code
    // cross-check round 1): the glue sources this frame from the
    // root joint's cached world values at apply time. Before any
    // runtime writes are enabled (C6/C7 integration), the root
    // frame must be demonstrated invariant or current for the
    // cycle it anchors -- the adapter's own composition never
    // reads a cached parent world for an adapter-owned joint
    // (brief section 2.6), and the root frame may not become the
    // one stale read that survives. Package A produces no writes,
    // so the obligation is discharged by recording it here and at
    // the glue's sourcing site; it rides with C6/C7 integration.
    LLQuaternion root_world_rotation;
    LLVector3 root_world_position = LLVector3::zero;

    std::vector<FSVRAdapterJointInput> joints;

    // The previous valid solved targets, an explicit input to the
    // solvers' continuity term (brief section 3.6). The plan holds
    // no cross-cycle pose state of its own; this map is data the
    // caller supplies, never a C1-accumulated baseline (H1
    // no-drift check).
    std::map<S32, LLQuaternion> previous_targets;
};

// Read-only view handed to the solvers: the plan input plus the
// base (arbitrated) pose composed over the root frame.
struct FSVRAdapterSolverContext
{
    const FSVRAdapterPlanInput* plan = nullptr;
    std::vector<LLQuaternion> base_world_rotations;
    std::vector<LLVector3> base_world_positions;
    std::map<S32, LLQuaternion> previous_targets;
};

// The solver boundary is the target-world list (brief section 5):
// solvers return desired WORLD orientations keyed by joint index;
// an absent index is "no constraint" for that joint. Solvers write
// nothing themselves; C1 owns the final committed output (OQ-2
// [R1] terminology). Implementations apply the C0 limit rules to
// their own outputs (brief section 3.4): C6 uses C0 section 5.1;
// C7 uses C0 sections 5.2/5.3 after its RP3 pass, with RP3's
// internal writes provisional under the OQ-2 [R1] containment
// conditions.
class FSVRAdapterSpineSolver
{
public:
    virtual ~FSVRAdapterSpineSolver() = default;
    virtual void solveSpine(const FSVRAdapterSolverContext& context,
                            std::map<S32, LLQuaternion>& out_target_worlds) = 0;
};

class FSVRAdapterArmSolver
{
public:
    virtual ~FSVRAdapterArmSolver() = default;
    // shoulder_world_* are resolved from the solved spine before
    // this call (brief section 3.4 step 4; D6: hand targets never
    // re-open the spine solve).
    virtual void solveArm(const FSVRAdapterSolverContext& context,
                          FSVRAdapterSide side,
                          const LLQuaternion& shoulder_world_rotation,
                          const LLVector3& shoulder_world_position,
                          std::map<S32, LLQuaternion>& out_target_worlds) = 0;
};

// Package A runtime solvers: no constraint, ever. The real C6/C7
// solvers land with their own components; until then even a fully
// gated apply computes an empty write list.
class FSVRAdapterNullSpineSolver final : public FSVRAdapterSpineSolver
{
public:
    void solveSpine(const FSVRAdapterSolverContext& /*context*/,
                    std::map<S32, LLQuaternion>& /*out_target_worlds*/) override {}
};

class FSVRAdapterNullArmSolver final : public FSVRAdapterArmSolver
{
public:
    void solveArm(const FSVRAdapterSolverContext& /*context*/,
                  FSVRAdapterSide /*side*/,
                  const LLQuaternion& /*shoulder_world_rotation*/,
                  const LLVector3& /*shoulder_world_position*/,
                  std::map<S32, LLQuaternion>& /*out_target_worlds*/) override {}
};

// ---------------------------------------------------------------------------
// The C7/RP3 write-through boundary (brief section 5, OQ-2 [R1];
// code cross-check round-1 amendment).
//
// RP3 does not fit the target-world boundary above: LLJointSolverRP3
// returns nothing -- it WRITES rotations (lljointsolverrp3.cpp:390-391),
// and its setWorldRotation path derives locals from the parent's
// CACHED world matrix (lljoint.cpp:834), which inside the apply is
// the previous cycle's (brief section 2.6). The brief resolved OQ-2
// by containment, not by a compute-only redesign of RP3: RP3's
// writes are provisional solver operations inside C1's synchronous
// transaction (conditions at brief section 3.4 step 5 [R1]). This
// boundary is the mechanism for that resolution; a future C7 built
// on RP3 implements THIS interface for its RP3 pass, and may not
// claim the brief by implementing the plain target-world interface
// alone.
//
// The contract, as obligations of the boundary:
// (a) PROVISIONAL ONLY. A solver operating through the host writes
//     provisional locals. Nothing it writes is a committed output;
//     C1 owns the final committed output (OQ-2 [R1] terminology).
// (b) ALWAYS-CURRENT PARENT WORLDS (containment condition (i)).
//     The host keeps no cached worlds across a provisional write:
//     every world read is composed from the current provisional
//     locals, so a parent-world read after a provisional
//     parent-changing write necessarily reflects it. This is the
//     refresh condition discharged by construction rather than by
//     refresh bookkeeping -- there is no cache that could go stale.
// (c) NON-OBSERVABILITY / ATOMICITY (containment condition (ii)).
//     Provisional state lives only inside the synchronous
//     transaction. If the solver does not complete, the transaction
//     discards the provisional state for that side entire: the
//     side's joints keep the values the transaction had already
//     resolved for them, and no provisional local reaches the
//     commit. In the viewer this is structural -- the glue performs
//     committed writes only from the finished plan result, after
//     updateMotions' blend, in the same synchronous call -- and at
//     plan level it is the discard rule above.
// (d) READBACK, POST-CLAMP, COMMIT. On completion the transaction
//     reads the provisional locals back, applies the arm post-clamp
//     (C0 sections 5.2/5.3, via FSVRAdapterArmPostClamp) to the
//     readback, recomputes the wrist-position error from the FINAL
//     clamped rotations (the values C1 commits; brief section 3.4,
//     H7 extension), and commits through the ordinary gate-5
//     eligibility filter. A readback local for an ineligible joint
//     is dropped, exactly as a rogue target is.
//     The post-clamp is MANDATORY for a write-through completion
//     (code cross-check round 2, R2-01): a write-through solver
//     supplied without a post-clamp fails closed -- that side's
//     write-through result is aborted before the solver runs, the
//     side keeps the pose the transaction already resolved for it,
//     and the side's outcome names the missing clamp
//     (FSVRAdapterArmAbortReason::MISSING_POST_CLAMP). There is no
//     unclamped write-through transaction.
//     Gate-5 rejection CASCADES over the readback (round 2,
//     R2-02): a provisionally written joint's readback local was
//     computed against its provisional parents, so if a written
//     joint is rejected at gate 5, every provisionally written
//     descendant of it in the same pass is rejected too -- its
//     readback depended on a parent the commit did not take.
//     Rejected joints keep the locals the transaction resolved
//     for them before the arm phase.
// ---------------------------------------------------------------------------
class FSVRAdapterWriteThroughHost
{
public:
    virtual ~FSVRAdapterWriteThroughHost() = default;

    // Provisional write of one joint local. Only the joints of the
    // side's arm chain (the domain the host was opened for) may be
    // written; any other joint -- another chain's, a collar's, the
    // head's or neck's -- is rejected (returns false) and nothing
    // is recorded. A provisional write is never a committed output
    // (contract point (a)).
    virtual bool writeProvisionalLocal(S32 joint_index,
                                       const LLQuaternion& local) = 0;
    // The current provisional local of a joint (its transaction
    // value if written, the value the transaction resolved for it
    // at host opening otherwise).
    virtual bool readProvisionalLocal(S32 joint_index,
                                      LLQuaternion& out_local) const = 0;
    // World rotation/position composed from the CURRENT provisional
    // state (contract point (b)).
    virtual bool readWorldRotation(S32 joint_index,
                                   LLQuaternion& out_rotation) const = 0;
    virtual bool readWorldPosition(S32 joint_index,
                                   LLVector3& out_position) const = 0;
};

// What a write-through solve reports back to the transaction.
struct FSVRAdapterWriteThroughOutcome
{
    // False aborts the transaction for this side: the provisional
    // state is discarded entire (contract point (c)).
    bool completed = false;
    // The wrist world position the solve aimed at, for the
    // post-clamp wrist-error recompute (contract point (d)).
    bool has_wrist_target = false;
    LLVector3 wrist_target_world = LLVector3::zero;
};

// A write-through arm solver: C7's RP3 pass, when C7 lands. Runs
// INSIDE the transaction (boundary contract above), after the
// shoulders have been resolved from the solved spine; it operates
// on the side's chain through the host and reports an outcome.
// It never re-opens the spine (D6).
class FSVRAdapterWriteThroughArmSolver
{
public:
    virtual ~FSVRAdapterWriteThroughArmSolver() = default;
    virtual FSVRAdapterWriteThroughOutcome solveArmWriteThrough(
        const FSVRAdapterSolverContext& context,
        FSVRAdapterSide side,
        FSVRAdapterWriteThroughHost& host) = 0;
};

// The arm post-clamp (C0 sections 5.2/5.3), invoked BY the
// transaction on the write-through readback, before commit (brief
// section 3.4: C1's ordering duty is that the post-RP3 clamp runs
// before the arm locals commit). The clamp implementation itself
// is C7's (the brief: C6 and C7 call the C0 limiter inside their
// own compute); this interface is where C7's clamp plugs into the
// transaction so the ordering is the transaction's guarantee, not
// the solver's promise. It receives the readback locals and the
// worlds composed from them, and edits the readback locals in
// place; the transaction recomposes from the edited locals.
// A write-through transaction without a clamp does not run at
// all: the clamp is mandatory, not optional (contract point (d),
// R2-01 fail-closed rule).
class FSVRAdapterArmPostClamp
{
public:
    virtual ~FSVRAdapterArmPostClamp() = default;
    virtual void clampArm(
        const FSVRAdapterSolverContext& context,
        FSVRAdapterSide side,
        const std::map<S32, LLQuaternion>& readback_world_rotations,
        const std::map<S32, LLVector3>& readback_world_positions,
        std::map<S32, LLQuaternion>& in_out_readback_locals) = 0;
};

struct FSVRAdapterWrite
{
    S32 joint_index = -1;
    LLQuaternion local; // the committed local rotation
};

// Why a side's write-through result was aborted (boundary
// contract points (c)/(d)). Surfaced on the plan result -- the
// plan layer's diagnostic surface, traced the same way
// FSVRAdapterGate is (C9) -- and named by
// FSVRAdapterApply::armAbortReasonName.
enum class FSVRAdapterArmAbortReason
{
    NONE = 0,
    SOLVER_INCOMPLETE,  // the solver did not complete; its
                        // provisional state was discarded entire
    MISSING_POST_CLAMP  // a write-through solver was supplied
                        // without the mandatory post-clamp; the
                        // side failed closed before the solver ran
                        // (R2-01)
};

// Per-side write-through outcome (boundary contract point (d)).
// wrist_error is the distance between the wrist world position
// composed from the final clamped rotations C1 committed and the
// target the solver reported -- the accepted error of the clamp
// (spec section 6.2/6.3: constraints win). The wrist is the
// structural joint whose parent is the side's elbow; when the
// chain carries no wrist joint, or the solver reported no target,
// wrist_error_valid is false.
struct FSVRAdapterArmOutcome
{
    bool write_through_ran = false;
    bool write_through_aborted = false;
    FSVRAdapterArmAbortReason abort_reason = FSVRAdapterArmAbortReason::NONE;
    bool wrist_error_valid = false;
    F32 wrist_error = 0.f;
};

struct FSVRAdapterPlanResult
{
    // True when the transaction ran (every gate passed), even if
    // the solvers returned no constraints and nothing was written.
    bool applied = false;
    FSVRAdapterGate first_failing_gate = FSVRAdapterGate::NONE;
    bool spine_solver_invoked = false;
    bool arm_solver_invoked_left = false;
    bool arm_solver_invoked_right = false;
    // The committed writes, in D6 order (spine pass, then arms).
    std::vector<FSVRAdapterWrite> writes;
    // Per input joint: the final local rotation (the write local
    // where written, the base local otherwise) and the composed
    // world rotation/position after the transaction.
    std::vector<LLQuaternion> resolved_locals;
    std::vector<LLQuaternion> resolved_world_rotations;
    std::vector<LLVector3> resolved_world_positions;
    // Write-through boundary outcomes, per side (zero when the
    // boundary did not run for that side).
    FSVRAdapterArmOutcome arm_outcome_left;
    FSVRAdapterArmOutcome arm_outcome_right;
};

class FSVRAdapterApply
{
public:
    // The production cycle classification, factored out of the
    // glue so the H1 harness covers the mapping itself and not
    // only the classification values the gates consume: the
    // Animation Preview flag dominates; a paused controller
    // reaching the post-blend hook got there through FORCE_UPDATE.
    // (A paused controller's ordinary update, and hidden updates,
    // never reach the hook at all; their kinds exist for the
    // gate's defense in depth and are fed by the harness only.)
    static FSVRAdapterCycleKind classifyCycle(bool preview_cycle,
                                              bool controller_paused);

    // The pure plan (brief section 4). No globals, no cross-cycle
    // pose state: identical inputs give identical outputs.
    static FSVRAdapterPlanResult computePlan(
        const FSVRAdapterPlanInput& input,
        FSVRAdapterSpineSolver& spine_solver,
        FSVRAdapterArmSolver& arm_solver);

    // The plan with the C7/RP3 write-through boundary (above).
    // When write_through_solver is non-null it takes the arm phase
    // for both sides (the plain arm solver is not invoked), and
    // post_clamp is then MANDATORY: a null post_clamp fails each
    // side closed (contract point (d), R2-01) -- the solver is not
    // run, nothing from the side reaches the commit, and the
    // side's outcome names MISSING_POST_CLAMP. There is no
    // unclamped write-through path, in fixtures or otherwise.
    // With a null write_through_solver this is exactly the
    // three-argument plan.
    static FSVRAdapterPlanResult computePlan(
        const FSVRAdapterPlanInput& input,
        FSVRAdapterSpineSolver& spine_solver,
        FSVRAdapterArmSolver& arm_solver,
        FSVRAdapterWriteThroughArmSolver* write_through_solver,
        FSVRAdapterArmPostClamp* post_clamp);

    // Brief section 3.3 gate 5, exposed for the harness. A joint is
    // eligible when it is registered, belongs to a solver domain,
    // and either arbitration grants it a tracking base or the
    // domain solver's gate-4 constraint authority covers it; a
    // disabled joint is eligible only under the spine-only L4
    // exception.
    static bool isJointEligible(const FSVRAdapterPlanInput& input,
                                const FSVRAdapterJointInput& joint,
                                bool spine_authority,
                                bool arm_authority);

    static const char* gateName(FSVRAdapterGate gate);
    static const char* armAbortReasonName(FSVRAdapterArmAbortReason reason);
};

// Token-once wrapper used by the viewer glue: each frame token is
// processed at most once (the "exactly once per motion cycle"
// mechanism, brief section 4), and a repeat returns a TOKEN no-op
// without invoking the solvers. Holds frame identity only, never
// pose state (brief section 3.6).
//
// INTEGRATION CHECK RECORDED (code cross-check round 1): the token
// is the coordinator's app_frame_token, not an independently
// generated motion-cycle identifier. Before runtime writes are
// enabled (C6/C7 integration), verify that an app frame corresponds
// 1:1 to self-avatar motion cycles on this path, or supply an
// independent motion-cycle id as the token.
class FSVRAdapterApplyOnce
{
public:
    FSVRAdapterPlanResult apply(const FSVRAdapterPlanInput& input,
                                FSVRAdapterSpineSolver& spine_solver,
                                FSVRAdapterArmSolver& arm_solver);
    // As apply(), with the write-through boundary (see the
    // five-argument computePlan). Token-once suppresses the
    // write-through solver exactly as it suppresses the others.
    FSVRAdapterPlanResult apply(const FSVRAdapterPlanInput& input,
                                FSVRAdapterSpineSolver& spine_solver,
                                FSVRAdapterArmSolver& arm_solver,
                                FSVRAdapterWriteThroughArmSolver* write_through_solver,
                                FSVRAdapterArmPostClamp* post_clamp);
    // Number of transactions that ran (diagnostics).
    U32 applyCount() const { return mApplyCount; }

private:
    bool mHasToken = false;
    U64 mLastToken = 0;
    U32 mApplyCount = 0;
};

class LLCharacter;

// Viewer glue (implementation at the bottom of
// llvradapterapply.cpp; excluded from FSVR_ADAPTER_PLAN_ONLY
// harness builds). Called by LLVOAvatarSelf::applyVRAdapterPostBlend
// exactly once per post-blend motion cycle. preview_cycle is the
// Animation Preview state the override reads from the avatar.
void FSVRAdapterApplyForCharacter(LLCharacter* character, bool preview_cycle);

#endif // LL_LLVRADAPTERAPPLY_H
