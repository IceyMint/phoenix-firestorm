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

// P2-07B C6-E (CP-3/CP-4): C5's session-neutral view, defined in
// llvrcalibration.h. Forward-declared here so the CP-4
// composition helper's declaration names it without coupling
// this header to the calibration TU; the definition site and
// every caller see the complete type.
struct FSVRSessionNeutralView;

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

// P2-07B C6-E (CP-2; Package B spec section 7.2):
// spine_sample_valid is exactly the conjunction
// E5 ^ E6 ^ E7 ^ E8 ^ !E9 over these inputs. E1-E4 ride their
// own gates in computePlan and are NOT inputs here. Pure data;
// the glue supplies the cycle's values from the session
// machinery (CP-3), the manager (E7), and C5's transaction
// state (E9).
struct FSVRSpineSampleInputs
{
    bool scale_input_valid = false;      // E5: scale-input
        // validity (P2-07B Stage 1, CP-S1-4): the C5 view's
        // scale source is not NONE (a MEASURED_RECORD view's
        // record was validated USABLE_* at its producers; a
        // PROVISIONAL_SYNC view's scale is the baseline
        // constant) AND the settings snapshot's lean gain is
        // valid (bounded domain, CP-S1-3). Renamed from
        // record_usable_for_head: the redesign falsified the
        // old name -- a sync-established neutral carries no
        // usable record at all.
    bool neutral_established = false;    // E6: the C5 view's
        // established flag, post any C8 invalidation
    bool hmd_position_valid = false;     // E7: the manager
        // accessor's three-term predicate, this cycle (the
        // bool getHmdPosition returns)
    bool sitting_signals_clear = false;  // E8: the section
        // 4.2 step-3 signals read not-sitting, this cycle
        // (the C5 view's per-cycle fact, ICR-2)
    bool calibration_in_progress = false; // E9:
        // FSVRCalibration::transactionActive(), read at
        // population (negated in the conjunction)
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
    // The legacy all-bones runtime-effective form (C4's
    // derivation over the published set: non-empty and no bone
    // shadow-only). RETAINED AS A RECORDED FACT ONLY (P2-07B
    // C6-E, CP-5 part 3): traces and H1 fixtures read it, but
    // it no longer gates -- gate 1 and gate 4 consume the
    // per-domain scoped fields below, because a domain's
    // shadow state must never block the other domain.
    bool decisions_runtime_effective = false;
    // P2-07B C6-E (CP-5 part 3): the scoped gate-1 facts, one
    // per solver domain, derived by the glue from the
    // arbitrator's emitted shadow_only form (the pure
    // derivations in llvrtrackingarbitrator.h). Gate 1 passes
    // when either domain's fact holds; gate 4 additionally
    // requires each domain's own fact for that domain's
    // authority, so no authority ever forms on shadow
    // decisions for its domain.
    bool spine_decisions_runtime_effective = false;
    bool arm_decisions_runtime_effective = false;

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

    // C6/C7 positional targets and solver inputs (brief CP-2;
    // additions only -- no existing field changes meaning).
    // P2-07B C6-E (CP-2/CP-4): the production glue now
    // populates spine_sample_valid from the E5-E9 conjunction
    // and composes the head-target block per cycle; the wrist
    // block stays written-false (Package B spec section 7.4).
    // Production remains inert until an authorization build
    // (CP-5) lets gate 1 pass; harnesses populate all of it.

    // FSVRDiagSoftPreference pass-through (D3): the C6 soft term
    // is read from here (the snapshot via the glue, CP-7).
    bool diag_soft_preference = true;
    // Continuity identity (CP-7/DP-4): the settings snapshot
    // generation these inputs were assembled under. A change
    // retires the caller's continuity history (previous targets
    // and pole state) before the next apply.
    U64 settings_generation = 0;

    // P2-07B C8 (CP-3): the session continuity epoch these
    // inputs were assembled under -- C8's dedicated counter,
    // advanced by a finalized LOCAL transition or a skeleton
    // reinitialization since the previous apply. A change
    // retires the caller's continuity history exactly as a
    // settings-generation change does (Package B spec section
    // 6.3): those two events produce neither a frame-token
    // gap nor a generation change, so without this stamp both
    // histories would carry across them. Supplied by the glue
    // from FSVRContinuity at assembly; a stamp, not a callback.
    U64 continuity_epoch = 0;

    // C6's target: the cornea-midpoint world position (already
    // composed and ratio-scaled by the CP-3 seam's caller;
    // Packages B/C supply it in production), with the
    // skeleton-derived cornea offset in the mHead local frame.
    // head_target_valid == false is "no constraint": C6 returns
    // an empty map.
    bool head_target_valid = false;
    LLVector3 head_target_world = LLVector3::zero;
    LLVector3 cornea_offset_head_local = LLVector3::zero;

    // C7's targets, per side (index 0 = LEFT, 1 = RIGHT):
    // wrist world positions, already composed (CP-3's
    // provisional seam in Package A). wrist_target_valid[side]
    // == false is "no constraint" for that side.
    bool wrist_target_valid[2] = {false, false};
    LLVector3 wrist_target_world[2] = {LLVector3::zero,
                                       LLVector3::zero};

    // Caller-owned pole history, per side [R2 -- C6C7-03
    // transport]: the pole direction (world frame) the caller
    // adopted from the last committed apply, carried beside
    // previous_targets under the same lifecycle (retired on
    // authority lapse, frame-token discontinuity, or settings
    // generation change; LOCAL-transition and skeleton
    // invalidation are C8's signals, brief OQ-3). C7 reads pole
    // history only from the solver context's copies of these
    // fields; it stores none itself.
    bool previous_pole_valid[2] = {false, false};
    LLVector3 previous_pole_world[2] = {LLVector3::zero,
                                        LLVector3::zero};

    // P2-07B C5: an active calibration transaction (Package B
    // spec section 3.1). Set only by the glue from C5's
    // transaction state; the apply's calibration-hold branch
    // keys on this field alone. Independent of gates 1-5 by
    // design (spec section 3.1 authorization contract).
    bool calibration_hold_active = false;
};

// Read-only view handed to the solvers: the plan input plus the
// base (arbitrated) pose composed over the root frame.
struct FSVRAdapterSolverContext
{
    const FSVRAdapterPlanInput* plan = nullptr;
    std::vector<LLQuaternion> base_world_rotations;
    std::vector<LLVector3> base_world_positions;
    std::map<S32, LLQuaternion> previous_targets;
    // The plan input's caller-owned pole history, copied in by
    // the transaction beside previous_targets [R2 -- C6C7-03
    // transport]. Indexed 0 = LEFT, 1 = RIGHT as on the plan.
    bool previous_pole_valid[2] = {false, false};
    LLVector3 previous_pole_world[2] = {LLVector3::zero,
                                        LLVector3::zero};
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
    // The pole the solver actually selected this call [R2 --
    // C6C7-03 transport]: has_pole is set ONLY when the RP3
    // procedure ran and selected a pole; the aim/flex modes and
    // the no-target return leave it false. pole_world is a world
    // direction. The transaction forwards it to the side's arm
    // outcome only after the side has been accepted (post-clamp
    // and commit processing complete), so a discarded side
    // exposes no provisional pole.
    bool has_pole = false;
    LLVector3 pole_world = LLVector3::zero;
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
    // The pole the caller adopts as this side's history for the
    // next cycle [R2 -- C6C7-03 transport]: pole_valid is set
    // ONLY when the side ran, was not aborted, its post-clamp
    // ran, and its result was committed -- the assignment
    // happens at the end of the side's transaction processing,
    // never mechanically beside the solver's return. An aborted
    // or discarded side exposes no provisional pole.
    bool pole_valid = false;
    LLVector3 pole_world = LLVector3::zero;
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

// P2-07B C6-E (CP-2): the E5-E9 conjunction as a named pure
// function, so H10 tests the production composition, never a
// reimplementation (Package B spec section 9). Defined above
// the plan-only line in llvradapterapply.cpp with the other
// extracted plan helpers.
// == in.scale_input_valid && in.neutral_established
//  && in.hmd_position_valid && in.sitting_signals_clear
//  && !in.calibration_in_progress
bool decideSpineSampleValid(const FSVRSpineSampleInputs& in);

// P2-07B C6-E (CP-4): the per-cycle head-target composition
// (Package B spec section 7.3), pure over its named inputs.
// Returns false -- and composes nothing -- unless every input
// the composition needs is present for this cycle (the
// session neutral established). On true, out_head_target_world
// is C6's shipped composeHeadTarget applied to the neutral
// cornea re-expressed in the CURRENT root frame plus the HMD
// displacement from the session anchor mapped through the
// rotation path's vector form,
//   d_world = (P(d_local) * q_yaw) * root_world_rotation,
// P(dx, dy, dz) = (-dz, -dx, dy) (brief section 2.13), and
// out_cornea_offset_head_local is the session's section 4.6
// value verbatim. Defined beside decideSpineSampleValid.
// P2-07B Stage 1 (CP-S1-3): lean_gain is the settings
// snapshot's provisional lean gain, consumed only when the
// session's scale source is PROVISIONAL_SYNC -- it
// multiplies the mapped displacement before the solver,
// and the composition's ratio argument is selected by the
// scale source (the record's head_ratio for MEASURED_RECORD,
// the provisional baseline constant otherwise).
// FSVRDiagForceUnitScale keeps its shipped meaning (it
// forces the *calibration* ratio to 1.0); the gain is a
// user preference and survives the diagnostic.
bool composeHeadTargetBlock(
    const FSVRSessionNeutralView& session,
    const LLVector3& hmd_position_local,
    F32 session_base_yaw_rad,
    const LLQuaternion& root_world_rotation,
    const LLVector3& root_world_position,
    bool diag_force_unit_scale,
    F32 lean_gain,
    LLVector3& out_head_target_world,
    LLVector3& out_cornea_offset_head_local);

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

    // P2-07B C5 (CP-5): the calibration hold set -- spec
    // section 3.1's in-scope joints by skeleton joint name
    // (the D7 spine chain, mNeck, mHead, and per side
    // collar/shoulder/elbow/wrist; fingers are not held).
    // The canonical list lives in this TU so the plan's
    // hold branch and C5's transaction machinery
    // (FSVRCalibration::holdJointNames forwards here) can
    // never drift, and so the plan keeps its one-directional
    // dependency shape (C5 depends on the plan, never the
    // reverse). Any addition is a spec amendment.
    static const std::vector<std::string>& calibrationHoldJointNames();

    // P2-07B C8 (CP-3): the glue's continuity predicate,
    // extracted verbatim from FSVRAdapterApplyForCharacter so
    // H8 fixtures the production expression (the house pattern:
    // C4's couplingWouldFire, C5's exposed helpers). Continuity
    // carries iff a previous apply exists, the frame token is
    // its immediate successor, and BOTH the settings generation
    // and the continuity epoch are unchanged since it. The
    // epoch term is C8's addition: a mismatch retires both
    // histories pre-transaction, through the glue's existing
    // whole-history clear, whether or not a solver is invoked.
    // P2-07B Stage 1 (CP-S1-2, S1-BRIEF-R1-01): the
    // installation-generation term is Stage 1's addition,
    // on the same terms -- the carried history's install
    // generation (the session view's, recorded when the
    // history was stored) must equal the current view's
    // install generation. A resync replaces the neutral the
    // history was produced against, so unchanged token,
    // settings generation, and epoch do NOT make old-anchor
    // history a valid continuity witness under a new one.
    // C8's epochs remain independently authoritative; this
    // is an additional necessary condition, never a
    // replacement.
    static bool continuityCarries(bool have_previous,
                                  U64 frame_token, U64 previous_token,
                                  U64 settings_generation,
                                  U64 previous_settings_generation,
                                  U64 continuity_epoch,
                                  U64 previous_continuity_epoch,
                                  U64 install_generation,
                                  U64 previous_install_generation);

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
