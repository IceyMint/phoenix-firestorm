/**
 * @file llvrcalibration.h
 * @brief P2-07B C5: calibration command, transaction, and
 *        session-neutral acquisition (Package B spec sections
 *        4 and 5; brief P2-07B-C5-IMPLEMENTATION-BRIEF [R2],
 *        SHIP).
 *
 * C5 owns the calibration record's production: the /xr
 * calibrate command (CP-1, in chatbar_as_cmdline.cpp), the
 * section 4.2 transaction as an explicit state machine
 * (CP-2), the pure helpers the transaction and the section 5
 * acquisition share (CP-3), the hold coordination from C5's
 * side (CP-4, glue in llvradapterapply.cpp; the hold branch
 * itself is CP-5, inside computePlan), and the section 5
 * session-neutral acquisition machinery (CP-6). The record
 * is committed solely through C4's
 * FSVRTrackingSettings::writeCalibrationRecord() -- exactly
 * one whole-map write per successful transaction.
 *
 * Factoring (house pattern, C4/C6C7): the state machines and
 * pure helpers sit above the FSVR_CALIBRATION_PURE_ONLY
 * guard in the .cpp (no globals, harness-compilable); the
 * viewer shell (command entry, manager/avatar/eye-state
 * reads, chat reports, the store call) sits below it. H5
 * drives the machines with fixture input structs and counts
 * the emitted actions (report / commit / install-neutral).
 *
 * Interface additions folded in from the C6-E brief [R2]
 * (accepted as C5-owned): ICR-1 -- the session view carries
 * anchor_forward_yaw_rad, the stage-2-convention yaw of the
 * forward of the adopted sample whose window established
 * the anchor, written at establishment/install; ICR-2 --
 * the view carries sitting_signals_clear, the section 4.2
 * step-3 not-sitting fact evaluated per cycle at view
 * production (the shell refreshes it pre-assembly and in
 * the post-apply step hook on every glue cycle, which is
 * every cycle on which the view can be consumed by C6-E's
 * composition path -- C6E-CODE-R1-01).
 */

#ifndef LL_LLVRCALIBRATION_H
#define LL_LLVRCALIBRATION_H

#include "llmath.h"
#include "llsd.h"

#include "llvradapterapply.h"
#include "llvrtrackingsettings.h"
#include "v3dmath.h"

#include <string>
#include <vector>

// The calibration geometry the spec's harness tolerances
// (1e-9) are stated over uses the tree's own F64 vector,
// LLVector3d: LLVector3 is F32, and the landmark/ratio math
// is done in F64 throughout (F32 inputs widen exactly). The
// tree's operator*(LLVector3d, LLQuaternion) applies a
// rotation in F64, which CP-3g uses.

// ---------------------------------------------------------------------------
// CP-3a: stationarity (OPEN-6 machinery; spec section 4.2
// step 4, shared with section 5 step 2).
// ---------------------------------------------------------------------------

struct FSVRStationarityParams
{
    // DP-B4 outputs. Every field arrives from the measurement
    // protocol; none has a value in the brief or in the code
    // C5 lands (see FSVRCalibration::productionStationarityParams).
    U32 min_samples = 0;      // minimum sample count
    F64 min_duration_s = 0.0; // minimum observation duration
    F64 max_spread_norm = 0.0;  // maximum permitted spread (norm)
    F64 max_spread_axis = 0.0;  // maximum permitted spread (per-axis)
};

struct FSVRCalSample
{
    F64 time_s = 0.0; // monotonic seconds (shell-supplied)
    bool hmd_valid = false; // getHmdPosition's result
    LLVector3 hmd_pos;      // LOCAL, metres
    bool wrist_valid[2] = {false, false};
    LLVector3 wrist_pos[2]; // LOCAL, metres
    // C8 epoch changed within/at this sample.
    bool transition_marker = false;
};

enum class FSVRStationarityReason
{
    STATIONARY,
    MOVING,
    TRACKER_LOSS,
    TRANSITION,
    INSUFFICIENT
};

struct FSVRStationarityVerdict
{
    FSVRStationarityReason reason = FSVRStationarityReason::INSUFFICIENT;
    // Meaningful only when reason == STATIONARY:
    LLVector3 hmd_representative; // the window's anchor value
    bool side_measured[2] = {false, false};
    // Per side, simultaneous-frame (means of the side's valid
    // samples; distances are computed per sample, never from
    // these endpoints -- DP-C5-10).
    LLVector3 wrist_representative[2];
};

// ---------------------------------------------------------------------------
// CP-3c: ratio resolution and record construction (spec
// sections 4.5 / 4.3).
// ---------------------------------------------------------------------------

// The landmark set: plain values, assembled by the CP-2/CP-6
// shells (the transaction machine from the held cycle and
// the accepted window; fixtures directly).
struct FSVRCalLandmarkSet
{
    // Avatar side, agent frame, from the held pose (always
    // computable: the held wrists are read regardless of
    // player tracking, spec section 4.5 invariant 5).
    LLVector3d avatar_wrist_world[2]; // index 0 = LEFT, 1 = RIGHT
    LLVector3d cornea_midpoint_agent; // E_A (agent frame, as
                                      // read -- brief section
                                      // 2.4: no conversion)
    // h_A (spec section 4.5): E_A.z - F_A. Capture-side
    // documentation of the anchor correspondence; not stored,
    // not consumed by the ratio rules.
    F64 h_a = 0.0;
    // The avatar's head-frame right direction, unit, agent
    // frame (the helper's explicit basis input, DP-C5-9's
    // avatar-side counterpart).
    LLVector3d avatar_right;
    // Player side, from the accepted window: per-sample
    // simultaneous quantities (DP-C5-10), per side. A side
    // with an empty distance vector had no valid wrist
    // sample in the window.
    std::vector<F64> player_dist_samples[2];    // |wrist - HMD|
    std::vector<F64> player_lateral_samples[2]; // (wrist - HMD) . right
};

struct FSVRCalRatioResolution
{
    bool valid = false; // step-5 validation outcome
    bool head_only = false;      // arm fields absent
    bool ratio_measured = false; // L_P was measured this capture
    bool ratio_carried = false;  // [R3] carry-forward applied
    bool ratio_placeholder = false; // disclosed 1.0 placeholder
    F64 head_ratio = 0.0;
    F64 avatar_measure_LA = 0.0;       // stored avatar_cornea_height
    F64 player_measure_LP_stored = 0.0; // stored player_hmd_height
    // Arm fields, meaningful only when !head_only:
    F64 arm_ratio[2] = {0.0, 0.0};
    F64 avatar_reach[2] = {0.0, 0.0};
    F64 player_reach[2] = {0.0, 0.0};
};

// ---------------------------------------------------------------------------
// CP-3d/3e: held-pose verification (spec section 3.2).
// ---------------------------------------------------------------------------

struct FSVRPoseTolerance
{
    // False when the derivation itself failed (non-finite
    // measured base, or a tolerance that would exceed the
    // pose it is meant to verify): verification then fails
    // outright (brief CP-3d).
    bool valid = false;
    F64 orientation_rad = 0.0;
    F64 position_m = 0.0;
};

// ---------------------------------------------------------------------------
// CP-3f: appearance settle (DP-B9).
// ---------------------------------------------------------------------------

struct FSVRCalSettleObservation
{
    std::vector<LLVector3> hold_joint_offsets; // rest_offsets, this cycle
    F32 pelvis_to_foot = 0.f;
    bool cornea_valid = false;
    LLVector3 cornea_midpoint_agent; // E_A, this cycle
};

// ---------------------------------------------------------------------------
// CP-6: the session-neutral view (C6-E consumes this;
// DP-C5-11, plus ICR-1/ICR-2).
// ---------------------------------------------------------------------------

struct FSVRSessionNeutralView
{
    bool established = false;
    // Record basis (validator output; head fields valid for
    // USABLE_HEAD_ONLY and USABLE_FULL):
    FSVRCalibrationRecordValidation record;
    LLVector3 anchor_hmd_local; // current LOCAL space
    // ICR-1: the stage-2-convention yaw (atan2f(f.x, -f.z))
    // of the forward of the adopted sample whose window
    // established the anchor. Written at
    // establishment/install alongside the anchor; meaningless
    // when established is false.
    F32 anchor_forward_yaw_rad = 0.f;
    LLVector3 cornea_offset_head_local; // section 4.6 value
    LLVector3 neutral_cornea_root;      // root frame, fixed at
                                        // establishment
    // ICR-2: the section 4.2 step-3 not-sitting fact for the
    // cycle being populated, refreshed by the shell on every
    // stepped glue cycle. Not latched at acquisition.
    bool sitting_signals_clear = false;
};

// ---------------------------------------------------------------------------
// CP-2: the calibration transaction machine (pure).
// ---------------------------------------------------------------------------

// One stepped cycle's facts, assembled by the shell (glue
// step hook) or by H5 fixtures.
struct FSVRTransactionStepInputs
{
    F64 now_s = 0.0; // monotonic seconds (shell-supplied)
    S64 captured_utc = 0; // for the commit's record

    // The cycle's plan pair. The input's calibration flag was
    // populated from live transaction state at assembly; the
    // result is the pose the apply just produced.
    const FSVRAdapterPlanInput* plan_input = nullptr;
    const FSVRAdapterPlanResult* plan_result = nullptr;

    // Section 4.2 step-3 gate facts (current reads):
    bool sitting_on_object = false; // isSitting() && getParent()
    bool parented = false;          // getParent() != null
    bool ground_sit_active = false; // ANIM_AGENT_SIT_GROUND_CONSTRAINED
    bool in_air = false;            // mInAir
    bool hmd_up_valid = false;
    LLVector3 hmd_up; // LOCAL
    bool hmd_forward_valid = false;
    LLVector3 hmd_forward; // LOCAL
    bool hmd_position_valid = false; // getHmdPosition (three-term)
    LLVector3 hmd_position;          // LOCAL

    // Player-side sample facts (SAMPLING):
    bool wrist_valid[2] = {false, false}; // availability folded in
    LLVector3 wrist_position[2];          // LOCAL
    // The shell's player right basis for this cycle (DP-C5-9:
    // cross(forward, up) at the shell, one line); consumed at
    // window acceptance for the lateral reaches.
    bool player_right_valid = false;
    LLVector3 player_right; // LOCAL, unit

    // C8's LOCAL-transition epoch as observed this cycle.
    U64 local_transition_epoch = 0;

    // Landmark reads (settle observations every stepped
    // cycle; RESOLVE consumes the current cycle's):
    bool eye_valid = false; // left_valid || right_valid
    LLVector3 cornea_midpoint_agent; // E_A, as read
    F32 pelvis_to_foot = 0.f;
    F32 foot_plane_z = 0.f; // F_A (spec section 4.5 formula,
                            // evaluated by the shell)

    // The prior record's validation (read by the shell at
    // transaction entry; carry-forward's input).
    FSVRCalibrationRecordValidation prior_record;

    // The stationarity parameter source for this step (the
    // shell passes the production source's value; H5 passes
    // labeled synthetic sets).
    bool stationarity_installed = false;
    FSVRStationarityParams stationarity_params;
};

// An action the machine emits for the shell to execute. The
// shell executes a step's actions in order; the harness
// counts and inspects them (that is how "exactly one write"
// is fixtured without a settings store).
struct FSVRTransactionAction
{
    enum class Kind
    {
        REPORT,         // nearby-chat text (text)
        COMMIT,         // writeCalibrationRecord(record), once
        INSTALL_NEUTRAL // install the verified capture as the
                        // session neutral (payload below)
    };
    Kind kind = Kind::REPORT;
    std::string text; // REPORT
    LLSD record;      // COMMIT: the complete section 4.3 map
    // INSTALL_NEUTRAL payload:
    FSVRCalibrationRecordValidation installed_record;
    LLVector3 install_anchor_hmd_local;
    F32 install_anchor_forward_yaw_rad = 0.f;
    LLVector3 install_cornea_offset_head_local;
    LLVector3 install_neutral_cornea_root;
};

// The brief's state list. COMMIT and INSTALL name the two
// completion phases of DP-B11(a)'s ordering; since
// C5-CODE-R1-02 the machine performs both atomically at the
// end of the RESOLVE step (one ordered action batch, the
// transaction terminally DONE from that step) and never
// rests in COMMIT or INSTALL across stepped cycles -- a
// stepped gap there let an abort land between the record
// write and the neutral install.
enum class FSVRTransactionState
{
    IDLE,
    ENTERING,
    HOLDING,
    VERIFYING,
    GATES,
    SAMPLING,
    RESOLVE,
    COMMIT,
    INSTALL,
    DONE,
    ABORTED
};

class FSVRCalibrationTransaction
{
public:
    // Entry (the shell has already applied CP-1's refusals:
    // no nesting, live VR session). False if already active.
    bool begin();
    bool active() const
    {
        return mState != FSVRTransactionState::IDLE &&
               mState != FSVRTransactionState::DONE &&
               mState != FSVRTransactionState::ABORTED;
    }
    FSVRTransactionState state() const { return mState; }
    F64 lastSteppedSeconds() const { return mLastSteppedS; }

    // Advance one stepped cycle; returns the actions the
    // shell must execute, in order.
    std::vector<FSVRTransactionAction> step(
        const FSVRTransactionStepInputs& inputs);

    // Starvation (CP-4 item 3): if the transaction has not
    // stepped within the bound, end it; the returned actions
    // carry the abort report. Called by the shell's
    // preCycleCheck before the flag is populated for a
    // resuming cycle.
    std::vector<FSVRTransactionAction> checkStarvation(F64 now_s);

private:
    std::vector<FSVRTransactionAction> abortWith(const char* reason);
    void feedSettle(const FSVRTransactionStepInputs& inputs,
                    const FSVRAdapterPlanInput& plan_input);
    bool settleExpired() const;

    FSVRTransactionState mState = FSVRTransactionState::IDLE;
    F64 mLastSteppedS = 0.0;
    bool mHaveStepped = false;

    // Hold bookkeeping (resolved at the first held cycle):
    std::vector<S32> mHoldIndices;
    std::vector<std::string> mMissingHoldJoints;
    bool mHoldResolved = false;
    S32 mHeadIndex = -1;
    S32 mWristIndex[2] = {-1, -1};
    FSVRPoseTolerance mTolerance;
    // The held cycle verification ran over (HOLDING's capture):
    FSVRAdapterPlanInput mHeldInput;
    FSVRAdapterPlanResult mHeldResult;
    bool mHaveHeldCycle = false;

    // Settle (DP-B9 / DP-C5-5):
    bool mHaveSettleObservation = false;
    FSVRCalSettleObservation mPrevSettle;
    U32 mSettleStableCount = 0;
    U32 mSettleCycles = 0;
    bool mSettled = false;

    // Transition epoch baseline (first stepped cycle):
    bool mHaveEpochBaseline = false;
    U64 mEpochBaseline = 0;

    // SAMPLING window:
    std::vector<FSVRCalSample> mWindow;
    U64 mWindowEpoch = 0;

    // RESOLVE/COMMIT/INSTALL carry:
    FSVRStationarityVerdict mVerdict;
    FSVRCalRatioResolution mResolution;
    LLVector3 mAcceptedForward;
    bool mAcceptedForwardValid = false;
    // The player right basis of the acceptance cycle
    // (DP-C5-9: derived at window acceptance; the lateral
    // reaches of every window sample ride it).
    LLVector3 mAcceptedRight;
    bool mAcceptedRightValid = false;
    LLVector3 mDerivedCorneaOffset;
    LLVector3 mNeutralCorneaRoot;
    LLSD mBuiltRecord;
};

// ---------------------------------------------------------------------------
// CP-6: the session-neutral acquisition machine (pure).
// ---------------------------------------------------------------------------

struct FSVRSessionStepInputs
{
    F64 now_s = 0.0;

    // A fresh record validation read (the shell re-reads and
    // re-validates whenever the machine is stepped while
    // UNCALIBRATED/SUSPENDED, and after the account/record
    // invalidation entry point):
    bool record_read = false;
    FSVRCalibrationRecordValidation record;

    // The cycle's plan pair (reconstruction reads and the
    // bind composition consume them):
    const FSVRAdapterPlanInput* plan_input = nullptr;
    const FSVRAdapterPlanResult* plan_result = nullptr;

    // Section 5 step-2 gate facts (the section 4.2 step-3
    // signal set):
    bool sitting_on_object = false;
    bool parented = false;
    bool ground_sit_active = false;
    bool in_air = false;
    bool hmd_up_valid = false;
    LLVector3 hmd_up; // LOCAL
    bool hmd_forward_valid = false;
    LLVector3 hmd_forward; // LOCAL
    bool hmd_position_valid = false;
    LLVector3 hmd_position; // LOCAL
    bool wrist_valid[2] = {false, false};
    LLVector3 wrist_position[2]; // LOCAL

    U64 local_transition_epoch = 0;

    // Reconstruction reading for this cycle:
    bool eye_valid = false; // genuinely prim-derived
    LLVector3 cornea_midpoint_agent; // E_A, as read

    bool stationarity_installed = false;
    FSVRStationarityParams stationarity_params;
};

enum class FSVRSessionState
{
    UNCALIBRATED,
    ACQUIRING,
    ESTABLISHED,
    SUSPENDED
};

class FSVRSessionNeutral
{
public:
    FSVRSessionState state() const { return mState; }
    const FSVRSessionNeutralView& view() const { return mView; }

    void step(const FSVRSessionStepInputs& inputs);

    // ICR-2: the per-cycle not-sitting fact, refreshed by the
    // shell on every glue cycle (independent of the
    // acquisition stepping above). This is the fact's only
    // writer: the shell refreshes it pre-assembly through
    // FSVRCalibration::refreshSittingSignals and again in
    // the post-apply step hook, both from the one signal
    // read (C6E-CODE-R1-01).
    void noteSittingSignals(bool clear)
    {
        mView.sitting_signals_clear = clear;
    }

    // Invalidation entry points (CP-6; production of the
    // signals is C8's / the shell's):
    void invalidateForLocalTransition();
    void invalidateForSkeletonReinit();
    void invalidateForAccountOrRecordChange();

    // CP-2's INSTALL path (DP-B11(a)): the one route to
    // ESTABLISHED that does not run the acquisition
    // procedure. Called by the shell when it executes the
    // transaction's INSTALL_NEUTRAL action.
    void installFromCapture(
        const FSVRCalibrationRecordValidation& record,
        const LLVector3& anchor_hmd_local,
        F32 anchor_forward_yaw_rad,
        const LLVector3& cornea_offset_head_local,
        const LLVector3& neutral_cornea_root);

private:
    void dropToSuspended();
    void resetAcquisition();

    FSVRSessionState mState = FSVRSessionState::UNCALIBRATED;
    FSVRSessionNeutralView mView;
    FSVRCalibrationRecordValidation mRecordBasis;
    bool mHaveRecordBasis = false;
    std::vector<FSVRCalSample> mWindow;
    std::vector<LLVector3> mReconstructionOffsets;
    bool mHaveEpochBaseline = false;
    U64 mEpochBaseline = 0;
};

// ---------------------------------------------------------------------------
// FSVRCalibration: the pure helpers (CP-3) as statics, plus
// the viewer-shell entry points (defined below the
// FSVR_CALIBRATION_PURE_ONLY guard in the .cpp).
// ---------------------------------------------------------------------------

class LLCharacter;

class FSVRCalibration
{
public:
    // ---- CP-3 pure helpers ----

    // The section 3.1 hold set: exactly these 16 skeleton
    // joint names (spec section 3.1; brief section 2.7). The
    // apply's hold branch and the transaction resolve the
    // set by name against the plan input's joint list; the
    // list is shared so the two can never drift.
    static const std::vector<std::string>& holdJointNames();
    // Resolve the hold set against a plan input's joints.
    // Returns the indices of the joints found, in
    // holdJointNames() order; out_missing (when non-null)
    // receives the names not present (a hold failure, never
    // a silent shrink of the set).
    static std::vector<S32> resolveHoldJoints(
        const FSVRAdapterPlanInput& input,
        std::vector<std::string>* out_missing);

    // CP-3a. DP-B4's structural rules; the parameter values
    // are the caller's (production: the named source below,
    // which returns unmeasured -- DP-C5-12). The structural
    // invariants are enforced inside the helper,
    // independently of the caller's values (C5-CODE-R1-04):
    // invalid/nonfinite parameters are rejected, a window
    // needs at least two genuinely tracked, finite samples
    // with monotonic timestamps whatever min_samples says,
    // and no degenerate or invalid window classifies as
    // STATIONARY.
    static FSVRStationarityVerdict decideStationarity(
        const std::vector<FSVRCalSample>& window,
        const FSVRStationarityParams& params);
    // The one named production source for the stationarity
    // parameters. Returns false (unmeasured) until DP-B4's
    // measured values are installed; while uninstalled,
    // SAMPLING cannot complete in production and section 5
    // acquisition cannot establish on stationarity grounds.
    static bool productionStationarityParams(
        FSVRStationarityParams& out_params);

    // CP-3b. One implementation, two callers (PB-R1-06):
    // the HMD up-vector against world up in LOCAL space
    // (LOCAL is Y-up; the frame's +Y), threshold cos(20
    // degrees) (DP-C5-6 -- a posture constant, not an OPEN-6
    // noise bound).
    static bool decideUprightOrientation(const LLVector3& hmd_up);

    // CP-3c. Section 4.5's resolution rules, in order,
    // including the [R3] carry-forward implied denominator
    // and the disclosed placeholder.
    static FSVRCalRatioResolution resolveCalibrationRatios(
        const FSVRCalLandmarkSet& landmarks,
        const FSVRCalibrationRecordValidation& prior);
    // Exactly the section 4.3 map and nothing else.
    static LLSD buildCalibrationRecord(
        const FSVRCalRatioResolution& resolution, S64 captured_utc);

    // CP-3e. The expectation composition: world positions
    // and rotations for a joint list from (rest_offset,
    // local rotations) over the plan input's root frame,
    // replicating the transaction composition's rule
    // (llvradapterapply.cpp: pos = parent_pos + rest_offset
    // * parent_rot) WITHOUT the production path's
    // intermediate quaternion normalization -- the probe in
    // CP-3d exists to measure that rounding difference.
    static void composeBindWorlds(
        const FSVRAdapterPlanInput& input,
        const std::vector<LLQuaternion>& locals,
        std::vector<LLQuaternion>& out_rotations,
        std::vector<LLVector3>& out_positions);

    // CP-3d. The self-consistency probe: compose the bind
    // pose through both composition paths (the production
    // transaction composition via computePlan over an
    // identity-local copy of the input, and CP-3e's
    // expectation composition) and derive each check's
    // tolerance as max(measured x 8, precision floor)
    // (DP-C5-4 as conditioned [R2]).
    static FSVRPoseTolerance derivePoseTolerance(
        const FSVRAdapterPlanInput& bind_composition_inputs);
    // The dual check (spec section 3.2): orientation (each
    // hold joint's resolved local against its bind local,
    // identity) and position (each hold joint's resolved
    // world against CP-3e's expectation). out_first_failing
    // (when non-null) receives the first failing joint's
    // plan index from either check, orientation first.
    static bool verifyHeldPose(
        const FSVRAdapterPlanInput& input,
        const FSVRAdapterPlanResult& result,
        const std::vector<S32>& hold_joint_indices,
        const FSVRPoseTolerance& tolerance,
        S32& out_first_failing_joint);
    // The two checks separately (CP-3d's components; H5
    // case 15 fixtures the position check alone accepting
    // what the orientation check rejects).
    static bool heldPoseOrientationOk(
        const FSVRAdapterPlanResult& result,
        const std::vector<S32>& hold_joint_indices,
        const FSVRPoseTolerance& tolerance,
        S32* out_first_failing);
    static bool heldPosePositionOk(
        const FSVRAdapterPlanInput& input,
        const FSVRAdapterPlanResult& result,
        const std::vector<S32>& hold_joint_indices,
        const FSVRPoseTolerance& tolerance,
        S32* out_first_failing);

    // CP-3f. Observed stability across consecutive composed
    // cycles (DP-B9): joint offsets, mPelvisToFoot, and the
    // derived cornea landmark each within the section 3.2
    // recomposition tolerance (position_m -- positive on
    // every rig via CP-3d's precision floor).
    static bool settleObservationStable(
        const FSVRCalSettleObservation& prev,
        const FSVRCalSettleObservation& cur,
        const FSVRPoseTolerance& tolerance);

    // CP-3g. offset = inverse(head_world_rotation) *
    // (cornea_midpoint_world - head_world_position). E_A
    // enters as read: the eye state's agent space IS the
    // composition frame (brief section 2.4's contract); no
    // conversion exists in this path and none may be added.
    // Computed in F64 internally over the F32 transport
    // types in the signature.
    static LLVector3 deriveCorneaOffsetHeadLocal(
        const LLVector3& cornea_midpoint_world,
        const LLVector3& head_world_position,
        const LLQuaternion& head_world_rotation);
    // Section 4.6 check (iii): repeated reconstructions
    // over the window agree within the section 3.2
    // recomposition-derived tolerance.
    static bool corneaReconstructionStable(
        const std::vector<LLVector3>& offsets_over_window,
        F64 tolerance_m);

    // Brief-level mechanics constants (DP-C5-5, DP-C5-4):
    static constexpr F64 POSE_TOLERANCE_SAFETY_FACTOR = 8.0;
    static constexpr U32 SETTLE_CONSECUTIVE_STABLE = 3;
    static constexpr U32 SETTLE_WINDOW_CYCLES = 300;
    static constexpr F64 STARVATION_BOUND_S = 10.0;

    // ---- Viewer shell (below the guard in the .cpp) ----

    // CP-1's entry point: refuses (in progress / no VR
    // session) via nearby chat, else begins the transaction.
    static void requestCalibration();
    // CP-4 item 2: the glue populates the plan input's hold
    // flag from this.
    static bool transactionActive();
    // CP-4 item 6: the glue's amended guard consults this --
    // true iff a calibration transaction is active or the
    // session machine is ACQUIRING.
    static bool cycleWorkPending();
    // CP-4 item 3: starvation abort, executed (with its
    // report) before the flag is populated for this cycle.
    static void preCycleCheck();
    // C6E-CODE-R1-01: the pre-assembly sitting refresh. The
    // glue calls this with the cycle's character BEFORE it
    // copies sessionNeutralView() for plan-input assembly,
    // so a same-cycle consumer (C6-E's E8) reads this
    // cycle's signals, not the previous cycle's. Ownership
    // is unchanged: the fact lives in the session machine's
    // view and its only writer is noteSittingSignals; this
    // entry point performs the same write the post-apply
    // step hook performs, computed by the same signal read
    // over the same character, so the cycle's two refresh
    // points agree by construction and neither creates a
    // second store.
    static void refreshSittingSignals(LLCharacter* character);
    // CP-4 item 4: the glue's post-apply step hook. Reads
    // the cycle's fresh shell facts, advances both machines,
    // and executes the transaction's emitted actions in
    // order (reports; the one commit; the neutral install).
    static void stepFromAdapterCycle(
        LLCharacter* character,
        const FSVRAdapterPlanInput& input,
        const FSVRAdapterPlanResult& result);
    // CP-6's read view for C6-E (read-only to consumers).
    static const FSVRSessionNeutralView& sessionNeutralView();
    // CP-6's invalidation entry points (signal production
    // is C8's; these are the consuming surface):
    static void invalidateNeutralForLocalTransition();
    static void invalidateNeutralForSkeletonReinit();
    static void invalidateNeutralForAccountOrRecordChange();
    // The monotonic LOCAL-transition epoch C5's window code
    // compares. Production of the increments is C8's; the
    // counter lives here and the local-transition
    // invalidation entry point advances it, so any caller
    // of that entry point moves the epoch consistently.
    static U64 localTransitionEpoch();
};

#endif // LL_LLVRCALIBRATION_H
