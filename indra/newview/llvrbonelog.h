/**
 * @file llvrbonelog.h
 * @brief P2-07B feel-regrounding diagnostic: per-cycle motion
 *        recorder ("bonelog").
 *
 * Leah's request (2026-11-05): log how the headset moves and
 * how the in-game character moves -- world space, all bones --
 * so the follow/feel design can be reground from recorded
 * divergence instead of from a desk spec.
 *
 * OBSERVATION-ONLY. The recorder reads state; it writes
 * nothing back. It is gated by the environment variable
 * FS_VR_BONELOG=1 (parsed once, the LLOpenXRTrace way) --
 * deliberately NOT a setting: no settings-schema change, and
 * nothing about it reaches the solver, the calibration
 * machines, or Gate-1 authorization.
 *
 * Output: /tmp/firestorm_bonelog.jsonl, truncated when the
 * writer thread starts. One JSON object per line: a one-time
 * "header" record naming the skeleton joints in traversal
 * order, then one sample record per adapter cycle, with
 * "sync_executed" and "calibrate_requested" event records
 * inline on the same clock (LLOpenXRTrace::nowUs, the OpenXR
 * trace's steady clock, so bonelog samples join trace
 * records via t_ms / cycle == app_frame_token).
 *
 * Execution contract (BONELOG-R1-01): producers run on the
 * main thread (every call site is a main-thread site: the
 * adapter glue, the calibration step hook, the chat command
 * path). A producer serializes into reusable, preallocated
 * storage and hands the finished record to a bounded ring
 * queue with an O(1) swap under a briefly held mutex -- it
 * never performs file I/O and never waits for disk. A
 * dedicated writer thread performs ALL file operations
 * (open/write/flush/close). When the ring is full the record
 * is dropped and a cumulative drop counter incremented; the
 * writer reports the running total in-band
 * ({"event":"bonelog_drops","dropped":N}) whenever it
 * changes, and once more as its final record at shutdown.
 * Any recorder failure -- allocation, thread start, I/O --
 * disables the recorder quietly and permanently; no recorder
 * path can throw into, block, or stall the tracking frame.
 *
 * Schema headers are NOT droppable records (BONELOG-R2-02):
 * a skeleton header is the only mapping from a sample's bone
 * indices to joint names, so a header is tracked as pending
 * from the moment its skeleton takes effect until it is
 * actually enqueued; while a header is pending the cycle's
 * sample is skipped entirely, and no sample under the new
 * skeleton can reach the file ahead of its header. Motion
 * samples may drop; schema may not.
 *
 * Failure semantics (BONELOG-R1-03): the environment gate
 * (enabled()) and the writer's health are distinct states.
 * Once the writer observes an I/O failure it marks itself
 * failed; producers then skip traversal and serialization
 * entirely and never retry. All emitted strings pass through
 * a complete JSON escaper (quote, backslash, and every
 * control character below U+0020). The writer thread's whole
 * body runs under a top-level catch-all (BONELOG-R2-01): an
 * exception escaping a std::thread entry would call
 * std::terminate, so any writer exception marks the writer
 * failed and the recorder DEAD using atomic stores only --
 * no logging or formatting inside the catch -- and the
 * thread object stays joinable for the shutdown join.
 */

#ifndef LL_LLVRBONELOG_H
#define LL_LLVRBONELOG_H

#include "v3math.h"
#include "llquaternion.h"
#include "stdtypes.h"

#include <string>
#include <vector>

class LLCharacter;
struct FSVRAdapterPlanInput;
struct FSVRAdapterPlanResult;

// ---- P2-07B DIAG-1 recorded-data structs ----------------
// Serialization-side views of the DIAG-1 additions (brief
// sections 2.2 / 2.3 / 2.5). They carry values only: no
// state pointer, address, or identity token ever appears
// in them -- the blend slots' source_headrot flag is the
// semantic marker the glue's identity comparison produced,
// and it is the only identity-derived datum in the file.

struct FSVRBoneLogPlanDomain
{
    S32 domain = 0; // FSVRAdapterDomain value, serialized by name
    bool decision_grants_base = false;
    bool disabled = false;
};

struct FSVRBoneLogPlanData
{
    std::vector<LLQuaternion> base_locals;
    std::vector<LLVector3> base_world_positions;
    std::vector<LLQuaternion> base_world_rotations;
    std::vector<LLQuaternion> resolved_locals;
    std::vector<LLVector3> resolved_world_positions;
    std::vector<LLQuaternion> resolved_world_rotations;
    std::vector<FSVRBoneLogPlanDomain> domains;
    bool applied = false;
    const char* gate_name = nullptr; // nullptr serializes as null
    bool spine_solver_invoked = false;
    bool head_group_refusal = false;
    U32 writes_count = 0;
    // P2-07B Fix 4 (CP-F4-3): the plan result's retained
    // solver target worlds, parallel to the plan joint
    // list. Serialized only when solver_targets_present
    // (the solver ran and returned at least one target);
    // entries whose mask is 0 serialize as null. Absent
    // entirely otherwise, so pre-Fix-4 parses of the plan
    // block are unchanged (F4-F1) and target_vs_observed
    // is untouched (F4-F2).
    bool solver_targets_present = false;
    std::vector<LLQuaternion> solver_targets;
    std::vector<char> solver_target_mask;
    // P2-07B Fix 3 (brief section 4): the plan result's
    // root-channel record. root_offset is the cycle's
    // computed standing root offset (world vector),
    // present only when the standing channel produced
    // one -- absent-means-absent, never a fabricated zero
    // (F3-F). posture_behaviour names the posture
    // behaviour in force for the cycle (a fsvrPosture-
    // BehaviourName string owned by the calibration TU);
    // nullptr serializes as absent, so pre-Fix-3 plan
    // blocks parse identically (the F4-F1 standard).
    bool root_offset_present = false;
    LLVector3 root_offset = LLVector3::zero;
    const char* posture_behaviour = nullptr;
    // P2-07B Fix 3 R5 (brief section 8.1): the cycle's
    // sampled avatar state (a fsvrAvatarPostureStateName
    // string; the subtype rides in the name), the
    // root-follow suspension fact, and the home base's
    // validity and generation. Each serializes only when
    // its *_present companion is set (avatar_state: when
    // non-null), so plan blocks from cycles that carried
    // none of the R5 facts parse identically (the F4-F1
    // standard).
    const char* avatar_state = nullptr;
    bool root_follow_suspended_present = false;
    bool root_follow_suspended = false;
    bool home_base_present = false;
    bool home_base_valid = false;
    U32 home_base_generation = 0;
    // FSVRPostureSwap: the configured value and its inert
    // status, present under the same companion discipline
    // (root_follow_suspended_present gates the R5 group).
    bool posture_swap_configured = false;
    bool posture_swap_inert = false;
};

struct FSVRBoneLogHeadRotSample
{
    LLQuaternion neck_state;
    LLQuaternion head_state;
    LLQuaternion last_head_rot_before;
    LLQuaternion last_head_rot_after;
    bool instant = false;
    U64 generation = 0;
};

struct FSVRBoneLogBlendSlot
{
    bool additive = false;
    S32 priority = 0;
    F32 weight = 0.f;
    LLQuaternion rotation;
    bool has_rot_sum = false;
    F32 rot_sum_after = 0.f;
    F32 delta = 0.f;
    LLQuaternion blended_after;
    LLQuaternion added_after;
    bool source_headrot = false;
};

struct FSVRBoneLogBlendJoint
{
    U64 generation = 0;
    LLQuaternion seed_local;
    std::vector<FSVRBoneLogBlendSlot> slots;
    F32 rot_weight_sum = 0.f;
    LLQuaternion blended_rot;
    LLQuaternion added_rot;
    LLQuaternion result_local;
    // CODE-R1-01: the glue's consistency validation
    // (llpose.h fsvrValidateBlendObsSource) found an
    // identity-attributed slot whose recorded values
    // disagree with the published ask / effective mode.
    // Serialized as "source_integrity":"failed" on this
    // joint's blend entry; absent when validation passed
    // or no slot was attributed.
    bool source_integrity_failure = false;
};

struct FSVRBoneLogBlendSample
{
    bool neck_present = false;
    FSVRBoneLogBlendJoint neck;
    bool head_present = false;
    FSVRBoneLogBlendJoint head;
};

class FSVRBoneLog
{
public:
    // True when FS_VR_BONELOG enables recording. The env
    // value is parsed once and cached (LLOpenXRTrace::enabled
    // semantics: convertToBOOL, nonempty-unparseable = on).
    // This is the environment gate only; it says nothing
    // about writer health (see the failure-semantics note in
    // the file header).
    static bool enabled();

    // One sample for one adapter cycle. Call at the tail of
    // the adapter glue, after the cycle's committed writes
    // and the calibration step hook, so the skeleton read
    // below reflects this cycle's outcome. hmd_position_* are
    // the glue's own consumed HMD position read for the cycle.
    // The adapter consumes position, not an orientation
    // quaternion; the sample's orientation is reconstructed
    // from the manager's adopted forward/up (see the
    // provenance note in llvrbonelog.cpp: forward, up and the
    // consumed position all derive from ONE adopted locate
    // per cycle, and the reconstruction is numerically
    // equivalent to that locate's orientation within
    // floating-point tolerance -- it is not claimed
    // bit-identical).
    // DIAG-1: headrot/blend are the glue-assembled,
    // generation-checked captures for this cycle (nullptr
    // when no valid current-cycle capture exists -- the
    // corresponding block is then absent from the sample,
    // never fabricated or stale).
    static void sampleCycle(LLCharacter* character,
                            const FSVRAdapterPlanInput& input,
                            const FSVRAdapterPlanResult& result,
                            bool hmd_position_valid,
                            const LLVector3& hmd_position_local,
                            const FSVRBoneLogHeadRotSample* headrot,
                            const FSVRBoneLogBlendSample* blend);

    // Event records. calibrate_requested is emitted at the
    // entry of FSVRCalibration::requestCalibration (every
    // /xr calibrate invocation, refused ones included -- a
    // refused request is diagnostic fact). sync_executed is
    // emitted at executeSync's install point, immediately
    // after installFromCapture, carrying the installed pair
    // AND the executing cycle's frame token: the install
    // lands inside that cycle's step hook, so the sample of
    // the same cycle follows the event in the file and still
    // shows the pre-install solve. The first subsequent
    // ELIGIBLE adapter plan can consume the installed
    // neutral. The presence of target_vs_observed
    // establishes that a head target was composed and its
    // error against the observed skeleton was recorded. It
    // does not establish whether the positional solver
    // executed or converged. Solver execution must be
    // established from independent gate/result evidence.
    static void eventCalibrateRequested();
    static void eventSyncExecuted(
        U64 frame_token,
        const LLVector3& anchor_hmd_local,
        F32 anchor_forward_yaw_rad,
        const LLVector3& cornea_offset_head_local,
        const LLVector3& neutral_cornea_root);
    // P2-07B Fix 3 (brief sections 2.1/2.4): the
    // establishment record extended with the posture
    // flavour installed and the scale contract in force
    // per channel (fsvrPostureName / fsvrScaleContractName
    // strings owned by the calibration TU). Any nullptr
    // name serializes as absent, so the record's pre-Fix-3
    // reading is unchanged (F3-F).
    static void eventSyncExecuted(
        U64 frame_token,
        const LLVector3& anchor_hmd_local,
        F32 anchor_forward_yaw_rad,
        const LLVector3& cornea_offset_head_local,
        const LLVector3& neutral_cornea_root,
        const char* posture,
        const char* lean_contract,
        const char* root_contract);
    // P2-07B Fix 3 (brief section 2.5): emitted once per
    // establishment gap by the calibration shell (the
    // pure FSVRPostureGapTracker decides) -- the
    // undetermined-posture state announcing itself in the
    // record, never per cycle.
    static void eventPostureUndetermined();
    // P2-07B Fix 3 (brief section 2.4): an invalid scale
    // parameter failed its channel closed (never
    // clamped). channel is "lean" or "root".
    static void eventScaleParameterInvalid(const char* channel);
    // P2-07B Fix 3 [FIX3-CODE-R1-03]: the standing root
    // channel computed a valid offset, but the live
    // agent write is gated OFF while OQ-2 (the
    // playspace boundary policy) remains undecided --
    // unbounded following may not be selected by
    // implementation silence. Emitted by the adapter
    // glue once per establishment, edge-latched like
    // the scale-parameter events: the gate announces
    // itself in the record, never per cycle.
    static void eventRootFollowGated();
    // P2-07B Fix 3 [FIX3-CODE-R3]: the root-follow
    // bound (FSVRRootFollowBoundMeters) is absent or
    // invalid, so OQ-2's ruled policy (stop at the
    // bound) is not in force and live root movement
    // stays gated off -- the gated state with its cause
    // named. Same record shape as eventRootFollowGated;
    // emitted by the adapter glue once per
    // establishment alongside it, edge-latched the same
    // way.
    static void eventRootFollowBoundInvalid();
    // P2-07B Fix 3 R5 (brief sections 8.1-8.2): the avatar
    // state's transitions and the home-base / suspension
    // lifecycle, emitted by the adapter glue on the
    // transition edges only (never per cycle). from/to are
    // fsvrAvatarPostureStateName strings; reason is a
    // stable short string ("teleport", "sitting",
    // "session", "region").
    static void eventAvatarStateChanged(const char* from_state,
                                        const char* to_state);
    static void eventHomeBaseEstablished(U32 generation);
    static void eventHomeBaseInvalidated(const char* reason);
    static void eventTeleportSuspected(F32 distance_m);
    static void eventRootFollowDetectorUnproven();
    static void eventDetectorBandObservation(
        F32 max_continuity_m, F32 teleport_step_max_m,
        F32 teleport_min_m, U32 teleport_count,
        U32 unexplained_count, F32 unexplained_max_m,
        F32 teleport_event_min_m, F32 teleport_event_max_m,
        U32 teleport_events_completed,
        U32 teleport_events_incomplete);
    static void eventAvatarStateStaleSample();
    static void eventRegionDiscontinuity(F32 distance_m);
    static void eventTeleportEventMeasured(F32 net_m);
    static void eventRootFollowSuspended(const char* reason);

    // P2-07B Fix 4 (brief section 2.5): base-provenance
    // lifecycle events, emitted by the adapter glue when
    // the provenance seam reports them. A reset with
    // joint_name nullptr is a wholesale reset (character
    // change, skeleton reinit); a named reset is a
    // per-joint invalidation (membership loss, unavailable
    // or inconsistent blend record). Unclassified carries
    // the seed-gate divergence that dropped an additive
    // contribution. No silent fallbacks: every provenance
    // discontinuity the seam detects lands here.
    static void eventBaseProvenanceReset(const char* joint_name,
                                         const char* reason);
    static void eventBaseProvenanceUnclassified(
        const char* joint_name, F32 divergence_deg);

    // Orderly stop at viewer exit: signals the writer thread
    // to drain the queue, emit its final drop record, close
    // the file and exit, then joins it. Called from
    // LLAppViewer::cleanup() alongside the other worker
    // -thread shutdowns. No-op when recording is disabled or
    // was never started; idempotent.
    static void shutdown();

    // ---- Pure record serialization -----------------------
    // The builders below are pure functions of their inputs
    // (no viewer state, no I/O): the recorder's producers
    // gather state and call them, and the standalone format
    // test (indra/newview/tests/llvrbonelog_format_test.cpp)
    // calls them directly. Each clears `out` first, then
    // appends one complete JSONL record (trailing newline
    // included). Non-finite floats serialize as JSON null.
    static void buildHeaderRecord(std::string& out,
                                  const std::vector<std::string>& joint_names,
                                  long long t_ms);
    // DIAG-1 header v2: adds "schema":2 and the plan joint
    // names (plan index space for the sample's "plan"
    // block). Emitted at recorder start and re-emitted
    // whenever either the skeleton generation or the plan
    // joint list changes, under the same header-pending
    // discipline. v1 parsers read v2 files by ignoring the
    // new keys; absence of "schema" in a header means 1.
    static void buildHeaderRecord(std::string& out,
                                  const std::vector<std::string>& joint_names,
                                  const std::vector<std::string>& plan_joint_names,
                                  long long t_ms);
    static void buildCalibrateRequestedRecord(std::string& out,
                                              long long t_ms);
    static void buildSyncExecutedRecord(
        std::string& out,
        long long t_ms,
        U64 frame_token,
        const LLVector3& anchor_hmd_local,
        F32 anchor_forward_yaw_rad,
        const LLVector3& cornea_offset_head_local,
        const LLVector3& neutral_cornea_root);
    // Fix 3 extended form: the three Fix 3 names append
    // as optional keys after neutral_cornea_root, each
    // present only when its pointer is non-null.
    static void buildSyncExecutedRecord(
        std::string& out,
        long long t_ms,
        U64 frame_token,
        const LLVector3& anchor_hmd_local,
        F32 anchor_forward_yaw_rad,
        const LLVector3& cornea_offset_head_local,
        const LLVector3& neutral_cornea_root,
        const char* posture,
        const char* lean_contract,
        const char* root_contract);
    static void buildPostureUndeterminedRecord(std::string& out,
                                               long long t_ms);
    static void buildScaleParameterInvalidRecord(
        std::string& out,
        long long t_ms,
        const char* channel);
    static void buildRootFollowGatedRecord(std::string& out,
                                           long long t_ms);
    static void buildRootFollowBoundInvalidRecord(std::string& out,
                                                  long long t_ms);
    // P2-07B Fix 3 R5 event records (see the event*
    // declarations). A null name/reason serializes as null.
    static void buildAvatarStateChangedRecord(std::string& out,
                                              long long t_ms,
                                              const char* from_state,
                                              const char* to_state);
    static void buildHomeBaseEstablishedRecord(std::string& out,
                                               long long t_ms,
                                               U32 generation);
    static void buildHomeBaseInvalidatedRecord(std::string& out,
                                               long long t_ms,
                                               const char* reason);
    static void buildTeleportSuspectedRecord(std::string& out,
                                             long long t_ms,
                                             F32 distance_m);
    static void buildRootFollowDetectorUnprovenRecord(
        std::string& out, long long t_ms);
    static void buildDetectorBandObservationRecord(
        std::string& out, long long t_ms, F32 max_continuity_m,
        F32 teleport_step_max_m, F32 teleport_min_m,
        U32 teleport_count, U32 unexplained_count,
        F32 unexplained_max_m, F32 teleport_event_min_m,
        F32 teleport_event_max_m, U32 teleport_events_completed,
        U32 teleport_events_incomplete);
    static void buildAvatarStateStaleSampleRecord(
        std::string& out, long long t_ms);
    static void buildRegionDiscontinuityRecord(
        std::string& out, long long t_ms, F32 distance_m);
    static void buildTeleportEventMeasuredRecord(
        std::string& out, long long t_ms, F32 net_m);
    static void buildRootFollowSuspendedRecord(std::string& out,
                                               long long t_ms,
                                               const char* reason);
    static void buildDropsRecord(std::string& out,
                                 long long t_ms,
                                 U64 dropped_total);
    // Fix 4 provenance events (see eventBaseProvenance*).
    // joint_name nullptr serializes "joint":null.
    static void buildBaseProvenanceResetRecord(
        std::string& out,
        long long t_ms,
        const char* joint_name,
        const char* reason);
    static void buildBaseProvenanceUnclassifiedRecord(
        std::string& out,
        long long t_ms,
        const char* joint_name,
        F32 divergence_deg);
    // hmd_q_valid false serializes "q":null (position kept);
    // hmd_valid false serializes the whole "hmd" as null.
    // camera_valid false serializes "camera":null.
    // target_valid false omits the target_vs_observed block.
    static void buildSampleRecord(
        std::string& out,
        long long t_ms,
        U64 cycle,
        bool hmd_valid,
        const LLVector3& hmd_position,
        bool hmd_q_valid,
        const LLQuaternion& hmd_q,
        const LLVector3& root_position,
        const LLQuaternion& root_rotation,
        bool camera_valid,
        const LLVector3& camera_position,
        const LLQuaternion& camera_rotation,
        const std::vector<LLVector3>& bone_positions,
        const std::vector<LLQuaternion>& bone_rotations,
        bool target_valid,
        const LLVector3& target_cornea_root,
        F32 target_vs_observed_err_m);
    // DIAG-1 sample: the v1 fields verbatim, plus the
    // "plan" block (present iff plan != nullptr; the
    // producer passes it on every v2 sample), the
    // "headrot" block (present iff headrot != nullptr) and
    // the "blend" block (present iff blend != nullptr;
    // per-joint entries present per its *_present flags,
    // keyed by joint name).
    static void buildSampleRecordV2(
        std::string& out,
        long long t_ms,
        U64 cycle,
        bool hmd_valid,
        const LLVector3& hmd_position,
        bool hmd_q_valid,
        const LLQuaternion& hmd_q,
        const LLVector3& root_position,
        const LLQuaternion& root_rotation,
        bool camera_valid,
        const LLVector3& camera_position,
        const LLQuaternion& camera_rotation,
        const std::vector<LLVector3>& bone_positions,
        const std::vector<LLQuaternion>& bone_rotations,
        bool target_valid,
        const LLVector3& target_cornea_root,
        F32 target_vs_observed_err_m,
        const FSVRBoneLogPlanData* plan,
        const FSVRBoneLogHeadRotSample* headrot,
        const FSVRBoneLogBlendSample* blend);

#ifdef FSVR_BONELOG_TEST_HOOKS
    // ---- Test-only hooks --------------------------------
    // Compiled ONLY under FSVR_BONELOG_TEST_HOOKS (the
    // standalone state test in indra/newview/tests/); never
    // defined in production builds. They let the test rig
    // arm an injected writer exception, stall the writer to
    // force a full ring, and observe recorder state.
    static void testArmWriterThrow(bool arm);
    static void testSetWriterStall(bool stall);
    static bool testWriterFailed();
    static int testRunState();
    static U64 testDroppedCount();
    static U32 testQueuedCount();
    static bool testHeaderPending();
    // BONELOG-R3-01 startup-race rig: the barrier holds
    // the startup caller inside ensureStarted (just after
    // writer-thread construction) until an armed writer
    // failure has fully completed, forcing the interleaving
    // where the writer dies before the caller continues.
    static void testArmStartupBarrier(bool arm);
    static bool testFailureComplete();
    static U32 testWriterEntryCount();
#endif
};

#endif // LL_LLVRBONELOG_H
