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
    static void sampleCycle(LLCharacter* character,
                            const FSVRAdapterPlanInput& input,
                            const FSVRAdapterPlanResult& result,
                            bool hmd_position_valid,
                            const LLVector3& hmd_position_local);

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
    static void buildDropsRecord(std::string& out,
                                 long long t_ms,
                                 U64 dropped_total);
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
