/**
 * @file llvrbonelog.cpp
 * @brief P2-07B feel-regrounding diagnostic: per-cycle motion
 *        recorder ("bonelog"). See llvrbonelog.h.
 *
 * Implementation notes:
 *
 * - Execution contract (BONELOG-R1-01): producers (all on
 *   the main thread) serialize into reusable storage and
 *   hand records to a bounded ring queue with an O(1) swap;
 *   they never touch the file. A dedicated writer thread
 *   performs all file operations. A full ring drops the
 *   record and increments a cumulative counter the writer
 *   reports in-band. Round 1 wrote synchronously on the
 *   frame thread; that design's flush/write latency was
 *   unbounded and is removed.
 *
 * - Hook currency: the sample is taken at the tail of
 *   FSVRAdapterApplyForCharacter, after the cycle's committed
 *   writes (LLJoint::setRotation) and after the calibration
 *   step hook. LLJoint::getWorldPosition()/getWorldRotation()
 *   recompose on demand through updateWorldPRSParent(), which
 *   walks the parent chain refreshing any joint whose local
 *   rotation/position is dirty -- and setRotation touches
 *   exactly those dirty flags. The world transforms read here
 *   are therefore composed from THIS cycle's just-written
 *   locals, not from a cached matrix. (This is the PRS path;
 *   the cached LLMatrix4 path the C1 header warns about is a
 *   different representation and is not read here.) The poses
 *   are the adapter-tail state; a later animation or render
 *   stage that rewrites locals is outside this boundary by
 *   design.
 *
 * - HMD orientation provenance (BONELOG-R1-02): the adapter
 *   consumes the HMD position from the manager's adopted
 *   sample (LLOpenXRManager::getHmdPosition); the recorder
 *   reads forward/up via getHmdForward/getHmdUp at the
 *   adapter tail. All three derive from ONE adopted locate:
 *   adopt_located_views() (llopenxrmanager.cpp:3751) writes
 *   latest_hmd_forward (:3833), latest_pose_generation
 *   (:3835), the adopted_sample_* stamps (:3839-3844),
 *   adopted_sample_position -- decided over the SAME views
 *   array by decide_hmd_position_adoption (:3853-3873) --
 *   and latest_hmd_up (:3892), in one block, from the views
 *   of a single xrLocateViews under one locate generation;
 *   its failure branch clears them together (:3875-3887).
 *   adopt_located_views has exactly two call sites, both
 *   manager frame-path functions (submitFrameMonoImpl
 *   :4571, beginPacedFrame :5769). The position is consumed
 *   at llvradapterapply.cpp:1258 and forward/up are read at
 *   the tail of the SAME glue invocation on the main thread;
 *   the only manager calls in between are const getters, so
 *   nothing re-adopts between consumption and this read.
 *   The reconstruction (forward = (0,0,-1) * q, up =
 *   (0,1,0) * q are the locate orientation's basis images;
 *   rebuild q over (r, u, -f) with r = u x (-f) via
 *   LLQuaternion's three-axis constructor) is therefore of
 *   the same adopted sample, and is numerically equivalent
 *   to its orientation within floating-point tolerance --
 *   normalization and basis recombination are not bit-exact
 *   operations and no bit-identity is claimed. Before
 *   reconstructing, forward/up are validated (see
 *   reconstructHmdOrientation): non-finite or degenerate
 *   input yields "q":null with the position kept. When the
 *   adapter's own position read failed, "hmd" is null (no
 *   valid sample was consumed this cycle).
 *
 * - Root frame: "root" is the adapter transaction root
 *   frame (input.root_world_* -- the root frame the cycle
 *   composed and solved in). That is the frame the session
 *   neutral (neutral_cornea_root) and the head target live
 *   in, so target_cornea_root below is the target
 *   re-expressed in its own frame, directly comparable to
 *   the installed neutral. The transaction root is NOT
 *   asserted to be the avatar object's own transform; the
 *   root joint's observed world pose is in the bones array
 *   (bones[0]).
 *
 * - Camera: LLViewerCamera's state read at adapter-tail
 *   time -- the view the viewer resolved for this frame.
 *   The literal submitted OpenXR view pose is the
 *   xr_submit records of the OpenXR trace, which remains
 *   the submitted-view authority; "cycle" carries the app
 *   frame token so the two join exactly.
 *
 * - target_vs_observed: present only when the cycle
 *   composed a head target and the skeleton has an mHead.
 *   It compares the composed target with the cornea
 *   observed on the written skeleton at the hook. A
 *   composed target does not by itself prove a positional
 *   solve was authorized, executed, or converged; read the
 *   block as target-versus-observed error, nothing more.
 *
 * - Robustness (BONELOG-R1-03): the writer checks the
 *   stream after every write batch and flush; on any
 *   failure it marks itself failed permanently, and
 *   producers check that flag BEFORE traversing or
 *   serializing, so a dead writer costs one atomic load
 *   per call. Nothing retries an open or a failed write.
 *   Producer-side exceptions (allocation, thread start)
 *   are caught at the entry points and disable the
 *   recorder the same way. Non-finite floats are emitted
 *   as JSON null (sanitizer choice); everything else is
 *   formatted %.9g, the float32-reproducible style of the
 *   OpenXR trace.
 *
 * - Writer exception containment (BONELOG-R2-01): the
 *   writer body runs under a top-level catch-all in
 *   writerMain. Stream-flag checks cannot catch exceptions
 *   (e.g. std::bad_alloc from open, reserve, or string
 *   growth), and an exception escaping a std::thread entry
 *   is std::terminate. On catch the writer marks itself
 *   failed and the recorder DEAD with atomic stores only
 *   (no logging/formatting in the catch), and the thread
 *   returns normally, so its std::thread stays joinable.
 *
 * - Startup publication ordering (BONELOG-R3-01):
 *   ensureStarted publishes BS_RUNNING BEFORE constructing
 *   the writer thread. A fresh thread may run -- and fail
 *   -- before its creator executes another statement, so a
 *   startup-side store after construction could land after
 *   the writer's terminal stores and resurrect BS_DEAD.
 *   With the publish first there is no such store: once
 *   the writer has started executing, nothing on the
 *   startup path writes run_state again. The healthy
 *   checks also consult writer_failed, so BS_RUNNING alone
 *   is never treated as a healthy writer.
 *
 * - Schema headers (BONELOG-R2-02): refreshSkeleton
 *   advancing its cache is not publication. A header is
 *   pending from its skeleton transition until enqueueLine
 *   accepts it; while pending, sampleCycleImpl skips the
 *   cycle's sample entirely and retries the header next
 *   cycle. Queue order then guarantees no sample is ever
 *   interpreted under a stale header.
 */

#if !defined(FSVR_BONELOG_SERIALIZE_ONLY) && \
    !defined(FSVR_BONELOG_TEST_RIG)
#include "llviewerprecompiledheaders.h"
#endif

#include "llvrbonelog.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#ifndef FSVR_BONELOG_SERIALIZE_ONLY

#ifdef FSVR_BONELOG_TEST_RIG
// Standalone state test: minimal stand-ins for the viewer
// classes this file touches, declared by the test header
// and implemented by the test main. Production and
// SERIALIZE_ONLY builds never take this branch.
#include "llvrbonelog_test_stubs.h"
#else
#include "llvradapterapply.h"

#include "llcharacter.h"
#include "lljoint.h"
#include "llopenxrmanager.h"
#include "llopenxrtrace.h"
#include "llstring.h"
#include "llviewercamera.h"
#endif

#include <atomic>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <thread>
#ifdef FSVR_BONELOG_TEST_HOOKS
#include <chrono>
#include <stdexcept>
#endif

#endif // FSVR_BONELOG_SERIALIZE_ONLY

namespace
{

// %.9g float32-reproducible formatting; non-finite -> null.
void appendFloat(std::string& out, F32 value)
{
    if (!std::isfinite(value))
    {
        out += "null";
        return;
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.9g",
                  static_cast<double>(value));
    out += buf;
}

void appendVec(std::string& out, const LLVector3& v)
{
    out += '[';
    appendFloat(out, v.mV[VX]);
    out += ',';
    appendFloat(out, v.mV[VY]);
    out += ',';
    appendFloat(out, v.mV[VZ]);
    out += ']';
}

void appendQuat(std::string& out, const LLQuaternion& q)
{
    // Spec order [x, y, z, w] == LL's mQ[VX..VS].
    out += '[';
    appendFloat(out, q.mQ[VX]);
    out += ',';
    appendFloat(out, q.mQ[VY]);
    out += ',';
    appendFloat(out, q.mQ[VZ]);
    out += ',';
    appendFloat(out, q.mQ[VS]);
    out += ']';
}

// Forward declaration: defined below with the other
// serialization primitives; the DIAG-1 block appenders
// above it use it for the gate/domain name strings.
void appendJsonString(std::string& out, const std::string& s);

void appendBool(std::string& out, bool value)
{
    out += value ? "true" : "false";
}

// DIAG-1: the plan block's domain names (FSVRAdapterDomain
// values NONE/SPINE/ARM_LEFT/ARM_RIGHT, in enum order).
const char* planDomainName(S32 domain)
{
    switch (domain)
    {
    case 0: return "none";
    case 1: return "spine";
    case 2: return "arm_left";
    case 3: return "arm_right";
    }
    return "unknown";
}

void appendWorldArray(std::string& out,
                      const std::vector<LLVector3>& positions,
                      const std::vector<LLQuaternion>& rotations)
{
    const size_t count =
        positions.size() < rotations.size() ? positions.size()
                                            : rotations.size();
    out += '[';
    for (size_t i = 0; i < count; ++i)
    {
        if (i > 0)
        {
            out += ',';
        }
        out += "{\"p\":";
        appendVec(out, positions[i]);
        out += ",\"q\":";
        appendQuat(out, rotations[i]);
        out += '}';
    }
    out += ']';
}

void appendQuatArray(std::string& out,
                     const std::vector<LLQuaternion>& quats)
{
    out += '[';
    for (size_t i = 0; i < quats.size(); ++i)
    {
        if (i > 0)
        {
            out += ',';
        }
        appendQuat(out, quats[i]);
    }
    out += ']';
}

// DIAG-1 "plan" block (brief section 2.2): raw quantities
// only -- derived factors are analysis-side, never
// recorded.
void appendPlanBlock(std::string& out,
                     const FSVRBoneLogPlanData& plan)
{
    out += ",\"plan\":{\"base_locals\":";
    appendQuatArray(out, plan.base_locals);
    out += ",\"base_worlds\":";
    appendWorldArray(out, plan.base_world_positions,
                     plan.base_world_rotations);
    out += ",\"resolved_locals\":";
    appendQuatArray(out, plan.resolved_locals);
    out += ",\"resolved_worlds\":";
    appendWorldArray(out, plan.resolved_world_positions,
                     plan.resolved_world_rotations);
    // P2-07B Fix 4 (CP-F4-3): the retained solver target
    // worlds, in plan-joint order, present only when the
    // plan carried any; untargeted joints serialize as
    // null. Absent entirely otherwise (F4-F1).
    if (plan.solver_targets_present)
    {
        out += ",\"solver_targets\":[";
        for (size_t i = 0; i < plan.solver_targets.size(); ++i)
        {
            if (i > 0)
            {
                out += ',';
            }
            if (i < plan.solver_target_mask.size() &&
                plan.solver_target_mask[i])
            {
                appendQuat(out, plan.solver_targets[i]);
            }
            else
            {
                out += "null";
            }
        }
        out += ']';
    }
    out += ",\"domains\":[";
    for (size_t i = 0; i < plan.domains.size(); ++i)
    {
        if (i > 0)
        {
            out += ',';
        }
        out += "{\"domain\":";
        appendJsonString(out, planDomainName(plan.domains[i].domain));
        out += ",\"grants_base\":";
        appendBool(out, plan.domains[i].decision_grants_base);
        out += ",\"disabled\":";
        appendBool(out, plan.domains[i].disabled);
        out += '}';
    }
    out += "],\"applied\":";
    appendBool(out, plan.applied);
    out += ",\"gate\":";
    if (plan.gate_name)
    {
        appendJsonString(out, plan.gate_name);
    }
    else
    {
        out += "null";
    }
    out += ",\"spine_solver_invoked\":";
    appendBool(out, plan.spine_solver_invoked);
    out += ",\"head_group_refusal\":";
    appendBool(out, plan.head_group_refusal);
    out += ",\"writes\":";
    out += std::to_string(plan.writes_count);
    // P2-07B Fix 3 (brief section 4): the posture
    // behaviour in force for the cycle and the standing
    // channel's computed root offset, both optional --
    // absent entirely when the plan carried neither, so
    // pre-Fix-3 plan blocks parse identically (F3-F).
    if (plan.posture_behaviour)
    {
        out += ",\"posture_behaviour\":";
        appendJsonString(out, plan.posture_behaviour);
    }
    if (plan.root_offset_present)
    {
        out += ",\"root_offset\":";
        appendVec(out, plan.root_offset);
    }
    out += '}';
}

// DIAG-1 "headrot" block (brief section 2.3): the motion's
// ask for this cycle. Values only -- the capture's state
// identities never appear here or anywhere in the file.
void appendHeadRotBlock(std::string& out,
                        const FSVRBoneLogHeadRotSample& headrot)
{
    out += ",\"headrot\":{\"neck_state\":";
    appendQuat(out, headrot.neck_state);
    out += ",\"head_state\":";
    appendQuat(out, headrot.head_state);
    out += ",\"last_head_rot_before\":";
    appendQuat(out, headrot.last_head_rot_before);
    out += ",\"last_head_rot_after\":";
    appendQuat(out, headrot.last_head_rot_after);
    out += ",\"instant\":";
    appendBool(out, headrot.instant);
    out += ",\"generation\":";
    out += std::to_string(
        static_cast<unsigned long long>(headrot.generation));
    out += '}';
}

// DIAG-1 blend entry for one joint (brief section 2.5).
void appendBlendJoint(std::string& out,
                      const FSVRBoneLogBlendJoint& joint)
{
    out += "{\"generation\":";
    out += std::to_string(
        static_cast<unsigned long long>(joint.generation));
    out += ",\"seed_local\":";
    appendQuat(out, joint.seed_local);
    out += ",\"slots\":[";
    for (size_t i = 0; i < joint.slots.size(); ++i)
    {
        const FSVRBoneLogBlendSlot& slot = joint.slots[i];
        if (i > 0)
        {
            out += ',';
        }
        out += "{\"additive\":";
        appendBool(out, slot.additive);
        out += ",\"priority\":";
        out += std::to_string(slot.priority);
        out += ",\"weight\":";
        appendFloat(out, slot.weight);
        out += ",\"rotation\":";
        appendQuat(out, slot.rotation);
        if (slot.has_rot_sum)
        {
            out += ",\"rot_sum_after\":";
            appendFloat(out, slot.rot_sum_after);
        }
        else
        {
            out += ",\"delta\":";
            appendFloat(out, slot.delta);
        }
        out += ",\"blended_after\":";
        appendQuat(out, slot.blended_after);
        out += ",\"added_after\":";
        appendQuat(out, slot.added_after);
        if (slot.source_headrot)
        {
            out += ",\"source\":\"headrot\"";
        }
        out += '}';
    }
    out += "],\"rot_weight_sum\":";
    appendFloat(out, joint.rot_weight_sum);
    out += ",\"blended_rot\":";
    appendQuat(out, joint.blended_rot);
    out += ",\"added_rot\":";
    appendQuat(out, joint.added_rot);
    out += ",\"result_local\":";
    appendQuat(out, joint.result_local);
    if (joint.source_integrity_failure)
    {
        // CODE-R1-01: explicit record-integrity failure
        // indication (brief section 2.5: a mismatch "is a
        // record-integrity failure"). The attributed
        // slot's source marker above still stands.
        out += ",\"source_integrity\":\"failed\"";
    }
    out += '}';
}

// Complete JSON string escaping: quote, backslash, the
// named control escapes, and \u00XX for every other byte
// below U+0020. Bytes >= 0x80 pass through raw (the emitted
// file is UTF-8; skeleton names are ASCII in practice, and
// raw passthrough keeps valid UTF-8 valid).
void appendJsonString(std::string& out, const std::string& s)
{
    out += '"';
    for (char ch : s)
    {
        const unsigned char c = static_cast<unsigned char>(ch);
        switch (c)
        {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\b': out += "\\b"; break;
        case '\f': out += "\\f"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (c < 0x20)
            {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x",
                              static_cast<unsigned int>(c));
                out += buf;
            }
            else
            {
                out += static_cast<char>(c);
            }
            break;
        }
    }
    out += '"';
}

} // namespace

// ---- Pure record serialization ---------------------------
// These builders are the ONLY record-format code: producers
// and the format test both call them, so what the test
// parses is byte-for-byte what the recorder writes.

// static
void FSVRBoneLog::buildHeaderRecord(
    std::string& out,
    const std::vector<std::string>& joint_names,
    long long t_ms)
{
    out.clear();
    out += "{\"header\":true,\"t_ms\":";
    out += std::to_string(t_ms);
    out += ",\"joints\":[";
    bool first = true;
    for (const std::string& name : joint_names)
    {
        if (!first)
        {
            out += ',';
        }
        first = false;
        appendJsonString(out, name);
    }
    out += "]}\n";
}

// static
void FSVRBoneLog::buildHeaderRecord(
    std::string& out,
    const std::vector<std::string>& joint_names,
    const std::vector<std::string>& plan_joint_names,
    long long t_ms)
{
    out.clear();
    out += "{\"header\":true,\"schema\":2,\"t_ms\":";
    out += std::to_string(t_ms);
    out += ",\"joints\":[";
    bool first = true;
    for (const std::string& name : joint_names)
    {
        if (!first)
        {
            out += ',';
        }
        first = false;
        appendJsonString(out, name);
    }
    out += "],\"plan_joints\":[";
    first = true;
    for (const std::string& name : plan_joint_names)
    {
        if (!first)
        {
            out += ',';
        }
        first = false;
        appendJsonString(out, name);
    }
    out += "]}\n";
}

// static
void FSVRBoneLog::buildCalibrateRequestedRecord(std::string& out,
                                                long long t_ms)
{
    out.clear();
    out += "{\"event\":\"calibrate_requested\",\"t_ms\":";
    out += std::to_string(t_ms);
    out += "}\n";
}

// static
void FSVRBoneLog::buildSyncExecutedRecord(
    std::string& out,
    long long t_ms,
    U64 frame_token,
    const LLVector3& anchor_hmd_local,
    F32 anchor_forward_yaw_rad,
    const LLVector3& cornea_offset_head_local,
    const LLVector3& neutral_cornea_root)
{
    buildSyncExecutedRecord(out, t_ms, frame_token,
                            anchor_hmd_local,
                            anchor_forward_yaw_rad,
                            cornea_offset_head_local,
                            neutral_cornea_root,
                            nullptr, nullptr, nullptr);
}

// static
void FSVRBoneLog::buildSyncExecutedRecord(
    std::string& out,
    long long t_ms,
    U64 frame_token,
    const LLVector3& anchor_hmd_local,
    F32 anchor_forward_yaw_rad,
    const LLVector3& cornea_offset_head_local,
    const LLVector3& neutral_cornea_root,
    const char* posture,
    const char* lean_contract,
    const char* root_contract)
{
    out.clear();
    out += "{\"event\":\"sync_executed\",\"t_ms\":";
    out += std::to_string(t_ms);
    // The executing cycle's app frame token (see the header
    // note): samples carry the same value as "cycle".
    out += ",\"cycle\":";
    out += std::to_string(
        static_cast<unsigned long long>(frame_token));
    out += ",\"anchor_hmd_local\":";
    appendVec(out, anchor_hmd_local);
    out += ",\"anchor_forward_yaw_rad\":";
    appendFloat(out, anchor_forward_yaw_rad);
    out += ",\"cornea_offset_head_local\":";
    appendVec(out, cornea_offset_head_local);
    out += ",\"neutral_cornea_root\":";
    appendVec(out, neutral_cornea_root);
    // P2-07B Fix 3 (brief sections 2.1/2.4): the
    // establishment's posture flavour and per-channel
    // scale contracts, appended only when named -- a
    // record carrying none of them is byte-identical to
    // its pre-Fix-3 form (F3-F).
    if (posture)
    {
        out += ",\"posture\":";
        appendJsonString(out, posture);
    }
    if (lean_contract)
    {
        out += ",\"lean_contract\":";
        appendJsonString(out, lean_contract);
    }
    if (root_contract)
    {
        out += ",\"root_contract\":";
        appendJsonString(out, root_contract);
    }
    out += "}\n";
}

// static
void FSVRBoneLog::buildPostureUndeterminedRecord(
    std::string& out, long long t_ms)
{
    out.clear();
    out += "{\"event\":\"posture_undetermined\",\"t_ms\":";
    out += std::to_string(t_ms);
    out += "}\n";
}

// static
void FSVRBoneLog::buildScaleParameterInvalidRecord(
    std::string& out, long long t_ms, const char* channel)
{
    out.clear();
    out += "{\"event\":\"scale_parameter_invalid\",\"t_ms\":";
    out += std::to_string(t_ms);
    out += ",\"channel\":";
    appendJsonString(out, channel ? channel : "");
    out += "}\n";
}

// static
void FSVRBoneLog::buildRootFollowGatedRecord(
    std::string& out, long long t_ms)
{
    out.clear();
    out += "{\"event\":\"root_follow_gated\",\"t_ms\":";
    out += std::to_string(t_ms);
    out += "}\n";
}

// static
void FSVRBoneLog::buildRootFollowBoundInvalidRecord(
    std::string& out, long long t_ms)
{
    out.clear();
    out += "{\"event\":\"root_follow_bound_invalid\",\"t_ms\":";
    out += std::to_string(t_ms);
    out += "}\n";
}

// static
void FSVRBoneLog::buildDropsRecord(std::string& out,
                                   long long t_ms,
                                   U64 dropped_total)
{
    out.clear();
    out += "{\"event\":\"bonelog_drops\",\"t_ms\":";
    out += std::to_string(t_ms);
    out += ",\"dropped\":";
    out += std::to_string(
        static_cast<unsigned long long>(dropped_total));
    out += "}\n";
}

// static
void FSVRBoneLog::buildBaseProvenanceResetRecord(
    std::string& out,
    long long t_ms,
    const char* joint_name,
    const char* reason)
{
    out.clear();
    out += "{\"event\":\"base_provenance_reset\",\"t_ms\":";
    out += std::to_string(t_ms);
    out += ",\"joint\":";
    if (joint_name)
    {
        appendJsonString(out, joint_name);
    }
    else
    {
        out += "null";
    }
    out += ",\"reason\":";
    appendJsonString(out, reason ? reason : "");
    out += "}\n";
}

// static
void FSVRBoneLog::buildBaseProvenanceUnclassifiedRecord(
    std::string& out,
    long long t_ms,
    const char* joint_name,
    F32 divergence_deg)
{
    out.clear();
    out += "{\"event\":\"base_provenance_unclassified\","
           "\"t_ms\":";
    out += std::to_string(t_ms);
    out += ",\"joint\":";
    if (joint_name)
    {
        appendJsonString(out, joint_name);
    }
    else
    {
        out += "null";
    }
    out += ",\"seed_divergence_deg\":";
    appendFloat(out, divergence_deg);
    out += "}\n";
}

// static
void FSVRBoneLog::buildSampleRecordV2(
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
    const FSVRBoneLogBlendSample* blend)
{
    out.clear();
    out += "{\"t_ms\":";
    out += std::to_string(t_ms);
    // The app frame token: the join key to the OpenXR
    // trace's per-iteration records (app_frame / render /
    // xr_submit).
    out += ",\"cycle\":";
    out += std::to_string(
        static_cast<unsigned long long>(cycle));

    // --- hmd: the raw headset LOCAL pose as consumed.
    if (hmd_valid)
    {
        out += ",\"hmd\":{\"p\":";
        appendVec(out, hmd_position);
        out += ",\"q\":";
        if (hmd_q_valid)
        {
            appendQuat(out, hmd_q);
        }
        else
        {
            out += "null";
        }
        out += '}';
    }
    else
    {
        out += ",\"hmd\":null";
    }

    // --- root: the transaction root frame (see file note).
    out += ",\"root\":{\"p\":";
    appendVec(out, root_position);
    out += ",\"q\":";
    appendQuat(out, root_rotation);
    out += '}';

    // --- camera: LLViewerCamera state at sample time.
    out += ",\"camera\":";
    if (camera_valid)
    {
        out += "{\"p\":";
        appendVec(out, camera_position);
        out += ",\"q\":";
        appendQuat(out, camera_rotation);
        out += '}';
    }
    else
    {
        out += "null";
    }

    // --- bones: every joint, world space, skeleton order.
    const size_t bone_count =
        bone_positions.size() < bone_rotations.size()
            ? bone_positions.size()
            : bone_rotations.size();
    out += ",\"bones\":[";
    for (size_t i = 0; i < bone_count; ++i)
    {
        if (i > 0)
        {
            out += ',';
        }
        out += "{\"i\":";
        out += std::to_string(i);
        out += ",\"p\":";
        appendVec(out, bone_positions[i]);
        out += ",\"q\":";
        appendQuat(out, bone_rotations[i]);
        out += '}';
    }
    out += ']';

    // --- target_vs_observed: the cycle's composed cornea
    // target in the root frame, and the error of the cornea
    // observed on the written skeleton against the world
    // target. NOT a solver-convergence measure (see the
    // file note).
    if (target_valid)
    {
        out += ",\"target_vs_observed\":{\"target_cornea_root\":";
        appendVec(out, target_cornea_root);
        out += ",\"observed_err_m\":";
        appendFloat(out, target_vs_observed_err_m);
        out += '}';
    }

    // --- DIAG-1 blocks (brief sections 2.2 / 2.3 / 2.5):
    // new keys only; every v1 field above is unchanged in
    // name, type and nullability.
    if (plan)
    {
        appendPlanBlock(out, *plan);
    }
    if (headrot)
    {
        appendHeadRotBlock(out, *headrot);
    }
    if (blend && (blend->neck_present || blend->head_present))
    {
        out += ",\"blend\":{";
        bool first_entry = true;
        if (blend->neck_present)
        {
            out += "\"mNeck\":";
            appendBlendJoint(out, blend->neck);
            first_entry = false;
        }
        if (blend->head_present)
        {
            if (!first_entry)
            {
                out += ',';
            }
            out += "\"mHead\":";
            appendBlendJoint(out, blend->head);
        }
        out += '}';
    }

    out += "}\n";
}

// static
void FSVRBoneLog::buildSampleRecord(
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
    F32 target_vs_observed_err_m)
{
    buildSampleRecordV2(out, t_ms, cycle,
                        hmd_valid, hmd_position,
                        hmd_q_valid, hmd_q,
                        root_position, root_rotation,
                        camera_valid, camera_position, camera_rotation,
                        bone_positions, bone_rotations,
                        target_valid, target_cornea_root,
                        target_vs_observed_err_m,
                        nullptr, nullptr, nullptr);
}

#ifndef FSVR_BONELOG_SERIALIZE_ONLY

namespace
{

const char* const BONELOG_ENV = "FS_VR_BONELOG";
const char* const BONELOG_PATH = "/tmp/firestorm_bonelog.jsonl";
// Bounded ring capacity, in records. At the solve cadence
// this absorbs several seconds of writer stall; beyond it,
// records drop (counted) rather than block the frame.
const U32 RING_SLOTS = 256;
// Per-slot capacity reserved once at recorder start. Slots
// (and the producer scratch) keep the largest capacity they
// grow to, so steady-state production allocates nothing.
const size_t RING_SLOT_RESERVE = 32768;

// Recorder lifecycle states (run_state).
enum BoneLogRunState
{
    BS_NOT_STARTED = 0,
    BS_RUNNING,
    BS_STOPPING,
    BS_DEAD
};

// Producer-side hard disable, independent of the heap state
// below so the entry points can check it before touching
// anything that allocates. Set by any caught producer
// exception; never cleared.
std::atomic<bool> s_hard_disabled{false};

#ifdef FSVR_BONELOG_TEST_HOOKS
// Test-only fault/stall controls (standalone state test).
// No production code sets these; the setters exist only as
// the FSVRBoneLog::test* hooks under the same define.
std::atomic<bool> s_test_writer_throw_armed{false};
std::atomic<bool> s_test_writer_stall{false};

// Test-only startup-race barrier (BONELOG-R3-01): when
// armed, the startup caller blocks inside ensureStarted
// (just after writer-thread construction) until the
// writer's failure has fully completed -- both terminal
// stores done -- forcing deterministically the
// interleaving where the writer dies before the caller
// executes another statement.
std::atomic<bool> s_test_startup_barrier_armed{false};
std::atomic<U32> s_test_writer_entries{0};
std::mutex s_test_barrier_mutex;
std::condition_variable s_test_barrier_cv;
bool s_test_failure_complete = false; // s_test_barrier_mutex

// Writer side: an injected startup failure has completed.
void testSignalWriterFailureComplete()
{
    if (!s_test_startup_barrier_armed.load())
    {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(s_test_barrier_mutex);
        s_test_failure_complete = true;
    }
    s_test_barrier_cv.notify_all();
}

// Startup-caller side: wait for the signal above. Bounded
// so a mis-armed rig degrades to a slow pass, never a hang.
void testStartupBarrierHold()
{
    if (!s_test_startup_barrier_armed.load())
    {
        return;
    }
    std::unique_lock<std::mutex> lock(s_test_barrier_mutex);
    s_test_barrier_cv.wait_for(
        lock, std::chrono::seconds(30),
        [] { return s_test_failure_complete; });
}
#endif

struct BoneLogState
{
    // Header/skeleton cache (producer thread only): the
    // traversal is rebuilt when the character or its
    // skeleton serial changes. Rebuilding the cache is not
    // publication: header_pending stays set from the
    // transition until the header record is actually
    // enqueued, and samples are skipped while it is set
    // (BONELOG-R2-02).
    LLCharacter* header_character = nullptr;
    U32 header_serial = 0;
    bool header_pending = false;
    std::vector<LLJoint*> joints;
    std::vector<std::string> joint_names;
    S32 head_index = -1;

    // DIAG-1: the last PUBLISHED plan joint list. A cycle
    // whose plan list differs pends a v2 header re-emission
    // under the same discipline as a skeleton change
    // (brief section 2.1, Q-3 settled: header re-emission
    // is the mechanism).
    std::vector<std::string> plan_names;
    bool plan_pending = false;

    // Producer scratch (main thread only): reused across
    // samples; capacity persists, so steady-state samples
    // allocate nothing.
    std::string scratch;
    std::vector<LLVector3> bone_positions;
    std::vector<LLQuaternion> bone_rotations;
    FSVRBoneLogPlanData plan_scratch;

    // Bounded producer/consumer ring. The mutex is held
    // only for O(1) index/swap operations -- never across
    // serialization, traversal, or I/O.
    std::mutex mutex;
    std::condition_variable cv;
    std::vector<std::string> ring;
    U32 ring_head = 0;
    U32 ring_tail = 0;
    U32 ring_count = 0;
    bool stop_requested = false;
    // Records produced but not written: ring-full drops by
    // producers, discards by a failed writer. Guarded by
    // mutex; reported by the writer.
    U64 dropped = 0;
    U64 drops_reported = 0; // writer thread only

    // Lifecycle. run_state/writer_failed are atomic because
    // producers read them lock-free on the frame path;
    // writer_failed is written by the writer thread.
    std::atomic<int> run_state{BS_NOT_STARTED};
    std::atomic<bool> writer_failed{false};
    std::mutex start_mutex;
    std::thread writer; // touched only on the main thread
};

// The state is heap-allocated on first use and intentionally
// never destroyed: shutdown joins the writer from
// LLAppViewer::cleanup(), and no static-destruction order
// may ever invalidate state a late call could touch.
BoneLogState& bonelogState()
{
    static BoneLogState* state = new BoneLogState();
    return *state;
}

long long nowMs()
{
    // The OpenXR trace's steady clock (same epoch), in ms:
    // bonelog t_ms and trace monotonic_us/1000 are one clock.
    return LLOpenXRTrace::nowUs() / 1000;
}

bool finiteVec3(const LLVector3& v)
{
    return std::isfinite(v.mV[VX]) && std::isfinite(v.mV[VY]) &&
           std::isfinite(v.mV[VZ]);
}

bool finiteQuat(const LLQuaternion& q)
{
    return std::isfinite(q.mQ[VX]) && std::isfinite(q.mQ[VY]) &&
           std::isfinite(q.mQ[VZ]) && std::isfinite(q.mQ[VS]);
}

// Reconstruct the adopted locate's orientation from its
// forward/up basis images, with validation (BONELOG-R1-02).
// The floors are degeneracy guards, not precision gates:
// genuine basis images of a unit quaternion have length ~1
// and are near-orthogonal (cross magnitude ~1). Lengths
// must exceed 1e-4; the cross magnitude must exceed 1e-3
// (with |up| ~ 1 that is sin(angle) -- it rejects only
// near-parallel or collapsed input). Returns false on any
// non-finite or degenerate input; the caller then writes
// "q":null and keeps the position.
bool reconstructHmdOrientation(const LLVector3& forward,
                               const LLVector3& up,
                               LLQuaternion& out_orientation)
{
    if (!finiteVec3(forward) || !finiteVec3(up))
    {
        return false;
    }
    const F32 forward_len = forward.length();
    const F32 up_len = up.length();
    if (!(forward_len > 1e-4f) || !(up_len > 1e-4f))
    {
        return false;
    }
    // Basis images of the raw locate orientation:
    // forward = (0,0,-1) * q, up = (0,1,0) * q. Rebuild q
    // over (r, u, -f), r = u x (-f).
    const LLVector3 z_img = -forward / forward_len;
    LLVector3 x_img = up % z_img;
    const F32 x_len = x_img.length();
    if (!(x_len > 1e-3f))
    {
        return false;
    }
    x_img = x_img / x_len;
    const LLVector3 y_img = z_img % x_img;
    const LLQuaternion q(x_img, y_img, z_img);
    if (!finiteQuat(q))
    {
        return false;
    }
    out_orientation = q;
    return true;
}

// The writer thread body: owns the file outright (open,
// write, flush, close all happen here and nowhere else).
// Nothing in here is exception-safe by itself -- stream
// flags report ordinary I/O failure, but open, reserve,
// string growth and serialization can throw (bad_alloc).
// Any exception escaping this function is contained by
// writerMain's catch-all (BONELOG-R2-01).
void writerMainLoop()
{
    BoneLogState& state = bonelogState();
#ifdef FSVR_BONELOG_TEST_HOOKS
    if (s_test_writer_throw_armed.load())
    {
        // Injected fault (state test only): the throw must
        // be contained exactly like a real exception from
        // the open/reserve/write work below.
        throw std::runtime_error(
            "bonelog injected writer failure");
    }
#endif
    std::ofstream stream;
    stream.open(BONELOG_PATH, std::ios::out | std::ios::trunc);
    bool stream_ok = stream.is_open() && stream.good();
    if (!stream_ok)
    {
        // Permanently disabled output: producers see the
        // flag and stop producing; this loop only drains
        // (discarding, counted) until shutdown.
        state.writer_failed.store(true);
    }
    std::string out;
    out.reserve(RING_SLOT_RESERVE);
    std::string drops_line;
    for (;;)
    {
        {
            std::unique_lock<std::mutex> lock(state.mutex);
            state.cv.wait(lock, [&state] {
#ifdef FSVR_BONELOG_TEST_HOOKS
                // Test-only writer stall (state test):
                // while armed, the writer never drains, so
                // the producer ring can be forced full. The
                // setters exist only under this define; a
                // production build never stalls.
                if (s_test_writer_stall.load() &&
                    !state.stop_requested)
                {
                    return false;
                }
#endif
                return state.ring_count > 0 || state.stop_requested;
            });
        }
        // Drain everything currently queued.
        for (;;)
        {
            {
                std::lock_guard<std::mutex> lock(state.mutex);
                if (state.ring_count == 0)
                {
                    break;
                }
                out.swap(state.ring[state.ring_head]);
                state.ring_head =
                    (state.ring_head + 1) % RING_SLOTS;
                --state.ring_count;
            }
            if (stream_ok)
            {
                stream << out;
                if (!stream)
                {
                    stream_ok = false;
                    state.writer_failed.store(true);
                }
            }
            else
            {
                // Discarded by a dead writer: produced but
                // not written, so it counts as dropped.
                std::lock_guard<std::mutex> lock(state.mutex);
                ++state.dropped;
            }
            out.clear();
        }
        if (stream_ok)
        {
            stream.flush();
            if (!stream)
            {
                stream_ok = false;
                state.writer_failed.store(true);
            }
        }
        // Report the running drop total whenever it changed.
        U64 dropped_now = 0;
        {
            std::lock_guard<std::mutex> lock(state.mutex);
            dropped_now = state.dropped;
        }
        if (stream_ok && dropped_now != state.drops_reported)
        {
            FSVRBoneLog::buildDropsRecord(drops_line, nowMs(),
                                          dropped_now);
            stream << drops_line;
            stream.flush();
            if (stream)
            {
                state.drops_reported = dropped_now;
            }
            else
            {
                stream_ok = false;
                state.writer_failed.store(true);
            }
        }
        {
            std::lock_guard<std::mutex> lock(state.mutex);
            if (state.stop_requested && state.ring_count == 0)
            {
                break;
            }
        }
    }
    // Final drop record: always emitted on an orderly stop
    // with a healthy stream, even when the total is zero --
    // its presence proves the file was closed by the writer
    // and not abandoned mid-stream.
    if (stream_ok)
    {
        U64 dropped_total = 0;
        {
            std::lock_guard<std::mutex> lock(state.mutex);
            dropped_total = state.dropped;
        }
        FSVRBoneLog::buildDropsRecord(drops_line, nowMs(),
                                      dropped_total);
        stream << drops_line;
        stream.flush();
    }
    if (stream.is_open())
    {
        stream.close();
    }
    state.run_state.store(BS_DEAD);
}

// Writer-thread entry. The whole body sits under a
// top-level catch-all (BONELOG-R2-01): an uncaught
// exception escaping a std::thread entry function calls
// std::terminate, which would take the viewer down over a
// diagnostic's failure. On catch: mark the writer failed
// and the recorder DEAD -- producers cease on
// writer_failed (checked before any traversal or
// serialization), and ensureStarted refuses every
// non-NOT_STARTED state, so BS_DEAD is terminal here too
// -- with atomic stores only: no logging, formatting, or
// recovery attempt that could itself throw. The function
// then returns normally, so the std::thread object remains
// joinable for shutdown()'s orderly join.
void writerMain()
{
    BoneLogState& state = bonelogState();
#ifdef FSVR_BONELOG_TEST_HOOKS
    s_test_writer_entries.fetch_add(1);
#endif
    try
    {
        writerMainLoop();
    }
    catch (...)
    {
        state.writer_failed.store(true);
        state.run_state.store(BS_DEAD);
#ifdef FSVR_BONELOG_TEST_HOOKS
        // Both terminal stores are done: release a startup
        // caller held at the R3-01 barrier.
        testSignalWriterFailureComplete();
#endif
    }
}

// Start the ring + writer on first use. Any failure is
// permanent and quiet. Called from producers (main thread)
// with no locks held. BS_RUNNING is published BEFORE the
// writer thread is constructed (BONELOG-R3-01; see the
// ordering note at the top of this file), and every
// healthy check consults writer_failed as well: a failed
// writer -- a dead stream draining to shutdown, or a
// startup failure whose BS_DEAD store is still in flight
// behind its writer_failed store -- is never reported
// healthy on run_state alone.
bool ensureStarted()
{
    BoneLogState& state = bonelogState();
    const int run = state.run_state.load();
    if (run == BS_RUNNING)
    {
        return !state.writer_failed.load();
    }
    if (run != BS_NOT_STARTED)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(state.start_mutex);
    if (state.run_state.load() != BS_NOT_STARTED)
    {
        return state.run_state.load() == BS_RUNNING &&
               !state.writer_failed.load();
    }
    try
    {
        state.ring.assign(RING_SLOTS, std::string());
        for (std::string& slot : state.ring)
        {
            slot.reserve(RING_SLOT_RESERVE);
        }
        // Publish before construction: from the moment the
        // thread exists it may run (and fail) without this
        // thread executing another statement, and its
        // terminal BS_DEAD must be the last startup-era
        // store to run_state.
        state.run_state.store(BS_RUNNING);
        state.writer = std::thread(writerMain);
#ifdef FSVR_BONELOG_TEST_HOOKS
        // Test-only (state test): hold here until an armed
        // writer failure has fully completed, forcing the
        // BONELOG-R3-01 interleaving deterministically.
        testStartupBarrierHold();
#endif
    }
    catch (...)
    {
        state.writer_failed.store(true);
        state.run_state.store(BS_DEAD);
        return false;
    }
    // Construction succeeded -- but the writer may already
    // have failed (its catch publishes writer_failed and
    // BS_DEAD). Report what the writer's state says, not
    // the value published above.
    return !state.writer_failed.load();
}

// Hand one serialized record to the writer. The caller's
// string is swapped with the slot's (empty, capacity
// -retaining) string, so the producer keeps a reusable
// buffer. Never blocks on I/O; a full ring drops the
// record and counts it.
bool enqueueLine(std::string& line)
{
    BoneLogState& state = bonelogState();
    if (state.writer_failed.load())
    {
        return false;
    }
    if (!ensureStarted())
    {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(state.mutex);
        if (state.stop_requested)
        {
            return false;
        }
        if (state.ring_count >= RING_SLOTS)
        {
            ++state.dropped;
            return false;
        }
        state.ring[state.ring_tail].swap(line);
        state.ring_tail = (state.ring_tail + 1) % RING_SLOTS;
        ++state.ring_count;
    }
    state.cv.notify_one();
    return true;
}

// Pre-order traversal of the joint tree, parents before
// children, children in mChildren order: the skeleton order
// of the header record and of every sample's bones array.
void collectJoints(LLJoint* joint, std::vector<LLJoint*>& out)
{
    if (!joint)
    {
        return;
    }
    out.push_back(joint);
    for (LLJoint* child : joint->mChildren)
    {
        collectJoints(child, out);
    }
}

// Rebuild the cached traversal when the character or its
// skeleton generation changed; returns true when a fresh
// header record must be published before the next sample.
// Publication itself is tracked by header_pending in
// sampleCycleImpl (BONELOG-R2-02): this function advancing
// the cache does not emit anything.
bool refreshSkeleton(LLCharacter* character)
{
    BoneLogState& state = bonelogState();
    const U32 serial = character->getSkeletonSerialNum();
    if (state.header_character == character &&
        state.header_serial == serial &&
        !state.joints.empty())
    {
        return false;
    }
    state.joints.clear();
    collectJoints(character->getRootJoint(), state.joints);
    state.joint_names.clear();
    state.joint_names.reserve(state.joints.size());
    state.head_index = -1;
    for (S32 i = 0; i < static_cast<S32>(state.joints.size()); ++i)
    {
        state.joint_names.push_back(state.joints[i]->getName());
        if (state.joints[i]->getName() == "mHead")
        {
            state.head_index = i;
        }
    }
    state.header_character = character;
    state.header_serial = serial;
    return true;
}

void sampleCycleImpl(LLCharacter* character,
                     const FSVRAdapterPlanInput& input,
                     const FSVRAdapterPlanResult& result,
                     bool hmd_position_valid,
                     const LLVector3& hmd_position_local,
                     const FSVRBoneLogHeadRotSample* headrot,
                     const FSVRBoneLogBlendSample* blend)
{
    BoneLogState& state = bonelogState();
    if (state.writer_failed.load())
    {
        // Dead writer: no traversal, no serialization
        // (BONELOG-R1-03).
        return;
    }
    if (refreshSkeleton(character))
    {
        // New skeleton schema in force: its header must be
        // published before any sample under it (BONELOG
        // -R2-02). The cache advancing is not publication.
        state.header_pending = true;
    }
    // DIAG-1: a changed plan joint list pends a v2 header
    // re-emission under the same discipline (section 2.1).
    {
        bool plan_changed =
            input.joints.size() != state.plan_names.size();
        if (!plan_changed)
        {
            for (size_t i = 0; i < input.joints.size(); ++i)
            {
                if (input.joints[i].name != state.plan_names[i])
                {
                    plan_changed = true;
                    break;
                }
            }
        }
        if (plan_changed)
        {
            state.plan_pending = true;
        }
    }
    if (state.header_pending || state.plan_pending)
    {
        std::vector<std::string> current_plan_names;
        current_plan_names.reserve(input.joints.size());
        for (const FSVRAdapterJointInput& joint : input.joints)
        {
            current_plan_names.push_back(joint.name);
        }
        FSVRBoneLog::buildHeaderRecord(state.scratch,
                                       state.joint_names,
                                       current_plan_names,
                                       nowMs());
        if (enqueueLine(state.scratch))
        {
            state.header_pending = false;
            state.plan_pending = false;
            state.plan_names = current_plan_names;
        }
        else
        {
            // Ring full (or stopping): skip this cycle's
            // sample entirely and retry the header next
            // cycle. A sample written now would be read
            // under the previous header's joint names.
            return;
        }
    }
    if (state.joints.empty())
    {
        return;
    }

    // --- bones: every joint, world space, skeleton order.
    // getWorldPosition/getWorldRotation recompose through
    // updateWorldPRSParent from this cycle's locals (see
    // the currency note at the top of this file).
    state.bone_positions.clear();
    state.bone_rotations.clear();
    state.bone_positions.reserve(state.joints.size());
    state.bone_rotations.reserve(state.joints.size());
    for (LLJoint* joint : state.joints)
    {
        state.bone_positions.push_back(joint->getWorldPosition());
        state.bone_rotations.push_back(joint->getWorldRotation());
    }

    // --- hmd orientation: reconstructed from the manager's
    // adopted forward/up, validated (see the provenance
    // note at the top of this file).
    bool hmd_q_valid = false;
    LLQuaternion hmd_q;
    if (hmd_position_valid)
    {
        LLVector3 forward;
        LLVector3 up;
        LLOpenXRManager& openxr = LLOpenXRManager::instance();
        if (openxr.getHmdForward(forward) &&
            openxr.getHmdUp(up))
        {
            hmd_q_valid =
                reconstructHmdOrientation(forward, up, hmd_q);
        }
    }

    // --- camera: LLViewerCamera state at sample time (the
    // submitted view itself is the xr_submit trace's).
    LLViewerCamera* camera = LLViewerCamera::getInstance();
    const bool camera_valid = camera != nullptr;
    const LLVector3 camera_pos =
        camera_valid ? camera->getOrigin() : LLVector3::zero;
    const LLQuaternion camera_q =
        camera_valid ? camera->getQuaternion() : LLQuaternion();

    // --- target vs observed: only when the cycle composed
    // a head target and the skeleton has an mHead.
    bool target_valid = false;
    LLVector3 target_root;
    F32 observed_err = 0.f;
    if (input.head_target_valid && state.head_index >= 0)
    {
        const size_t head_idx =
            static_cast<size_t>(state.head_index);
        const LLVector3 achieved_cornea =
            state.bone_positions[head_idx] +
            input.cornea_offset_head_local *
                state.bone_rotations[head_idx];
        observed_err =
            (achieved_cornea - input.head_target_world).length();
        target_root =
            (input.head_target_world - input.root_world_position) *
                ~input.root_world_rotation;
        target_valid = true;
    }

    // --- DIAG-1 plan block (brief section 2.2): the
    // cycle's base and resolved poses in plan index space,
    // from the input and the plan result the recorder
    // already receives. Base locals are the input's own;
    // base worlds are the result's Route-A retention.
    FSVRBoneLogPlanData& plan = state.plan_scratch;
    plan.base_locals.clear();
    plan.domains.clear();
    plan.base_locals.reserve(input.joints.size());
    plan.domains.reserve(input.joints.size());
    for (const FSVRAdapterJointInput& joint : input.joints)
    {
        plan.base_locals.push_back(joint.base_local);
        FSVRBoneLogPlanDomain domain;
        domain.domain = static_cast<S32>(joint.domain);
        domain.decision_grants_base = joint.decision_grants_base;
        domain.disabled = joint.disabled;
        plan.domains.push_back(domain);
    }
    plan.base_world_positions = result.base_world_positions;
    plan.base_world_rotations = result.base_world_rotations;
    plan.resolved_locals = result.resolved_locals;
    plan.resolved_world_positions = result.resolved_world_positions;
    plan.resolved_world_rotations = result.resolved_world_rotations;
    plan.applied = result.applied;
    plan.gate_name =
        result.first_failing_gate == FSVRAdapterGate::NONE
            ? nullptr
            : FSVRAdapterApply::gateName(result.first_failing_gate);
    plan.spine_solver_invoked = result.spine_solver_invoked;
    plan.head_group_refusal = result.head_group_refusal;
    plan.writes_count =
        static_cast<U32>(result.writes.size());
    // P2-07B Fix 4 (CP-F4-3): the result's retained solver
    // target worlds, re-expressed in plan-joint order;
    // joints the solver did not target serialize as null.
    // An empty retention (the solver never ran, or
    // returned no targets) leaves the array absent
    // entirely (F4-F1). The scratch is reused across
    // cycles, so the vectors are rebuilt every cycle.
    plan.solver_targets_present =
        !result.solver_target_worlds.empty();
    plan.solver_targets.clear();
    plan.solver_target_mask.clear();
    if (plan.solver_targets_present)
    {
        plan.solver_targets.reserve(input.joints.size());
        plan.solver_target_mask.reserve(input.joints.size());
        for (S32 i = 0;
             i < static_cast<S32>(input.joints.size()); ++i)
        {
            auto it = result.solver_target_worlds.find(i);
            if (it != result.solver_target_worlds.end())
            {
                plan.solver_targets.push_back(it->second);
                plan.solver_target_mask.push_back(1);
            }
            else
            {
                plan.solver_targets.push_back(LLQuaternion());
                plan.solver_target_mask.push_back(0);
            }
        }
    }

    // P2-07B Fix 3 (brief section 4): the result's posture
    // behaviour in force and computed root offset, copied
    // only when the plan carried them (absent stays
    // absent in the record, F3-F).
    plan.posture_behaviour =
        result.posture_behaviour_present
            ? fsvrPostureBehaviourName(result.posture_behaviour)
            : nullptr;
    plan.root_offset_present = result.root_offset_present;
    plan.root_offset = result.root_offset;

    FSVRBoneLog::buildSampleRecordV2(
        state.scratch, nowMs(), input.frame_token,
        hmd_position_valid, hmd_position_local,
        hmd_q_valid, hmd_q,
        input.root_world_position, input.root_world_rotation,
        camera_valid, camera_pos, camera_q,
        state.bone_positions, state.bone_rotations,
        target_valid, target_root, observed_err,
        &plan, headrot, blend);
    enqueueLine(state.scratch);
}

} // namespace

// static
bool FSVRBoneLog::enabled()
{
    // Parsed once, cached -- the LLOpenXRTrace::enabled
    // shape (convertToBOOL; a nonempty value that does not
    // parse still enables). The spec's contract value is 1.
    static const bool s_enabled = [] {
        const std::string value = LLStringUtil::getenv(BONELOG_ENV);
        bool parsed = false;
        if (!value.empty() && LLStringUtil::convertToBOOL(value, parsed))
        {
            return parsed;
        }
        return !value.empty();
    }();
    return s_enabled;
}

// static
void FSVRBoneLog::sampleCycle(LLCharacter* character,
                              const FSVRAdapterPlanInput& input,
                              const FSVRAdapterPlanResult& result,
                              bool hmd_position_valid,
                              const LLVector3& hmd_position_local,
                              const FSVRBoneLogHeadRotSample* headrot,
                              const FSVRBoneLogBlendSample* blend)
{
    if (!enabled() || !character || s_hard_disabled.load())
    {
        return;
    }
    try
    {
        sampleCycleImpl(character, input, result,
                        hmd_position_valid, hmd_position_local,
                        headrot, blend);
    }
    catch (...)
    {
        // No recorder failure may interrupt the frame.
        s_hard_disabled.store(true);
    }
}

// static
void FSVRBoneLog::eventCalibrateRequested()
{
    if (!enabled() || s_hard_disabled.load())
    {
        return;
    }
    try
    {
        BoneLogState& state = bonelogState();
        if (state.writer_failed.load())
        {
            return;
        }
        buildCalibrateRequestedRecord(state.scratch, nowMs());
        enqueueLine(state.scratch);
    }
    catch (...)
    {
        s_hard_disabled.store(true);
    }
}

// static
void FSVRBoneLog::eventSyncExecuted(
    U64 frame_token,
    const LLVector3& anchor_hmd_local,
    F32 anchor_forward_yaw_rad,
    const LLVector3& cornea_offset_head_local,
    const LLVector3& neutral_cornea_root)
{
    if (!enabled() || s_hard_disabled.load())
    {
        return;
    }
    try
    {
        BoneLogState& state = bonelogState();
        if (state.writer_failed.load())
        {
            return;
        }
        buildSyncExecutedRecord(state.scratch, nowMs(),
                                frame_token, anchor_hmd_local,
                                anchor_forward_yaw_rad,
                                cornea_offset_head_local,
                                neutral_cornea_root);
        enqueueLine(state.scratch);
    }
    catch (...)
    {
        s_hard_disabled.store(true);
    }
}

// static
void FSVRBoneLog::eventSyncExecuted(
    U64 frame_token,
    const LLVector3& anchor_hmd_local,
    F32 anchor_forward_yaw_rad,
    const LLVector3& cornea_offset_head_local,
    const LLVector3& neutral_cornea_root,
    const char* posture,
    const char* lean_contract,
    const char* root_contract)
{
    if (!enabled() || s_hard_disabled.load())
    {
        return;
    }
    try
    {
        BoneLogState& state = bonelogState();
        if (state.writer_failed.load())
        {
            return;
        }
        buildSyncExecutedRecord(state.scratch, nowMs(),
                                frame_token, anchor_hmd_local,
                                anchor_forward_yaw_rad,
                                cornea_offset_head_local,
                                neutral_cornea_root,
                                posture, lean_contract,
                                root_contract);
        enqueueLine(state.scratch);
    }
    catch (...)
    {
        s_hard_disabled.store(true);
    }
}

// static
void FSVRBoneLog::eventPostureUndetermined()
{
    if (!enabled() || s_hard_disabled.load())
    {
        return;
    }
    try
    {
        BoneLogState& state = bonelogState();
        if (state.writer_failed.load())
        {
            return;
        }
        buildPostureUndeterminedRecord(state.scratch, nowMs());
        enqueueLine(state.scratch);
    }
    catch (...)
    {
        s_hard_disabled.store(true);
    }
}

// static
void FSVRBoneLog::eventScaleParameterInvalid(const char* channel)
{
    if (!enabled() || s_hard_disabled.load())
    {
        return;
    }
    try
    {
        BoneLogState& state = bonelogState();
        if (state.writer_failed.load())
        {
            return;
        }
        buildScaleParameterInvalidRecord(state.scratch, nowMs(),
                                         channel);
        enqueueLine(state.scratch);
    }
    catch (...)
    {
        s_hard_disabled.store(true);
    }
}

// static
void FSVRBoneLog::eventRootFollowGated()
{
    if (!enabled() || s_hard_disabled.load())
    {
        return;
    }
    try
    {
        BoneLogState& state = bonelogState();
        if (state.writer_failed.load())
        {
            return;
        }
        buildRootFollowGatedRecord(state.scratch, nowMs());
        enqueueLine(state.scratch);
    }
    catch (...)
    {
        s_hard_disabled.store(true);
    }
}

// static
void FSVRBoneLog::eventRootFollowBoundInvalid()
{
    if (!enabled() || s_hard_disabled.load())
    {
        return;
    }
    try
    {
        BoneLogState& state = bonelogState();
        if (state.writer_failed.load())
        {
            return;
        }
        buildRootFollowBoundInvalidRecord(state.scratch, nowMs());
        enqueueLine(state.scratch);
    }
    catch (...)
    {
        s_hard_disabled.store(true);
    }
}

// static
void FSVRBoneLog::eventBaseProvenanceReset(const char* joint_name,
                                           const char* reason)
{
    if (!enabled() || s_hard_disabled.load())
    {
        return;
    }
    try
    {
        BoneLogState& state = bonelogState();
        if (state.writer_failed.load())
        {
            return;
        }
        buildBaseProvenanceResetRecord(state.scratch, nowMs(),
                                       joint_name, reason);
        enqueueLine(state.scratch);
    }
    catch (...)
    {
        s_hard_disabled.store(true);
    }
}

// static
void FSVRBoneLog::eventBaseProvenanceUnclassified(
    const char* joint_name, F32 divergence_deg)
{
    if (!enabled() || s_hard_disabled.load())
    {
        return;
    }
    try
    {
        BoneLogState& state = bonelogState();
        if (state.writer_failed.load())
        {
            return;
        }
        buildBaseProvenanceUnclassifiedRecord(
            state.scratch, nowMs(), joint_name, divergence_deg);
        enqueueLine(state.scratch);
    }
    catch (...)
    {
        s_hard_disabled.store(true);
    }
}

// static
void FSVRBoneLog::shutdown()
{
    if (!enabled())
    {
        return;
    }
    BoneLogState& state = bonelogState();
    if (state.run_state.load() == BS_NOT_STARTED)
    {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.stop_requested = true;
        if (state.run_state.load() == BS_RUNNING)
        {
            state.run_state.store(BS_STOPPING);
        }
    }
    state.cv.notify_all();
    // Join scope, stated accurately: after the stop signal
    // the writer's remaining QUEUED WORK is bounded --
    // draining at most RING_SLOTS already-serialized records
    // to a local file, then one final record and close. That
    // bounds the amount of work, not the elapsed time of
    // this join: the writer's only waits are the condition
    // variable (notified above) and those filesystem
    // operations, and a stalled filesystem can still stall
    // shutdown -- the same exposure as any orderly flush at
    // exit, accepted for a diagnostic. If this hook is never
    // reached (forced process exit), the process dies with
    // the thread; no path blocks on it.
    if (state.writer.joinable())
    {
        state.writer.join();
    }
}

#ifdef FSVR_BONELOG_TEST_HOOKS

// ---- Test-only hooks (standalone state test) ------------
// Defined only under FSVR_BONELOG_TEST_HOOKS; no production
// build contains them.

// static
void FSVRBoneLog::testArmWriterThrow(bool arm)
{
    s_test_writer_throw_armed.store(arm);
}

// static
void FSVRBoneLog::testSetWriterStall(bool stall)
{
    s_test_writer_stall.store(stall);
    // Wake the writer so it observes the new stall state
    // (and, when clearing, resumes draining).
    bonelogState().cv.notify_all();
}

// static
bool FSVRBoneLog::testWriterFailed()
{
    return bonelogState().writer_failed.load();
}

// static
int FSVRBoneLog::testRunState()
{
    return bonelogState().run_state.load();
}

// static
U64 FSVRBoneLog::testDroppedCount()
{
    BoneLogState& state = bonelogState();
    std::lock_guard<std::mutex> lock(state.mutex);
    return state.dropped;
}

// static
U32 FSVRBoneLog::testQueuedCount()
{
    BoneLogState& state = bonelogState();
    std::lock_guard<std::mutex> lock(state.mutex);
    return state.ring_count;
}

// static
bool FSVRBoneLog::testHeaderPending()
{
    return bonelogState().header_pending;
}

// static
void FSVRBoneLog::testArmStartupBarrier(bool arm)
{
    {
        std::lock_guard<std::mutex> lock(s_test_barrier_mutex);
        s_test_failure_complete = false;
    }
    s_test_startup_barrier_armed.store(arm);
}

// static
bool FSVRBoneLog::testFailureComplete()
{
    std::lock_guard<std::mutex> lock(s_test_barrier_mutex);
    return s_test_failure_complete;
}

// static
U32 FSVRBoneLog::testWriterEntryCount()
{
    return s_test_writer_entries.load();
}

#endif // FSVR_BONELOG_TEST_HOOKS

#endif // FSVR_BONELOG_SERIALIZE_ONLY
