/**
 * @file llvrcalibration.cpp
 * @brief P2-07B C5: calibration command, transaction, and
 *        session-neutral acquisition. See llvrcalibration.h
 *        for the contract.
 *
 * Layout (brief CP-2): the pure state machines and helpers
 * sit above the FSVR_CALIBRATION_PURE_ONLY guard (no
 * globals, harness-compilable); the viewer shell sits below
 * it. The transaction commits solely through C4's
 * FSVRTrackingSettings::writeCalibrationRecord(), exactly
 * once per successful transaction, executed by the shell
 * from the machine's COMMIT action.
 */

#include "llviewerprecompiledheaders.h"

#include "llvrcalibration.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

LLVector3d toVec3d(const LLVector3& v)
{
    return LLVector3d(v.mV[VX], v.mV[VY], v.mV[VZ]);
}

LLVector3 toVec3(const LLVector3d& v)
{
    return LLVector3(static_cast<F32>(v.mdV[VX]),
                     static_cast<F32>(v.mdV[VY]),
                     static_cast<F32>(v.mdV[VZ]));
}

// Angular deviation between two rotations, in radians, in
// [0, pi]. The angle is extracted in F64 from the (F32)
// production quaternion product, so rounding-scale
// deviations are measured, not hidden.
F64 quatAngleBetween(const LLQuaternion& a, const LLQuaternion& b)
{
    LLQuaternion an = a;
    an.normalize();
    LLQuaternion bn = b;
    bn.normalize();
    LLQuaternion d = an * ~bn;
    F64 w = std::fabs(static_cast<F64>(d.mQ[VW]));
    if (w > 1.0)
    {
        w = 1.0;
    }
    return 2.0 * std::acos(w);
}

bool finiteVec(const LLVector3& v)
{
    return std::isfinite(v.mV[VX]) && std::isfinite(v.mV[VY]) &&
           std::isfinite(v.mV[VZ]);
}

// The stage-2 yaw convention (llagentcamera.cpp:379, cited
// by the C6-E brief's ICR-1): atan2f(f.x, -f.z) of the
// adopted sample's forward.
F32 stage2YawOfForward(const LLVector3& forward)
{
    return std::atan2(forward.mV[VX], -forward.mV[VZ]);
}

// C5-CODE-R1-03: adopted-forward evidence for ICR-1's
// session base yaw. A yaw exists only when the adopted
// sample's forward is flagged valid, finite in every
// component, and nondegenerate in the horizontal plane --
// the stage-2 yaw is atan2(f.x, -f.z), so a forward with
// no horizontal component (straight up or down) has no yaw
// to take. A tracked position alone establishes none of
// this. There is no default: establishment gates on this
// predicate and a zero yaw is never substituted for
// missing evidence.
bool adoptedForwardHasYaw(const LLVector3& forward, bool valid)
{
    if (!valid || !finiteVec(forward))
    {
        return false;
    }
    const F64 hx = static_cast<F64>(forward.mV[VX]);
    const F64 hz = static_cast<F64>(forward.mV[VZ]);
    const F64 horizontal = std::sqrt(hx * hx + hz * hz);
    return std::isfinite(horizontal) && horizontal > 0.0;
}

} // namespace

// ---------------------------------------------------------------------------
// CP-3 helpers.
// ---------------------------------------------------------------------------

// static
const std::vector<std::string>& FSVRCalibration::holdJointNames()
{
    // Spec section 3.1's in-scope set. The canonical list
    // lives in the plan TU (FSVRAdapterApply::
    // calibrationHoldJointNames) so the plan's hold branch
    // and this module share one list by construction and
    // the dependency direction stays one-way (this module
    // depends on the plan; the plan never depends back).
    return FSVRAdapterApply::calibrationHoldJointNames();
}

// static
std::vector<S32> FSVRCalibration::resolveHoldJoints(
    const FSVRAdapterPlanInput& input,
    std::vector<std::string>* out_missing)
{
    std::vector<S32> indices;
    if (out_missing)
    {
        out_missing->clear();
    }
    for (const std::string& name : holdJointNames())
    {
        S32 found = -1;
        for (S32 i = 0; i < static_cast<S32>(input.joints.size()); ++i)
        {
            if (input.joints[i].name == name)
            {
                found = i;
                break;
            }
        }
        if (found >= 0)
        {
            indices.push_back(found);
        }
        else if (out_missing)
        {
            out_missing->push_back(name);
        }
    }
    return indices;
}

// static
bool FSVRCalibration::productionStationarityParams(
    FSVRStationarityParams& out_params)
{
    // DP-C5-12 / OPEN-6: the one named production source.
    // DP-B4's measured values do not exist yet; this source
    // returns UNMEASURED and no plausible-looking constants
    // appear here or anywhere in C5. Installing the
    // measured values is a data-only change at this one
    // source.
    (void)out_params;
    return false;
}

// static
FSVRStationarityVerdict FSVRCalibration::decideStationarity(
    const std::vector<FSVRCalSample>& window,
    const FSVRStationarityParams& params)
{
    FSVRStationarityVerdict verdict;

    // C5-CODE-R1-04: the structural invariants are enforced
    // here, inside the helper, independently of the
    // caller-provided parameter values -- a degenerate
    // parameter set or a corrupt window must never classify
    // as STATIONARY. This validates structure only; no
    // OPEN-6 threshold is invented (the bounds themselves
    // remain the caller's, from DP-B4's protocol).
    //
    // Parameter validity first: a bound that is negative or
    // nonfinite cannot certify stillness, so such parameters
    // classify every window as failing the bound (MOVING)
    // -- the fail-closed reading: loud in the transaction
    // (abort), window-restarting in acquisition.
    if (!std::isfinite(params.min_duration_s) ||
        params.min_duration_s < 0.0 ||
        !std::isfinite(params.max_spread_norm) ||
        params.max_spread_norm < 0.0 ||
        !std::isfinite(params.max_spread_axis) ||
        params.max_spread_axis < 0.0)
    {
        verdict.reason = FSVRStationarityReason::MOVING;
        return verdict;
    }
    // DP-B4's structural rules, in order. A window
    // containing any invalid-HMD sample is TRACKER_LOSS
    // (loss voids the window; it is never averaged
    // through) -- extended, per the same principle, to any
    // sample that is not a genuine finite tracked reading:
    // a nonfinite HMD position, a nonfinite wrist position
    // on a wrist-valid sample, or a broken timestamp
    // (nonfinite, or earlier than its predecessor's -- the
    // shell supplies monotonic seconds) voids the window
    // identically.
    for (size_t i = 0; i < window.size(); ++i)
    {
        const FSVRCalSample& sample = window[i];
        bool genuine = sample.hmd_valid && finiteVec(sample.hmd_pos) &&
                       std::isfinite(sample.time_s) &&
                       (i == 0 || sample.time_s >= window[i - 1].time_s);
        for (S32 s = 0; genuine && s < 2; ++s)
        {
            genuine = !sample.wrist_valid[s] ||
                      finiteVec(sample.wrist_pos[s]);
        }
        if (!genuine)
        {
            verdict.reason = FSVRStationarityReason::TRACKER_LOSS;
            return verdict;
        }
    }
    // A window containing a transition marker is TRANSITION
    // (in-flight samples are invalid across a redefinition
    // of their frame).
    for (const FSVRCalSample& sample : window)
    {
        if (sample.transition_marker)
        {
            verdict.reason = FSVRStationarityReason::TRANSITION;
            return verdict;
        }
    }
    // Fewer than min_samples or shorter than min_duration_s
    // is INSUFFICIENT. The two-sample floor is structural
    // and applies whatever the caller passes: a single
    // sample is never accepted (spec section 4.2 step 4),
    // even under a parameter set with min_samples == 1.
    if (window.size() < 2 ||
        window.size() < static_cast<size_t>(params.min_samples))
    {
        verdict.reason = FSVRStationarityReason::INSUFFICIENT;
        return verdict;
    }
    const F64 duration = window.back().time_s - window.front().time_s;
    if (!std::isfinite(duration) || duration < params.min_duration_s)
    {
        verdict.reason = FSVRStationarityReason::INSUFFICIENT;
        return verdict;
    }

    // Spread about the window's center (DP-C5-10: the mean
    // is that center), per-axis and norm; both must be
    // within bound.
    LLVector3d mean(0.0, 0.0, 0.0);
    for (const FSVRCalSample& sample : window)
    {
        mean += toVec3d(sample.hmd_pos);
    }
    mean *= 1.0 / static_cast<F64>(window.size());
    // Derived values are validated, not assumed
    // (C5-CODE-R1-04): std::max does not propagate NaN, so
    // a nonfinite mean or spread would otherwise fall
    // through every bound comparison to STATIONARY. A
    // spread that cannot be finitely computed has not been
    // shown to be within any bound: MOVING.
    if (!std::isfinite(mean.mdV[VX]) || !std::isfinite(mean.mdV[VY]) ||
        !std::isfinite(mean.mdV[VZ]))
    {
        verdict.reason = FSVRStationarityReason::MOVING;
        return verdict;
    }

    F64 max_axis = 0.0;
    F64 max_norm = 0.0;
    for (const FSVRCalSample& sample : window)
    {
        const LLVector3d dev = toVec3d(sample.hmd_pos) - mean;
        max_axis = std::max({max_axis, std::fabs(dev.mdV[VX]),
                             std::fabs(dev.mdV[VY]),
                             std::fabs(dev.mdV[VZ])});
        max_norm = std::max(max_norm, dev.length());
    }
    if (!std::isfinite(max_axis) || !std::isfinite(max_norm) ||
        max_axis > params.max_spread_axis ||
        max_norm > params.max_spread_norm)
    {
        verdict.reason = FSVRStationarityReason::MOVING;
        return verdict;
    }

    verdict.reason = FSVRStationarityReason::STATIONARY;
    verdict.hmd_representative = toVec3(mean);
    // Wrist representatives: per side, the mean of that
    // side's valid samples. (L_P's distances are computed
    // per sample by the caller, never from these endpoints.)
    for (S32 s = 0; s < 2; ++s)
    {
        LLVector3d sum(0.0, 0.0, 0.0);
        U32 count = 0;
        for (const FSVRCalSample& sample : window)
        {
            if (sample.wrist_valid[s])
            {
                sum += toVec3d(sample.wrist_pos[s]);
                ++count;
            }
        }
        if (count > 0)
        {
            verdict.side_measured[s] = true;
            sum *= 1.0 / static_cast<F64>(count);
            verdict.wrist_representative[s] = toVec3(sum);
        }
    }
    return verdict;
}

// static
bool FSVRCalibration::decideUprightOrientation(const LLVector3& hmd_up)
{
    // DP-C5-6: cos(20 degrees) -- a posture constant, not
    // an OPEN-6 noise bound. One threshold serves both
    // callers (PB-R1-06).
    static constexpr F64 UPRIGHT_COS_THRESHOLD = 0.93969262078590838;
    const LLVector3d up = toVec3d(hmd_up);
    const F64 len = up.length();
    if (!(len > 0.0) || !std::isfinite(len))
    {
        return false;
    }
    // LOCAL is Y-up: the world-up direction in the frame is
    // the frame's +Y.
    const F64 dot = up.mdV[VY] / len;
    return dot >= UPRIGHT_COS_THRESHOLD;
}

// static
FSVRCalRatioResolution FSVRCalibration::resolveCalibrationRatios(
    const FSVRCalLandmarkSet& landmarks,
    const FSVRCalibrationRecordValidation& prior)
{
    FSVRCalRatioResolution res;

    // CP-3c rule 1: the per-side avatar distances are
    // always computable from the held pose alone.
    F64 a[2];
    for (S32 s = 0; s < 2; ++s)
    {
        a[s] = (landmarks.avatar_wrist_world[s] -
                landmarks.cornea_midpoint_agent)
                   .length();
    }
    // Per-side player distances exist only for sides with
    // valid wrist samples: the mean of the side's
    // per-sample simultaneous distances (DP-C5-10).
    bool side_valid[2];
    F64 p[2] = {0.0, 0.0};
    for (S32 s = 0; s < 2; ++s)
    {
        side_valid[s] = !landmarks.player_dist_samples[s].empty();
        if (side_valid[s])
        {
            F64 sum = 0.0;
            for (F64 d : landmarks.player_dist_samples[s])
            {
                sum += d;
            }
            p[s] = sum / static_cast<F64>(
                             landmarks.player_dist_samples[s].size());
        }
    }

    const bool both = side_valid[0] && side_valid[1];
    const bool one = side_valid[0] != side_valid[1];
    const S32 only_side = side_valid[0] ? 0 : 1;

    F64 L_A = 0.0;
    if (both)
    {
        // Rule 2 (full capture): the paired measured row.
        L_A = (a[0] + a[1]) / 2.0;
        const F64 L_P = (p[0] + p[1]) / 2.0;
        res.ratio_measured = true;
        res.head_only = false;
        res.avatar_measure_LA = L_A;
        res.player_measure_LP_stored = L_P;
        res.head_ratio = (L_P != 0.0) ? (L_A / L_P) : 0.0;
        // Arm ratios per side (spec section 4.5): the
        // head-relative lateral reach, avatar / player.
        for (S32 s = 0; s < 2; ++s)
        {
            const LLVector3d rel =
                landmarks.avatar_wrist_world[s] -
                landmarks.cornea_midpoint_agent;
            res.avatar_reach[s] =
                rel.mdV[VX] * landmarks.avatar_right.mdV[VX] +
                rel.mdV[VY] * landmarks.avatar_right.mdV[VY] +
                rel.mdV[VZ] * landmarks.avatar_right.mdV[VZ];
            F64 lat_sum = 0.0;
            for (F64 v : landmarks.player_lateral_samples[s])
            {
                lat_sum += v;
            }
            res.player_reach[s] =
                landmarks.player_lateral_samples[s].empty()
                    ? 0.0
                    : lat_sum / static_cast<F64>(
                                    landmarks.player_lateral_samples[s].size());
            res.arm_ratio[s] = (res.player_reach[s] != 0.0)
                                   ? (res.avatar_reach[s] / res.player_reach[s])
                                   : 0.0;
        }
    }
    else if (one)
    {
        // Rule 5 (DP-C5-13): measured head ratio from the
        // valid side alone; arm fields absent (the
        // validator admits no partial arm set).
        L_A = a[only_side];
        const F64 L_P = p[only_side];
        res.ratio_measured = true;
        res.head_only = true;
        res.avatar_measure_LA = L_A;
        res.player_measure_LP_stored = L_P;
        res.head_ratio = (L_P != 0.0) ? (L_A / L_P) : 0.0;
    }
    else
    {
        // Zero-wrist rows: L_A is the standalone avatar-side
        // mean over BOTH avatar sides (rule 1's table).
        L_A = (a[0] + a[1]) / 2.0;
        res.head_only = true;
        res.avatar_measure_LA = L_A;
        const bool prior_valid =
            prior.status == FSVRCalibrationRecordValidation::
                                RecordStatus::USABLE_FULL ||
            prior.status == FSVRCalibrationRecordValidation::
                                RecordStatus::USABLE_HEAD_ONLY;
        if (prior_valid)
        {
            // Rule 3 ([R3] carry-forward): preserve the
            // prior RATIO only; the stored player field is
            // the implied denominator L_A_new / r_prev,
            // derived to keep the section 4.3 identity
            // exact -- not a measured player reach.
            res.ratio_carried = true;
            res.head_ratio = prior.head_ratio;
            res.player_measure_LP_stored =
                (prior.head_ratio != 0.0)
                    ? (L_A / prior.head_ratio)
                    : 0.0;
        }
        else
        {
            // Rule 4 (disclosed placeholder): head_ratio
            // 1.0, explicitly unmeasured; the stored player
            // field is L_A / head_ratio so the identity
            // holds.
            res.ratio_placeholder = true;
            res.head_ratio = 1.0;
            res.player_measure_LP_stored = L_A;
        }
    }

    // Rule 6 (spec section 4.2 step 5): finiteness and
    // plausibility only -- no invented numeric caps. A
    // zero/negative measure is not a T-pose capture; a
    // measured L_P of zero fails here (its ratio would be
    // non-finite); arm ratios must be positive when
    // present (DP-C5-9's loud-abort failure mode).
    bool ok = std::isfinite(res.head_ratio) && res.head_ratio > 0.0 &&
              std::isfinite(res.avatar_measure_LA) &&
              res.avatar_measure_LA > 0.0 &&
              std::isfinite(res.player_measure_LP_stored);
    if (res.ratio_carried)
    {
        // The stored identity as constructed: the implied
        // denominator must be positive and finite whenever
        // L_A is.
        ok = ok && res.player_measure_LP_stored > 0.0;
    }
    if (res.ratio_measured)
    {
        ok = ok && res.player_measure_LP_stored > 0.0;
    }
    if (!res.head_only)
    {
        for (S32 s = 0; s < 2; ++s)
        {
            ok = ok && std::isfinite(res.arm_ratio[s]) &&
                 res.arm_ratio[s] > 0.0 &&
                 std::isfinite(res.avatar_reach[s]) &&
                 std::isfinite(res.player_reach[s]);
        }
    }
    res.valid = ok;
    return res;
}

// static
LLSD FSVRCalibration::buildCalibrationRecord(
    const FSVRCalRatioResolution& resolution, S64 captured_utc)
{
    // Exactly the section 4.3 map and nothing else (key
    // set equality is an H5 fixture): version stays 1;
    // floor_verified is true under DP-B1's amended meaning
    // (the capture's anchor was verified through the
    // section 4.2 gates).
    LLSD record = LLSD::emptyMap();
    record["version"] =
        static_cast<S32>(FSVRTrackingSettings::CALIBRATION_RECORD_SCHEMA_VERSION);
    // LLSD integers are S32; the validator reads captured_utc
    // only as an integer (diagnostics only).
    record["captured_utc"] = static_cast<S32>(captured_utc);
    record["floor_verified"] = true;
    record["head_ratio"] = resolution.head_ratio;
    record["avatar_cornea_height"] = resolution.avatar_measure_LA;
    record["player_hmd_height"] = resolution.player_measure_LP_stored;
    if (!resolution.head_only)
    {
        record["arm_ratio_left"] = resolution.arm_ratio[0];
        record["arm_ratio_right"] = resolution.arm_ratio[1];
        record["avatar_reach_left"] = resolution.avatar_reach[0];
        record["avatar_reach_right"] = resolution.avatar_reach[1];
        record["player_reach_left"] = resolution.player_reach[0];
        record["player_reach_right"] = resolution.player_reach[1];
    }
    return record;
}

// static
void FSVRCalibration::composeBindWorlds(
    const FSVRAdapterPlanInput& input,
    const std::vector<LLQuaternion>& locals,
    std::vector<LLQuaternion>& out_rotations,
    std::vector<LLVector3>& out_positions)
{
    // CP-3e: the transaction composition's rule over
    // plan-input data, without the production path's
    // intermediate normalization (see the header).
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
        out_rotations[i] = locals[i] * parent_rot;
        out_positions[i] = parent_pos + (joint.rest_offset * parent_rot);
    }
}

namespace
{

// C5-CODE-R1-01 / PB-R1-03: the root-relative neutral
// cornea, shared by both establishment paths (the
// transaction's RESOLVE and section 5 acquisition).
// composeBindWorlds composes over the plan input's root
// frame, so its output is WORLD space; the stored neutral
// must instead be expressed in the root frame itself,
// because C6-E applies the avatar's current root frame to
// it per cycle -- storing the world value would apply the
// root transform twice. Composing the same bind
// configuration with an identity root frame yields the
// root-relative value directly: identical chain
// arithmetic, the root transform factored out by
// construction.
LLVector3 neutralCorneaRootRelative(
    const FSVRAdapterPlanInput& plan, S32 head_index,
    const LLVector3& cornea_offset_head_local)
{
    FSVRAdapterPlanInput root_frame = plan;
    root_frame.root_world_position = LLVector3::zero;
    root_frame.root_world_rotation = LLQuaternion();
    const S32 count = static_cast<S32>(root_frame.joints.size());
    const std::vector<LLQuaternion> identity_locals(count, LLQuaternion());
    std::vector<LLQuaternion> bind_rotations;
    std::vector<LLVector3> bind_positions;
    FSVRCalibration::composeBindWorlds(root_frame, identity_locals,
                                       bind_rotations, bind_positions);
    return bind_positions[head_index] +
           (cornea_offset_head_local * bind_rotations[head_index]);
}

} // namespace

// static
FSVRPoseTolerance FSVRCalibration::derivePoseTolerance(
    const FSVRAdapterPlanInput& bind_composition_inputs)
{
    FSVRPoseTolerance tol;
    const FSVRAdapterPlanInput& input = bind_composition_inputs;
    const S32 count = static_cast<S32>(input.joints.size());
    if (count == 0)
    {
        return tol;
    }

    // Path A: the production transaction composition --
    // computePlan over a copy whose base locals are the
    // bind locals (identity) and whose hold flag is off.
    // computePlan computes the base composition in every
    // outcome, so the returned resolved pose IS path A's
    // bind composition regardless of gates.
    FSVRAdapterPlanInput probe = input;
    for (FSVRAdapterJointInput& joint : probe.joints)
    {
        joint.base_local = LLQuaternion();
    }
    probe.calibration_hold_active = false;
    FSVRAdapterNullSpineSolver null_spine;
    FSVRAdapterNullArmSolver null_arm;
    const FSVRAdapterPlanResult path_a =
        FSVRAdapterApply::computePlan(probe, null_spine, null_arm);

    // Path B: CP-3e's expectation composition.
    const std::vector<LLQuaternion> identity_locals(
        count, LLQuaternion());
    std::vector<LLQuaternion> path_b_rotations;
    std::vector<LLVector3> path_b_positions;
    composeBindWorlds(input, identity_locals, path_b_rotations,
                      path_b_positions);

    if (static_cast<S32>(path_a.resolved_world_positions.size()) != count)
    {
        return tol;
    }

    // The probe's own magnitudes: S = the largest
    // composed-position magnitude across both paths.
    // Finiteness is tracked explicitly: std::max would
    // silently drop a NaN deviation (every comparison with
    // NaN is false), so a non-finite composition must
    // poison the derivation through this flag, never
    // through the accumulated maxima (CP-3d: a non-finite
    // measured base fails the derivation).
    bool finite = true;
    F64 S = 0.0;
    for (S32 i = 0; i < count; ++i)
    {
        const F64 len_a =
            toVec3d(path_a.resolved_world_positions[i]).length();
        const F64 len_b = toVec3d(path_b_positions[i]).length();
        finite = finite && std::isfinite(len_a) && std::isfinite(len_b);
        if (std::isfinite(len_a))
        {
            S = std::max(S, len_a);
        }
        if (std::isfinite(len_b))
        {
            S = std::max(S, len_b);
        }
    }

    // Measured bases: the maximum discrepancy between the
    // two paths, per check, across the hold set.
    const std::vector<S32> hold = resolveHoldJoints(input, nullptr);
    F64 base_orientation = 0.0;
    F64 base_position = 0.0;
    for (S32 idx : hold)
    {
        const F64 dev_orientation = quatAngleBetween(
            path_a.resolved_world_rotations[idx], path_b_rotations[idx]);
        const F64 dev_position =
            (toVec3d(path_a.resolved_world_positions[idx]) -
             toVec3d(path_b_positions[idx]))
                .length();
        finite = finite && std::isfinite(dev_orientation) &&
                 std::isfinite(dev_position);
        if (std::isfinite(dev_orientation))
        {
            base_orientation = std::max(base_orientation, dev_orientation);
        }
        if (std::isfinite(dev_position))
        {
            base_position = std::max(base_position, dev_position);
        }
    }
    if (!finite)
    {
        return tol;
    }

    const F64 eps = std::numeric_limits<double>::epsilon();
    const F64 floor_position = 1024.0 * eps * std::max(1.0, S);
    const F64 floor_orientation = 1024.0 * eps;
    const F64 tol_position = std::max(
        base_position * POSE_TOLERANCE_SAFETY_FACTOR, floor_position);
    const F64 tol_orientation = std::max(
        base_orientation * POSE_TOLERANCE_SAFETY_FACTOR,
        floor_orientation);

    // Validity (CP-3d): a non-finite measured base, or a
    // tolerance larger than the pose it is meant to verify,
    // fails the derivation outright.
    if (!std::isfinite(base_position) || !std::isfinite(base_orientation) ||
        !std::isfinite(tol_position) || !std::isfinite(tol_orientation))
    {
        return tol;
    }
    if (tol_position >= std::max(S, 1.0) ||
        tol_orientation >= 1.5707963267948966 /* pi/2 */)
    {
        return tol;
    }
    tol.valid = true;
    tol.orientation_rad = tol_orientation;
    tol.position_m = tol_position;
    return tol;
}

// static
bool FSVRCalibration::heldPoseOrientationOk(
    const FSVRAdapterPlanResult& result,
    const std::vector<S32>& hold_joint_indices,
    const FSVRPoseTolerance& tolerance,
    S32* out_first_failing)
{
    if (out_first_failing)
    {
        *out_first_failing = -1;
    }
    if (!tolerance.valid)
    {
        return false;
    }
    const LLQuaternion identity;
    for (S32 idx : hold_joint_indices)
    {
        if (idx < 0 ||
            idx >= static_cast<S32>(result.resolved_locals.size()))
        {
            return false;
        }
        if (quatAngleBetween(result.resolved_locals[idx], identity) >
            tolerance.orientation_rad)
        {
            if (out_first_failing)
            {
                *out_first_failing = idx;
            }
            return false;
        }
    }
    return true;
}

// static
bool FSVRCalibration::heldPosePositionOk(
    const FSVRAdapterPlanInput& input,
    const FSVRAdapterPlanResult& result,
    const std::vector<S32>& hold_joint_indices,
    const FSVRPoseTolerance& tolerance,
    S32* out_first_failing)
{
    if (out_first_failing)
    {
        *out_first_failing = -1;
    }
    if (!tolerance.valid)
    {
        return false;
    }
    const S32 count = static_cast<S32>(input.joints.size());
    if (static_cast<S32>(result.resolved_world_positions.size()) != count)
    {
        return false;
    }
    const std::vector<LLQuaternion> identity_locals(
        count, LLQuaternion());
    std::vector<LLQuaternion> expected_rotations;
    std::vector<LLVector3> expected_positions;
    composeBindWorlds(input, identity_locals, expected_rotations,
                      expected_positions);
    for (S32 idx : hold_joint_indices)
    {
        if (idx < 0 || idx >= count)
        {
            return false;
        }
        const F64 dev = (toVec3d(result.resolved_world_positions[idx]) -
                         toVec3d(expected_positions[idx]))
                            .length();
        if (!(dev <= tolerance.position_m))
        {
            if (out_first_failing)
            {
                *out_first_failing = idx;
            }
            return false;
        }
    }
    return true;
}

// static
bool FSVRCalibration::verifyHeldPose(
    const FSVRAdapterPlanInput& input,
    const FSVRAdapterPlanResult& result,
    const std::vector<S32>& hold_joint_indices,
    const FSVRPoseTolerance& tolerance,
    S32& out_first_failing_joint)
{
    out_first_failing_joint = -1;
    if (!tolerance.valid)
    {
        return false;
    }
    S32 failing = -1;
    if (!heldPoseOrientationOk(result, hold_joint_indices, tolerance,
                               &failing))
    {
        out_first_failing_joint = failing;
        return false;
    }
    if (!heldPosePositionOk(input, result, hold_joint_indices, tolerance,
                            &failing))
    {
        out_first_failing_joint = failing;
        return false;
    }
    return true;
}

// static
bool FSVRCalibration::settleObservationStable(
    const FSVRCalSettleObservation& prev,
    const FSVRCalSettleObservation& cur,
    const FSVRPoseTolerance& tolerance)
{
    if (!tolerance.valid)
    {
        return false;
    }
    const F64 tol = tolerance.position_m;
    // (i) Per-joint offset change at or below the section
    // 3.2 recomposition tolerance.
    if (prev.hold_joint_offsets.size() != cur.hold_joint_offsets.size())
    {
        return false;
    }
    for (size_t i = 0; i < cur.hold_joint_offsets.size(); ++i)
    {
        const F64 dev = (toVec3d(cur.hold_joint_offsets[i]) -
                         toVec3d(prev.hold_joint_offsets[i]))
                            .length();
        if (!(dev <= tol))
        {
            return false;
        }
    }
    // (ii) mPelvisToFoot stable across consecutive reads.
    if (std::fabs(static_cast<F64>(cur.pelvis_to_foot) -
                  static_cast<F64>(prev.pelvis_to_foot)) > tol)
    {
        return false;
    }
    // (iii) The derived cornea landmark stable across
    // consecutive cycles. Compared only when both cycles
    // carry a reading; a reading appearing or disappearing
    // between cycles is not stability.
    if (prev.cornea_valid != cur.cornea_valid)
    {
        return false;
    }
    if (cur.cornea_valid)
    {
        const F64 dev = (toVec3d(cur.cornea_midpoint_agent) -
                         toVec3d(prev.cornea_midpoint_agent))
                            .length();
        if (!(dev <= tol))
        {
            return false;
        }
    }
    return true;
}

// static
LLVector3 FSVRCalibration::deriveCorneaOffsetHeadLocal(
    const LLVector3& cornea_midpoint_world,
    const LLVector3& head_world_position,
    const LLQuaternion& head_world_rotation)
{
    // offset = R_head^-1 . (E_A - p_head), with the tree's
    // own F64 rotation application (operator*(LLVector3d,
    // LLQuaternion) in llquaternion.cpp). E_A enters as
    // read -- no conversion (brief section 2.4).
    LLQuaternion rot = head_world_rotation;
    rot.normalize();
    const LLVector3d d =
        toVec3d(cornea_midpoint_world) - toVec3d(head_world_position);
    const LLVector3d offset = d * ~rot;
    return toVec3(offset);
}

// static
bool FSVRCalibration::corneaReconstructionStable(
    const std::vector<LLVector3>& offsets_over_window, F64 tolerance_m)
{
    if (offsets_over_window.empty())
    {
        return false;
    }
    LLVector3d mean(0.0, 0.0, 0.0);
    for (const LLVector3& offset : offsets_over_window)
    {
        if (!finiteVec(offset))
        {
            return false;
        }
        mean += toVec3d(offset);
    }
    mean *= 1.0 / static_cast<F64>(offsets_over_window.size());
    for (const LLVector3& offset : offsets_over_window)
    {
        if ((toVec3d(offset) - mean).length() > tolerance_m)
        {
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// CP-2: the calibration transaction machine.
// ---------------------------------------------------------------------------

bool FSVRCalibrationTransaction::begin()
{
    if (active())
    {
        return false;
    }
    mState = FSVRTransactionState::ENTERING;
    mHaveStepped = false;
    mLastSteppedS = 0.0;
    mHoldIndices.clear();
    mMissingHoldJoints.clear();
    mHoldResolved = false;
    mHeadIndex = -1;
    mWristIndex[0] = -1;
    mWristIndex[1] = -1;
    mTolerance = FSVRPoseTolerance();
    mHaveHeldCycle = false;
    mHaveSettleObservation = false;
    mPrevSettle = FSVRCalSettleObservation();
    mSettleStableCount = 0;
    mSettleCycles = 0;
    mSettled = false;
    mHaveEpochBaseline = false;
    mEpochBaseline = 0;
    mWindow.clear();
    mWindowEpoch = 0;
    mVerdict = FSVRStationarityVerdict();
    mResolution = FSVRCalRatioResolution();
    mAcceptedForwardValid = false;
    mBuiltRecord = LLSD();
    return true;
}

std::vector<FSVRTransactionAction> FSVRCalibrationTransaction::abortWith(
    const char* reason)
{
    // Abort semantics (brief CP-2): report the specific
    // reason, perform no write, end the transaction, never
    // touch the session neutral.
    mState = FSVRTransactionState::ABORTED;
    FSVRTransactionAction action;
    action.kind = FSVRTransactionAction::Kind::REPORT;
    action.text = reason;
    return {action};
}

std::vector<FSVRTransactionAction>
FSVRCalibrationTransaction::checkStarvation(F64 now_s)
{
    if (active() && mHaveStepped &&
        now_s - mLastSteppedS > FSVRCalibration::STARVATION_BOUND_S)
    {
        return abortWith("calibration failed: interrupted");
    }
    return {};
}

void FSVRCalibrationTransaction::feedSettle(
    const FSVRTransactionStepInputs& inputs,
    const FSVRAdapterPlanInput& plan_input)
{
    // Settle observations are read off the stepped cycle:
    // the hold joints' rest_offsets (from the plan input),
    // mPelvisToFoot, and E_A (from the step inputs).
    FSVRCalSettleObservation cur;
    cur.hold_joint_offsets.reserve(mHoldIndices.size());
    for (S32 idx : mHoldIndices)
    {
        if (idx >= 0 && idx < static_cast<S32>(plan_input.joints.size()))
        {
            cur.hold_joint_offsets.push_back(
                plan_input.joints[idx].rest_offset);
        }
    }
    cur.pelvis_to_foot = inputs.pelvis_to_foot;
    cur.cornea_valid = inputs.eye_valid;
    cur.cornea_midpoint_agent = inputs.cornea_midpoint_agent;

    ++mSettleCycles;
    if (mHaveSettleObservation)
    {
        if (FSVRCalibration::settleObservationStable(mPrevSettle, cur,
                                                      mTolerance))
        {
            ++mSettleStableCount;
            if (mSettleStableCount >=
                FSVRCalibration::SETTLE_CONSECUTIVE_STABLE)
            {
                mSettled = true;
            }
        }
        else
        {
            mSettleStableCount = 0;
        }
    }
    mPrevSettle = cur;
    mHaveSettleObservation = true;
}

bool FSVRCalibrationTransaction::settleExpired() const
{
    return !mSettled && mSettleCycles > FSVRCalibration::SETTLE_WINDOW_CYCLES;
}

std::vector<FSVRTransactionAction> FSVRCalibrationTransaction::step(
    const FSVRTransactionStepInputs& inputs)
{
    if (!active())
    {
        return {};
    }
    mLastSteppedS = inputs.now_s;
    mHaveStepped = true;

    // A LOCAL transition anywhere in the transaction
    // aborts it (spec section 6.4): the baseline is the
    // first stepped cycle's epoch; any later change is
    // fatal, in every state.
    if (!mHaveEpochBaseline)
    {
        mEpochBaseline = inputs.local_transition_epoch;
        mHaveEpochBaseline = true;
    }
    else if (inputs.local_transition_epoch != mEpochBaseline)
    {
        return abortWith("calibration failed: local space changed");
    }

    const bool have_plan = inputs.plan_input && inputs.plan_result &&
                           inputs.plan_result->first_failing_gate !=
                               FSVRAdapterGate::TOKEN &&
                           static_cast<S32>(
                               inputs.plan_result->resolved_locals.size()) ==
                               static_cast<S32>(inputs.plan_input->joints.size());
    const bool held_cycle =
        have_plan && inputs.plan_input->calibration_hold_active;

    switch (mState)
    {
    case FSVRTransactionState::ENTERING:
    {
        // The hold flag is set from this cycle on (CP-4);
        // settle observation begins. The tolerance probe
        // derives from this cycle's plan input (hold start).
        if (have_plan)
        {
            mTolerance = FSVRCalibration::derivePoseTolerance(*inputs.plan_input);
            if (!mHoldResolved)
            {
                mHoldIndices = FSVRCalibration::resolveHoldJoints(
                    *inputs.plan_input, &mMissingHoldJoints);
                mHoldResolved = true;
                for (S32 i = 0;
                     i < static_cast<S32>(inputs.plan_input->joints.size());
                     ++i)
                {
                    const std::string& name = inputs.plan_input->joints[i].name;
                    if (name == "mHead")
                    {
                        mHeadIndex = i;
                    }
                    else if (name == "mWristLeft")
                    {
                        mWristIndex[0] = i;
                    }
                    else if (name == "mWristRight")
                    {
                        mWristIndex[1] = i;
                    }
                }
            }
        }
        mState = FSVRTransactionState::HOLDING;
        [[fallthrough]];
    }
    case FSVRTransactionState::HOLDING:
    {
        if (have_plan)
        {
            feedSettle(inputs, *inputs.plan_input);
        }
        if (!mMissingHoldJoints.empty())
        {
            // A hold-set joint absent from the loaded
            // skeleton is a hold failure (the pose cannot
            // be the defined pose), never a silent shrink.
            return abortWith("calibration failed: T-pose not achieved");
        }
        if (held_cycle)
        {
            // One full held cycle composed: verification
            // inputs are complete.
            mHeldInput = *inputs.plan_input;
            mHeldResult = *inputs.plan_result;
            mHaveHeldCycle = true;
            mState = FSVRTransactionState::VERIFYING;
        }
        return {};
    }
    case FSVRTransactionState::VERIFYING:
    {
        S32 failing = -1;
        const bool ok =
            mHaveHeldCycle &&
            FSVRCalibration::verifyHeldPose(mHeldInput, mHeldResult,
                                             mHoldIndices, mTolerance,
                                             failing);
        if (!ok)
        {
            return abortWith("calibration failed: T-pose not achieved");
        }
        mState = FSVRTransactionState::GATES;
        return {};
    }
    case FSVRTransactionState::GATES:
    {
        // Section 4.2 step 3, in the spec's order. There is
        // no floor gate (section 2 anchor semantics).
        if (inputs.sitting_on_object || inputs.parented)
        {
            return abortWith("calibration failed: sitting");
        }
        if (inputs.ground_sit_active)
        {
            return abortWith("calibration failed: ground sitting");
        }
        if (inputs.in_air)
        {
            return abortWith("calibration failed: not grounded");
        }
        if (!inputs.hmd_up_valid ||
            !FSVRCalibration::decideUprightOrientation(inputs.hmd_up))
        {
            return abortWith("calibration failed: not upright");
        }
        if (!inputs.hmd_position_valid)
        {
            return abortWith("calibration failed: headset not tracked");
        }
        mWindow.clear();
        mWindowEpoch = inputs.local_transition_epoch;
        mState = FSVRTransactionState::SAMPLING;
        // DP-C5-12: with the production bound unmeasured,
        // the machine aborts at window start rather than
        // collecting samples it cannot judge.
        if (!inputs.stationarity_installed)
        {
            return abortWith(
                "calibration failed: stationarity bound not yet measured");
        }
        return {};
    }
    case FSVRTransactionState::SAMPLING:
    {
        if (have_plan)
        {
            feedSettle(inputs, *inputs.plan_input);
        }
        if (settleExpired())
        {
            return abortWith("calibration failed: appearance not settled");
        }
        FSVRCalSample sample;
        sample.time_s = inputs.now_s;
        sample.hmd_valid = inputs.hmd_position_valid;
        sample.hmd_pos = inputs.hmd_position;
        for (S32 s = 0; s < 2; ++s)
        {
            sample.wrist_valid[s] = inputs.wrist_valid[s];
            sample.wrist_pos[s] = inputs.wrist_position[s];
        }
        sample.transition_marker =
            (inputs.local_transition_epoch != mWindowEpoch);
        mWindow.push_back(sample);

        const FSVRStationarityVerdict verdict =
            FSVRCalibration::decideStationarity(
                mWindow, inputs.stationarity_params);
        switch (verdict.reason)
        {
        case FSVRStationarityReason::STATIONARY:
            // C5-CODE-R1-03: forward evidence is a mandatory
            // gate on acceptance -- ICR-1's base yaw derives
            // from the accepted cycle's adopted forward, and
            // a stationary position window does not by itself
            // establish one. Without a valid, finite,
            // nondegenerate forward the window is restarted
            // (the tracker-loss disposition: never accepted,
            // never averaged through, never defaulted) and
            // sampling continues.
            if (!adoptedForwardHasYaw(inputs.hmd_forward,
                                      inputs.hmd_forward_valid))
            {
                mWindow.clear();
                return {};
            }
            mVerdict = verdict;
            mAcceptedForward = inputs.hmd_forward;
            mAcceptedForwardValid = inputs.hmd_forward_valid;
            mAcceptedRight = inputs.player_right;
            mAcceptedRightValid = inputs.player_right_valid;
            mState = FSVRTransactionState::RESOLVE;
            return {};
        case FSVRStationarityReason::MOVING:
            return abortWith("calibration failed: not stationary");
        case FSVRStationarityReason::TRACKER_LOSS:
            // Loss voids the window; it restarts, never
            // averaged through (DP-B4).
            mWindow.clear();
            return {};
        case FSVRStationarityReason::TRANSITION:
            return abortWith("calibration failed: local space changed");
        case FSVRStationarityReason::INSUFFICIENT:
            return {};
        }
        return {};
    }
    case FSVRTransactionState::RESOLVE:
    {
        if (have_plan)
        {
            feedSettle(inputs, *inputs.plan_input);
        }
        if (!mSettled)
        {
            if (settleExpired())
            {
                return abortWith(
                    "calibration failed: appearance not settled");
            }
            return {};
        }
        // Landmarks are read from THIS cycle's held pose
        // (after settle, under the hold) and this cycle's
        // eye-state read.
        if (!held_cycle || mHeadIndex < 0 || mWristIndex[0] < 0 ||
            mWristIndex[1] < 0)
        {
            // Not a usable held cycle; wait for one.
            return {};
        }
        if (!inputs.eye_valid)
        {
            return abortWith(
                "calibration failed: cornea landmark unavailable");
        }
        const FSVRAdapterPlanResult& result = *inputs.plan_result;

        FSVRCalLandmarkSet landmarks;
        for (S32 s = 0; s < 2; ++s)
        {
            landmarks.avatar_wrist_world[s] =
                toVec3d(result.resolved_world_positions[mWristIndex[s]]);
        }
        landmarks.cornea_midpoint_agent =
            toVec3d(inputs.cornea_midpoint_agent);
        landmarks.h_a =
            static_cast<F64>(inputs.cornea_midpoint_agent.mV[VZ]) -
            static_cast<F64>(inputs.foot_plane_z);
        // The avatar's right: head-local -Y under the held
        // head world rotation (agent frame: X forward, Y
        // left, Z up, so the head frame's right is its -Y).
        {
            const LLVector3d right_local(0.0, -1.0, 0.0);
            landmarks.avatar_right =
                right_local * result.resolved_world_rotations[mHeadIndex];
        }
        // Player side, per sample from the accepted window
        // (DP-C5-10: distances per sample from that
        // sample's simultaneous pair, then averaged by the
        // helper -- never from averaged endpoints).
        for (const FSVRCalSample& sample : mWindow)
        {
            for (S32 s = 0; s < 2; ++s)
            {
                if (!sample.wrist_valid[s])
                {
                    continue;
                }
                const LLVector3d rel =
                    toVec3d(sample.wrist_pos[s]) - toVec3d(sample.hmd_pos);
                landmarks.player_dist_samples[s].push_back(rel.length());
                // The lateral component rides the player
                // right basis captured at window acceptance
                // (DP-C5-9: the shell derives it from the
                // adopted orientation basis at acceptance;
                // the head-relative reach is defined against
                // that basis for every sample in the
                // window).
                if (mAcceptedRightValid)
                {
                    const LLVector3d right = toVec3d(mAcceptedRight);
                    landmarks.player_lateral_samples[s].push_back(
                        rel.mdV[VX] * right.mdV[VX] +
                        rel.mdV[VY] * right.mdV[VY] +
                        rel.mdV[VZ] * right.mdV[VZ]);
                }
            }
        }
        // A full capture whose lateral basis was unavailable
        // at acceptance cannot produce arm ratios: the
        // lateral vectors are empty while distances exist,
        // and the helper's arm math then yields
        // player_reach 0 -> invalid -> step-5 abort. That
        // is the loud failure DP-C5-9 records, not a silent
        // wrong record.

        mResolution = FSVRCalibration::resolveCalibrationRatios(
            landmarks, inputs.prior_record);
        if (!mResolution.valid)
        {
            return abortWith("calibration failed: invalid capture");
        }
        // Section 4.6's production offset, from the held
        // cycle's composed head transform; E_A as read.
        mDerivedCorneaOffset = FSVRCalibration::deriveCorneaOffsetHeadLocal(
            inputs.cornea_midpoint_agent,
            result.resolved_world_positions[mHeadIndex],
            result.resolved_world_rotations[mHeadIndex]);
        // The root-frame neutral cornea (PB-R1-03): the
        // offset composed over the bind configuration of
        // this cycle's plan input, stored root-relative
        // (C5-CODE-R1-01: the shared helper composes with
        // an identity root frame; C6-E applies the current
        // root frame per cycle).
        mNeutralCorneaRoot = neutralCorneaRootRelative(
            *inputs.plan_input, mHeadIndex, mDerivedCorneaOffset);
        mBuiltRecord = FSVRCalibration::buildCalibrationRecord(
            mResolution, inputs.captured_utc);
        // C5-CODE-R1-02: completion is ONE indivisible
        // logical step. The record commit and the neutral
        // install (DP-B11(a)'s ordering) are emitted as a
        // single ordered action batch from THIS step, and
        // the transaction is terminally DONE from this
        // moment: no stepped cycle intervenes, so neither
        // the epoch abort check at this function's head nor
        // starvation (preCycleCheck) can land between the
        // record write and the install. (The pre-fix machine
        // emitted COMMIT here and INSTALL from a later
        // COMMIT state; a LOCAL transition or a >10 s gap in
        // that interval wrote the new record and then
        // aborted -- a written record reporting failure,
        // with the session neutral still on the old one.)
        //
        // Atomicity assumption, stated plainly: C4's
        // writeCalibrationRecord() returns void -- it is a
        // single whole-map setLLSD on the per-account store
        // with no failure channel (persistence is the
        // store's native save-on-quit). The batch therefore
        // cannot branch on a write failure; what it
        // guarantees is that no abort, transition, or
        // starvation path can separate the write from the
        // install, and the shell executes the batch
        // synchronously, in order.
        FSVRTransactionAction commit;
        commit.kind = FSVRTransactionAction::Kind::COMMIT;
        commit.record = mBuiltRecord;

        FSVRTransactionAction install;
        install.kind = FSVRTransactionAction::Kind::INSTALL_NEUTRAL;
        install.installed_record =
            FSVRTrackingSettings::validateCalibrationRecord(mBuiltRecord);
        install.install_anchor_hmd_local = mVerdict.hmd_representative;
        // Gated at window acceptance (C5-CODE-R1-03): the
        // accepted forward is validated evidence, so the
        // yaw is derived from it unconditionally -- there
        // is no defaulted-yaw path.
        install.install_anchor_forward_yaw_rad =
            stage2YawOfForward(mAcceptedForward);
        install.install_cornea_offset_head_local = mDerivedCorneaOffset;
        install.install_neutral_cornea_root = mNeutralCorneaRoot;

        FSVRTransactionAction report;
        report.kind = FSVRTransactionAction::Kind::REPORT;
        if (!mResolution.head_only)
        {
            report.text = "calibration complete";
        }
        else if (mResolution.ratio_placeholder)
        {
            report.text =
                "calibration complete (head only) — displacement scale "
                "unmeasured (ratio 1.0) until a full capture measures it";
        }
        else if (mResolution.ratio_carried)
        {
            report.text =
                "calibration complete (head only — hand ratios not "
                "captured)";
        }
        else
        {
            // One-wrist measured head-only (DP-C5-13): the
            // head scale IS measured; "head only" names the
            // record shape only.
            report.text =
                "calibration complete (head only — hand ratios not "
                "captured)";
        }
        mState = FSVRTransactionState::DONE;
        std::vector<FSVRTransactionAction> actions = {commit, install,
                                                      report};
        if (mResolution.ratio_carried)
        {
            FSVRTransactionAction carried;
            carried.kind = FSVRTransactionAction::Kind::REPORT;
            carried.text = "scale carried from previous calibration";
            actions.push_back(carried);
        }
        return actions;
    }
    case FSVRTransactionState::COMMIT:
    case FSVRTransactionState::INSTALL:
    case FSVRTransactionState::DONE:
    case FSVRTransactionState::ABORTED:
    case FSVRTransactionState::IDLE:
        return {};
    }
    return {};
}

// ---------------------------------------------------------------------------
// CP-6: the session-neutral acquisition machine.
// ---------------------------------------------------------------------------

void FSVRSessionNeutral::resetAcquisition()
{
    mWindow.clear();
    mReconstructionOffsets.clear();
    mHaveEpochBaseline = false;
}

void FSVRSessionNeutral::dropToSuspended()
{
    mView = FSVRSessionNeutralView();
    mState = FSVRSessionState::SUSPENDED;
    resetAcquisition();
}

void FSVRSessionNeutral::invalidateForLocalTransition()
{
    if (mState == FSVRSessionState::ESTABLISHED ||
        mState == FSVRSessionState::ACQUIRING)
    {
        dropToSuspended();
    }
}

void FSVRSessionNeutral::invalidateForSkeletonReinit()
{
    if (mState == FSVRSessionState::ESTABLISHED ||
        mState == FSVRSessionState::ACQUIRING)
    {
        dropToSuspended();
    }
}

void FSVRSessionNeutral::invalidateForAccountOrRecordChange()
{
    if (mState == FSVRSessionState::ESTABLISHED ||
        mState == FSVRSessionState::ACQUIRING)
    {
        dropToSuspended();
    }
}

void FSVRSessionNeutral::installFromCapture(
    const FSVRCalibrationRecordValidation& record,
    const LLVector3& anchor_hmd_local, F32 anchor_forward_yaw_rad,
    const LLVector3& cornea_offset_head_local,
    const LLVector3& neutral_cornea_root)
{
    const bool sitting_clear = mView.sitting_signals_clear;
    mView = FSVRSessionNeutralView();
    mView.sitting_signals_clear = sitting_clear;
    mView.established = true;
    mView.record = record;
    mView.anchor_hmd_local = anchor_hmd_local;
    mView.anchor_forward_yaw_rad = anchor_forward_yaw_rad;
    mView.cornea_offset_head_local = cornea_offset_head_local;
    mView.neutral_cornea_root = neutral_cornea_root;
    mRecordBasis = record;
    mHaveRecordBasis = true;
    mState = FSVRSessionState::ESTABLISHED;
    resetAcquisition();
}

void FSVRSessionNeutral::step(const FSVRSessionStepInputs& inputs)
{
    if (mState == FSVRSessionState::ESTABLISHED)
    {
        return;
    }

    // Record basis: a fresh validation read while
    // UNCALIBRATED/SUSPENDED (re)starts acquisition, or
    // retires it when the record is (now) absent.
    if (inputs.record_read &&
        (mState == FSVRSessionState::UNCALIBRATED ||
         mState == FSVRSessionState::SUSPENDED))
    {
        if (inputs.record.status ==
            FSVRCalibrationRecordValidation::RecordStatus::NO_RECORD)
        {
            mHaveRecordBasis = false;
            mState = FSVRSessionState::UNCALIBRATED;
            resetAcquisition();
            return;
        }
        mRecordBasis = inputs.record;
        mHaveRecordBasis = true;
        mState = FSVRSessionState::ACQUIRING;
        resetAcquisition();
    }
    if (mState != FSVRSessionState::ACQUIRING)
    {
        return;
    }

    // A LOCAL transition during acquisition drops the
    // machine to SUSPENDED (CP-6): re-establishment runs
    // this same procedure in the new LOCAL space.
    if (!mHaveEpochBaseline)
    {
        mEpochBaseline = inputs.local_transition_epoch;
        mHaveEpochBaseline = true;
    }
    else if (inputs.local_transition_epoch != mEpochBaseline)
    {
        dropToSuspended();
        return;
    }

    // Section 5 step 2's gates must hold through the
    // window; a failing step voids the accumulated window.
    const bool gates_ok =
        !inputs.sitting_on_object && !inputs.parented &&
        !inputs.ground_sit_active && !inputs.in_air &&
        inputs.hmd_up_valid &&
        FSVRCalibration::decideUprightOrientation(inputs.hmd_up);
    if (!gates_ok)
    {
        mWindow.clear();
        mReconstructionOffsets.clear();
        return;
    }

    FSVRCalSample sample;
    sample.time_s = inputs.now_s;
    sample.hmd_valid = inputs.hmd_position_valid;
    sample.hmd_pos = inputs.hmd_position;
    for (S32 s = 0; s < 2; ++s)
    {
        sample.wrist_valid[s] = inputs.wrist_valid[s];
        sample.wrist_pos[s] = inputs.wrist_position[s];
    }
    sample.transition_marker = false; // epoch handled above
    mWindow.push_back(sample);

    // Section 4.6 reconstruction, per window sample: a
    // reading counts only if genuinely prim-derived; the
    // offset runs through CP-3g with the composed head
    // transform at the reading (the cycle's plan result).
    if (inputs.eye_valid && inputs.plan_input && inputs.plan_result &&
        inputs.plan_result->first_failing_gate != FSVRAdapterGate::TOKEN)
    {
        const FSVRAdapterPlanInput& plan = *inputs.plan_input;
        const FSVRAdapterPlanResult& result = *inputs.plan_result;
        for (S32 i = 0; i < static_cast<S32>(plan.joints.size()); ++i)
        {
            if (plan.joints[i].name == "mHead" &&
                i < static_cast<S32>(result.resolved_world_positions.size()))
            {
                mReconstructionOffsets.push_back(
                    FSVRCalibration::deriveCorneaOffsetHeadLocal(
                        inputs.cornea_midpoint_agent,
                        result.resolved_world_positions[i],
                        result.resolved_world_rotations[i]));
                break;
            }
        }
    }

    if (!inputs.stationarity_installed)
    {
        // The same OPEN-6 gate as the transaction's
        // (DP-C5-12): acquisition cannot complete on
        // stationarity grounds until DP-B4's values are
        // installed. The window stays bounded meanwhile.
        if (mWindow.size() > 3600)
        {
            mWindow.erase(mWindow.begin(),
                          mWindow.begin() + 1800);
        }
        return;
    }

    const FSVRStationarityVerdict verdict =
        FSVRCalibration::decideStationarity(mWindow,
                                            inputs.stationarity_params);
    if (verdict.reason == FSVRStationarityReason::TRACKER_LOSS ||
        verdict.reason == FSVRStationarityReason::MOVING)
    {
        // Not a trustworthy window: restart it (the neutral
        // is never struck from a moving or voided window).
        mWindow.clear();
        mReconstructionOffsets.clear();
        return;
    }
    if (verdict.reason != FSVRStationarityReason::STATIONARY)
    {
        return;
    }

    // Establishment: reconstruction must have produced at
    // least one genuinely prim-derived reading, all
    // finite, stable across the window within the section
    // 3.2 tolerance family (derived over the current plan
    // input). Failure leaves the machine ACQUIRING; the
    // record is untouched and no skeleton value is ever
    // substituted (section 4.6).
    if (!inputs.plan_input)
    {
        return;
    }
    const FSVRPoseTolerance tol =
        FSVRCalibration::derivePoseTolerance(*inputs.plan_input);
    if (!tol.valid || mReconstructionOffsets.empty() ||
        !FSVRCalibration::corneaReconstructionStable(
            mReconstructionOffsets, tol.position_m))
    {
        return;
    }
    LLVector3d mean_offset(0.0, 0.0, 0.0);
    for (const LLVector3& offset : mReconstructionOffsets)
    {
        mean_offset += toVec3d(offset);
    }
    mean_offset *= 1.0 / static_cast<F64>(mReconstructionOffsets.size());
    const LLVector3 session_offset = toVec3(mean_offset);

    // The root-frame neutral cornea (PB-R1-03): the offset
    // composed over the avatar-neutral/bind configuration,
    // fixed at establishment, stored root-relative
    // (C5-CODE-R1-01: the shared helper, as in the
    // transaction's RESOLVE).
    const FSVRAdapterPlanInput& plan = *inputs.plan_input;
    S32 head_index = -1;
    for (S32 i = 0; i < static_cast<S32>(plan.joints.size()); ++i)
    {
        if (plan.joints[i].name == "mHead")
        {
            head_index = i;
            break;
        }
    }
    if (head_index < 0)
    {
        return;
    }
    const LLVector3 neutral_root = neutralCorneaRootRelative(
        plan, head_index, session_offset);

    // C5-CODE-R1-03: establishment requires this cycle's
    // adopted-forward evidence -- ICR-1's base yaw is
    // written below from it, and a measured neutral heading
    // cannot be replaced by a default. Without it,
    // establishment is withheld (the machine stays
    // ACQUIRING and retries on later cycles); the window
    // and reconstruction state are left intact.
    if (!adoptedForwardHasYaw(inputs.hmd_forward,
                              inputs.hmd_forward_valid))
    {
        return;
    }

    const bool sitting_clear = mView.sitting_signals_clear;
    mView = FSVRSessionNeutralView();
    mView.sitting_signals_clear = sitting_clear;
    mView.established = true;
    mView.record = mRecordBasis;
    mView.anchor_hmd_local = verdict.hmd_representative;
    mView.anchor_forward_yaw_rad = stage2YawOfForward(inputs.hmd_forward);
    mView.cornea_offset_head_local = session_offset;
    mView.neutral_cornea_root = neutral_root;
    mState = FSVRSessionState::ESTABLISHED;
    resetAcquisition();
}

#ifndef FSVR_CALIBRATION_PURE_ONLY

// ---------------------------------------------------------------------------
// Viewer shell. Everything below reads live viewer state
// (OpenXR manager, avatar, eye state), executes the pure
// machines' actions (nearby-chat reports, the one store
// write, the neutral install), and owns the process-static
// machine instances. The transaction state is session
// state in these statics, never persisted: a viewer
// shutdown mid-transaction leaves nothing to resume
// (brief CP-2).
// ---------------------------------------------------------------------------

#include "fscommon.h"
#include "llagentcamera.h"
#include "llanimationstates.h"
#include "llavatarappearancedefines.h"
#include "llcharacter.h"
#include "llopenxrmanager.h"
#include "llvoavatar.h"

#include <chrono>
#include <ctime>

namespace
{

FSVRCalibrationTransaction s_transaction;
FSVRSessionNeutral s_session_neutral;
FSVRCalibrationRecordValidation s_prior_record;
U64 s_local_transition_epoch = 0;

F64 shellNowSeconds()
{
    using namespace std::chrono;
    return duration_cast<duration<double>>(
               steady_clock::now().time_since_epoch())
        .count();
}

// The section 4.2 step-3 avatar signals, read once over one
// avatar (plain member reads). This is the ONE read of the
// signal set: the post-apply step hook consumes the four
// facts individually (transaction / session step inputs),
// and both of the ICR-2 fact's refresh points -- the step
// hook and the pre-assembly refreshSittingSignals -- take
// the fact's value from this same read over the same
// character, so the cycle's two refresh points can never
// disagree (C6E-CODE-R1-01).
struct ShellSittingSignals
{
    bool sitting_on_object = false;
    bool parented = false;
    bool ground_sit = false;
    bool in_air = false;

    // The ICR-2 not-sitting fact over these signals.
    bool clear() const
    {
        return !sitting_on_object && !parented && !ground_sit &&
               !in_air;
    }
};

ShellSittingSignals readSittingSignals(LLVOAvatar* avatar)
{
    ShellSittingSignals signals;
    signals.parented = avatar->getParent() != nullptr;
    signals.sitting_on_object = avatar->isSitting() && signals.parented;
    signals.ground_sit =
        avatar->isMotionActive(ANIM_AGENT_SIT_GROUND_CONSTRAINED);
    signals.in_air = avatar->mInAir;
    return signals;
}

void executeActions(const std::vector<FSVRTransactionAction>& actions)
{
    for (const FSVRTransactionAction& action : actions)
    {
        switch (action.kind)
        {
        case FSVRTransactionAction::Kind::REPORT:
            FSCommon::report_to_nearby_chat(action.text);
            break;
        case FSVRTransactionAction::Kind::COMMIT:
            // The one commit point (spec section 4.4):
            // exactly one whole-map write per successful
            // transaction, through C4's surface.
            FSVRTrackingSettings::writeCalibrationRecord(action.record);
            break;
        case FSVRTransactionAction::Kind::INSTALL_NEUTRAL:
            s_session_neutral.installFromCapture(
                action.installed_record,
                action.install_anchor_hmd_local,
                action.install_anchor_forward_yaw_rad,
                action.install_cornea_offset_head_local,
                action.install_neutral_cornea_root);
            break;
        }
    }
}

} // namespace

// static
void FSVRCalibration::requestCalibration()
{
    // CP-1 items 4-5: the entry refusals. The running
    // transaction is undisturbed by a refused second
    // invocation.
    if (s_transaction.active())
    {
        FSCommon::report_to_nearby_chat(
            "calibration failed: already in progress");
        return;
    }
    LLOpenXRManager& openxr = LLOpenXRManager::instance();
    if (!openxr.isSessionRunning() || !openxr.isVRModeActive())
    {
        FSCommon::report_to_nearby_chat("calibration failed: no VR session");
        return;
    }
    // The prior record's validation, read once at entry:
    // carry-forward's input (CP-3c rule 3).
    s_prior_record = FSVRTrackingSettings::validateCalibrationRecord(
        FSVRTrackingSettings::calibrationRecordRaw());
    s_transaction.begin();
}

// static
bool FSVRCalibration::transactionActive()
{
    return s_transaction.active();
}

// static
bool FSVRCalibration::cycleWorkPending()
{
    // CP-4 item 6: a calibration transaction is active, or
    // the session machine is ACQUIRING. An established
    // neutral needs no per-cycle derivation, and an
    // uncalibrated session waits on a record, not on
    // cycles; a SUSPENDED machine re-enters ACQUIRING on a
    // stepped cycle once frames publish again.
    return s_transaction.active() ||
           s_session_neutral.state() == FSVRSessionState::ACQUIRING;
}

// static
void FSVRCalibration::preCycleCheck()
{
    // CP-4 item 3: a starved transaction ends (abort
    // report emitted) before the flag can be populated
    // for this cycle.
    if (s_transaction.active())
    {
        executeActions(s_transaction.checkStarvation(shellNowSeconds()));
    }
}

// static
void FSVRCalibration::refreshSittingSignals(LLCharacter* character)
{
    // C6E-CODE-R1-01: the glue invokes this before it
    // copies the session view for plan-input assembly, so
    // the view's ICR-2 fact is this cycle's. The write is
    // the session machine's own noteSittingSignals, fed by
    // the shell's one signal read over the glue's
    // character -- the same write, value, and source the
    // post-apply step hook performs later in the cycle.
    // No state lives here: a non-avatar character simply
    // leaves the fact as the step hook last wrote it (the
    // same early-out the step hook itself takes).
    LLVOAvatar* avatar = dynamic_cast<LLVOAvatar*>(character);
    if (!avatar)
    {
        return;
    }
    s_session_neutral.noteSittingSignals(
        readSittingSignals(avatar).clear());
}

// static
void FSVRCalibration::stepFromAdapterCycle(
    LLCharacter* character, const FSVRAdapterPlanInput& input,
    const FSVRAdapterPlanResult& result)
{
    if (result.first_failing_gate == FSVRAdapterGate::TOKEN)
    {
        // A duplicate frame token: the apply did not run;
        // this is not a stepped cycle for either machine.
        return;
    }
    LLVOAvatar* avatar = dynamic_cast<LLVOAvatar*>(character);
    if (!avatar)
    {
        return;
    }
    const F64 now_s = shellNowSeconds();

    // The section 4.2 step-3 avatar signals, read once
    // through the shell's one signal read (plain member
    // reads -- the steady-state cost class CP-4 item 4
    // budgets for).
    const ShellSittingSignals signals = readSittingSignals(avatar);
    const bool parented = signals.parented;
    const bool sitting_on_object = signals.sitting_on_object;
    const bool ground_sit = signals.ground_sit;
    const bool in_air = signals.in_air;
    // ICR-2: the per-cycle not-sitting fact, refreshed on
    // every stepped glue cycle. The pre-assembly refresh
    // (refreshSittingSignals) wrote this same value from
    // this same read earlier in the cycle; this write is
    // the hook's own refresh and stays so a cycle whose
    // step hook runs without an assembly-side consumer
    // still refreshes the fact.
    s_session_neutral.noteSittingSignals(signals.clear());

    if (!s_transaction.active() &&
        s_session_neutral.state() == FSVRSessionState::ESTABLISHED)
    {
        // Neither machine has work: the hook's cost in
        // steady state is the flag reads above (CP-4
        // item 4).
        return;
    }

    // Manager samples (read-only consumption of the landed
    // C2/C3 accessors).
    LLOpenXRManager& openxr = LLOpenXRManager::instance();
    LLVector3 hmd_position;
    const bool hmd_position_valid = openxr.getHmdPosition(hmd_position);
    LLVector3 hmd_up;
    const bool hmd_up_valid = openxr.getHmdUp(hmd_up);
    LLVector3 hmd_forward;
    const bool hmd_forward_valid = openxr.getHmdForward(hmd_forward);
    const bool hand_tracking = openxr.getHandTrackingAvailable();
    bool wrist_valid[2] = {false, false};
    LLVector3 wrist_position[2];
    if (hand_tracking)
    {
        wrist_valid[0] = openxr.getWristPosition(false, wrist_position[0]);
        wrist_valid[1] = openxr.getWristPosition(true, wrist_position[1]);
    }
    // DP-C5-9: the player right basis -- right =
    // cross(forward, up), normalized (for forward -Z and
    // up +Y, forward x up yields +X, matching OpenXR
    // LOCAL's right-handed convention).
    LLVector3 player_right;
    bool player_right_valid = false;
    if (hmd_forward_valid && hmd_up_valid)
    {
        player_right = LLVector3(
            hmd_forward.mV[VY] * hmd_up.mV[VZ] -
                hmd_forward.mV[VZ] * hmd_up.mV[VY],
            hmd_forward.mV[VZ] * hmd_up.mV[VX] -
                hmd_forward.mV[VX] * hmd_up.mV[VZ],
            hmd_forward.mV[VX] * hmd_up.mV[VY] -
                hmd_forward.mV[VY] * hmd_up.mV[VX]);
        player_right_valid = player_right.normalize() > 0.f;
    }

    // The eye state (section 4.6): E_A is the mean of
    // left/right when both are valid, the left alone in
    // mono; genuinely prim-derived iff at least one valid
    // is set (brief section 2.4). Consumed as read.
    LLAgentCamera::ViewpointOverrideEyeState eye;
    gAgentCamera.getViewpointOverrideEyeState(eye);
    const bool eye_valid = eye.left_valid || eye.right_valid;
    LLVector3 cornea_midpoint;
    if (eye.left_valid && eye.right_valid)
    {
        cornea_midpoint = (eye.left_pos_agent + eye.right_pos_agent) * 0.5f;
    }
    else if (eye.left_valid)
    {
        cornea_midpoint = eye.left_pos_agent;
    }
    else if (eye.right_valid)
    {
        cornea_midpoint = eye.right_pos_agent;
    }

    // F_A (spec section 4.5's formula, the tree's own
    // accessors).
    const F32 foot_plane_z =
        avatar->getRenderPosition().mV[VZ] +
        avatar->getVisualParamWeight(
            LLAvatarAppearanceDefines::AVATAR_HOVER) -
        avatar->getPelvisToFoot();

    FSVRStationarityParams params;
    const bool params_installed = productionStationarityParams(params);
    const U64 epoch = s_local_transition_epoch;

    if (s_transaction.active())
    {
        FSVRTransactionStepInputs step;
        step.now_s = now_s;
        step.captured_utc = static_cast<S64>(std::time(nullptr));
        step.plan_input = &input;
        step.plan_result = &result;
        step.sitting_on_object = sitting_on_object;
        step.parented = parented;
        step.ground_sit_active = ground_sit;
        step.in_air = in_air;
        step.hmd_up_valid = hmd_up_valid;
        step.hmd_up = hmd_up;
        step.hmd_forward_valid = hmd_forward_valid;
        step.hmd_forward = hmd_forward;
        step.hmd_position_valid = hmd_position_valid;
        step.hmd_position = hmd_position;
        for (S32 s = 0; s < 2; ++s)
        {
            step.wrist_valid[s] = wrist_valid[s];
            step.wrist_position[s] = wrist_position[s];
        }
        step.player_right_valid = player_right_valid;
        step.player_right = player_right;
        step.local_transition_epoch = epoch;
        step.eye_valid = eye_valid;
        step.cornea_midpoint_agent = cornea_midpoint;
        step.pelvis_to_foot = avatar->getPelvisToFoot();
        step.foot_plane_z = foot_plane_z;
        step.prior_record = s_prior_record;
        step.stationarity_installed = params_installed;
        step.stationarity_params = params;
        executeActions(s_transaction.step(step));
    }

    // The session machine steps alongside the transaction
    // (CP-6). Its record basis is re-read whenever it is
    // stepped while UNCALIBRATED/SUSPENDED (an established
    // neutral's basis changes only through the named
    // invalidation entry points).
    if (s_session_neutral.state() != FSVRSessionState::ESTABLISHED)
    {
        FSVRSessionStepInputs step;
        step.now_s = now_s;
        const FSVRSessionState session_state = s_session_neutral.state();
        if (session_state == FSVRSessionState::UNCALIBRATED ||
            session_state == FSVRSessionState::SUSPENDED)
        {
            step.record_read = true;
            step.record = FSVRTrackingSettings::validateCalibrationRecord(
                FSVRTrackingSettings::calibrationRecordRaw());
        }
        step.plan_input = &input;
        step.plan_result = &result;
        step.sitting_on_object = sitting_on_object;
        step.parented = parented;
        step.ground_sit_active = ground_sit;
        step.in_air = in_air;
        step.hmd_up_valid = hmd_up_valid;
        step.hmd_up = hmd_up;
        step.hmd_forward_valid = hmd_forward_valid;
        step.hmd_forward = hmd_forward;
        step.hmd_position_valid = hmd_position_valid;
        step.hmd_position = hmd_position;
        for (S32 s = 0; s < 2; ++s)
        {
            step.wrist_valid[s] = wrist_valid[s];
            step.wrist_position[s] = wrist_position[s];
        }
        step.local_transition_epoch = epoch;
        step.eye_valid = eye_valid;
        step.cornea_midpoint_agent = cornea_midpoint;
        step.stationarity_installed = params_installed;
        step.stationarity_params = params;
        s_session_neutral.step(step);
    }
}

// static
const FSVRSessionNeutralView& FSVRCalibration::sessionNeutralView()
{
    return s_session_neutral.view();
}

// static
void FSVRCalibration::invalidateNeutralForLocalTransition()
{
    // The epoch advances with the invalidation, so any
    // in-flight window comparing epochs sees the change
    // even if the entry point is the only signal a caller
    // raises.
    ++s_local_transition_epoch;
    s_session_neutral.invalidateForLocalTransition();
}

// static
void FSVRCalibration::invalidateNeutralForSkeletonReinit()
{
    s_session_neutral.invalidateForSkeletonReinit();
}

// static
void FSVRCalibration::invalidateNeutralForAccountOrRecordChange()
{
    s_session_neutral.invalidateForAccountOrRecordChange();
}

// static
U64 FSVRCalibration::localTransitionEpoch()
{
    return s_local_transition_epoch;
}

#endif // FSVR_CALIBRATION_PURE_ONLY
