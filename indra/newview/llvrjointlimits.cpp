/**
 * @file llvrjointlimits.cpp
 * @brief P2-07B C6/C7 CP-1 implementation: see llvrjointlimits.h.
 *        The clamp procedures follow C0 section 5 step for step.
 */

#include "llviewerprecompiledheaders.h"

#include "llvrjointlimits.h"

#include <cmath>

namespace
{

constexpr F32 kRadToDeg = 180.f / F_PI;
constexpr F32 kDegToRad = F_PI / 180.f;
// Feasibility tolerance for the ellipse score: the plane caps
// score exactly 1.000 on their boundary (C0 section 5.2 worked
// numbers), so admission must not reject a boundary point over
// float dust; the clamp's verification gate uses the same
// tolerance, so nothing admitted here can fail verification
// there by more than this margin.
constexpr F32 kScoreTolerance = 1e-3f;
constexpr F32 kAngleToleranceDeg = 0.01f;

// Quaternions are sign-canonicalized (w >= 0) before any angle
// extraction, so the twist angle lands in (-180, 180] and the
// decomposition is single-valued.
LLQuaternion canonicalized(const LLQuaternion& q_in)
{
    LLQuaternion q = q_in;
    q.normalize();
    if (q.mQ[VS] < 0.f)
    {
        q = LLQuaternion(-q.mQ[VX], -q.mQ[VY], -q.mQ[VZ], -q.mQ[VS]);
    }
    return q;
}

F32 clampf(F32 v, F32 lo, F32 hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

} // namespace

bool FSVRJointLimits::spineCapsForJoint(const std::string& joint_name,
                                        FSVRSpineJointCaps& out_caps)
{
    // Spec section 6.3 (flexion / extension / lateral, degrees).
    struct Row
    {
        const char* name;
        F32 flexion;
        F32 extension;
        F32 lateral;
    };
    static const Row kTable[] = {
        {"mSpine1", 16.f, 8.f, 8.f},
        {"mSpine2", 24.f, 12.f, 12.f},
        {"mTorso", 5.f, 4.f, 5.f},
        {"mSpine3", 8.f, 6.f, 8.f},
        {"mSpine4", 8.f, 7.f, 8.f},
        {"mChest", 9.f, 7.f, 9.f},
    };
    for (const Row& row : kTable)
    {
        if (joint_name == row.name)
        {
            out_caps.flexion_deg = row.flexion;
            out_caps.extension_deg = row.extension;
            out_caps.lateral_deg = row.lateral;
            return true;
        }
    }
    return false;
}

const FSVRShoulderCaps& FSVRJointLimits::shoulderCaps()
{
    static const FSVRShoulderCaps kCaps;
    return kCaps;
}

void FSVRJointLimits::decomposeSwingTwistZ(const LLQuaternion& q_in,
                                           LLQuaternion& swing_out,
                                           LLQuaternion& twist_out)
{
    const LLQuaternion q = canonicalized(q_in);
    const F32 tau = 2.f * std::atan2(q.mQ[VZ], q.mQ[VS]);
    twist_out = LLQuaternion(tau, LLVector3(0.f, 0.f, 1.f));
    // swing = q (x) twist^-1 in Hamilton terms; in LL operator
    // order (a * b is the Hamilton product b (x) a) that is
    // ~twist * q. The result is the shortest-arc rotation taking
    // +Z to the tip direction: the tip depends only on it.
    swing_out = ~twist_out * q;
    swing_out.normalize();
}

LLQuaternion FSVRJointLimits::composeSwingTwistZ(
    const LLQuaternion& swing, const LLQuaternion& twist)
{
    // Hamilton swing (x) twist == LL twist * swing (see
    // decomposeSwingTwistZ). The twist acts in the joint's own
    // frame first, so it cannot move the tip.
    LLQuaternion result = twist * swing;
    result.normalize();
    return result;
}

F32 FSVRJointLimits::twistAngleDegZ(const LLQuaternion& q_in)
{
    const LLQuaternion q = canonicalized(q_in);
    return 2.f * std::atan2(q.mQ[VZ], q.mQ[VS]) * kRadToDeg;
}

void FSVRJointLimits::decomposeSwingTwistY(const LLQuaternion& q_in,
                                           LLQuaternion& swing_out,
                                           LLQuaternion& twist_out)
{
    const LLQuaternion q = canonicalized(q_in);
    const F32 tau = 2.f * std::atan2(q.mQ[VY], q.mQ[VS]);
    twist_out = LLQuaternion(tau, LLVector3(0.f, 1.f, 0.f));
    swing_out = ~twist_out * q;
    swing_out.normalize();
}

LLQuaternion FSVRJointLimits::composeSwingTwistY(
    const LLQuaternion& swing, const LLQuaternion& twist)
{
    LLQuaternion result = twist * swing;
    result.normalize();
    return result;
}

F32 FSVRJointLimits::twistAngleDegY(const LLQuaternion& q_in)
{
    const LLQuaternion q = canonicalized(q_in);
    return 2.f * std::atan2(q.mQ[VY], q.mQ[VS]) * kRadToDeg;
}

void FSVRJointLimits::spineTipAnglesDeg(const LLQuaternion& swing,
                                        F32& phi_flexion_deg,
                                        F32& phi_lateral_deg)
{
    // d is the +Z tip direction under the swing. atan2 is total
    // except at (0, 0); a (0, 0) argument pair seeds 0 (C0
    // section 5.1 steps 2 and 5).
    LLVector3 d = LLVector3(0.f, 0.f, 1.f) * swing;
    d.normalize();
    phi_flexion_deg = (d.mV[VX] == 0.f && d.mV[VZ] == 0.f)
        ? 0.f
        : std::atan2(d.mV[VX], d.mV[VZ]) * kRadToDeg;
    phi_lateral_deg = (d.mV[VY] == 0.f && d.mV[VZ] == 0.f)
        ? 0.f
        : std::atan2(-d.mV[VY], d.mV[VZ]) * kRadToDeg;
}

F32 FSVRJointLimits::spineEllipseScore(F32 phi_flexion_deg,
                                       F32 phi_lateral_deg,
                                       const FSVRSpineJointCaps& caps)
{
    const F32 cf = phi_flexion_deg >= 0.f ? caps.flexion_deg
                                          : caps.extension_deg;
    const F32 cl = caps.lateral_deg;
    const F32 a = phi_flexion_deg / cf;
    const F32 b = phi_lateral_deg / cl;
    return a * a + b * b;
}

bool FSVRJointLimits::spineFeasible(F32 phi_flexion_deg,
                                    F32 phi_lateral_deg,
                                    const FSVRSpineJointCaps& caps)
{
    // Sign-matched rectangle AND inscribed ellipse (C0 5.1
    // step 3). The ellipse is inscribed in the rectangle, so
    // the score test alone bounds the rectangle too; both are
    // checked because the rule states both.
    if (phi_flexion_deg < -caps.extension_deg - kAngleToleranceDeg ||
        phi_flexion_deg > caps.flexion_deg + kAngleToleranceDeg ||
        phi_lateral_deg < -caps.lateral_deg - kAngleToleranceDeg ||
        phi_lateral_deg > caps.lateral_deg + kAngleToleranceDeg)
    {
        return false;
    }
    return spineEllipseScore(phi_flexion_deg, phi_lateral_deg, caps) <=
           1.f + kScoreTolerance;
}

LLQuaternion FSVRJointLimits::clampSpineLocal(
    const LLQuaternion& local, const FSVRSpineJointCaps& caps)
{
    LLQuaternion swing;
    LLQuaternion twist;
    decomposeSwingTwistZ(local, swing, twist);

    LLVector3 d = LLVector3(0.f, 0.f, 1.f) * swing;
    d.normalize();
    const bool degenerate = d.mV[VZ] <= 0.f;

    F32 phi_f = 0.f;
    F32 phi_l = 0.f;
    spineTipAnglesDeg(swing, phi_f, phi_l);
    if (!degenerate && spineFeasible(phi_f, phi_l, caps))
    {
        // Feasible: return the input untouched, so the twist
        // (and everything else) is bit-preserved.
        return local;
    }

    // Step 4: radial scale-back in normalized tip space. The
    // sign-matched caps are fixed per component sign, and the
    // scale factor is positive, so scaling the normalized pair
    // scales (phi_f, phi_l) by the same factor.
    const F32 score = spineEllipseScore(phi_f, phi_l, caps);
    F32 phi_f_clamped = phi_f;
    F32 phi_l_clamped = phi_l;
    if (score > 1.f)
    {
        const F32 scale = 1.f / std::sqrt(score);
        phi_f_clamped = phi_f * scale;
        phi_l_clamped = phi_l * scale;
    }

    // Reconstruct the tip: d' proportional to
    // (tan phi_f', -tan phi_l', 1), normalized.
    LLVector3 d_new(std::tan(phi_f_clamped * kDegToRad),
                    -std::tan(phi_l_clamped * kDegToRad), 1.f);
    d_new.normalize();
    LLQuaternion swing_new;
    swing_new.shortestArc(LLVector3(0.f, 0.f, 1.f), d_new);

    // Step 5's mandatory verification hard gate (applied on the
    // degenerate path, and on any clamp that had to move the
    // tip): recompute the tip coordinates from d' and check all
    // three feasibility conditions; on numerical failure the
    // output is zero swing.
    F32 verify_f = 0.f;
    F32 verify_l = 0.f;
    spineTipAnglesDeg(swing_new, verify_f, verify_l);
    if (!spineFeasible(verify_f, verify_l, caps))
    {
        swing_new = LLQuaternion();
    }

    // Recompose with the twist untouched (step 5: q' = swing'
    // composed with the carried twist).
    return composeSwingTwistZ(swing_new, twist);
}

void FSVRJointLimits::shoulderChartDeg(const LLVector3& d_chest_frame,
                                       bool left, F32& theta_deg,
                                       F32& psi_deg, F32& x_deg,
                                       F32& y_deg)
{
    LLVector3 d = d_chest_frame;
    d.normalize();
    const F32 s = left ? 1.f : -1.f;
    const F32 f = d.mV[VX];
    const F32 o = s * d.mV[VY];
    const F32 u = -d.mV[VZ];
    // Theta is read from u directly (theta = arccos u, the
    // identity the header records): total on the whole sphere
    // and exact at both poles, unlike the stereographic radius
    // 2*atan(rho), whose denominator 1+u vanishes at the
    // anatomical-up pole (theta = 180). [R2 -- C6C7-CODE-01:
    // the previous form divided f and o by the clamped
    // denominator; at the exact pole f = o = 0, so theta read
    // 0 instead of 180 and the direction passed feasibility
    // unchanged -- a hard-limit escape.]
    theta_deg = std::acos(clampf(u, -1.f, 1.f)) * kRadToDeg;
    // Azimuth: psi = atan2(o, f) -- the stereographic
    // components' common positive factor 1/(1+u) cancels in
    // the ratio, so psi stays accurate arbitrarily close to
    // the pole. At the poles themselves f and o vanish
    // together and the azimuth is undefined; the chart assigns
    // the deterministic fallback psi = 0 (the flexion plane).
    // At the up pole that reads (x, y) = (180, 0): infeasible
    // under every cap pair, so clampShoulderRelative's scale-
    // back projects the direction into the permitted region
    // at the fallback azimuth instead of admitting it.
    if (f == 0.f && o == 0.f)
    {
        psi_deg = 0.f;
    }
    else
    {
        psi_deg = std::atan2(o, f) * kRadToDeg;
    }
    const F32 psi_rad = psi_deg * kDegToRad;
    x_deg = theta_deg * std::cos(psi_rad);
    y_deg = theta_deg * std::sin(psi_rad);
}

F32 FSVRJointLimits::shoulderEllipseScore(F32 x_deg, F32 y_deg)
{
    const FSVRShoulderCaps& caps = shoulderCaps();
    const F32 cf = x_deg >= 0.f ? caps.flexion_deg : caps.extension_deg;
    const F32 cl = y_deg >= 0.f ? caps.abduction_deg : caps.adduction_deg;
    const F32 a = x_deg / cf;
    const F32 b = y_deg / cl;
    return a * a + b * b;
}

bool FSVRJointLimits::shoulderFeasible(F32 x_deg, F32 y_deg)
{
    const FSVRShoulderCaps& caps = shoulderCaps();
    if (x_deg < -caps.extension_deg - kAngleToleranceDeg ||
        x_deg > caps.flexion_deg + kAngleToleranceDeg ||
        y_deg < -caps.adduction_deg - kAngleToleranceDeg ||
        y_deg > caps.abduction_deg + kAngleToleranceDeg)
    {
        return false;
    }
    return shoulderEllipseScore(x_deg, y_deg) <= 1.f + kScoreTolerance;
}

LLQuaternion FSVRJointLimits::clampShoulderRelative(
    const LLQuaternion& q_rel_in, bool left, bool* clamped_out)
{
    if (clamped_out)
    {
        *clamped_out = false;
    }
    const FSVRShoulderCaps& caps = shoulderCaps();

    LLQuaternion swing;
    LLQuaternion twist;
    decomposeSwingTwistY(q_rel_in, swing, twist);
    const F32 tau_deg = twistAngleDegY(q_rel_in);
    const F32 tau_clamped =
        clampf(tau_deg, -caps.external_deg, caps.internal_deg);
    const bool twist_clamped = tau_clamped != tau_deg;

    const LLVector3 d0(0.f, left ? 1.f : -1.f, 0.f);
    LLVector3 d = d0 * swing;
    d.normalize();
    F32 theta_deg = 0.f;
    F32 psi_deg = 0.f;
    F32 x_deg = 0.f;
    F32 y_deg = 0.f;
    shoulderChartDeg(d, left, theta_deg, psi_deg, x_deg, y_deg);
    const bool swing_feasible = shoulderFeasible(x_deg, y_deg);

    if (swing_feasible && !twist_clamped)
    {
        return q_rel_in;
    }

    LLQuaternion swing_new = swing;
    if (!swing_feasible)
    {
        // Radial scale-back in theta at fixed psi (C0 5.2
        // step 4): theta' = theta / sqrt(S), S the ellipse
        // score > 1.
        const F32 score = shoulderEllipseScore(x_deg, y_deg);
        const F32 theta_new =
            score > 1.f ? theta_deg / std::sqrt(score) : theta_deg;
        const F32 rho_new = std::tan(theta_new * kDegToRad * 0.5f);
        const F32 psi_rad = psi_deg * kDegToRad;
        const F32 a_new = rho_new * std::cos(psi_rad);
        const F32 b_new = rho_new * std::sin(psi_rad);
        const F32 denom = 1.f + rho_new * rho_new;
        const F32 s = left ? 1.f : -1.f;
        LLVector3 d_new(2.f * a_new / denom, s * 2.f * b_new / denom,
                        -(1.f - rho_new * rho_new) / denom);
        d_new.normalize();
        swing_new.shortestArc(d0, d_new);
    }

    LLQuaternion twist_new = twist;
    if (twist_clamped)
    {
        twist_new = LLQuaternion(tau_clamped * kDegToRad,
                                 LLVector3(0.f, 1.f, 0.f));
    }

    if (clamped_out)
    {
        *clamped_out = true;
    }
    return composeSwingTwistY(swing_new, twist_new);
}

F32 FSVRJointLimits::elbowSegmentAngleDeg(const LLVector3& a_pos,
                                          const LLVector3& b_pos,
                                          const LLVector3& c_pos)
{
    const LLVector3 u = b_pos - a_pos;
    const LLVector3 v = c_pos - b_pos;
    const F32 lu = u.length();
    const F32 lv = v.length();
    if (lu < 1e-9f || lv < 1e-9f)
    {
        return 0.f;
    }
    const F32 cos_angle = clampf((u * v) / (lu * lv), -1.f, 1.f);
    return std::acos(cos_angle) * kRadToDeg;
}
