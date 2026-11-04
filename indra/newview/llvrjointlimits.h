/**
 * @file llvrjointlimits.h
 * @brief P2-07B C6/C7 CP-1: the shared anatomical joint-limits
 *        module (implementation spec section 6.3; C0 rig
 *        verification section 5). Brief
 *        P2-07B-C6C7-IMPLEMENTATION-BRIEF, SHIP round 3.
 *
 * This module is the ONLY implementation of the C0 section
 * 5.1/5.2/5.3 clamping rules. Both solvers (C6 spine, C7 arm)
 * and the C7 post-clamp (CP-6) call it; no second copy of the
 * rule may live in a solver. Every function is pure: each takes
 * the limits-enabled condition as resolved by its caller (the
 * FSVRDiagBendLimits diagnostic suspends the clamp at the call
 * sites, spec section 6.1); the module holds no flag state.
 *
 * Frames and conventions are C0's: spine joints clamp on the
 * composed LOCAL rotation (bind is the identity rotation; the
 * viewer never builds bone-aligned frames, C0 section 4.1);
 * shoulders clamp on the relative rotation in the mChest
 * composed frame -- q_rel = q_Shoulder * ~q_Chest, the corrected
 * composition order of C0 section 4.2.1 (LLQuaternion::operator*
 * is the Hamilton product b (x) a: the naive inverse-first order
 * is wrong in this codebase) -- and the shoulder's raw local
 * quaternion (relative to mCollar) must never be clamped against
 * these numbers directly.
 */

#ifndef LL_LLVRJOINTLIMITS_H
#define LL_LLVRJOINTLIMITS_H

#include "llmath.h"

#include <string>

// Spine per-joint caps (spec section 6.3, C0 section 5.1):
// swing flexion / extension / lateral, degrees. The spec's
// twist column is a research bound the anatomical model does
// not use: the solver authors no twist at all (design decision
// D2), and authored twist is never clamped.
struct FSVRSpineJointCaps
{
    F32 flexion_deg = 0.f;
    F32 extension_deg = 0.f;
    F32 lateral_deg = 0.f;
};

// Shoulder caps (spec section 6.3, C0 section 5.2), in the
// swing chart of the mChest composed frame: flexion / extension
// about the chart's forward axis, abduction / adduction about
// the out axis, and the axial (twist about +Y) pair. The axial
// signs are geometry-derived (C0 section 4.2): internal rotation
// is + about +Y on BOTH sides; C0-03's runtime fixture remains
// open (brief OQ-2).
struct FSVRShoulderCaps
{
    F32 flexion_deg = 150.f;
    F32 extension_deg = 60.f;
    F32 abduction_deg = 150.f;
    F32 adduction_deg = 45.f;
    F32 internal_deg = 70.f;
    F32 external_deg = 90.f;
};

// Elbow segment-angle cap (spec section 6.3, C0 section 5.3):
// the angle between the A->B and B->C directions may not
// exceed this; hyperextension (bending to the opposite side of
// the bend plane) is forbidden outright.
constexpr F32 FSVR_ELBOW_MAX_SEGMENT_DEG = 145.f;

namespace FSVRJointLimits
{

// The spec section 6.3 spine table. Returns false for a joint
// with no caps entry (not a spine solver joint).
bool spineCapsForJoint(const std::string& joint_name,
                       FSVRSpineJointCaps& out_caps);
const FSVRShoulderCaps& shoulderCaps();

// ------------------------------------------------------------------
// Swing/twist decomposition about a fixed local axis.
//
// decomposeSwingTwistZ splits a local rotation (bind identity)
// into swing (the part that moves the +Z tip direction) and
// twist (rotation about +Z). The tip direction depends only on
// the swing; the twist angle is tau = 2*atan2(z, w) on the
// sign-canonicalized quaternion (w >= 0). composeSwingTwistZ is
// the exact inverse: twist * swing in LL operator order (the
// Hamilton product swing (x) twist -- the twist acts in the
// joint's own frame first, so the tip is untouched by it).
// ------------------------------------------------------------------
void decomposeSwingTwistZ(const LLQuaternion& q,
                          LLQuaternion& swing_out,
                          LLQuaternion& twist_out);
LLQuaternion composeSwingTwistZ(const LLQuaternion& swing,
                                const LLQuaternion& twist);
F32 twistAngleDegZ(const LLQuaternion& q);

// Same decomposition about +Y (the shoulder's axial axis in
// the mChest composed frame).
void decomposeSwingTwistY(const LLQuaternion& q,
                          LLQuaternion& swing_out,
                          LLQuaternion& twist_out);
LLQuaternion composeSwingTwistY(const LLQuaternion& swing,
                                const LLQuaternion& twist);
F32 twistAngleDegY(const LLQuaternion& q);

// ------------------------------------------------------------------
// Spine rule (C0 section 5.1). Tip coordinates from the swing:
// phi_f = atan2(d_x, d_z) (positive = flexion), phi_l =
// atan2(-d_y, d_z) (positive = bend to the avatar's right),
// where d is the +Z axis rotated by the swing. Feasibility is
// the sign-matched rectangle AND the inscribed ellipse
// (phi_f/cf)^2 + (phi_l/cl)^2 <= 1 with cf the sign-matched
// flexion/extension cap; the ellipse score is that left side.
// ------------------------------------------------------------------
void spineTipAnglesDeg(const LLQuaternion& swing,
                       F32& phi_flexion_deg,
                       F32& phi_lateral_deg);
F32 spineEllipseScore(F32 phi_flexion_deg, F32 phi_lateral_deg,
                      const FSVRSpineJointCaps& caps);
bool spineFeasible(F32 phi_flexion_deg, F32 phi_lateral_deg,
                   const FSVRSpineJointCaps& caps);
// Clamp a composed spine local. A feasible input is returned
// bit-identical (no decompose/recompose round-trip); a clamped
// input keeps its twist bit-exact (the twist quaternion is
// carried through, never re-derived). The degenerate case
// (d_z <= 0) follows C0 section 5.1 step 5, including the
// mandatory verification hard gate with zero-swing fallback.
LLQuaternion clampSpineLocal(const LLQuaternion& local,
                             const FSVRSpineJointCaps& caps);

// ------------------------------------------------------------------
// Shoulder rule (C0 section 5.2), in the mChest composed frame.
// The direction d (the bind arm direction d0 = (0, +/-1, 0)
// rotated by q_rel) is charted by the stereographic substrate
// anchored at anatomical down: f = d_x, o = s*d_y (s = +1 left,
// -1 right; "out" positive for both arms), u = -d_z;
// A = f/(1+u), B = o/(1+u); theta = 2*atan(rho) (the angle from
// arm-down, identically arccos u), psi = atan2(B, A); chart
// components (x, y) = (theta*cos psi, theta*sin psi). The
// ellipse score is (x/cf)^2 + (y/cl)^2 with sign-matched caps;
// the plane caps score exactly 1.000 on their boundary.
// shoulderChartDeg reports theta/psi and the components for a
// direction already expressed in the chest frame. The chart's
// stereographic pole sits at anatomical up (theta = 180): the
// azimuth is undefined there, so the chart assigns the
// deterministic fallback psi = 0 (the flexion plane) and the
// direction reads (180, 0) -- infeasible, and projected into
// the permitted region by the clamp's scale-back [R2 --
// C6C7-CODE-01].
// ------------------------------------------------------------------
void shoulderChartDeg(const LLVector3& d_chest_frame, bool left,
                      F32& theta_deg, F32& psi_deg,
                      F32& x_deg, F32& y_deg);
F32 shoulderEllipseScore(F32 x_deg, F32 y_deg);
bool shoulderFeasible(F32 x_deg, F32 y_deg);
// Clamp a chest-relative shoulder rotation: swing scaled back
// radially in theta at fixed psi, twist about +Y clamped to
// [-external, +internal]. A feasible input is returned
// bit-identical. clamped_out (optional) reports whether the
// rotation changed.
LLQuaternion clampShoulderRelative(const LLQuaternion& q_rel,
                                   bool left,
                                   bool* clamped_out = nullptr);

// ------------------------------------------------------------------
// Elbow rule (C0 section 5.3): the segment angle delta between
// the A->B and B->C directions (0 = straight). Callers enforce
// the cap by rotating B; this helper is the measurement.
// ------------------------------------------------------------------
F32 elbowSegmentAngleDeg(const LLVector3& a_pos,
                         const LLVector3& b_pos,
                         const LLVector3& c_pos);

} // namespace FSVRJointLimits

#endif // LL_LLVRJOINTLIMITS_H
