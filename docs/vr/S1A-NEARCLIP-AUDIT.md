# S1A-01 — Corneal near-clip source & rendering audit

Branch: `vr/s1a-cornea-nearclip-audit-20261020` (base 7ad6967806).
Scope per the Round 6 Implementation Contract (revised), part B:
document the 0.01 m near-clip code path and its interaction with the
rendering projection. NO behaviour change in this commit. All
citations read directly from the base tree, 2026-10-20.

## The mechanism

- Constants, `indra/newview/llagentcamera.cpp` ~L133–134:
  `DEFAULT_CAMERA_NEAR_CLIP = 0.1f`,
  `VIEWPOINT_OVERRIDE_CAMERA_NEAR_CLIP = 0.01f`.
- The override value applies whenever a `viewpoint_override`
  attachment mesh is found (`findViewpointOverrideMesh()`), i.e.
  exactly when Leah's VR camera is active. There is no user setting
  for it; it is a compile-time constant.
- The near value reaches the camera through TWO write paths that
  must agree (both keyed on the same predicate, both writing the
  same constants — verified):
  1. `LLAgentCamera::updateCamera()` (~L1928–1931):
     `LLViewerCamera::setNear(override ? 0.01 : 0.1)` at camera
     placement time, in the same block that moves the camera to the
     marker's render position.
  2. The display path, `llviewerdisplay.cpp` ~L784:
     `LLViewerCamera::setNear(gAgentCamera.getPreferredNearClip())`,
     where `LLAgentCamera::getPreferredNearClip()`
     (llagentcamera.cpp ~L2801) returns 0.01 when the marker exists,
     else 0.1.
- Consumption: `LLViewerCamera::setPerspective()`
  (llviewercamera.cpp ~L294) builds the projection from the camera's
  near value, so the 0.01 m plane is the EFFECTIVE render near plane
  for every VR frame — it clips rendered geometry (and shapes depth
  precision), it does not hide anything by visibility logic. This
  matches the contract's Phase 0 requirement to trace the effective
  near, not just the requested one: with two writers, the rendered
  value is the only trustworthy one.

## What 0.01 m means at a cornea

The camera sits at the left cornea surface (marker pivot). Geometry
closer than 1 cm to that point is clipped by the near plane:
- Glasses: frames/bridge typically sit ~1–2 cm in front of the
  cornea — mostly OUTSIDE the clip distance, but the closest parts
  of a frame or lens edge can cross inside 1 cm depending on the
  avatar's face shape and the item's fit.
- Fringe/bangs: hair that falls in front of the eyes can pass within
  1 cm of the cornea point, and hair cards SWING with animation —
  expect intermittent clipping ("popping") at the boundary rather
  than a clean absence.
- Nose: ~4–6 cm away — never near-clipped; visible only at gaze
  angles where it enters the frustum (extreme down-gaze), as in life.
- Eyelashes/system eye geometry: at/behind the cornea point; behind
  the camera plane, not near-clip victims.

## Distinguishing causes (contract B1) — source notes

A near-face visual artifact in the headset is NOT automatically a
near-clip failure. Candidate causes and how source discriminates:
- Near plane: distance from the marker pivot < 0.01 m at the moment
  of clipping; correlates with head/gaze motion that brings geometry
  across the 1 cm shell.
- Backface culling: hair cards are single-sided; from the cornea,
  the NEAR side of a fringe card may be its back face — invisible
  regardless of distance. Material-level fact, check per hairstyle.
- Alpha behaviour: hair alpha masks/blending + depth ordering can
  erase or halo strands independent of the near plane.
- True intersection: geometry crossing the camera point itself
  (marker placement vs animated attachment transforms).
- Projection faults: Phase 0/1 territory (D1/D2/D11), not S1a.

## S1A-02 readiness

The user reproduction matrix (contract tests A–D: baseline, eyes
only, slow head rotation, body animation — her real avatar, hair and
glasses, Quest 3) can run against the CURRENT packaged build; no code
change is needed before it. Test F (experimental alternate near
plane) requires a separate build and is only authorized if A–D
surface an unresolved artifact.

## Open items for the record

- Phase 0's trace must record the effective near per frame (both
  writers agree today; the trace is what proves they keep agreeing).
- Whether 1 cm is satisfactory for Leah's actual hair/glasses is a
  headset question (S1A-02), not a source question. "Satisfactory,
  no change" is a legitimate successful outcome per the contract.
