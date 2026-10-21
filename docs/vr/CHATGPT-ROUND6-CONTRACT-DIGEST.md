# Round 6 — Implementation Contract (digest of ChatGPT's revised contract)

Source: ChatGPT (GPT-6, High), "Critique stereo VR plan", 2026-10-20.
Two replies: the Round 6 Implementation Contract, then the REVISED
contract after Leah's not-mouselook correction. The revised contract
governs. Muse verified its load-bearing new claim (LLHeadRotMotion
consumes the shared "LookAtPoint" animation datum —
llcharacter/llheadrotmotion.cpp:258,646) against the tree.

## Branches (from implementation base 7ad6967806)
- `vr/phase0-trace-validity-20261020` — Phase 0.
- `vr/s1a-cornea-nearclip-audit-20261020` — S1a-VR.
Independent; neither descends from the other; separate PRs against the
base; `vr/stereo-design` stays the documentation branch.

## Phase 0 — instrumentation & validity (contract part A)
Scope: reproducible baselines for D1, D2, D5, D6, D11, D13, D14; the
three validity defects fixed in isolated commits AFTER the baseline
is captured. NO production changes to projection, crop, frame
lifecycle, look control, recenter, or avatar visibility in Phase 0.
- Trace: keep /tmp/firestorm_openxr.log (human); add structured
  JSONL /tmp/firestorm_openxr_trace.jsonl, enabled by FS_OPENXR_TRACE=1,
  default OFF. Records carry: schema v1, run_id, event type, viewer
  frame_id, XR frame id, monotonic µs (never mix XrTime with
  steady_clock without a verified conversion), source commit, mode.
  Every rendered frame and every XR attempt incl. failures; buffered
  writes (≤1 flush/sec + shutdown), dropped-record count, 60-frame
  frame-time summaries; no personal data.
- Render record (per frame, after camera matrices are final): camera
  pos/orientation + coordinate space, head world rotation, cornea
  marker pos + validation, gaze/look-at target type, HMD data
  provenance (which LocateViews sequence fed this frame), calibration
  state, clamp state, camera MODE (must explicitly identify
  non-mouselook VR); configured VFOV/aspect, actual + cached
  projection matrices, view matrix, near/far INCLUDING the EFFECTIVE
  near plane used by setPerspective (correction-reply addition: not
  just the requested value), GL viewport, world-render rect, source
  texture dims, zoom/subregion; mXRSubmitSource identity/format,
  world viewport's position inside it, active post passes, capture
  point relative to renderFinalize (D14), framebuffer-zero use.
- XR record (per submission): Wait/Begin/Locate/End ordering,
  timestamps, results; predictedDisplayTime/Period; shouldRender;
  located view count; viewStateFlags decoded; per-eye located pose,
  all four XrFovf angles, tangent extents/aspect; swapchain dims/
  format/sampleCount; acquire/wait/copy/release outcomes (an eye is
  successful only on completed copy + valid FBO + successful
  release); input + cropped source rects; synthetic submitted FOV;
  submitted pose/space; layer/view counts; GL errors; fallbacks.
- Analyzer (offline): FOV asymmetry (D1), render-vs-submitted ray
  divergence (D1/D2/D11), viewport/texture area ratio (D11), crop
  retained area (D3/D11), pixel scaling, render→XR phase (D6),
  render-vs-compositor orientation in a justified common frame (D5),
  source-bounds violations (D13), color-path differences (D14),
  validity tallies (D7–D9). Unknown conversions report NOT_COMPARABLE,
  never invented differences.
- S11 fixture (MVP): stationary in-world build — ≥8×8 m floor grid
  (1 m + 0.25 m divisions), calibration wall ~3 m from the reference
  cornea point (0.25 m grid), marked centre axis + eye-height line,
  known-size square/circle markers (0.1/0.25/0.5 m), angular markers
  at ±15/30/45/60°, distinct objects at 0.75/2/5 m, grey+RGB colour
  strip; opaque unlit/fullbright; manifest of dims/coords/materials;
  fixture NEVER tracks avatar/HMD; pose recorded per test.
- Five Quest 3 test stages: (1) optical baseline, (2) coverage,
  (3) projection/pose coherence, (4) motion stability (still, slow
  yaw/pitch, clamp boundary, torso-anim-with-still-HMD, lean),
  (5) gaze/head behaviour as-is (no Rule #9 implementation). Every
  test PASS/FAIL/BLOCKED with evidence; un-runnable = BLOCKED, not PASS.
- Validity fixes (isolated commits, after baseline): V1 partial
  layer → exactly two valid views or layerCount=0, per-eye success
  accounting, cleanup on all failure paths. V2 viewStateFlags →
  check orientation/position VALID (+tracked separately); invalid
  pose drives neither head nor a submitted view; NO silent recalibration.
  V3 swapchain → sampleCount=1 for the GL_TEXTURE_2D blit path, log
  recommended/max, validate FBO completeness, fail XR init cleanly on
  rejection. Plus D13 safeguard: never blit an XR-sized rect out of
  framebuffer 0; explicit, bounded fallback or recorded failure.
- Commits: P0-01 trace infra → P0-02 render instrumentation → P0-03
  XR instrumentation → P0-04 analyzer + fixture + BASELINE CAPTURE →
  P0-05..P0-08 validity fixes → P0-09 evidence report.
- Exit criteria (all): trace toggleable with no release-behaviour
  change; correlatable frame ids; FOV/crop/source/swapchain geometry
  quantified; pose/timing provenance known; colour path characterized;
  fixture reproducible; 5 stages evidenced or BLOCKED with reasons;
  partial layers impossible; invalid tracking gated; swapchain
  compatible; OOB copying prevented; baseline vs post-fix results
  distinguishable; no production redesign smuggled in.

## S1a-VR — corneal near-clip audit (contract part B, revised)
Restoration work is OUT (Leah's VR hides nothing; mouselook findings
are reference-only Annex 1). Purpose: how the forced 0.01 m near clip
(llagentcamera.cpp ~L133–134, applied ~L1928–1941; consumed via
getPreferredNearClip → llviewerdisplay.cpp ~L784 setNear →
setPerspective) affects near-face geometry: fringe, hair crossing the
cornea, glasses frames/bridge/lenses, nose at extreme down-gaze,
face accessories. Must distinguish near-clip from backface culling,
alpha behaviour, depth ordering, marker placement, attachment
transforms, and true camera-inside-geometry.
- Protocol (her real avatar/hair/glasses, left cornea marker):
  A baseline forward/stationary (confirm effective near = 0.01 m);
  B eye/gaze movement only; C slow head yaw/pitch incl. down-gaze;
  D body animation (idle/sit/dance); E per-issue geometry cause
  analysis (screenshot shows symptom; source/geometry assigns cause);
  F ONLY if an artifact is unresolved: experimental alternate-near
  build, production default stays 0.01 m, never a quiet permanent change.
- Evidence record per issue: scene, camera mode (VR/non-mouselook),
  marker, geometry/material, requested vs effective near, user
  observation, reproduction motion, suspected cause, evidence,
  S1b need, PASS/FAIL/BLOCKED/UNKNOWN.
- Settings: FSVRSelfVisibilityMode WITHDRAWN. Optional developer-only
  FSVRNearClipDiagnostic (default off, no behaviour change when off).
- Exit: mode confirmed; full visibility unchanged; effective near
  verified; her actual hair/glasses tested standing/eyes/head/animated;
  artifacts categorized with evidence; no blanket changes; S1b
  handoffs concrete per case. "1 cm is satisfactory; no S1b fix needed"
  is a legitimate successful outcome.
- Commits: S1A-01 near-clip source/rendering audit → S1A-02 user
  reproduction matrix → S1A-03 optional isolated diagnostics →
  S1A-04 S1b requirements handoff. Documentation-only completion is
  acceptable where evidence justifies it.

## Rule #9 Phase 2 hazard (contract part C7 — flagged, verified)
LLHUDEffectLookAt supplies ONE shared LookAtPoint datum consumed by
BOTH LLHeadRotMotion and the eye motion. Re-admitting the ladder
naively restores head motion too. Phase 2 must separate eye/head
outputs and verify: HMD head ownership absolute; attention moves
eyes only; mouse precedence; native expiry/resume actually selects
the next autonomous target (verify, don't assume); single sampling,
no feedback/double-count; desktop keeps body-ordered behaviour.
Phase 2 item — NOT a Phase 0 fix.

## Forbidden until their phases (contract part C6)
Phase 1: asymmetric production projection, production crop removal,
shader/frustum/cache restructure. Phase 2: look-at re-admission,
head/eye ownership changes, eyes-only mouse, arbitration/deadzone,
recenter, frame-lifecycle move, compositor policy choice,
AvatarEyeFrame. Phase 3/4: corneal masks, per-eye self-geometry,
stereo. Indefinitely (unless separately authorized): positional
camera from HMD, required VR controllers, automatic gaze steering.

## Review evidence (every PR)
Base/head commits; files+functions; rule + diagnosis addressed;
original behaviour with line refs; new behaviour + why; builds
actually attempted with true results; tests actually run with
outputs/trace paths; NOT RUN/BLOCKED marked; regressions; evidence
links; explicit review request. No "Quest 3 verified" claim without
a real Quest 3/WiVRn session.

## Authorization (ChatGPT's closing, concurred by Muse)
"Start P0-01 through P0-04 and S1A-01 immediately in parallel.
Capture the baseline before applying validity fixes."
