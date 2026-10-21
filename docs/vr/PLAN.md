# Firestorm-Snowy VR — stereo & proper-projection plan (living document)

Status: revised plan after external critique (2026-10-20). Design rules in
`DESIGN-RULES.md` govern everything here; where this plan and the rules
disagree, the rules win.

## Where we are today (source-verified 2026-10-20)

- OpenXR integration (`LLOpenXRManager`) submits MONO: one desktop render
  is center-cropped and blitted into both eyes with an averaged center
  pose and a symmetrized FOV manufactured from the left eye.
- `xrWaitFrame`/`xrLocateViews` run in `swap()`, after rendering — the
  render pose is always one frame stale.
- Camera: hard locked to the avatar's head; position = the
  `viewpoint_override` attachment's render position; that attachment is
  a single prim worn on the LEFT EYEBALL attach point, placed at the
  left cornea. HMD rotation drives the head look-at, clamped to SL's
  ±72° LLHeadRotMotion limit relative to the torso.
- No OpenXR input actions; UI is desktop-only; reference space LOCAL;
  runtime is WiVRn (Monado).

## The eye model (decided 2026-10-20)

The eyes are the AVATAR's eyes (DESIGN-RULES #6 — immersion/embodiment:
the player must feel they ARE the character, including its size and eye
spacing). Concretely:

- Left eye position: the existing `viewpoint_override` prim's render
  position (left eyeball attach point, cornea-placed, visually verified
  by the user).
- Right eye position: a second identical prim, `viewpoint_override_right`,
  worn on the RIGHT EYEBALL attach point, cornea-placed the same way.
  Stereo is active only when both markers are worn; left-only = mono-left
  mode; neither = VR off.
- Physical HMD IPD is NOT the stereo baseline. Skeleton eye joints
  (mEyeLeft/mEyeRight) are a fallback only — their positions are not
  trusted to match where the eye mesh renders (system-eye morphs and
  worn mesh-eye attachments are not bound to the joints).
- Eye orientations: head joint world rotation COMPOSED WITH the
  avatar's eye-gaze offset, for both eyes (rules #3/#6 as amended by
  rule #7, 2026-10-20): the avatar's gaze moves the camera — if the
  avatar looks right, the user looks right. This holds until an
  eye-tracking headset exists, at which point physical eye tracking
  is intended to take over the gaze role.
  Two cautions from the round-3 concurrence (both accepted):
  (a) FEEDBACK LOOP: the look-at system must never chase its own
  tail — resolve the look-at target and animation ONCE per frame,
  then sample the resulting eye orientation and marker positions into
  the AvatarEyeFrame; never re-resolve per eye, and never let the
  camera update feed back into the gaze target within the frame.
  Also verify HMD rotation is not double-counted via head + gaze.
  (b) LIMITS: the head's ±72° clamp applies to the head alone; the
  gaze offset is separately bounded by the avatar's own eye-motion
  limits (eyes naturally rotate relative to the head, so the composed
  camera may exceed the head-only angle — that is correct, not a
  clamp violation). Watch for saccade-driven camera motion discomfort
  in testing.
- Fallbacks (skeleton eye joints, sagittal mirroring) are DIAGNOSTIC or
  explicitly-selected emergency modes only — never an automatic route
  into stereo. Stereo requires both markers, full stop.
- Marker rigour (from the round-2 review): the sampled point is the
  prim's PIVOT; each marker must be validated as worn by the local
  avatar on the CORRECT eyeball attach point (left/right), alive, with
  a usable drawable; duplicate names must never select arbitrarily;
  missing/detaching markers cause deliberate mode transitions.
- Both eye positions must come from the SAME completed animation state:
  attachment drawables can be updated damped/async relative to the
  skeleton, so Phase 2 freezes an immutable AvatarEyeFrame (head
  rotation + both marker positions + projection params + prediction
  timestamp) behind a synchronization barrier, and both eye passes
  consume only that snapshot.
- Eyeball attach points moving with gaze animation: RESOLVED
  2026-10-20 (rule #7) — this is WANTED. The avatar's gaze affects the
  camera; markers riding the eyeballs with gaze is correct behaviour,
  and the AvatarEyeFrame captures the gaze-influenced state as-is.
  (Instrumentation may still log it for diagnostics; it is not a bug
  to fix and eye animation is never disabled to hide it.)

## Phases

### Phase 0 — Instrument the current mono path + validity gate
Camera/FOV traces, frame-timing capture, color-patch (sRGB) diagnostic,
record actual render dimensions vs swapchain dimensions. Exit: measurable
optical, timing and color baselines; no behaviour change.
Test priorities for the Quest 3 symptoms (round-3 review, in order):
(1) calibrated grid rendered with the runtime's actual left-eye
XrFovf tangents — peripheral geometry baseline; (2) remove synthetic
FOV and center crop, verify full source-to-target viewport coverage;
(3) log rendered projection, submitted FOV, submitted pose and located
pose TOGETHER to catch compositor disagreement; (4) stationary-head
vs moving-head grid comparison — separate projection swim from
pose/reprojection swim; (5) gaze-driven camera tested independently of
HMD rotation (feedback/double-count check). A temporary visible border
at the exact per-eye source extent is a useful diagnostic for where
coverage is lost (source render vs compositor mapping) — diagnostic
only, not a feature.
Also in Phase 0, fix three validity defects already present in the
current source (found in round-2 review, verified against the tree):
(1) submitFrameMono() must never submit a partial projection layer —
under a PRIMARY_STEREO configuration a frame submits a full 2-view
layer or no layer (today one failed eye produces an invalid 1-view
layer); (2) XrViewState::viewStateFlags must be checked after
xrLocateViews before orientations are used; (3) validate the swapchain
sample count actually chosen against the blit path's texture target
(multisample swapchains vs GL_TEXTURE_2D attachment).

### Phase 1 — Proper left-eye rendering
Real asymmetric projection from the runtime's XrFovf tangents, as ONE
coherent camera mode (rendering, shader uniforms, screen-space effects and
frustum derivation must agree — the camera keeps several projection paths:
calcProjection cache, updateFrustumPlanes, picking). Native eye-sized
render target, tangent aspect NOT forced to match image aspect, center
crop and synthetic symmetric FOV deleted, color submission validated
against the Phase 0 diagnostic. Head/animation rules unchanged.
Exit: left-eye image geometrically correct, no crop/resample.

### Phase 2 — Timing + compositor policy
XR frame ownership moves before the avatar animation update and out of
swap(): WaitFrame -> LocateViews(predictedDisplayTime) -> clamp ->
animate/resolve joints ONCE -> freeze head anchor -> BeginFrame ->
render -> EndFrame. The submitted layer pose is an explicit policy
decision (physical pose, avatar head, and eye viewpoints are distinct
quantities): prototype world-locked (LOCAL) vs view-relative (VIEW)
submission and compare under four cases — rotation within the clamp,
beyond the clamp, torso animating while HMD still, physical lean.
Exit: no extra frame of pose delay; clamp behaviour predictable and
chosen, not accidental. The two policies are NOT equals: LOCAL is the
production candidate; VIEW is a diagnostic (Khronos guidance warns
against ordinary world content in VIEW space — the world would behave
head-locked). The exit bar is conjunctive: the chosen policy must
preserve the avatar-camera rules AND demonstrate acceptable spatial
stability in motion; if neither policy achieves both, Phase 2 documents
that limitation explicitly instead of declaring success. Note the
avatar-baseline complication (round-2 review): no rigid transform maps
a non-physical eye baseline onto the physical eyes, so reprojection
equivalence with a physical-IPD camera cannot be promised — validate
at much smaller and much larger avatar baselines, and log the
physical/avatar IPD ratio as a diagnostic (never an auto-correction).
Phase 2 also delivers the AvatarEyeFrame synchronization barrier
described in the eye model section: the frame snapshot freezes the
complete eye rig, not just the head anchor, and is consumed identically
by both eye passes.

### Phase 3 — Renderer refactor, no stereo yet
Split the display path into frame-wide update (once: visibility counters,
impostors, sun shadow, texture streaming, probes), per-eye view passes
(explicit EyeRenderContext: view, exact projection, target), and
presentation. Reason: the current display function mutates persistent
scene state, so calling it twice per frame corrupts the second pass.
Single left eye through the new path must be visually equivalent to
Phase 2 before proceeding.

### Phase 4 — Experimental stereo
Second eye pass using the right marker's position; working buffers
reused sequentially, sized to the MAXIMUM eye extent (the two eyes'
recommended extents are not guaranteed equal), each eye rendering with
its own exact viewport, projection and output rectangle; shared union visibility volume
covering BOTH eye frusta; hardware occlusion OFF initially; shared
shadow maps (union coverage); exposure shared between eyes; DoF and
vignette off in VR. Acceptance: near/far parallax matches the avatar's
measured eye spacing; rendering right-then-left produces the same
images as left-then-right (state-leak test); no eye-dependent geometry
inconsistencies.

### Phase 5 — Optimization & release modes
Restore stereo-safe occlusion, tune shadows/LOD, render-resolution
scale control, persistent Left / Right / Stereo setting (mono modes
still submit the full located view count with an explicit right-eye
policy), session-loss/disconnect recovery, desktop mirror polish.

## Confirmed facts & open items
- Headset: Meta Quest 3 via WiVRn (confirmed 2026-10-20). Note: the
  Quest 3 has no eye tracking, so Rule #7's sunset (physical eye
  tracking taking over the gaze role) cannot trigger on this headset.
- Leah's symptoms in the current mono path (2026-10-20): SWIM AT THE
  EDGES, and the VIEW DOES NOT COVER THE ENTIRE VIEW AREA. In her own
  words, the core complaint is that THE WORLD DOESN'T STAY STILL LIKE
  IN VRCHAT — world-stability ("lockedness") is the felt deficit, with
  VRChat on the same headset as the reference for correct. That maps
  to three causes this plan attacks in order: wrong peripheral
  projection (Phase 1), the one-frame-stale pose from swap()-time
  tracking (Phase 2), and compositor pose policy vs the avatar head
  (Phase 2). A previous
  attempt (with ChatGPT, before this plan) fixed coverage by ZOOMING
  the image to fill, which looked WORSE. Phase 0 therefore measures
  edge geometry and view coverage first; Phase 1 fills the panel with
  correct asymmetric tangents — never crop/zoom-to-fill.
- Compositor policy choice (Phase 2 experiments decide).
- Build rules: COMPLETE as recorded (Leah, 2026-10-20: "that's all
  the build rules — everything that you saw in the bash history").
  The full procedure lives in the BUILD-RULES notes from the
  safety-backup work: always build in the Ubuntu chroot, ReleaseFS_open,
  --avx2 --fmodstudio, channel Snowy, tests off, autobuild.

## Provenance
- Original plan + questions: Muse, 2026-10-20.
- External critique: ChatGPT (GPT-6, reasoning High) — see
  `CHATGPT-CRITIQUE-DIGEST.md`. Its corrections were verified against
  this tree's source before adoption.
- Eye model & two-marker design: Leah (rules) + Muse (extension),
  adopted 2026-10-20. Note it deliberately overrides the critique's
  recommendation to use runtime eye geometry / physical IPD.
