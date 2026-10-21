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
- Eye orientations: head joint world rotation for both eyes (per rules
  #3/#6); eye joint gaze rotation is never used.

## Phases

### Phase 0 — Instrument the current mono path
Camera/FOV traces, frame-timing capture, color-patch (sRGB) diagnostic,
record actual render dimensions vs swapchain dimensions. Exit: measurable
optical, timing and color baselines; no behaviour change.

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
chosen, not accidental.

### Phase 3 — Renderer refactor, no stereo yet
Split the display path into frame-wide update (once: visibility counters,
impostors, sun shadow, texture streaming, probes), per-eye view passes
(explicit EyeRenderContext: view, exact projection, target), and
presentation. Reason: the current display function mutates persistent
scene state, so calling it twice per frame corrupts the second pass.
Single left eye through the new path must be visually equivalent to
Phase 2 before proceeding.

### Phase 4 — Experimental stereo
Second eye pass using the right marker's position; buffers reused
sequentially at one eye's resolution; shared union visibility volume
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

## Open items
- Headset model in use (affects FOV asymmetry expectations).
- Compositor policy choice (Phase 2 experiments decide).
- Remaining build rules from Leah (only Rule #1 — always build in the
  Ubuntu chroot — recorded so far; see BUILD-RULES in the safety-backup
  notes).

## Provenance
- Original plan + questions: Muse, 2026-10-20.
- External critique: ChatGPT (GPT-6, reasoning High) — see
  `CHATGPT-CRITIQUE-DIGEST.md`. Its corrections were verified against
  this tree's source before adoption.
- Eye model & two-marker design: Leah (rules) + Muse (extension),
  adopted 2026-10-20. Note it deliberately overrides the critique's
  recommendation to use runtime eye geometry / physical IPD.
