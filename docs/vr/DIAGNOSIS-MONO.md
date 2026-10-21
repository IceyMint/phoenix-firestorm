# Diagnosis: what is actually wrong with the current mono VR path

Joint audit, 2026-10-20. Muse did a line-level read of the tree;
ChatGPT independently audited the same files on GitHub
(branch `port/try-filtered-patch-2026-03-02`), confirmed most items,
corrected two of Muse's, refuted one, and added D11–D15. Muse then
re-verified the corrections against the source — they hold. This is
the reconciled version; the reconciliation log is at the end.

Symptoms (Leah, Quest 3 via WiVRn): weird stretching at the edges;
view does not cover the entire view area; world doesn't stay still
like in VRChat. Not vibration.

## Findings

### D1 — The submitted FOV is fabricated (CONFIRMED)
`llopenxrmanager.cpp`, `xr_centered_mono_fov_for_source_crop` (~L293):
vertical = total span (angleUp − angleDown) halved into symmetric
up/down (the eye's real asymmetry discarded); horizontal is NOT from
the eye's angles — it is derived from the vertical half-angle × the
source rect's PIXEL aspect, so it changes with render-rect size, not
optics; both then scaled by crop fractions. The submitted tangents
match neither the lens nor, exactly, the render. The crop-fraction
formula is itself reasonable for a centred crop of a genuine
full-frame image — the fault is the model it feeds on.
**Symptoms: edge stretching, coverage, instability.**

### D2 — The render projection is symmetric (CONFIRMED, explanation corrected)
`llviewerdisplay.cpp` (~L261) sets the camera's vertical FOV to
angleUp − angleDown; `LLViewerCamera::setPerspective()` builds a
symmetric `glm::perspective` with the render rect's aspect. The eye's
optical principal-point offset (asymmetric tangents) is lost.
CORRECTION to the first draft of this document: the crop in D3 is a
CENTRED crop of a symmetric image and stays symmetric — there is no
"off-centre sub-window reinterpreted as on-axis" effect. The real
mismatch is three-way: (1) the symmetric render projection, (2) the
viewport region actually rendered, (3) the synthetic submitted FOV
reconstructed from texture size and crop — three geometries that are
not guaranteed to describe the same rays. **Symptoms: edge stretching,
coverage.**

### D3 — Center crop discards source pixels (CONFIRMED, causality amended)
`center_crop_rect_to_aspect` (~L259), used ~L1297, forces the source
to the left eye's tangent aspect. AMENDED: the cropped rect is then
blitted into the ENTIRE swapchain — the crop does not leave the
destination unfilled. What it does: discards useful source pixels,
narrows apparent angular coverage, magnifies (softening), and
misrepresents peripheral rays to the compositor. If "not covering"
ever means literal black borders, see D13 instead.
**Symptoms: coverage, edge stretching, softness.**

### D4 — RETRACTED as originally stated; replaced by the fallback hazard
Muse's draft claimed the default submit source is a low-resolution
desktop backbuffer upscaled into the swapchain. ChatGPT refuted this
and Muse's re-check confirms the refutation: in `llviewerwindow.cpp`
(~L286) `is_openxr_decoupled_render_requested()` returns TRUE when
the env var is unset — decoupled rendering (and native resolution)
are DEFAULT-ON opt-outs, and `submitFrameMono` prefers the pipeline's
`mXRSubmitSource`. The desktop backbuffer is only a fallback.
What survives of D4 is the fallback itself — see D13.

### D5 — Compositor is anchored to the wrong viewpoint (CONFIRMED)
Both eyes receive the identical averaged pose (`xr_center_pose_from_views`:
position average + quaternion nlerp of the located eye poses), the
identical fabricated FOV (D1), and the identical image (~L1298–1371).
The submitted pose is the physical HMD centre in LOCAL space; the
image was rendered from the avatar's head. HMD rotation does reach
the avatar through the head-look system, so the two are related —
but there is no explicit, self-consistent mapping between the actual
rendering viewpoint and the submitted pose, and the projection view
is supposed to describe the image being submitted. **Symptom: world
doesn't stay still — the strongest single candidate alongside D6.**

### D6 — The frame lifecycle runs backwards (CONFIRMED)
`submitFrameMono` is called from `swap()` (`llviewerdisplay.cpp`
~L1718): xrWaitFrame, xrBeginFrame and xrLocateViews all run AFTER
the frame was rendered; the located views are consumed by the NEXT
frame. A structural frame-phase delay, with XR pacing coupled to the
desktop display path (which can also return early under some
conditions). **Symptom: world doesn't stay still.**

### D7 — Partial projection layer (CONFIRMED, round-2 find)
`projection_layer.viewCount = submitted_view_count` (~L1380); the
layer is submitted whenever the count is > 0 (~L1392). One failed eye
→ an invalid 1-view layer under a PRIMARY_STEREO configuration.
Related: when the blit FBO is incomplete the code logs, but still
releases the swapchain image, so an unwritten image can be submitted
as if rendered. **Symptom: intermittent invalid/corrupt frames.**

### D8 — Tracking validity flags never checked (CONFIRMED)
After `xrLocateViews` (~L1136–1166), `XrViewState.viewStateFlags`
(orientation/position validity) is never read before the orientations
are cached into HMD forward/up and the submitted pose.
**Symptom: instability during tracking dropouts.**

### D9 — Swapchain sample count vs blit target (CONFIRMED, latent)
Swapchains request `recommendedSwapchainSampleCount` (~L901); the
blit attaches images as GL_TEXTURE_2D (~L1345). If a runtime returns
multisampled images, the blit FBO is incompatible (feeding D7). Not
established that WiVRn returns >1 on the Quest 3 — validate, don't
assume. **Symptom: potential missing/invalid frames.**

### D10 — Crop aspect from the left eye only (CONFIRMED)
The runtime aspect for the crop (~L1292) comes from `views[0]`,
applied identically to both swapchains regardless of the right eye's
FOV or extent. The right eye receives a left-eye-derived synthetic
projection. Part of mono compatibility, wrong by construction;
Leah's left eye is the immediate concern. **Symptoms: edge, coverage
(right eye).**

### D11 — Three rectangles are mixed up (NEW, HIGH — ChatGPT)
The decoupled world viewport is built from the left-eye tangent
aspect (`llviewerwindow.cpp` ~L339–398); `setup3DRender()` derives
the camera aspect from that viewport; but the submit path takes
`submit_input_rect` from the FULL dimensions of `mXRSubmitSource`
(`llopenxrmanager.cpp` ~L1240–1275) — not from the viewport the world
was actually rendered into. The texture can be larger than the
viewport, so the submit-side crop + FOV reconstruction (D1) operates
on the wrong rectangle and is not guaranteed to reconstruct the
rendered frustum (it can describe a narrower vertical field than was
rendered). **Symptoms: edge stretching, coverage. Together with
D1/D2, the core of the optical problem: render projection, rendered
viewport, selected source pixels, and submitted FOV must describe
the same rays — currently nothing guarantees that.**

### D12 — Angle-wrap hazard in the head-look math (NEW — ChatGPT, verified)
`llagentcamera.cpp` (~L1461–1468 and the surrounding block): the HMD
delta is computed by raw Euler subtraction,
`-(hmd_yaw - sViewpointOverrideHmdBaseYaw)` (likewise pitch/roll),
before clamping. Near the ±π wrap, two physically adjacent headings
(+179° / −179°) differ numerically by ~358°, and the clamp turns that
into a sudden jump of the avatar head's target orientation. Euler
decomposition is also ill-conditioned near extreme pitch.
**Symptom: sudden view jumps near angular boundaries — NOT the
constant edge stretching.** Fix direction: quaternion-composed
relative orientation.

### D13 — The framebuffer fallback can read outside the desktop buffer (NEW)
When `mXRSubmitSource` is unavailable, the code binds framebuffer 0
but keeps `submit_input_rect`, which may describe the larger XR-sized
viewport. GL clips out-of-bounds reads: literal missing image areas /
black regions are possible on this path. There must be no silent
fallback — log it prominently and use explicitly validated desktop
bounds. **Symptom: possible literal missing coverage, if active.**
Whether this path ever activates in practice is unknown — Phase 0
logging (below) decides.

### D14 — The XR source skips the final presentation pass (NEW)
`pipeline.cpp` (~L9060–9130): `mXRSubmitSource` is captured BEFORE
the final presentation shader (`gDeferredPostNoDoFNoiseProgram`).
Possibly intentional — not every desktop presentation effect belongs
in VR — but the headset image is therefore NOT the desktop's final
image. Colour-space validation (Phase 0) must measure the headset
path as it actually is, not assume desktop equivalence.
**Symptom: possible colour/sharpness differences; not geometry.**

### D15 — "VRChat-stable" needs a precise definition (NEW — framing)
By design (rules #1–#4, #7), this viewer will never have roomscale
positional parallax, and its camera follows a constrained, animated
avatar head — not the raw HMD. Stability acceptance for this project
means: (1) correct optical rays; (2) no camera movement from erroneous
tracking data; (3) no extra frame-phase latency; (4) consistent
behaviour under the constrained avatar-head model; (5) NO promise of
ordinary roomscale stability while physically leaning. Phase 1 must
not be judged a failure for (5) — that difference is the design.

## Ranking for Leah's actual complaints (joint)
1. **D1 + D2 + D11** — projection incoherence. Biggest expected win
   for edge stretching and apparent coverage.
2. **D5 + D6** — submitted-pose mismatch + late frame lifecycle.
   Biggest expected win for rotational world stability.
3. **D3** — crop: recovers real coverage, removes needless resampling.
4. **D12** — eliminates potential angular jumps.
5. **D4/D13** — establish what the source actually is per frame;
   kill the silent out-of-bounds fallback.
6. **D7–D9** — OpenXR validity/robustness.
7. **D10 + D14** — right-eye geometry; presentation consistency.

## Smallest useful verification experiment (before any refactor)
Capture ONE synchronized per-frame diagnostic record: allocated
left/right XrFovf; render projection matrix + world viewport rect;
mXRSubmitSource texture dims + source rect; which source path was
used (offscreen vs framebuffer 0); swapchain dims + sample count;
submitted layer poses/FOVs; HMD orientation used for the camera, its
sampling time, and the frame's predicted display time. Then, avatar
torso/gaze held still, slow head turns on a static grid:
(1) head still — if edges already stretch, projection/source geometry
is implicated; (2) slow rotation — if static looks right but motion
swims, timing/pose is implicated; (3) vary the desktop window size —
if headset coverage changes despite native-res rendering, the
rectangle coupling (D11) is active.

## Reconciliation log
- D1: Muse finding confirmed; ChatGPT amendment on the crop formula
  noted in D1.
- D2: confirmed with Muse's mechanism corrected (centred crop; the
  fault is three-way geometry incoherence). Muse accepts.
- D3: confirmed; causality amended (destination IS filled; the losses
  are pixels, coverage angle, and magnification). Muse accepts.
- D4: Muse's default-resolution claim REFUTED by ChatGPT and
  withdrawn after Muse re-read `llviewerwindow.cpp` (~L286:
  decoupled defaults to true when the env var is unset). Replaced by
  D13. (This also corrects Muse's earlier session notes describing
  these paths as "env-gated, off by default" — they are default-on
  opt-outs.)
- D5, D6, D7, D8, D10: confirmed as written. D9: confirmed as latent.
- D11–D15: added by ChatGPT; D11 and D12 verified by Muse against
  the source after the review (submit_input_rect from full texture
  dims at ~L1270; raw Euler subtraction at llagentcamera.cpp ~L1468).
