# ChatGPT critique of the Firestorm stereo plan — digest (2026-10-20)

> STATUS UPDATE (round 2, 2026-10-20): this digest records the ROUND-1
> critique. Two of its positions have since been superseded and must
> not be treated as instructions: (1) its recommendation of
> runtime-derived eye geometry / physical IPD is OVERRIDDEN by Design
> Rule #6 (avatar-eye embodiment) and the adopted two-marker eye model;
> (2) plan adoption is no longer "pending" in the round-1 sense —
> ChatGPT reviewed the documented plan and the actual source in round 2
> and CONCURRED on the eye model and the plan, with amendments now
> folded into PLAN.md. See CHATGPT-CONCURRENCE-DIGEST.md.

Full conversation (Leah's ChatGPT, GPT-6, reasoning High, ~9m19s thinking,
researched upstream Firestorm source + OpenXR spec + Monado/WiVRn docs):
"Critique stereo VR plan" — https://chatgpt.com/c/6ad7fc11-90f8-83ea-9151-826ea8aba34c
Brief: `chatgpt-brief-stereo.md` (this folder).

## Verdict
Directionally correct, but Phase 2 as written NOT approved. Stereo is three
independent problems (optics, render architecture, frame scheduling) plus a
fourth: the mapping between the physically tracked HMD and the deliberately
constrained avatar head. Sequential stereo remains a reasonable destination,
but only after the restructure below.

## Where ChatGPT corrected Muse (and Muse's source check on Leah's fork)
1. **The display function is side-effectful — you cannot just call it twice.**
   Verified in Leah's tree, `llviewerdisplay.cpp`: `LLDrawable::incrementVisible()`,
   `gPipeline.generateSunShadow()`, `LLVOAvatar::updateImpostors()`,
   `gPipeline.stateSort()` all live in the display path. Fix: split into
   frame-wide update (once) / per-eye view passes / presentation (once),
   behind an explicit per-eye render context.
2. **"Change one glm::perspective call" is not enough.** Verified in Leah's
   `llviewercamera.cpp`: `calcProjection()` keeps its own symmetric matrix
   cache (line 182) and `updateFrustumPlanes()` (line 205) rebuilds frustum
   geometry from the active projection; picking/world-to-screen use the
   camera too. The XR off-axis projection must be one coherent camera mode
   used by rendering, shader uniforms and screen-space effects, with the
   conservative culling frustum kept explicitly separate. Stereo culling
   volume must cover BOTH displaced eye frusta, not just wider angles.
3. **Compositor pose is the subtlest problem (Muse's brief was sloppy here).**
   "Submit the raw located poses" is wrong when the image was rendered from
   the clamped avatar head: physical head at 100° vs avatar at 72° lets
   runtime reprojection shift the image in ways that fight the design rules.
   Physical pose H, avatar head A and per-eye V = A·E must be treated as
   distinct quantities with an explicit submission policy; prototype and
   compare world-locked (LOCAL) vs view-relative (VIEW) layer behaviour.
   Four acceptance cases: within clamp / beyond clamp / torso animates while
   HMD still / physical lean.
4. **Timing move must go earlier than the animation update**, not just
   earlier than drawing: WaitFrame -> LocateViews(predicted time) -> clamp ->
   avatar anim + joint resolve ONCE -> freeze head anchor -> BeginFrame ->
   render -> EndFrame. XR frame ownership must leave `swap()` entirely
   (desktop window minimized/vsync must not pace the headset).
5. **Post chain is per-eye work, not a blit.** Verified in Leah's
   `pipeline.cpp`: `renderFinalize()` includes SSR, exposure, tone mapping,
   glow, AA (FXAA/SMAA), DoF, vignette. VR defaults: DoF off, vignette off;
   exposure SHARED between eyes (independent adaptation = binocular
   brightness mismatch); temporal histories per-eye or off. Also: sRGB
   double-gamma risk — needs a color-patch diagnostic before choosing the
   copy path; tangent-aspect != recommended image aspect (don't crop/distort
   to force a match).
6. **Stereo state safety:** start with a shared union-visible set and
   hardware occlusion DISABLED (occlusion state is per-main-camera; cube
   path already special-cases it). Render-order test: right-then-left must
   equal left-then-right, else state is leaking between eyes.
7. **IPD:** runtime relative eye transforms (E = H_center^-1 · H_eye)
   applied to the avatar head anchor — NOT avatar eye joints, NOT fixed
   63mm (debug only), 1:1 meters. Canted-eye rotational offsets may
   conflict with the "axes = head joint rotation" rule; document, don't
   silently violate.
8. **Mono modes need explicit compositor semantics:** a stereo view config
   still requires submitting the located view count; duplicating the left
   image into the right eye with wrong pose/FOV metadata distorts. Call it
   mono compatibility, never stereo.
9. Also flagged: WiVRn encode resolution is a separate budget from render
   resolution (benchmark render/encode/network separately); confirm viewer
   GL context and WiVRn use the same GPU (cross-GPU copies at eye res are
   costly); render-resolution scale is the first performance lever, not
   foveation (WiVRn foveated *encoding* doesn't cut viewer render cost);
   XR lifecycle hardening (shouldRender false, session loss, disconnect).

## Revised phase plan (ChatGPT's, endorsed by Muse)
- **Phase 0** — Instrument/validate current mono: camera/FOV traces, timing,
  color-space tests, real render dims. Exit: measurable baselines.
- **Phase 1** — Proper left-eye rendering: real asymmetric projection as a
  coherent camera mode, native eye-sized target, corrected color submission,
  no crop/synthetic FOV. Animation/head rules unchanged.
- **Phase 2** — Timing + compositor-model experiments: XR frame ownership
  before anim update, out of swap(); LOCAL vs VIEW submission compared
  under the four acceptance cases.
- **Phase 3** — Renderer refactor WITHOUT stereo: frame/view split, per-eye
  context, single left eye through the new path; visual equivalence to
  Phase 2 required.
- **Phase 4** — Experimental stereo: runtime eye geometry, sequential
  passes, shared visibility, occlusion off; geometry + render-order tests.
- **Phase 5** — Optimization/release: safe occlusion back, shadow/LOD tuning,
  Left/Right/Stereo settings, disconnect recovery.

Status: presented to Leah 2026-10-20; adoption pending her word.
