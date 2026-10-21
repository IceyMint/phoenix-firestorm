# ChatGPT round 2 — concurrence review (2026-10-20)

Same conversation as round 1 (GPT-6, reasoning High). ChatGPT read the
design docs on branch `vr/stereo-design` and the actual VR source on
`port/try-filtered-patch-2026-03-02` (llopenxrmanager.cpp plus
llviewerobject/llviewerjointattachment/llvoavatar sources) from GitHub.

## A. Avatar-eye embodiment override — CONCUR, with a limitation
OpenXR permits submitted eye poses/FOVs that differ from xrLocateViews;
an avatar-derived baseline is a coherent design for the embodiment goal.
Honest limit: no rigid transform maps a 35mm avatar baseline onto a
64mm physical one, so compositor reprojection of avatar-IPD images
cannot be made perfectly equivalent to a physical-IPD camera by
metadata consistency alone. Test, don't abandon: validate compositor
behaviour at much smaller AND much larger avatar baselines; record the
physical/avatar IPD ratio as a diagnostic (no auto-correction).

## B. Two-marker eye model — CONCUR, safeguards required
Markers beat skeleton origins for this project. Three findings, all now
acceptance criteria in PLAN.md:
- B1: attachment drawable updates can be DAMPED/ASYNC
  (gPipeline.updateMoveDampedAsync in LLVOAvatar::idleUpdateMisc) —
  marker positions can lag the head by a frame. Phase 2 must include a
  marker-transform synchronization barrier: freeze an immutable
  AvatarEyeFrame (head rotation + both marker positions + projections
  + timestamp) from ONE completed animation state.
- B2: eyeball attach points may move with GAZE animation (cornea
  markers riding eyeball rotation). Instrument and test; if they move,
  an explicit design decision is required — do not silently change
  the anchor or disable eye animation.
  RESOLVED 2026-10-20 by Leah (Design Rule #7): gaze-driven camera
  movement is WANTED — if the avatar looks right, the user looks
  right — until she has an eye-tracking headset, at which point
  physical eye tracking should take over the gaze role. Camera
  orientation = head rotation composed with the avatar's eye-gaze
  offset. B2 is a feature, not a defect.
- B3: marker validation: correct eyeball attach point per side, local
  avatar only, alive/usable drawable, duplicate names must not select
  arbitrarily, prim PIVOT is the sampled point (what the user placed
  at the cornea is the pivot — prims are positioned by pivot).
  Fallbacks (skeleton/mirror) are diagnostic/emergency only, NEVER an
  automatic route into stereo.

## C. Five-phase plan — CONCUR, with amendments (all adopted into PLAN.md)
- C1: LOCAL vs VIEW are not equals: VIEW is a diagnostic (Khronos
  guidance warns against world content in VIEW space), LOCAL is the
  production candidate. Phase 2 exit now requires the chosen policy to
  preserve the avatar-camera rules AND show acceptable spatial
  stability; if neither does, Phase 2 documents the limitation rather
  than declaring success.
- C2: marker sync / AvatarEyeFrame (see B1) is a Phase 2 deliverable.
- C3: Phase 4 buffers: one working set sized to the MAX eye extent;
  each eye renders with its own viewport/projection/rect (eye extents
  are not guaranteed equal). Render-order test stays mandatory.
- C4: Phase 0 validity gate — bugs found in CURRENT source (verified by
  Muse against the tree after the review):
  * submitFrameMono() submits a projection layer whenever
    submitted_view_count > 0, so one failed eye yields an invalid
    1-view layer under a PRIMARY_STEREO configuration
    (llopenxrmanager.cpp ~1301-1393). Must submit a full 2-view layer
    or no layer.
  * XrViewState::viewStateFlags never checked after xrLocateViews
    (~line 1143) before using orientations.
  * Swapchain sampleCount comes from recommendedSwapchainSampleCount
    (line 901) while the blit path attaches GL_TEXTURE_2D — validate
    the chosen sample count/texture target for multisample runtimes.

## Authorization scope (ChatGPT's words, paraphrased)
Phases 0 and 1 may be implemented now; Phase 2 experiments authorized;
neither compositor policy is pre-approved as final. Phase 4's
requirement: render from the avatar's two marked corneal positions,
head-joint orientation, both views from the same animation instant.
No need to revert to physical IPD, relax head constraints, or simplify
the animation model. (Implementation itself still awaits Leah's go —
she is in "discuss the plan first" mode.)
