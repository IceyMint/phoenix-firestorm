# Firestorm VR — Source-level recovery audit (2026-11-08)

**Status:** Source review, **not** a successful compilation or headset acceptance test.  
**Reviewed branch:** `vr/development`, including Muse's indev snapshot merged in PR #1, starting from `97dbd4a`.  
**Specification:** [Embodiment Vision & Acceptance Contract](EMBODIMENT-VISION-AND-ACCEPTANCE.md)  
**Scope:** OpenXR manager, camera/look-at, headrot/keyframe animation, pose blender and post-blend hook, tracking settings/arbitrator, adapter, arm and spine solvers, calibration/continuity, avatar eye markers, related settings and build glue.

This report distinguishes **CONFIRMED source behavior** from **INCOMPLETE feature** and **NEEDS RUNTIME TEST**. It does not infer functionality from “SHIP” text in commit messages or claimed test counts.

## Critical / block primary experience

### A. Stage-2 positional IK does not run in the ordinary build — CONFIRMED

- `indra/newview/llvrtrackingsettings.cpp:39-49`: the sole Stage-2 Gate-1 authorization is a build-time constant `FSVRGate1Authorization::NONE`.
- `indra/newview/llvrtrackingarbitrator.cpp:107-136`: Stage-2 decisions remain shadow-only without authorization.
- `indra/newview/llvradapterapply.cpp:1573-1603`: plan execution requires runtime-effective solver-domain decisions, as well as VR/session and valid positional samples.
- `indra/newview/llvradapterapply.cpp:2708-2712`: the runtime always passes `arm_sample_valid = false`.
- `indra/newview/llvradapterapply.cpp:3692-3706`: wrist target validity is always false for both sides.

**Impact:** The arm solver and positional spine infrastructure can exist and be exercised in isolated tests without driving the avatar in the shipped normal code. It does not meet the minimal HMD + two hands scenario. Do not mistake a successful calibration operation or an in-memory plan for a committed pose change.

### B. No verified tracked-controller input path — INCOMPLETE

- `indra/newview/llopenxrmanager.cpp:1774-1815,1971-1992,2136-2174`: optional `XR_EXT_hand_tracking` capability and native hand-joint tracker creation are present.
- `indra/newview/llopenxrmanager.cpp:3914-3977,6371-6460`: runtime publishes recognized hand-joint/wrist samples.
- The reviewed OpenXR manager does **not** set up OpenXR controller action sets, pose actions, action synchronization, or controller grip/aim spaces.

**Impact:** A wearer with a headset and hand controllers but no active optical hand tracking has no demonstrated controller-pose path into hand targets. Hand-joint tracking is not equivalent to controller tracking. Confirm supported controller sources and runtime action bindings before solver work.

### C. Recenter does not align avatar body-forward — CONFIRMED

- `indra/newview/llagentcamera.cpp:1930-1961`: accepted recenter changes only the HMD base yaw/pitch/roll, without a `gAgent.yaw` or avatar-body heading adjustment.
- `indra/newview/llagentcamera.cpp:1980-1993`: raw yaw subtraction is not wrapped before ±72° clamping. A sample crossing the ±π boundary can turn a tiny yaw difference into a clamp-limit head jump.
- `indra/newview/llopenxrmanager.cpp:2538-2617`: the runtime supplies a `poseInPreviousSpace` transform and the manager records it, but the queued `LLLocalSpaceTransition` retains only identity/time, not the transform.
- `indra/newview/llvrcontinuity.cpp:219-251,384-407`: production finalized transitions deliberately set `has_origin_transform = false`, so the newer session-neutral continuity path **suspends**, rather than rebases, positional calibration.

**Impact:** The problem is shared by head, body, reference-space and positional-neutral systems. A patch that only calls `gAgent.yaw` can leave calibration incoherent. The isolated experimental branch `vr/recenter-forward-alignment-20261108` was written against the older Phase-2 code and **must not be blindly merged** into the newer calibration system. Test straight, ±90°, 180°, wrap, repeated recenter and standing/seated.

### D. No true stereo yet — CONFIRMED incomplete milestone

- `indra/newview/llopenxrmanager.cpp:5267-5400`: the same source framebuffer is blitted to both swapchains; each projection view uses common `submit_pose` and `mono_fov`.
- `indra/newview/llopenxrmanager.cpp:5872-5908`: right cornea is frozen for provenance but not used as the rendered right-eye camera.

**Impact:** Mono-duplicated submission cannot satisfy anatomical binocular viewpoint or normal stereo depth. It is a development fallback, not completed VR.

## Important compatibility / design gaps

### E. Observer Mode and head-rotation-follow option — NOT IMPLEMENTED IN REVIEWED PATHS

- `indra/newview/llagentcamera.cpp:1760-1815,2478-2512`: presence of the viewpoint attachment activates a fixed head-look/camera policy; the final camera axes follow the animated `mHead` orientation unconditionally when the override is active.
- `indra/llcharacter/llkeyframemotion.cpp:744-805`: viewpoint override suppresses animated keyframes specifically on `mHead`, `mNeck`, `mSkull` while active.
- No Observer/Embodied mode authority switch, or distinct `Follow Animated Head Rotation` setting, was found in the reviewed VR subsystems.

**Impact:** Implement a single joint/camera authority switch: Embodied owns tracked joints; Observer leaves all joints animated, eye *position* follows the animated pose, headset controls *view rotation*, with optional animation-rotation composition. Do not simply turn off OpenXR rendering to simulate observer mode.

### F. Anatomy-specific scaling is not yet an end-to-end proven behavior — INCOMPLETE

- `indra/newview/llvrarmsolver.cpp:227-238`: provisional arm target has a ratio parameter but never receives a live wrist target from the adapter.
- `indra/newview/llvradapterapply.cpp:1046-1083`: the standing root ratio defaults to `1.0` unless explicitly overridden, and multiplies a preference gain; it is not automatically derived from avatar anatomy.
- `indra/newview/llvradapterapply.cpp:1086-1184,3415-3420`: standing root-follow is bounded/safety-gated and additionally requires a `teleport_detector_proven` setting.
- `indra/newview/app_settings/settings.xml:197-228`: physical eye-height/manual proportions are settings, but a setting does not prove that the full physical→avatar mapping is operational.

**Impact:** Preserve the scaling research, but verify a very small avatar physically reaches only within its own rig, with expected gestures, before enabling automatic root motion.

### G. The anatomical left-eye finder is weaker than the right-eye finder — CONFIRMED source asymmetry

- `indra/newview/llagentcamera.cpp:214-238`: left-eye finder returns the first attachment with the configured name, without validating eyeball attach point or checking ambiguity.
- `indra/newview/llagentcamera.cpp:499-540`: right-eye finder checks the right eyeball attachment and rejects multiple matches.

**Impact:** Duplicate/misplaced left-eye markers can silently select an inappropriate viewpoint. Apply explicit attachment identity/ambiguity rules and safe missing-marker behavior before declaring anatomical fidelity.

### H. Several visible settings remain non-operative — CONFIRMED

- `indra/newview/llvrtrackingsettings.cpp:723-739`: selective tracking remains unverified and Stage-2 authorization disabled. `FVRBodyFollowsHeadset` is documented as reserved.
- `indra/newview/llagentcamera.cpp:1980-1993`: legacy head rotation uses HMD pitch/roll rather than consuming per-bone shadow arbitration outputs.

**Impact:** Separate experimental/diagnostic controls from working user options. Do not present a shadow decision as an enabled capability.

## Worth keeping — positive source findings

1. `indra/llcharacter/llmotioncontroller.cpp:899-934` invokes the avatar's VR adapter **after** `mPoseBlender.blendAndApply()`; `indra/newview/llvoavatarself.cpp:992-1000` routes it only for the self avatar. The underlying insertion point is credible for non-destructive animation overlays.
2. `indra/newview/llvradapterapply.cpp:1443-1488,1639-1655,3952-3961` builds a base pose and commits specific local joint rotations, rather than replacing every animation joint wholesale. This is a useful mechanism **once tracking actually drives it**.
3. `indra/llcharacter/llkeyframemotion.cpp:744-805` restricts head-keyframe suppression to head/neck/skull in viewpoint mode; other animated joints continue being evaluated. Observer Mode will need to disable this suppression.
4. `indra/newview/llvrboneregistry.cpp:100-150` establishes auditable spine/arm/finger skeleton membership for an eventual anatomy-aware solver.
5. `indra/newview/llopenxrmanager.cpp:2136-2174,3914-3977` already supplies optional optical hand-joint samples that could complement controller actions.
6. `indra/newview/llvrcontinuity.cpp` has explicit discontinuity handling and `llvrbonelog.cpp` has diagnostic facilities; these may help validation once the intended runtime policy is specified.

## Proposed recovery sequence

| Milestone | Evidence that makes it complete |
| --- | --- |
| 0. Freeze and instrument | Record baseline branch/commit; builds reproducibly; traces accurately distinguish tracker input, animation base, committed avatar pose, camera, and OpenXR submission |
| 1. Recenter correctness | Body forward aligns with pre-recenter HMD facing; yaw wraps correctly; LOCAL-space transform and positional neutral are coherent; no unnecessary snap |
| 2. True per-eye stereo | Each eye has its own rendering camera, cornea anchor, pose/FOV and correct source; headset test confirms parallax and stable presentation |
| 3. Minimal head+hands embodiment | Quest controller poses produce wrist targets; arm IK commits intended joint rotations; a full-body sit AO continues animating unclaimed joints |
| 4. Proportional body tracking | Small-avatar arm/lean/root movements follow rig anatomy; standing/seated/teleport transitions tested; remove unnecessary gates or retain only justified safeguards |
| 5. Observer Mode | Full-body animation is preserved; OFF gives headset-controlled view from animated eye position; ON composes animated head rotation; mode transitions do not corrupt tracking |

**Do not start by enabling Gate 1**: the arm samples and targets are still false; the rest of the input and output contracts require validation. Do not replace the entire pose/animation system before testing the existing post-blend hook.

## Audit limits

- This is a static source review of the connected GitHub snapshot, not a compilation, a test suite invocation, or an in-headset observation. Runtime timing, user comfort, shader/render state, OpenXR runtime support, IK naturalness and simulator-side avatar motion remain unverified.
- “No controller action path” is scoped to the reviewed OpenXR implementation; if Muse has a local unpushed component, re-audit it when provided.
- Observer Mode is a recently agreed design requirement, so its absence is a *gap*, not evidence that Muse ignored an earlier specification.
- Some disabled runtime gates may have been deliberate safety choices; what matters for user acceptance is that they cannot honestly count as finished behavior.

**Recovery recommendation:** retain `vr/development`, do not restart the whole viewer, and repair/test milestones from the user-facing experience backward. Preserve isolated experiments until revalidated.
