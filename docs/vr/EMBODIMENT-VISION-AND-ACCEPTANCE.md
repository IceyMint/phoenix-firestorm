# Firestorm VR — Embodiment Vision & Acceptance Contract

**Status:** Agreed product vision; technical details remain subject to source audit and headset validation  
**Recorded:** 2026-11-08  
**Working branch:** `vr/development`  
**Project:** `IceyMint/phoenix-firestorm`

## 1. North star

**Build a VRChat-like immersive experience for Second Life using Firestorm, with anatomy-faithful avatars and animation-preserving, tracker-prioritized inverse kinematics (IK).**

The wearer experiences the world *as their Second Life avatar*, rather than as a generic human-sized VR camera. The avatar's actual shape, eye positions, proportions, and rig are the reference for the visual viewpoint and the retargeted body pose.

**VRChat is the behavioral reference. The Second Life avatar's anatomy is the spatial reference. Firestorm's existing animation system remains the animation reference.**

This project does **not** seek to run VRChat worlds, replace Firestorm's entire animation system, or reproduce GoGo Loco's pose menu. GoGo Loco is a useful *behavioral example* of allowing animations to continue on joints not currently controlled by trackers.

## 2. Primary mode: Embodied — “I am my avatar”

- The HMD controls avatar head orientation; the view follows the avatar's anatomical eye positions. Head movement must feel immediate, stable and natural.
- Available hand controllers/hand tracking control the avatar's hands. An anatomy-aware IK solution adjusts the arms, elbows, shoulders, and any necessary supporting joints so the hands can reach their targets without stretching an unusually small or proportioned avatar into a generic adult rig.
- Optional additional trackers control their corresponding body regions. Missing trackers do **not** imply freezing or resetting those regions.
- Physical movement and reaches are **retargeted proportionally** to the avatar's dimensions, rather than mapped one-for-one in world meters regardless of avatar size. Preserve recognizable gestures and joint plausibility. The exact translation/lean/root-follow rules require hardware validation.
- Avatar neck/head movement respects the rig and appropriate constraints. Supporting spine motion should be natural and must not steal unnecessary joints from the animation.
- Untracked joints keep their current Firestorm/Second Life animation result, including Animation Overrider (AO), sitting, walking, and other in-world animation sources.
- Tracker-driven joints and the minimum necessary IK support chain take priority over animations *for those joints only*. Do not disable a whole-body animation merely because the wearer tracks their head and hands.
- Recenter changes the avatar's **forward-facing body heading** to the direction the wearer was physically facing at the moment of recentering, then recenters the head relative to that newly aligned body. Recenter is yaw-only: it must **not** redefine gravity-referenced pitch or roll. It must not become continuous automatic body-follow.
- When seated, attached to a moving object, teleported, or under script-controlled rotation, behavior must be explicitly defined and safely gated. Never silently report body alignment when the avatar's heading did not change.

### Example: HMD + two controllers, no full-body trackers

A sitting animation continues to animate the hips, legs, and any unclaimed joints. The wearer can turn their head, reach with both hands, and move their upper body to the extent supported by the avatar's anatomy. Head/arm/spine IK affects only the joints it actually needs. The rest of the sitting animation is preserved.

## 3. Optional mode: Observer — “I'm inside someone else's body”

Observer Mode is a first-class, switchable mode, **not** a second renderer or a separate avatar rig.

- Normal Firestorm/Second Life animations control **the entire avatar**: head, torso, arms, hands, hips, legs, and accessories as applicable. HMD, controller, and tracker inputs do not take animation ownership of those joints.
- The eye viewpoint stays anchored to the avatar's anatomical eye positions and therefore travels with its animated body and head.
- Physical HMD rotation remains available to look around from that eye position.
- **Follow Animated Head Rotation** is a distinct, user-controllable Observer Mode option:
  - **OFF (default):** The camera viewpoint uses the animated eye *position*, but animation-driven head rotation does not forcibly turn the user's viewing direction. The HMD controls gaze direction.
  - **ON:** The avatar's animated head rotation contributes to the view orientation, while the user can still look around with the HMD. A head-turn animation can therefore turn the view.
- Switching modes or changing the head-rotation option must avoid unnecessary camera jumps and provide an accessible way to restore the comfortable option.
- Observer Mode does not automatically enable continuous body-follow or any new locomotion system. It means the wearer is a passenger with respect to **joint-animation authority**.

## 4. Anatomy-faithful viewpoint and stereo

- Use the avatar's explicit anatomical **left and right eye/cornea markers** for camera positions, not a guessed universal human eye height or an arbitrary offset inside the head.
- The viewer must respect small, large, and nonstandard avatar sizes. Visual scale comes from the avatar's anatomy.
- The target is **true stereo**, with independent left- and right-eye projections/renders. Duplicating one mono image into both OpenXR eyes is a development-stage fallback, **not** finished stereo.
- Camera orientation and the anatomical anchor must remain coherent with the final resolved avatar pose, including when animations and tracking contribute to different joints.
- Avatar-defined eye separation is an intended anatomical property; practical optical comfort and headset compatibility must be validated rather than assumed. A bad or absent marker must produce a detectable, safe fallback, not silently claim anatomy fidelity.
- Existing design intent includes seeing the avatar's hair, glasses, nose, and body when appropriate, without using ordinary desktop mouselook to hide the self-avatar.
- Do not let animated head motion, a calibration adjustment, or a tracking dropout produce avoidable view discontinuity or loss of HMD rotational responsiveness.

## 5. The pose-authority rule

A useful implementation model is:

1. **Resolve Firestorm's normal animation pose** for the frame; this is the base.
2. **Acquire and validate tracking samples** from the HMD, controllers, and any optional trackers, with timestamps and reference-space provenance.
3. **Retarget / solve IK** using the avatar's proportions and rig, claiming only the joints needed by the active tracking solution.
4. **Compose the final pose** by preserving animation values at all unclaimed joints and applying tracking/IK values only at the claimed joints.
5. **Resolve eye anchors and render** from the final pose in a coherent frame; make the OpenXR submission use the actual render state.
6. **When in Observer Mode**, skip tracker-based joint ownership and derive the animated viewpoint and optional head-rotation contribution from the fully animated pose.

This is a *behavioral contract*, not a claim that the current source already follows this architecture. The actual animation update order, root and seated transforms, pose ownership, and render synchronization must be verified against Firestorm source and the headset.

## 6. Required behavior checks

A change is not “done” because a document says SHIP or a unit test passes. Verify all applicable cases:

1. **Recenter:** With the wearer facing straight, 90° left, 90° right, or 180° from the current avatar heading, one recenter aligns avatar forward to physical facing, neutralizes the relative head yaw, and preserves pitch/roll. Test angle wrap boundaries and repeat recenter.
2. **Animation coexistence:** While a full-body sit, walk, or AO animation plays, head-and-hands tracking changes only the intended head, arms and necessary support joints. Untracked legs/hips and unrelated joints keep animating.
3. **Small-avatar reach:** A compact avatar reproduces a physical gesture within its own reach and proportions without stretching a limb to adult-human dimensions.
4. **Partial tracking/dropout:** Add or lose a controller, hand-tracking sample, or optional tracker without freezing the rest of the animated avatar or leaving persistent incorrect joint ownership.
5. **Observer OFF:** A head-turn animation visually moves the avatar's head and eye position, but does not turn the user's gaze; physical head rotation still looks around.
6. **Observer ON:** The same head-turn animation contributes to viewpoint orientation; HMD look-around still works. Toggling is accessible and avoids avoidable sudden snaps.
7. **Mode switch:** Moving between Observer and Embodied modes does not leave stale tracking ownership or corrupted calibration.
8. **True stereo:** Left and right eyes receive genuinely different renders from appropriate anatomical anchors with valid OpenXR projection and timing.
9. **Avatar variants:** Test tiny and conventional avatars, unusual eye placement, missing/ambiguous eye markers, different animation sets, and different sit/stand states.
10. **Safety / traceability:** Invalid poses, reference-space changes, teleports, and seat/parent transitions produce explicit, diagnosable outcomes rather than silent assumptions.

## 7. Engineering process — evidence over assumptions

- **Stop feature expansion while the fundamentals are incorrect.** Audit the current `vr/development` implementation from the actual call paths, not just prose and commit descriptions.
- Treat each change as a small, independently testable behavioral improvement with an expected result, a rollback path, and a headset-test checklist.
- Distinguish **design intent**, **source-level proof**, **automated-test evidence**, **successful build**, and **on-device observation**. Never promote one category to another without evidence.
- Preserve all existing branches and experimental code while assessing recovery; delete or replace a subsystem only after identifying why it is wrong.
- Reconcile any old recenter experiments with the newer P2-07B calibration/sync code before merging them.
- Keep independent head/eye rendering, animation ownership, avatar root, and reference-space transforms conceptually separate, but verify their frame ordering together.
- Prefer existing Firestorm systems for animations and avatar rigs rather than introducing a parallel replacement without a demonstrated need.

## 8. Still open; do not guess

The broad behavior is agreed, but the following are **not yet settled implementation specifications**:

- Exact physical-to-avatar translation scaling, root-follow and lean limits, seated movement, and how teleport/sit transitions suspend or resume them.
- IK distribution across neck/spine/shoulders, per-avatar joint constraints, and behavior when tracker targets are unreachable.
- Precise OpenXR-to-avatar coordinate conventions, seated recenter implementation, and how to align an avatar held by scripts.
- Whether an extreme avatar eye separation needs an optional comfort override, and what a safe missing-marker fallback should be.
- The rotational composition, interpolation and comfort treatment for Observer Mode's “Follow Animated Head Rotation.”
- Input/menu bindings and user-facing wording for switching modes and toggling Observer head rotation.

Decide these from the user's intended experience **and** reproducible runtime evidence, never from assumptions about what VRChat, Firestorm, or an existing patch “must” be doing.

---

**One-line acceptance standard:** *A VR user can inhabit a Second Life avatar naturally like in VRChat, with gestures scaled to that avatar's anatomy, its untracked body still animated, its eyes defining the view, and an optional passenger/Observer experience.*
