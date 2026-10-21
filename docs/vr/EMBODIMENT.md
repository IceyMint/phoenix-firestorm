# Embodiment track — "I AM the avatar"

Design principle (ChatGPT, round 5, adopted): **make perception
follow the avatar's actual state, never falsify it.** Rules #6
(avatar's eyes), #7 (avatar's gaze) and #8 (real self-visibility)
are this principle applied three times.

Framing (round 5): embodiment is a DISTINCT development track
alongside the stereo plan (PLAN.md). Audits and minimal changes can
start now; features that depend on the camera/projection should only
be enabled after Phase 1/2 establish a stable headset image — more
camera motion is not automatically more immersive, especially while
projection and stability are broken. Full round-5 text: the
"Critique stereo VR plan" ChatGPT conversation (2026-10-20); this
document is the reconciled record.

## The seeds and their verdicts

### S1 — Own-body presence + Rule #8 self-visibility — PURSUE (top rank)
Looking down must show YOUR body, outfit and animations. Per the
round-5 addendum, with Rule #8 this becomes **anatomically selective
self-visibility**: the avatar's real hair, glasses, nose and facial
geometry RENDER from the cornea views; only geometry that genuinely
intersects an eye is suppressed, locally, via a **corneal obstruction
mask** (distance + direction from the eye, coherent across both eyes
so nothing flickers between them). Visibility policy is per
attachment/per material class, user-selectable. Split:
- **S1a — visibility audit & restoration (can start NOW):** find and
  inventory the mechanisms that currently hide the self in first
  person (the addendum reply names three, with citations, in the
  chat transcript — re-derive at audit time), restore body/hair/
  glasses rendering behind a setting.
- **S1b — corneal masking (Phase 3/4):** the obstruction mask,
  integrated with the per-eye render passes.

### S2 — Ear-anchored audio — PURSUE (rank 2)
Audio listener moves from the desktop camera to the avatar's head
(between the markers, camera orientation): SL voice, ambience and
media spatialise around the avatar's ears; other avatars' voices come
from their positions relative to YOUR head as it turns in the clamp.

### S3 — Gaze interaction — PURSUE (rank 5)
No controllers exist; the avatar's gaze direction (Rule #7) becomes
the interaction ray — dwell/look-to-select — augmenting or replacing
the desktop mouse cursor drawn into the image.

### S4 — Face-proximity grace — PARK
Fade/near-plane handling when the avatar's face enters geometry.
Useful failure mode, not on the critical path.

### S5 — Social eye-contact loop — PARK (automatic form)
Your avatar glancing at a speaker already drags your view (Rule #7).
Deliberate/automatic gaze-steering as a feature: parked — comfort
and feedback-loop questions (see the round-3 cautions) unresolved.
**Superseded in practice by Design Rule #9 (Leah, 2026-10-20):**
attention is manual — the mouse steers her gaze (EYES ONLY; the head
belongs to the physical headset — Rule #9 as corrected by Leah).
"I'll prolly manually look at [people]." No
ADDED autopilot; S14's speaker cue may highlight WHO is talking,
Rule #9 decides where she looks. Rule #9 amendment (same evening):
manual steering is an override — when the mouse stops steering,
the eyes return to the avatar's own AUTONOMOUS look-at behaviour,
which natively includes glancing at speakers. S5's substance thus
arrives as the resting state of Rule #9 rather than as a feature.

### S6 — Micro-life (breathing/sway) — PRESERVE
Inherited head-joint micro-motion already reaches the camera.
No change; verify amounts during testing (too much = sickness,
too little = mannequin).

### S7 — Voice embodiment — SPLIT
Lip-sync (local voice driving the avatar's mouth/jaw): PURSUE.
Self-voice sidetone/monitoring: PARK.

### S8 — Entry ritual — PURSUE (rank 4)
A deliberate "step into the avatar" transition (desktop third-person
→ fade through to the cornea) when VR enables, instead of a hard cut.
Cheap; outsized presence payoff per ChatGPT's ranking.

### S9 — View blinking synced to avatar blinks — KILLED as a default
Involuntary view blackouts judged a comfort risk. (Designer's
prerogative: Leah may resurrect it as an opt-in experiment.)

### S10 — Own shadow & reflections — PURSUE
Your shadow correct and visible from first person (grounding);
reflections/mirrors showing your face where SL's systems allow.

## ChatGPT's additions (S11–S15)

- **S11 — Calibration/test scene: PURSUE (rank 3).** A controlled
  scene (grid, near/far objects, seated/social setups) used by the
  Phase 0 experiments and every embodiment test. Infrastructure that
  makes all other items measurable.
- **S12 — Selective own footsteps: PURSUE.** Hear your own feet when
  your avatar walks — selectively (surfaces/context), not a constant
  noise layer.
- **S13 — Worn/native UI ideas: PARK.**
- **S14 — Speaker identification: PURSUE.** Knowing who is speaking
  by their avatar (voice + look-at + subtle cue), reinforcing the
  social loop S2/S5 build on.
- **S15 — Seated-contact audit: PURSUE (joint rank 1 with S1).**
  When the avatar sits, the view must sit IN the seat: correct eye
  height, contact, no floating/interpenetration illusions. Social SL
  is mostly seated — this is where embodiment is won or lost daily.

## Ranking (round 5, by payoff per unit of risk)
1. S1 + S15 — self-visibility incl. hair/glasses + seated contact
2. S2 — ear-anchored audio
3. S11 — calibration scene
4. S8 — entry ritual
5. S3 — gaze interaction

## Status
Menu presented to Leah 2026-10-20. Her picks/overrules pending —
she is the designer; verdicts above are recommendations. Nothing here
is implementation-authorized yet (discussion phase, as with the
stereo plan; S1a and the Phase 0 work are the natural first "go"
candidates).
