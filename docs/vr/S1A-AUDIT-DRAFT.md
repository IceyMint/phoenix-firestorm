# S1a — Self-Visibility Source Audit (DRAFT for Muse's review)

> **CORRECTION / RESOLUTION (Leah, 2026-10-20 22:41):** her VR mode
> does NOT use mouselook, and "nothing is hidden when in VR and I
> like it like that." Every mechanism M1–M4 below is gated on
> CAMERA_MODE_MOUSELOOK and therefore does NOT apply to her VR path:
> in VR the full avatar (body, head, hair, glasses, all attachments)
> already renders, which is the behaviour she wants. This audit
> stands as the map of what MOUSELOOK would hide (relevant only if
> desktop first-person embodiment is ever wanted). The only item
> that still applies to VR is M5, the override's 0.01 m near clip.
> S1a's VR scope is therefore: near-clip behaviour for near-face
> geometry (glasses, fringe) + the later S1b corneal masking.

Read-only audit of Leah's Firestorm tree
(`/run/media/snow/Games/Software/source/phoenix-firestorm`, branch
`port/try-filtered-patch-2026-03-02`), 2026-10-20. Goal: inventory every
mechanism that can hide or cull the LOCAL avatar (or parts of it) from its
own first-person / mouselook / `viewpoint_override` cornea camera, per
Design Rule #8 ("anatomically selective self-visibility").

All citations are `file:line` inside `indra/newview/` unless noted, and
were read directly during this audit. Nothing on the PC was modified.

## Headline

There is **no single "hide self" switch**. Self-hiding in mouselook is
layered: (1) a master setting that gates the whole avatar, (2) a separate
gate that removes the system head/hair/eyelashes even when the master
setting is on, (3) a per-attachment-point data table that hides **every**
attachment worn on a head point — glasses and worn mesh hair included —
even when the master setting is on, and (4) the near plane.

**Leah's live user setting already has the master switch ON**
(`~/.firestorm_x64/user_settings/settings.xml`: `FirstPersonAvatarVisible`
Value = 1). So for her, the operative hides today are mechanisms M2, M3,
M4 and M5 below — her body below the neck should already render in
mouselook; her head, system hair, eyelashes, eyeballs, and all head-point
attachments (glasses, worn hair) do not.

**Key structural fact:** every hide below is keyed on
`gAgentCamera.cameraMouselook()` (CAMERA_MODE_MOUSELOOK). The
`viewpoint_override` path itself does **not** set mouselook mode, does
not touch any of these gates, and lives entirely in
`llagentcamera.cpp` (camera position/axes/near clip). See §"viewpoint_override
interaction" — applicability of M1–M4/M6 to the cornea camera is therefore
*conditional on the viewer also being in mouselook mode*.

---

## M1 — Master gate: whole self avatar skipped in mouselook unless `FirstPersonAvatarVisible`

- **Setting:** `app_settings/settings.xml:5295` `FirstPersonAvatarVisible`,
  Boolean, **default 0**, comment: *"Display avatar and attachments below
  neck while in mouse look"*. Persisted; exposed in Preferences/PhotoTools UI.
- **Plumbing:** static `LLVOAvatar::sVisibleInFirstPerson` declared
  `llvoavatar.h:377`, initialised `false` at `llvoavatar.cpp:648`, loaded
  from the setting at startup `llappviewer.cpp:638`, live-updated via
  `llviewercontrol.cpp:185-187` (`handleRenderAvatarMouselookChanged`,
  listener registered `llviewercontrol.cpp:1313`).
- **Gate:** `llagent.cpp:2559-2571` `LLAgent::needsRenderAvatar()` returns
  `false` when `cameraMouselook() && !sVisibleInFirstPerson` (also requires
  `mShowAvatar && mOutfitChosen` in general).
- **Effect:** `llvoavatar.cpp:6171` in `renderSkinned()`:
  `if (isSelf() && !gAgent.needsRenderAvatar()) return 0;` — the entire
  skinned body renders nothing. Rigged attachments follow the same gate via
  `llvoavatarself.cpp:3322-3325` `LLVOAvatarSelf::shouldRenderRigged()`
  returning `gAgent.needsRenderAvatar()`.
- **Hides:** the whole avatar (all system meshes + rigged attachments).
- **Applies to cornea camera?** Only when `cameraMouselook()` is true.
  **Not operative for Leah** (her setting is 1 — verified above).
- **Restoration idea:** for users with the setting off, treat
  `viewpoint_override_active` as satisfying this gate in `needsRenderAvatar()`.

## M2 — Head gate: system head, system hair and eyelashes removed in mouselook *even with M1 on*

- **Gate:** `llagent.cpp:2574-2579` `LLAgent::needsRenderHead()` returns
  true only if `(sVisibleInFirstPerson && LLPipeline::sReflectionRender)`
  or `(mShowAvatar && !cameraMouselook())` — i.e. in a normal (non-reflection)
  mouselook render it is **false regardless of the M1 setting**.
- **Call sites (all `!isSelf() || gAgent.needsRenderHead() || LLPipeline::sShadowRender`):**
  - `llvoavatar.cpp:6192` — opaque pass: system **head mesh**
    (`MESH_ID_HEAD`) skipped for self.
  - `llvoavatar.cpp:6251` — transparent pass `renderTransparent()`:
    **eyelash mesh** (`MESH_ID_EYELASH`, :6259) and **system hair mesh**
    (`MESH_ID_HAIR`, :6266) skipped for self.
  - `llvoavatar.cpp:6107` — low-shader-level path: same trio's
    `updateJointGeometry()` skipped for self.
- **Hides:** system head/face mesh, system hair, eyelashes. This is the
  mechanism that removes *system* hair in first person — directly in
  Rule #8's scope.
- **Applies to cornea camera?** Yes, under the same mouselook condition as M1;
  shadow passes exempted (`sShadowRender`), and reflection renders exempted
  via the `sReflectionRender` term in the gate itself.
- **Restoration idea:** make `needsRenderHead()` return true for self when
  the viewpoint override is active — with the caveat that this restores the
  *whole* head (face interior surfaces included); making it *selective*
  (hair yes, cornea-blocking face no) is the S1b corneal-mask work, not S1a.

## M3 — Eyeball gate: system eyeballs never render for self in mouselook

- **Site:** `llvoavatar.cpp:6298` in `renderRigid()`:
  `if (isSelf() && (!gAgent.needsRenderAvatar() || !gAgent.needsRenderHead())) return 0;`
  The eyeball meshes (`MESH_ID_EYEBALL_LEFT`/`RIGHT`, :6304-6313) sit below it.
- **Hides:** both system eyeballs whenever either gate is off — always, in
  mouselook.
- **Applies to cornea camera?** Yes (mouselook condition). Note: the camera
  sits *at* the left cornea, so suppressing the eyeballs is plausibly
  **desired** under Rule #8 (they are the one thing that genuinely
  intersects the viewpoint); flagged here for completeness, likely
  "keep hidden / S1b decides", not an S1a restore candidate.

## M4 — Per-attachment-point table: ALL attachments on head points hidden in mouselook

This is the biggest Rule #8 item: it hides **worn** hair and **glasses**.

- **Data:** `indra/newview/character/avatar_lad.xml`, attribute
  `visible_in_first_person` per `<attachment_point>`. The points set to
  `"false"` are exactly the head group (group 2) plus jaw/tongue:
  Skull id 2 (`avatar_lad.xml:20-28`, ATTACH_HEAD, joint mHead),
  Mouth id 11 (:121-129), Chin id 12 (:~132-140), Left Ear id 13,
  Right Ear id 14, **Left Eyeball id 15 (ATTACH_LEYE, joint mEyeLeft —
  the point the `viewpoint_override` prim itself is worn on)**,
  Right Eyeball id 16, Nose id 17 (block range :113-196, read directly),
  and Jaw id 47, Alt Left Ear id 48, Alt Right Ear id 49, Alt Left Eye
  id 50, Alt Right Eye id 51, Tongue id 52 (block range :527-590).
  All other attachment points are `"true"`.
- **Plumbing:** attribute read at `indra/llappearance/llavatarappearance.cpp:1899`
  into `AttachmentInfo::mVisibleFirstPerson` (default `false`,
  `indra/llappearance/llavatarappearance.h:436`); copied onto the joint
  attachment at `llvoavatar.cpp:8166`
  (`attachment->setVisibleInFirstPerson(info->mVisibleFirstPerson)`);
  flag accessors `llviewerjointattachment.h:68-69`.
- **Enforcement:** `llvoavatarself.cpp:1447-1503`
  `LLVOAvatarSelf::updateAttachmentVisibility(camera_mode)`, mouselook case
  at :1478-1491: an attachment is visible only if
  `(sVisibleInFirstPerson && attachment->getVisibleInFirstPerson())`
  or a mirror pass is rendering; otherwise
  `attachment->setAttachmentVisibility(false)`. (HUD attachments are
  exempted earlier, :1458-1461.) Called with the live camera mode from
  `llagent.cpp:3034`, `llagentcamera.cpp:1867`, `llvoavatarself.cpp:1436`
  and `:1650`, `pipeline.cpp:11082` and `:11697`.
- **Hide mechanics:** `llviewerjointattachment.cpp:350-372`
  `setAttachmentVisibility(false)` sets each attached drawable's spatial
  bridge type to 0, and `pipeline.cpp:12250-12256` `LLPipeline::hasRenderType`
  returns false for type 0 (STORM-365), so the objects are never drawn.
- **Hides (in mouselook):** everything worn on the points above — in
  practice: **glasses** (commonly Nose or Skull point), **worn mesh hair**
  (commonly Skull point), earrings (ear points), eye/nose/mouth
  accessories, and the `viewpoint_override` prim itself (harmless — it
  *is* the camera). With the M1 setting OFF it hides **all** non-HUD
  attachments (the `sVisibleInFirstPerson &&` term fails for every point).
- **Applies to cornea camera?** Yes under the mouselook condition; this is
  operative for Leah today despite her M1 setting being on.
- **Restoration idea:** flip the head-group points to
  `visible_in_first_person="true"` in `avatar_lad.xml` (data-only change),
  or special-case `updateAttachmentVisibility` when the override is
  active. Which worn items *should* reappear (vs. stay hidden because they
  cross the cornea) is a per-point/per-item policy decision — S1b's
  corneal mask territory; S1a can restore the points wholesale.

## M5 — Near plane: geometry within 1 cm of the cornea is clipped (override active)

- **Constants:** `llagentcamera.cpp:133` `DEFAULT_CAMERA_NEAR_CLIP = 0.1f`;
  `llagentcamera.cpp:134` `VIEWPOINT_OVERRIDE_CAMERA_NEAR_CLIP = 0.01f`.
- **Application:** `llagentcamera.cpp:1930-1931` sets the viewer camera
  near to 0.01 whenever a `viewpoint_override` mesh is worn (any camera
  mode), `llagentcamera.cpp:2801-2803` `getPreferredNearClip()` returns
  the same, and the render loop applies it per frame at
  `llviewerdisplay.cpp:784`. Default 0.1 m set at `llagentcamera.cpp:381`.
- **Hides:** nothing wholesale — but any restored geometry (nose tip,
  glasses lens/frame parts, hair strands) passing within **1 cm** of the
  override prim's pivot will be near-clipped. With the override off, the
  radius is 10 cm, which would eat much of a nose/ glasses from a cornea
  camera.
- **Applies to cornea camera?** Yes — this is the one mechanism the
  override itself controls.
- **Restoration idea:** none needed at 0.01 as a global value; residual
  clipping of sub-centimetre geometry belongs to S1b's near-fade work.

## M6 — RLVa restrictions (conditional; only with RLV enabled and an active restriction)

- `llagent.cpp:2563`: `RLV_BHVR_SHOWSELF` forces `needsRenderAvatar()` false.
- `llagent.cpp:2577`: `RLV_BHVR_SHOWSELFHEAD` forces `needsRenderHead()` false.
- `llvoavatarself.cpp:1468-1475`: under RLV, `@showself` hides all self
  attachments and `@showselfhead` hides head-group (RLV_ATTACHGROUP_HEAD)
  attachments in `updateAttachmentVisibility`, in any camera mode.
- **Applies to cornea camera?** Only while such a restriction is active.
  Noted for completeness; no action unless Leah runs RLV restrictions.

## M7 — Shadow pass interaction (minor; not a cornea-camera hide)

- `pipeline.cpp:10929-10937` `generateSunShadow()`: when in mouselook with
  `sVisibleInFirstPerson` on, it calls
  `updateAttachmentVisibility(CAMERA_MODE_THIRD_PERSON)` for the shadow
  pass so the self avatar's attachments can cast shadows, then the normal
  per-frame calls restore mouselook visibility. Recorded so it isn't
  mistaken for a hide: it *helps* self-shadowing (relevant to embodiment
  seed S10), and it means attachment visibility is mode-switched per pass.

## viewpoint_override interaction (summary)

From `llagentcamera.cpp` (all read directly):

- Activation is purely "a worn object named `viewpoint_override` exists":
  constant `llagentcamera.cpp:129`, finder `findViewpointOverrideMesh()`
  at :149 (name match :166), active flags at :1391 and :1928-1929.
- When active, `updateCamera()` sets the camera position to the override
  mesh's render position (:1938), forces near clip 0.01 (:1930-1931), sets
  camera axes from `mHeadp` world rotation (:1952-1961, fallback
  `mEyeLeftp`), and sets the head-anim-disable animation data
  (:1939-1941; animation, not visibility — out of scope, noted only).
- It does **not** call `changeCameraToMouselook`, set
  `sVisibleInFirstPerson`, or touch `needsRenderAvatar`/`needsRenderHead`/
  `updateAttachmentVisibility`. `llopenxrmanager.cpp` contains **no**
  avatar-visibility logic (grep for `needsRender*`,
  `sVisibleInFirstPerson`, `updateAttachmentVisibility` there returned
  nothing) — XR submits the normally rendered frame.
- Consequence: M1–M4/M6 apply to the cornea camera **iff** the viewer is
  in CAMERA_MODE_MOUSELOOK while the override is worn. If the override
  were worn in third-person mode, none of the mouselook hides would bite
  (and the rendered head would surround the cornea camera instead).
  Which mode Leah's VR sessions actually run in is a runtime fact, not
  determinable from source — see open questions.

## Checked and cleared (not self-hiding mechanisms)

- `llvoavatar.cpp:2285` and `:2399` — `needsRenderAvatar()` gates inside
  `lineSegmentIntersect` (picking/raycast), not rendering.
- `llvoavatar.cpp:3833-3845` — suppresses the self **nametag/chat bubble**
  in mouselook, not geometry.
- `llvoavatar.cpp:5835` (`head_scale`) — debug logging of joint scales only.
- `llspatialpartition.cpp:3177` (`isSelf`) — debug crosshair rendering only.
- `llvoavatar.cpp:5265/5295/5308/5379` (`self_in_mouselook`) — orientation/
  pelvis-follow behaviour in `updateOrientation()`, not visibility.

## Could NOT determine from source

1. Whether Leah's VR sessions run with `cameraMouselook() == true` while
   the override is worn / XR is on. No code found forces mouselook mode
   for either; applicability of M1–M4/M6 to her actual cornea view
   depends on this runtime fact. (Her `FirstPersonAvatarVisible=1` user
   setting suggests she uses mouselook-style first person, but that is
   inference, not evidence.)
2. Which attachment points Leah's actual hair and glasses are worn on.
   M4's impact assumes the common cases (hair→Skull, glasses→Nose/Skull);
   her inventory was not inspected.
3. Whether any *other* content-side hiding exists in her worn items
   themselves (e.g. scripts in attachments toggling visibility in
   mouselook) — out of source scope.
4. Exact per-frame ordering between `generateSunShadow`'s third-person
   visibility forcing (M7) and the XR submit path in `llopenxrmanager.cpp`
   — the manager file was grepped, not read end-to-end.
