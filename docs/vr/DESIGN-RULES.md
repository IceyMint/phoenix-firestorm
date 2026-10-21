# Firestorm-Snowy VR — Leah's design rules (authoritative)

Stated by Leah across 2026-10-20. These govern all VR work on the fork.
Do not propose violating them; where standard practice conflicts, the
rule wins and the conflict gets documented, not silently resolved.

1. **Head constraints stay.** The in-game head must never rotate past
   normal human body constraints — SL's own LLHeadRotMotion limit
   (±72°), applied relative to the avatar's torso. (2026-10-20 18:43)
2. **Animations are enjoyed.** Only the head animation is disabled in
   first-person VR mode (mHead/mNeck/mSkull keyframes); torso, AO,
   dances, sits all keep playing. (18:49)
3. **Camera hard locked to the head.** Position from the
   viewpoint_override attachment/head, orientation axes = head joint
   world rotation. (18:49)
4. **Physical position never moves the camera.** Only physical rotation
   drives the in-game head. No lean translation, no roomscale camera
   movement, no body-follow turning, no neck-pivot offset. (18:49)
5. **Left-eye correctness first.** Leah's right-eye vision is poor; she
   will not personally perceive stereo. Proper projection for the left
   eye is the priority; full stereo also approved (2026-10-20 19:00)
   for other users and because her PC can handle it.
6. **The eyes are the AVATAR's eyes.** (2026-10-20 19:59) The eye
   position in the view must match the avatar's eyes exactly —
   including eye SPACING. Reason: immersion/embodiment — the player
   must feel they ARE the character. A small avatar has narrower eye
   spacing than the player's real eyes, and the world should feel
   correspondingly big; a large avatar the reverse. This deliberately
   overrides the standard "use the player's physical IPD" practice
   (and ChatGPT's 2026-10-20 recommendation against avatar-derived
   eye spacing): physical-HMD IPD is NOT the stereo baseline here;
   the avatar is.
   HOW IT WORKS TODAY (Leah, 2026-10-20 20:02): the viewpoint_override
   attachment is placed on the avatar's LEFT EYE, at the cornea — that
   placement IS the camera anchor. (Smart detail: the eye joint's
   origin is the center of the eyeball; a camera there would sit
   inside the eyeball mesh. The cornea puts it at the surface.)
   WHY AN ATTACHMENT, NOT THE EYE JOINT (Leah, 2026-10-20 20:04): she
   does not trust that the eye joint is where the eye MESH actually
   appears. Well-founded: skeleton joint positions are data from the
   skeleton/appearance system, while rendered eyes may be morphed
   system eyes or separate worn mesh-eye attachments that are not
   bound to mEyeLeft/mEyeRight at all. The attachment is placed
   VISUALLY, by eye, on the rendered cornea — placement she can see
   and verify. Trust model for all eye positions in this project:
   visually placed markers beat skeleton data.
   CONSEQUENCE for stereo: Muse's skeleton-derived right eye
   (mEyeRight_world + attachment offset) is DEMOTED to fallback-only —
   it inherits exactly the distrust above.
   THE ATTACHMENT ITSELF (Leah, 2026-10-20 20:07): a SINGLE PRIM,
   worn on the LEFT EYEBALL attach point, positioned at the cornea.
   ADOPTED stereo extension (Muse proposal, Leah approved
   2026-10-20 20:10): a second identical single prim worn on the
   RIGHT EYEBALL attach point, positioned at the right cornea the
   same way. Code
   finds both worn markers by name; both eye positions are then
   visually placed facts and spacing follows the avatar exactly.
   Graceful degradation: right marker missing -> mono-left mode
   (Leah's own default anyway); left missing -> VR mode off, as
   today. Last-resort fallbacks (skeleton eye joints, then mirroring
   across the head's sagittal plane) are diagnostic or explicitly
   selected emergency modes ONLY — never an automatic route into
   stereo; stereo requires both markers. (Clarified in the round-2
   review, 2026-10-20.) Marker sampling uses the prim's PIVOT, and
   each marker must validate as the local avatar's, on the correct
   eyeball attach point, with a usable drawable.

7. **The avatar's gaze moves the camera.** (Leah, 2026-10-20 20:26)
   Round-2 item B2 asked whether eyeball-attach markers moving with
   the avatar's eye-gaze animation is a problem. Leah's answer: it is
   WANTED. If the avatar looks right, the user looks right — the
   avatar's eye gaze affects the camera, not just the head joint.
   This stands UNTIL she has an eye-tracking-capable headset; the
   intent is that physical eye tracking would eventually take over
   this role. Consequences for the plan: (a) B2 is a feature, not a
   defect — do not instrument it as a bug; the AvatarEyeFrame simply
   captures the gaze-influenced state like everything else. (b) The
   camera orientation model is: head joint world rotation COMPOSED
   WITH the avatar's eye-gaze offset (the eyes' look direction
   relative to the head), within the rule-1 clamp philosophy — the
   gaze is the avatar's own, already constrained by SL's look-at
   system. (c) Marker positions riding the eyeball attach points
   with gaze is consistent with this rule for the same reason.

8. **Self-visibility follows real life.** (Leah, 2026-10-20 21:14)
   "I want to see my hair. And anything you would see in real life.
   My own head doesn't disappear in real life." From the cornea view
   the user must see what a person actually sees of themselves:
   their HAIR framing the view (fringe/bangs at the top edge, long
   hair over the shoulders/chest looking down, hair moving with the
   animations), GLASSES FRAMES if the avatar wears them (Leah wears
   glasses in real life — "I see my glasses on my face right now" —
   and her avatar wears them too), the nose at extreme down-gaze,
   and the body below. The standard industry approach of hiding the
   whole head in first person is explicitly REJECTED. Implementation
   shape (for the embodiment plan): hide the face/skin/eyes that
   would block the cornea, keep hair and head-worn items visible,
   with near-distance fading for geometry that crosses the camera —
   selective visibility, not head deletion.

9. **Attention is manual, and the mouse moves ONLY the eyes.**
   (Leah, 2026-10-20 21:44; CORRECTED by Leah 21:58 — an earlier
   draft of this rule wrongly let the mouse turn the head past the
   eye limit; she caught it: "the mouse should only move eyes, the
   head is controlled by headset.")
   The mouse steers the avatar's EYES, within the avatar's own
   eye-motion limits, through the avatar's layered look-at
   machinery. The HEAD is controlled by the physical HEADSET —
   HMD rotation, Rule #1's ±72° clamp — never by the mouse. If a
   look target lies beyond the eyes' range, the eyes saturate at
   their limit looking toward it; to look further you turn your
   own head, exactly as in real life. Division of control is
   absolute: body turns the head, mouse moves the eyes. It is the
   input half of Rule #7: the user commands the gaze, the gaze
   moves the camera. Leah on social attention: "I'll prolly
   manually look at [people]" — there is NO autopilot steering her
   view toward speakers (embodiment seed S5 stays parked,
   superseded by this rule). Composition is therefore simple and
   conflict-free: head orientation from the HMD path, eye offset
   from the gaze (mouse-steered or autonomous), camera = the
   composition (Rule #7). Still needed: a small deadzone so mouse
   tremor never becomes view swim, and a defined mode boundary
   between mouse-as-pointer (UI/clicking) and mouse-as-gaze — with
   S3, cursor, gaze and view converge on the same point.
   AMENDMENT (Leah, same evening): manual control is an OVERRIDE,
   autonomy is the resting state. When the mouse is not actively
   steering them, the EYES return to the avatar's AUTONOMOUS
   control — her native look-at/interest behaviour (glancing at
   speakers, things that move, whatever the avatar herself would
   look at) — and per Rule #7 the camera follows that autonomous
   gaze too. So "no autopilot" means no ADDED steering system; the
   avatar's own attention was never removed, it is what manual
   control temporarily borrows. Hers by default, yours while you
   steer, hers again when you stop. RELEASE TRIGGER (Leah, same
   evening): the mouse NOT MOVING ends the override — stillness
   hands the eyes back to autonomous control. MECHANISM (Leah,
   pointing at existing behaviour): this is the viewer's OWN
   look-at target behaviour — in the official viewer the look-at
   target already moves with the mouse and resets after a few
   seconds. Rule #9 therefore RIDES the existing look-at machinery
   and its native reset timer rather than inventing a new gaze
   system or a new timer; the timer is good as it stands. Any
   retuning of the duration needs evidence from the prototype that
   the native value feels wrong in the headset.
