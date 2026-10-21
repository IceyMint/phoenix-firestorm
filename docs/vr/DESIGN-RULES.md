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

9. **Attention is manual and body-ordered.** (Leah, 2026-10-20 21:44)
   The mouse steers the avatar's gaze: moving it first moves the EYES
   (within the avatar's own eye-motion limits); pushing further turns
   the HEAD (within Rule #1's existing ±72° clamp). This is how a
   person actually looks — eyes for small shifts, head for large
   ones — performed by the avatar's own layered look-at machinery
   (eyes lead, head follows, with its own smoothing), not a custom
   free-look controller. It is the input half of Rule #7: the user
   commands the gaze, the gaze moves the camera. Leah on social
   attention: "I'll prolly manually look at [people]" — there is NO
   autopilot steering her view toward speakers (embodiment seed S5
   stays parked, superseded by this rule). Composition with the HMD
   (prototype questions, not blockers): physical head rotation stays
   primary; the mouse contributes an eye offset plus a saturating
   head offset inside the same clamp. Needs a small deadzone so mouse
   tremor never becomes view swim, and a defined mode boundary
   between mouse-as-pointer (UI/clicking) and mouse-as-gaze — with
   S3, cursor, gaze and view converge on the same point.
