# Quest/WiVRn recenter failure repair — 2026-11-09

Draft PR #2, branch `vr/recovery-recenter-local-20261108`, base `vr/development` (`7dbc74e7cd`). No merge or headset acceptance.

## Verdict and scope

The supplied real-headset failure is explained at source level: the old deployment gate rejects working WiVRn when `XR_RUNTIME_JSON` is absent, so its LOCAL events never reach `gAgent.yaw`. Even after that gate, `poseValid=false` could not authorize a turn. The existing log, including repeated FOCUSED LOCAL events, has no accepted body commands. An additional defect can hide a successful command: `LLVOAvatar::updateOrientation()` permits idle rendered-root deviation of about 60 degrees from `gAgent.getAtAxis()` and smooths body rotation. A command alone does not prove rendered alignment.

This repair recognizes system/user-installed WiVRn, measures an independent yaw bridge when the event has no transform, targets the actual rendered root heading, aligns that root once, and records subsequent rendered headings. It does not establish that the physical Quest test has passed. Keep the PR draft.

The `AOMLocalHeadAnims` registry repair is already incorporated in ancestor `2661c869f4`. Its default true, false user overrides and independent viewpoint animation suppression are preserved; this patch changes no settings or animation implementation.

## Evidence and coordinate reasoning

OpenXR explicitly leaves `poseInPreviousSpace` undefined when `poseValid=false`. Ordinary pre/post HMD samples do not establish the origin transform: physical head motion, prediction and sampling intervals are confounded. Querying adjacent timestamps alone is also insufficient for the audited WiVRn implementation. The available local WiVRn v26.6 source (`bef986d6`) forwards the recenter request to Monado; its `b_space_overseer.c::recenter_local_spaces()` immediately updates LOCAL/LOCAL_FLOOR offsets and emits `pose_valid=false`, without updating STAGE. This is supporting implementation evidence, not proof of the user's exact v26.9 binary.

Instead, the manager maintains recent **LOCAL-in-STAGE** orientations from successful `xrLocateSpace` calls. For an unknown event it compares a pre-change orientation with the post-change orientation in the same stable STAGE. These are static reference-space relationships; head movement is absent. Orientation VALID is required for this static relationship; the camera separately requires a tracked HMD sample. Unit finite quaternions, rigid geometry and preserved gravity are checked by the existing production math. If `R_old` and `R_new` map old/new LOCAL into STAGE, the bridge is `R_old * inverse(R_new)` in Firestorm's row-vector convention.

A pre-change sample must belong to the same session generation, precede the event, be no older than 250 ms, and lie after the most recent STAGE-change fence. STAGE changes clear history and invalidate pending bridges. Each LOCAL event retires its pre-change history, preventing multiple events from reusing one anchor. Missing STAGE, unavailable/invalid orientation, stale evidence or an unbridged batch fails closed. A malformed **poseValid=true** transform is not rescued by this fallback. Session, duplicate, activation, diagnostic-arm, tracked-sample, landing <=8 degrees, seated/parented, rotation-grab and teleport gates remain.

The bridge contains yaw only. It never sets `has_origin_transform` and is never passed as a fabricated rigid/positional transform. Positional continuity therefore suspends on poseValid=false; history retirement and calibration-transaction invalidation still occur. Full valid event transforms retain the existing positional rebase path. Calibration's turn annotation is the applied root rotation, separately from any larger agent command needed to close pre-existing root/agent disagreement.

OpenXR forward is -Z, up +Y; increasing `atan2(forward.x,-forward.z)` is physical right. Avatar forward is +X, up +Z; physical right is negative LL yaw. The production helper reconstructs old HMD yaw as `post_yaw - old_to_new_yaw`, wraps the delta from the old head base, and negates it for the avatar. The target is the **current rendered root heading plus that physical turn**. The agent command is wrapped `target - current_agent_heading`; the root is aligned to the same target once. Normal avatar animation and subsequent movement remain in control after that act; no continuous body-follow is added. AO pelvis/chest/head twists can remain distinct from root heading and require visual acceptance.

Runtime selection follows Linux x86-64 loader locations: explicit `XR_RUNTIME_JSON` or architecture-specific/plain active-runtime manifests in XDG config home, XDG config directories, then `/etc`. An explicit broken override cannot fall through. Strict JSON parsing requires WiVRn manifest identity and a resolvable library; the canonical library must actually be loaded in the process (`dl_iterate_phdr`), and successful instance properties must report WiVRn or its version-decorated name. Empty/failed/foreign identities reject. This is deployment verification, not cryptographic authentication. Unsupported runtimes remain diagnostic-only.

Primary contracts: [OpenXR reference-space event](https://registry.khronos.org/OpenXR/specs/1.1/man/html/XrEventDataReferenceSpaceChangePending.html), [loader runtime discovery](https://github.com/KhronosGroup/OpenXR-SDK-Source/blob/main/specification/loader/runtime.adoc).

## Validation

Runtime source: `f02a91e03f666ff54b68017118bef231481bd1b9` (preceded by `c9a9afa89e`). **167 production-code checks pass** with AddressSanitizer, UndefinedBehaviorSanitizer and `-Werror`. Six production viewer objects compile with the generated configuration: continuity, calibration, OpenXR manager, camera, adapter and display. The production settings loader/cache regression passes, including false overrides.

A live instance-only probe on this machine, with `XR_RUNTIME_JSON` explicitly unset, returned `xrCreateInstance=0`, `runtime_name=WiVRn 'v26.9-97-g11e36e07'`, `manifest_verified=1`, `accepted=1`. The selected XDG symlink resolves to `/home/snow/.local/share/openxr/1/openxr_wivrn.json`, with its canonical `/home/snow/.local/lib64/wivrn/libopenxr_wivrn.so` loaded. The probe made no `xrGetSystem` or `xrCreateSession` call and created no headset graphics session. This verifies the actual failing deployment's identity path; it does not verify STAGE or body turning. A second instance-only probe selected the actual `/usr/share/openxr/1/openxr_wivrn.json` through a temporary XDG directory, still without `XR_RUNTIME_JSON`. That installed library could not create an instance: client `v26.9` differs from the running service `v26.9-97-g11e36e07` (`XR_ERROR_RUNTIME_UNAVAILABLE`, -51). No version bypass was used and no runtime selection was persisted. Thus end-to-end validation of that separate system binary is blocked by its existing client/service mismatch; the selected working installation passed. `tests/recenter/runtime_identity_probe.cpp` reproduces the check using the production verifier; compile with the candidate include directories, Boost.JSON archive, OpenXR loader and `-ldl`.

The source tests do not execute live `xrPollEvent`, the manager queue or `xrLocateSpace` with a physical session. Those paths compile and require the capture below for end-to-end acceptance. Final package details follow below. The test runner compiles current production math/continuity/calibration code, not a copied yaw formula. Its runtime fixture loads a small test shared library to exercise the real manifest verifier without initializing a physical runtime.

Coverage includes missing `XR_RUNTIME_JSON`, XDG manifest selection, missing/unloaded library, explicit broken override, invalid JSON, decorated WiVRn identity, rejected foreign/empty identities; poseValid=false representation with yaw-only evidence; left/right/wrap/residual yaw; nonzero activation and STAGE orientation; actual root/agent disagreement; root alignment; timestamp/session/STAGE fences; invalid/tilted evidence; chronological composition and duplicate/missing changes; positional suspension and valid-transform continuity. Independent XR-to-avatar vector mapping checks the production heading plan and resulting root quaternion. The capture analyzer is also checked against a known settled fixture and the existing failure log. These are scripted tests, not physical captures of the repaired viewer.

Reproduce inside the configured Ubuntu chroot:

```sh
python3 indra/newview/tests/recenter/run_recenter_tests.py \
  --build-dir build-linux-x86_64 --sanitize --compile-viewer
python3 indra/newview/tests/settings/run_head_animation_settings_test.py \
  --build-dir build-linux-x86_64
```

Build: ReleaseFS_open, AVX2/FMOD, GCC 11.2.0, existing Ubuntu dependencies and incremental outputs, eight jobs, LL_TESTS disabled for the full viewer (standalone tests run separately). The pre-existing local FMOD archive override and explicit NDOF path stay build-only. A candidate-local TMPDIR avoids full `/tmp`. A local `hg` shim supplies the distinct build number because the version script treats a worktree `.git` file as Mercurial. None of these changes are committed to build infrastructure.

## Final package

- Runtime source: `f02a91e03f666ff54b68017118bef231481bd1b9`; subsequent evidence/probe-only commit does not change the viewer.
- Full Ubuntu configure, viewer link and packaging completed with exit 0 and `[100%] Built target llpackage`.
- Archive: `Phoenix-FirestormOS-Snowy_AVX2-7-2-3-80165.tar.xz`, 180,749,260 bytes.
- SHA-256: `2e5afd8e68be38f41217af8b9a6dd0c00d012e3fb1fd488d08120f068af67cdb`.
- `xz -t`, full archive traversal, required-member/executable-mode checks pass. Archived binary SHA-256 is `485e7390985db2f25a042631481b42759c293c53223c59f6b21d02f1ec9e9f65`, matching the packaging input. Binary contains the final `f02a91e03f` source marker and new stage-measurement, rendered-heading and separate agent-turn diagnostics. Archived `build_data.json` identifies 7.2.3.80165.
- Archived settings are byte-identical to the tested source and declare `AOMLocalHeadAnims` exactly once. No animation setting repair was lost.
- Final diff whitespace checks pass. The source checkout is clean after the evidence commit; the detached build retains its pre-existing local FMOD override. Packaging's non-ELF strip notices do not prevent successful packaging. This is an incremental build, not a clean rebuild.

## On-device acceptance and capture

1. Extract the new package into a fresh directory; retain the old viewer. Confirm version 7.2.3.80165. Launch using the normal WiVRn setup, with `XR_RUNTIME_JSON` unset for this test and `FS_OPENXR_P2_RECENTER=1`. Do not enable the VIEW diagnostic arm. From the extracted package directory, launch with:

   ```sh
   env -u XR_RUNTIME_JSON FS_OPENXR_P2_RECENTER=1 ./firestorm
   ```

   Log in and activate the usual viewpoint override and VR mode.
2. Wait at least one second with tracking active. Confirm the log reports `manifest_verification=ok`, `authenticated=true`, the v26.9 WiVRn name, and a stable STAGE anchor available. If not, retain the failure evidence; do not bypass the gates.
3. Stand still and recenter facing forward, right 90 degrees, left 90 degrees, backward, and across +/-180 degrees. Wait about three seconds after each press. Repeat recenter at the same heading, then test after nonzero initial activation heading. Each event must cause exactly one turn, no repeated drift, neutral head yaw and unchanged gravity pitch/roll. Test small turns too: the root must no longer retain idle-heading slack.
4. For the supplied failure class, require `pose_valid=false`, `xr_recenter_stage_measurement` with a valid bridge, then one `xr_recenter_applied` with `transform_valid=false` and `yaw_source="stable_stage"`. It reports physical/root turn separately from `agent_turn_rad`, and before-root, before-agent and target world headings. The next 120 rendered frames emit `xr_recenter_render_heading` samples after `renderFinalize`: agent, root, pelvis and viewer-object headings plus root error. A command log alone is not a pass. Stationary final root error should settle within five degrees, and observed body forward must match the wearer's intended facing in both headset and an independent observer/mirror. Object heading is not a confirmed simulator acknowledgment (`simulator_ack_verified=false`).
5. Repeat tilted/downward, with normal AO/idle/walk animations, and in a tiny/conventional avatar. Check that head gravity references, eyes, animation and viewpoint remain coherent. Root/pelvis/chest animation offsets should be inspected separately.
6. With an established positional neutral, recenter with poseValid=false: expect positional **suspend**, no invented rebase or mixed-coordinate calibration. Re-sync deliberately in the new frame. For poseValid=true fixtures, retain rebase behavior and metadata. Acquisition/calibration windows must abort across either transition.
7. Seated/parented, rotation grabbed and teleport cases must report skipped body alignment and leave body ownership intact. Missing STAGE, tracking loss, stale anchors, concurrent STAGE change or unresolved batches must reject safely. An unknown unbridged change holds head driving until fresh VR activation. Do not count a hold as a recenter success.
8. Extract only relevant diagnostics (the tool finds standard Firestorm log paths automatically):

```sh
python3 indra/newview/tests/recenter/analyze_recenter_capture.py --out recenter-capture
# Or use --log /path/to/Firestorm.log
```

The resulting `recenter-only.log` and `summary.json`, plus a short recording/observer verdict for each angle, are the acceptance evidence. The analyzer reports logged root settling and always leaves physical-headset and simulator-ack acceptance unverified; those require independent observation.

## Remaining limits and rollback

No repaired headset session, in-world visual test, simulator acknowledgment, comfort check or performance measurement has been performed. The additional reference-space query runs once per newly consumed head sample; its live latency cost is unmeasured. The eight-degree landing interlock remains conservative: an out-of-tolerance landing is reported, not silently accepted. STAGE availability/stability on the user's exact WiVRn v26.9 binary must be established by the capture.

Use the earlier 80164 package to roll back the candidate. Source rollback reverts the new recenter commits while retaining ancestor `2661c869f4` and its head-animation setting. Do not merge this draft until physical acceptance is recorded.
