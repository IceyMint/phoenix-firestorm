# Recovery milestone 1: recenter implementation and evidence

Status: proposed source change for review; **not headset acceptance**.

## Baseline and scope

- Reviewed baseline: `vr/development` at `7dbc74e7cd712b037b8c471f04075b056ba215c6`.
- Feature branch: `vr/recovery-recenter-local-20261108`, in a separate worktree. Existing dirty working branches were left in place.
- Read both [the embodiment contract](EMBODIMENT-VISION-AND-ACCEPTANCE.md) and [the recovery audit](RECOVERY-CODE-AUDIT-2026-11-08.md).
- Audited isolated experiment `vr/recenter-forward-alignment-20261108`: `a3b9e7b22a`, `98a7abc29e`, `8bf7de7b6b`. No experiment commit was cherry-picked. Its signed yaw/body-control gating informed this repair; its yaw-only event payload and additive composition did not preserve translated positional neutrals.
- Left-eye mono, separate eye provenance, projections and rendering interfaces remain as in the baseline. No Stage-2 gate was enabled.

## Source findings and changes

1. Camera head yaw now wraps the difference before the existing +/-72 degree clamp. Pitch/roll remain gravity referenced, with zero calibration bases.
2. The authenticated, current-session LOCAL event retains a checked old-to-new transform. OpenXR `poseInPreviousSpace` describes the **new origin in the old space**, so it is inverted for old-coordinate points. The app LOCAL space is created with an identity application offset. This conversion would need revision if that creation offset changes.
3. Transform checks require finite translation, unit quaternion, proper rigid geometry and preserved +Y up (absolute tolerance `1e-3`). Malformed/tilted evidence cannot authorize a rebase.
4. The manager still selects the newest eligible transition at the consumed sample time, but composes **all** eligible changes in `change_time_ns` order. Translation makes order significant. Superseded event transforms reach both the camera and the calibration observer. Existing session, generation, duplicate, activation and diagnostic gates remain.
5. On an accepted landing (existing 8 degree interlock), reconstruct the consumed HMD forward in the old frame, subtract the old camera base, and apply the signed yaw through `LLAgent::yaw`. This includes a nonzero landing residual; it does not assume the post-change HMD sample is exactly zero. Rotation occurs once on event consumption, not continuously.
6. Sitting, parented avatars, grabbed rotation and active teleports skip the body turn with an explicit trace reason. Their head base is re-expressed in the new LOCAL frame without claiming neutralization against a body that did not turn. A landing rejection also re-expresses a valid old base. Unknown transforms hold head driving until a fresh VR activation captures a known frame; there is no silent recovery from an invented heading.
7. An established positional neutral rebases its HMD anchor and heading. The actual accepted body turn is annotated on the matching undrained finalization, so neutral heading accounts for both the LOCAL coordinate change and avatar rotation. Scale source, posture, anatomical offsets, saved calibration record and install generation are preserved. An unestablished neutral is not promoted to established.
8. **Every** finalized batch advances continuity and calibration transition epochs, even on a successful neutral rebase. Old-coordinate positional history is retired, acquiring neutrals suspend, and active calibration transactions abort through their existing epoch comparison. Session loss and skeleton invalidation retain their previous behavior. This intentionally replaces C8's former always-suspend policy and its no-epoch-advance rebase policy.
9. The baseline camera used `mCameraPositionAgent`, which has no declaration in its class. Both baseline and candidate failed compilation there. A local `camera_position_agent` restores compilation without changing the selected position or camera routing.

Reference: [Khronos reference-space-change contract](https://registry.khronos.org/OpenXR/specs/1.1/man/html/XrEventDataReferenceSpaceChangePending.html) and [LOCAL space definition](https://github.com/KhronosGroup/OpenXR-Docs/blob/main/specification/sources/chapters/spaces.adoc).

## Automated evidence

Environment: Linux x86-64, GCC 16.2.1, existing Linux viewer CMake include/define configuration and dependency headers. Checks build this checkout's production sources and math sources, rather than linking an old viewer or mocking transforms.

```sh
python3 indra/newview/tests/recenter/run_recenter_tests.py \
  --build-dir /path/to/existing/build-linux-x86_64 \
  --compile-viewer
```

- **96 production-code checks passed**: both wrap directions; straight, +/-90 and 180 degree cases; nonzero activation/landing yaw; repeated recenter; translated/rotated anchors; preserved neutral metadata; unchanged-coordinate correspondence when no body turn is applied; chronological composition despite reversed delivery order; missing/invalid transforms, duplicate times, mixed generations, malformed quaternions, tilt and NaN rejection; observer dedup/body annotation; continuity epochs; session and skeleton fences.
- `llvrcontinuity.cpp`, `llvrcalibration.cpp`, `llopenxrmanager.cpp`, `llagentcamera.cpp` and the dependent `llvradapterapply.cpp` compiled to production objects with `-Werror`, Linux/X11/OpenSim defines and the existing dependency include set. The final teleport-gating edit was followed by another camera object compilation.
- `git diff --check` passed.
- Clean viewer configuration was attempted separately. After supplying `LL_BUILD_ENV`, it stopped at `ogg_vorbis` prebuilt installation because the configured autobuild executable is unavailable in this environment. **No complete linked viewer, package or launch is claimed.** Existing dependency headers were sufficient for the focused object checks.
- Address/undefined sanitizer mode was attempted, but linking stopped because `/usr/lib64/libasan.so.8.0.0` is missing. **No sanitizer pass is claimed.** The runner retains `--sanitize` for a correctly provisioned environment.

These checks do not execute live OpenXR event delivery, `LLAgent`/simulator synchronization, avatar animation, physical headset movement or compositor display. Test counts are assertions over fixtures, not headset scenarios completed.

## Trace evidence to collect on Quest 3

Existing trace controls and authenticated pinned-WiVRn deployment requirements still apply. Collect:

- `xr_recenter_probe` for runtime event provenance;
- `xr_recenter_applied`: requested agent yaw was applied, with `body_turn_applied`, `body_turn_rad`, old/new bases, generation and sample time. This is **not proof of rendered avatar heading**;
- `recenter_body_alignment_skipped` and `body_alignment_skip_reason` for seated/parented, grabbed or teleport gating;
- `recenter_interlock_rejected` with `transform_valid` for missing transform or landing failure;
- `xr_local_calibration_continuity`: `rebase` or `suspend`, transition count, continuity epoch, annotated body turn and whether the neutral remains established.

The body-turn annotation and observer drain run on the viewer thread: the camera annotates immediately after the take/turn, and the post-blend adapter drains before assembling positional inputs. Verify this ordering and the corresponding rendered body/head outcome in runtime traces; compile/fixture evidence does not establish visual timing.

## Pending headset checklist

- [ ] Facing straight, +/-90, 180 and across +/-pi: one recenter makes free-standing body forward match physical facing, relative head yaw zero, pitch/roll unchanged.
- [ ] Repeat recenter and recenter after nonzero activation heading; no accumulated body turn or wrap-clamp jump.
- [ ] Turn/look while tilted/downward: gravity pitch/roll stay intact. Verify both eye displays remain coherent with the existing left-eye mono path.
- [ ] With an established sync/calibration, include translated LOCAL origins and multiple queued changes: neutral survives valid transforms and positional output has no avoidable jump.
- [ ] Recenter during acquisition/calibration: transaction aborts and no mixed-coordinate neutral is installed.
- [ ] Sitting, moving seat/parent, grabbed rotation, teleport and release: body alignment is gated/reported correctly, and no false success or automatic body follow occurs.
- [ ] Invalid/missing transform, tracking loss, session restart and skeleton rebuild: inspect hold/suspend outcomes and fresh-activation recovery.
- [ ] Tiny/conventional avatars and AO/sit/walk animations: verify avatar root propagation, head targeting and camera ordering; no assumption that a successful agent yaw means immediate simulator or rendered-body alignment.

Milestone 1 remains pending headset acceptance. Seated body alignment, script-owned heading policy, arbitrary runtime support and recovery without restarting the VR activation remain outside this patch.

## Rollback

Close the PR or revert the recovery commit on its feature branch. The reviewed baseline and earlier experiment remain available. No merge, installation, live viewer replacement, settings migration or headset session was performed.
