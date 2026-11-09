# Firestorm missing head-animation control repair — 2026-11-09

PR: https://github.com/IceyMint/phoenix-firestorm/pull/2 (keep draft, no merge).
Branch: `vr/recovery-recenter-local-20261108`.
Built source: `2661c869f40589041aba8c52fde15882d13e86ee`.
Candidate: `Phoenix-FirestormOS-Snowy_AVX2-7-2-3-80164.tar.xz`.
SHA-256: `3279cc40ea0dba88ddc303c60f1b63b7e2198279b80f583a5bbbeffe245293f1`.

## Cause and minimal fix

`LLKeyframeMotion::applyKeyframes()` in `indra/llcharacter/llkeyframemotion.cpp:747` constructs the strict two-argument `LLCachedControl<bool>` for `AOMLocalHeadAnims`. The control was absent from every tracked settings file and from procedural declarations. `LLControlCache` in `indra/llxml/llcontrol.h:360` deliberately calls `LL_ERRS` when that constructor cannot find its control. This accounts for the reported error as soon as keyframe animation first executes in a profile that does not already define the control.

The lookup, default registry, settings-file list and loader are unchanged between the PR base `7dbc74e7cd712b037b8c471f04075b056ba215c6` and old PR head `bd1e17706d028826d8f78661d25ba10c8533133c`. The base registry, old PR registry and registry extracted from the original 80163 archive are byte-identical. This is a defect already present in the PR base, separate from the recenter implementation.

`app_settings/settings_files.xml` assigns the required application `settings.xml` to the Default/Global group. `LLAppViewer::initConfiguration()` loads Default with `set_defaults=true` before user overrides; `loadSettingsFromDirectory()` resolves the file and calls `LLControlGroup::loadFromFile()`. That loader declares unseen controls using their Type, Value, Comment and persistence metadata. `viewer_manifest.py` already includes the application XML files in the package. Neither the loader nor packaging rules need a change.

The only production change is an 11-line declaration in `indra/newview/app_settings/settings.xml`: Boolean, persistent, default true. True allows the existing local head/neck/skull/torso/chest keyframe animations. False retains the existing suppression option. The separate per-character viewpoint override can still suppress head/neck/skull independently. No animation, recenter, calibration, camera or OpenXR code was changed by this repair. No fallback declaration masks the missing registry entry.

## Verification

- New standalone regression links production `LLControlGroup` and `LLCachedControl` from the Linux viewer build. It checks declaration, Boolean type, persistence, enabled default, the exact strict cache constructor used by animation, cache notifications for false/true changes, a false user-file override and preservation of the true default.
- Regression fails against both base and pre-fix PR registries with `FAIL: head animation control declared`; passes against the fixed source and settings extracted from the final compressed archive.
- A separate direct lookup probe loads each registry and immediately constructs the strict cache. Both old registries reach the fatal handler with exactly `Control named "AOMLocalHeadAnims" not found.`; the fixed registry succeeds with `enabled=1`. The probe replaces the fatal callback only to record the message and exit 10 instead of generating a core dump. The production loader/cache code is unchanged.
- All 96 existing production-code recenter checks passed again with AddressSanitizer, UndefinedBehaviorSanitizer and `-Werror`. This is regression evidence, not recenter acceptance.
- Ubuntu chroot incremental ReleaseFS_open AVX2/FMOD viewer build, link and packaging completed with exit 0 and `[100%] Built target llpackage`. Existing build outputs/dependencies were reused; this was not a clean build. Version is 7.2.3.80164, channel Firestorm-Snowy, source 2661c869f4.
- `xz -t` and selected-file archive extraction passed. Extracted binary, registry, settings-file list and build_data.json match packaging inputs byte-for-byte. Registry matches the committed repair and declares the control exactly once; build_data.json identifies 80164. No compiler/linker errors were found. Non-ELF strip notices did not prevent packaging.
- Both original and repaired packaged viewers reached `STATE_LOGIN_WAIT` with isolated HOME directories, Xvfb/software OpenGL, explicit PulseAudio server and AutoLogin=false. Both exited normally (exit 0, `Goodbye!`) after `--quitafter 20`. This tests pre-login startup only: the original also passes because this path does not execute the affected avatar animation lookup. Earlier isolated attempts failed at unavailable display/audio initialization and do not establish a viewer regression. Console and viewer logs for the successful runs are included.
- `git diff --check` passed; the PR worktree was clean after the source commit. Existing working branches and user profiles were preserved.

Reproduce the focused tests inside the configured Ubuntu build environment:

```sh
python3 indra/newview/tests/settings/run_head_animation_settings_test.py \
  --build-dir build-linux-x86_64
# Optionally add --settings /path/to/extracted/app_settings/settings.xml
python3 indra/newview/tests/recenter/run_recenter_tests.py \
  --build-dir build-linux-x86_64 --sanitize
```

Build commands: `autobuild configure -A 64 -c ReleaseFS_open -- --avx2 --fmodstudio --chan Snowy -DLL_TESTS:BOOL=FALSE -DNDOF_LIBRARY:FILEPATH=/path/to/build-linux-x86_64/packages/lib/release/libndofdev.a`, then `autobuild build -A 64 -c ReleaseFS_open -- --avx2 --fmodstudio --chan Snowy --jobs 8 -DLL_TESTS:BOOL=FALSE`.

The existing local FMOD archive override is retained only in the detached build checkout (2.03.07, MD5 `8c40304390fe137d89c0f710f4c3e6ff`); it is not committed. The existing explicit NDOF dependency is reused. Candidate-local TMPDIR avoids the full shared /tmp. A local hg shim supplies the distinct build number 80164 because the version script expects a .git directory rather than a worktree .git file. AUTOBUILD_VARIABLES_FILE points to the existing build-variables/variables. The evidence-only commit after the built source does not change runtime files.

## Outstanding validation and rollback

No grid login, in-world avatar session, headset session, rendered recenter/body alignment, positional calibration continuity or comfort validation was performed. The precise lookup failure is reproduced in the production-code probe; the full reported in-world crash sequence is not reproduced end to end. Headset validation and the existing milestone acceptance checklist remain outstanding. Nothing was merged or installed as the user's viewer, and normal user settings were not modified. Only disposable test profiles were written.

Rollback this repair by reverting source commit 2661c869f4 on the PR branch. The original 80163 archive is preserved separately; the new archive has a distinct build number.
