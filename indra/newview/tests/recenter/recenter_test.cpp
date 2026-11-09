// Standalone production-code regression checks; see run_recenter_tests.py.
#include "llvrcontinuity.h"
#include "llvrcalibration.h"
#include "fsvropenxrruntime.h"
#include <dlfcn.h>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

static int checks = 0;
static void verify(bool ok, int line) { ++checks; if (!ok) { std::cerr << "Failed check at line " << line << '\n'; std::exit(1); } }
#define check(expression) verify((expression), __LINE__)
static bool near(F32 a, F32 b) { return fabsf(a-b) < 0.0001f; }
static bool near(const LLVector3& a, const LLVector3& b) { return (a-b).length() < 0.0001f; }
static FSVRFinalizedTransitionKey transition(long long time, F32 yaw, LLVector3 origin)
{
    FSVRFinalizedTransitionKey key;
    key.incarnation = 4; key.generation = 2; key.change_time_ns = time;
    // LLQuaternion is right-handed about +Y: physical right yaw is negative.
    key.has_origin_transform = fsvrLocalOriginTransform(
        origin, LLQuaternion(-yaw, LLVector3(0.f, 1.f, 0.f)), key.origin_transform);
    check(key.has_origin_transform);
    return key;
}
int main(int argc, char** argv)
{
    check(near(fsvrHeadYawDelta(-179.f*DEG_TO_RAD, 179.f*DEG_TO_RAD), 2.f*DEG_TO_RAD));
    check(near(fsvrHeadYawDelta(179.f*DEG_TO_RAD, -179.f*DEG_TO_RAD), -2.f*DEG_TO_RAD));
    for (F32 yaw : {0.f, F_PI_BY_TWO, -F_PI_BY_TWO, F_PI})
    {
        auto key = transition(10, yaw, LLVector3(2.f, 1.f, -3.f));
        auto evidence = fsvrComposeLocalTransitions({key});
        auto result = decideRebase(evidence, LLVector3(2.f, 1.f, -3.f));
        check(result.decision == FSVRRebaseDecision::Rebase);
        check(near(result.rebased_anchor, LLVector3::zero));
        check(near(fsvrHeadYawDelta(fsvrRebasedYaw(yaw, key.origin_transform), 0.f), 0.f));
        LLMatrix4 inverse = key.origin_transform; inverse.invert();
        const F32 body_turn = -fsvrHeadYawDelta(fsvrRebasedYaw(0.f, inverse), 0.f);
        check(near(fabsf(body_turn), fabsf(yaw)));
        if (fabsf(yaw) < F_PI) check(near(body_turn, -yaw));
        // Established neutral survives, including scale, posture and installation.
        FSVRSessionNeutral neutral;
        neutral.installFromCapture({}, LLVector3(2.f, 1.f, -3.f), 0.f,
            LLVector3(.1f, .2f, .3f), LLVector3(.4f, .5f, .6f),
            FSVRScaleSource::PROVISIONAL_SYNC, FSVRPosture::STANDING);
        const auto before = neutral.view();
        neutral.rebaseForLocalTransition(key.origin_transform, body_turn);
        check(neutral.state() == FSVRSessionState::ESTABLISHED);
        check(near(neutral.view().anchor_hmd_local, LLVector3::zero));
        check(near(neutral.view().anchor_forward_yaw_rad, 0.f));
        check(neutral.view().scale_source == before.scale_source);
        check(neutral.view().posture == before.posture);
        check(neutral.view().install_generation == before.install_generation);
        check(near(neutral.view().neutral_cornea_root, before.neutral_cornea_root));
        // Same heading, repeat recenter: no incremental body drift.
        auto repeated = transition(20, 0.f, LLVector3::zero);
        neutral.rebaseForLocalTransition(repeated.origin_transform, 0.f);
        check(near(neutral.view().anchor_forward_yaw_rad, 0.f));
        // Seated/grabbed body (zero turn): only coordinates change.
        check(near(fsvrHeadYawDelta(fsvrRebasedYaw(yaw, key.origin_transform),
            fsvrRebasedYaw(0.f, key.origin_transform)), fsvrHeadYawDelta(yaw, 0.f)));
    }
    // Activation/calibration at nonzero yaw, and a nonzero landing sample.
    auto offset = transition(10, 70.f*DEG_TO_RAD, LLVector3::zero);
    LLMatrix4 inverse = offset.origin_transform; inverse.invert();
    const F32 old_facing = fsvrRebasedYaw(5.f*DEG_TO_RAD, inverse);
    check(near(old_facing, 75.f*DEG_TO_RAD));
    check(near(-fsvrHeadYawDelta(old_facing, 30.f*DEG_TO_RAD), -45.f*DEG_TO_RAD));
    FSVRSessionNeutral uncalibrated;
    uncalibrated.rebaseForLocalTransition(offset.origin_transform, 0.f);
    check(!uncalibrated.view().established);
    check(uncalibrated.state() == FSVRSessionState::UNCALIBRATED);
    auto first = transition(10, F_PI_BY_TWO, LLVector3(2.f, 0.f, 0.f));
    auto second = transition(20, -F_PI_BY_TWO, LLVector3(0.f, 0.f, 3.f));
    const LLVector3 anchor(4.f, 2.f, -5.f);
    const auto chain = fsvrComposeLocalTransitions({second, first});
    check(near(anchor * chain.origin_transform,
        (anchor * first.origin_transform) * second.origin_transform));
    auto bad = first; bad.has_origin_transform = false;
    check(!fsvrComposeLocalTransitions({bad, second}).has_origin_transform);
    bad = first; bad.generation++;
    check(!fsvrComposeLocalTransitions({bad, second}).has_origin_transform);
    check(!fsvrComposeLocalTransitions({first, first}).has_origin_transform);
    LLMatrix4 matrix;
    check(!fsvrLocalOriginTransform(LLVector3::zero, LLQuaternion(0.f,0.f,0.f,0.f), matrix));
    check(!fsvrLocalOriginTransform(LLVector3::zero, LLQuaternion(.1f, LLVector3(1.f,0.f,0.f)), matrix));
    check(!fsvrLocalOriginTransform(LLVector3(std::numeric_limits<F32>::quiet_NaN(),0.f,0.f), LLQuaternion(), matrix));
    auto wrong = chain; wrong.transform_generation++;
    check(decideRebase(wrong, anchor).decision == FSVRRebaseDecision::Suspend);
    check(decideRebase(chain, LLVector3(std::numeric_limits<F32>::quiet_NaN(),0.f,0.f)).decision == FSVRRebaseDecision::Suspend);
    FSVRContinuityRouter lifecycle;
    check(!lifecycle.noteSessionIncarnation(4));
    check(lifecycle.noteSessionIncarnation(5));
    check(lifecycle.continuityEpoch() == 1);
    check(!lifecycle.noteSessionIncarnation(5));
    check(!lifecycle.noteSampledSkeletonGeneration(10));
    check(lifecycle.noteSampledSkeletonGeneration(11));
    check(lifecycle.continuityEpoch() == 2);
    FSVRFinalizationObserver observer;
    observer.noteFinalizedBatch({second, first});
    observer.noteFinalizedBatch({first});
    check(observer.pendingBatchCount() == 1);
    observer.noteBodyTurn(4,20,-.5f);
    auto batches = observer.drainBatches();
    check(near(batches[0][0].body_turn_rad, -.5f));
    FSVRContinuityRouter router;
    auto routed = router.routeFinalizedBatch(batches[0], anchor);
    check(routed.decision == FSVRRebaseDecision::Rebase);
    check(near(routed.body_turn_rad,-.5f));
    check(router.continuityEpoch() == 1);
    router.routeFinalizedBatch({});
    check(router.continuityEpoch() == 1);
    bad.has_origin_transform = false;
    check(router.routeFinalizedBatch({bad}).decision == FSVRRebaseDecision::Suspend);
    check(router.continuityEpoch() == 2);
    // Runtime selection without XR_RUNTIME_JSON: system/user XDG install.
    check(argc == 3);
    unsetenv("XR_RUNTIME_JSON");
    setenv("XDG_CONFIG_HOME", argv[1], 1);
    setenv("XDG_CONFIG_DIRS", "/nonexistent", 1);
    const auto candidates = fsvrRuntimeManifestCandidates();
    check(candidates.front() == std::string(argv[1]) + "/openxr/1/active_runtime.x86_64.json");
    std::string detail;
    check(!fsvrVerifyRuntimeManifest(detail)); // no selected file
    const std::string manifest = std::string(argv[1]) + "/openxr/1/active_runtime.json";
    {
        std::ofstream out(manifest);
        out << "{\"runtime\":{\"name\":\"WiVRn\",\"library_path\":\"" << argv[2] << "\"}}";
    }
    check(!fsvrVerifyRuntimeManifest(detail)); // library exists but not loaded
    void* library = dlopen(argv[2], RTLD_NOW | RTLD_LOCAL);
    check(library != nullptr);
    check(fsvrVerifyRuntimeManifest(detail)); // actual missing-pin regression
    check(fsvrSupportedRuntimeName("WiVRn 'v26.9-97-g11e36e07'"));
    check(!fsvrSupportedRuntimeName("WiVRnFake"));
    check(!fsvrSupportedRuntimeName("Monado"));
    check(!fsvrSupportedRuntimeName(""));
    setenv("XR_RUNTIME_JSON", "/missing/explicit.json", 1);
    check(fsvrRuntimeManifestCandidates().size() == 1);
    check(!fsvrVerifyRuntimeManifest(detail)); // explicit failure never falls back
    unsetenv("XR_RUNTIME_JSON");
    { std::ofstream out(manifest); out << "{broken json"; }
    check(!fsvrVerifyRuntimeManifest(detail));
    dlclose(library);

    // poseValid=false: old/new LOCAL orientations in an unchanged STAGE.
    // Exercise +/-90, wrap, residual landing, nonzero initial stage orientation.
    for (F32 physical : {0.f, F_PI_BY_TWO, -F_PI_BY_TWO, 179.f*DEG_TO_RAD, -179.f*DEG_TO_RAD})
    {
        F32 bridge = 0.f;
        check(fsvrStageYawBridge(LLQuaternion(-.3f, LLVector3::y_axis),
            LLQuaternion(-(.3f+physical), LLVector3::y_axis), bridge));
        check(near(fsvrHeadYawDelta(bridge, -physical), 0.f));
        auto missing = first;
        missing.has_origin_transform = false;
        missing.has_yaw_bridge = true;
        missing.old_to_new_yaw = bridge;
        F32 composed_yaw = 0.f;
        check(fsvrTransitionYaw({missing}, composed_yaw));
        check(near(fsvrHeadYawDelta(composed_yaw, bridge), 0.f));
        // Cannot turn yaw-only evidence into positional continuity.
        check(decideRebase(fsvrComposeLocalTransitions({missing}), anchor).decision == FSVRRebaseDecision::Suspend);
        const F32 landing = 3.f*DEG_TO_RAD;
        const F32 body = fsvrRecenterBodyTurn(landing, .2f, bridge);
        check(near(fsvrHeadYawDelta(body, .2f - physical - landing), 0.f));
        const F32 initial_agent = .7f;
        const auto aligned_root = fsvrRootYawAligned(LLQuaternion(-.6f, LLVector3::z_axis), initial_agent + body);
        const LLVector3 rendered_forward = LLVector3::x_axis * aligned_root;
        // Independent XR -> SL vector mapping: -Z forward -> +X,
        // +X right -> -Y. Include activation baseline and initial agent.
        const LLVector3 xr_forward = LLVector3(0.f,0.f,-1.f) *
            LLQuaternion(-(physical + landing - .2f), LLVector3::y_axis);
        const LLVector3 expected = LLVector3(-xr_forward.mV[VZ], -xr_forward.mV[VX], xr_forward.mV[VY]) *
            LLQuaternion(initial_agent, LLVector3::z_axis);
        check(near(rendered_forward, expected));
        // SL +X forward, +Y left, +Z up: physical right is negative LL yaw.
        LLVector3 body_forward = LLVector3::x_axis * LLQuaternion(body, LLVector3::z_axis);
        check(near(atan2f(body_forward.mV[VY], body_forward.mV[VX]), body));
    }
    check(fsvrStageAnchorEligible(100, 3, 200, 3, 0));
    check(!fsvrStageAnchorEligible(100, 2, 200, 3, 0));
    check(!fsvrStageAnchorEligible(200, 3, 200, 3, 0));
    check(!fsvrStageAnchorEligible(201, 3, 200, 3, 0));
    check(!fsvrStageAnchorEligible(100, 3, 250000101, 3, 0));
    check(!fsvrStageAnchorEligible(100, 3, 200, 3, 150));
    // Rendered root and commanded agent may disagree because of idle hysteresis.
    const LLQuaternion old_root(.4f, LLVector3::z_axis);
    const LLVector3 old_agent = LLVector3::x_axis * LLQuaternion(.9f, LLVector3::z_axis);
    const auto plan = fsvrPlanRecenterHeading(0.f, 0.f, -F_PI_BY_TWO, old_root, old_agent);
    check(plan.valid);
    check(near(plan.target_heading, .4f - F_PI_BY_TWO));
    check(near(plan.agent_turn, .4f - F_PI_BY_TWO - .9f));
    const LLVector3 result_agent = old_agent * LLQuaternion(plan.agent_turn, LLVector3::z_axis);
    const LLVector3 result_root = LLVector3::x_axis * fsvrRootYawAligned(old_root, plan.target_heading);
    check(near(result_agent, result_root));
    FSVRSessionNeutral lagged;
    lagged.installFromCapture({}, LLVector3::zero, 0.f, LLVector3::zero, LLVector3::zero,
        FSVRScaleSource::PROVISIONAL_SYNC, FSVRPosture::STANDING);
    // Calibration follows the actual root change, not the larger agent command
    // needed to close an existing agent/root discrepancy.
    lagged.rebaseForLocalTransition(first.origin_transform, plan.physical_turn);
    check(near(lagged.view().anchor_forward_yaw_rad, 0.f));
    check(!fsvrPlanRecenterHeading(0.f, 0.f, 0.f, old_root, LLVector3::z_axis).valid);
    check(!fsvrPlanRecenterHeading(0.f, 0.f, 0.f, LLQuaternion(0.f,0.f,0.f,0.f), old_agent).valid);
    F32 rejected_yaw;
    check(!fsvrStageYawBridge(LLQuaternion(0.f,0.f,0.f,0.f), LLQuaternion(), rejected_yaw));
    check(!fsvrStageYawBridge(LLQuaternion(), LLQuaternion(.1f, LLVector3::x_axis), rejected_yaw));
    auto yaw_only = first;
    yaw_only.has_origin_transform = false;
    yaw_only.has_yaw_bridge = true;
    yaw_only.old_to_new_yaw = .5f;
    check(fsvrTransitionYaw({second, yaw_only}, rejected_yaw));
    check(!fsvrTransitionYaw({yaw_only,yaw_only}, rejected_yaw));
    yaw_only.has_yaw_bridge = false;
    check(!fsvrTransitionYaw({yaw_only}, rejected_yaw));
    std::cout << checks << " production-code recenter checks passed\n";
}
