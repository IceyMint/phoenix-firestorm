// Standalone production-code regression checks; see run_recenter_tests.py.
#include "llvrcontinuity.h"
#include "llvrcalibration.h"
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
int main()
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
    std::cout << checks << " production-code recenter checks passed\n";
}
