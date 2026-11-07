/**
 * @file llvrbonelog_test_stubs.h
 * @brief P2-07B bonelog standalone STATE test: minimal
 *        stand-ins for the viewer classes llvrbonelog.cpp
 *        touches, so the recorder's full state machine
 *        (ring, writer thread, header publication) can be
 *        compiled and driven without the viewer.
 *
 * Included by llvrbonelog.cpp ONLY under
 * FSVR_BONELOG_TEST_RIG, and by llvrbonelog_state_test.cpp.
 * Production builds never see this file. Signatures mirror
 * the real classes at every point llvrbonelog.cpp uses
 * them (lljoint.h: getName/getWorldPosition/
 * getWorldRotation/mChildren; llcharacter.h:
 * getSkeletonSerialNum/getRootJoint; llopenxrmanager.h:
 * getHmdForward/getHmdUp; llopenxrtrace.h: nowUs).
 * LLStringUtil is the one exception: it is a typedef in
 * the real tree, so the real class is used (see below).
 */

#ifndef LL_LLVRBONELOG_TEST_STUBS_H
#define LL_LLVRBONELOG_TEST_STUBS_H

#include <chrono>
#include <map>
#include <string>
#include <vector>

// LLStringUtil is NOT stubbed: in the real tree it is a
// typedef (LLStringUtilBase<char>, llstring.h), and
// llstring.h already reaches this TU transitively through
// llvrbonelog.h's llmath includes. Its getenv /
// convertToBOOL implementations live in libllcommon, which
// the state test links -- so the recorder's enabled() runs
// against the real string utility, exactly as in the
// viewer. Including the header explicitly for clarity.
#include "llstring.h"

// ---- Adapter plan structs (real: llvradapterapply.h) ----
// Only the fields llvrbonelog.cpp reads. DIAG-1: the gate
// and domain enums mirror the real ones (same values, same
// gate-name strings) so the recorder's shared code paths
// compile and behave identically against the rig.
enum class FSVRAdapterGate
{
    NONE = 0,
    CYCLE,
    CONTEXT,
    FRAME_IDENTITY,
    VR_SESSION,
    POSITIONAL_MASTER,
    NO_SOLVER,
    TOKEN
};

enum class FSVRAdapterDomain
{
    NONE,
    SPINE,
    ARM_LEFT,
    ARM_RIGHT
};

struct FSVRAdapterApply
{
    static const char* gateName(FSVRAdapterGate gate)
    {
        switch (gate)
        {
        case FSVRAdapterGate::NONE: return "none";
        case FSVRAdapterGate::CYCLE: return "cycle";
        case FSVRAdapterGate::CONTEXT: return "context";
        case FSVRAdapterGate::FRAME_IDENTITY:
            return "frame_identity";
        case FSVRAdapterGate::VR_SESSION: return "vr_session";
        case FSVRAdapterGate::POSITIONAL_MASTER:
            return "positional_master";
        case FSVRAdapterGate::NO_SOLVER: return "no_solver";
        case FSVRAdapterGate::TOKEN: return "token";
        }
        return "unknown";
    }
};

struct FSVRAdapterJointInput
{
    std::string name;
    S32 parent_index = -1;
    LLVector3 rest_offset;
    LLQuaternion base_local;
    FSVRAdapterDomain domain = FSVRAdapterDomain::NONE;
    bool decision_grants_base = false;
    bool disabled = false;
};

struct FSVRAdapterPlanInput
{
    U64 frame_token = 0;
    bool head_target_valid = false;
    LLVector3 head_target_world;
    LLVector3 cornea_offset_head_local;
    LLVector3 root_world_position;
    LLQuaternion root_world_rotation;
    std::vector<FSVRAdapterJointInput> joints;
};

struct FSVRAdapterPlanResult
{
    bool applied = false;
    FSVRAdapterGate first_failing_gate = FSVRAdapterGate::NONE;
    bool spine_solver_invoked = false;
    bool head_group_refusal = false;
    // Fix 4 (CP-F4-3): mirrored because llvrbonelog.cpp
    // reads it in the plan-block fill.
    std::map<S32, LLQuaternion> solver_target_worlds;
    std::vector<S32> writes;
    std::vector<LLQuaternion> resolved_locals;
    std::vector<LLQuaternion> resolved_world_rotations;
    std::vector<LLVector3> resolved_world_positions;
    std::vector<LLQuaternion> base_world_rotations;
    std::vector<LLVector3> base_world_positions;
};

// ---- LLJoint (real: lljoint.h) ----
class LLJoint
{
public:
    std::string mName;
    LLVector3 mWorldPosition;
    LLQuaternion mWorldRotation;
    std::vector<LLJoint*> mChildren;

    const std::string& getName() const { return mName; }
    LLVector3 getWorldPosition() { return mWorldPosition; }
    LLQuaternion getWorldRotation() { return mWorldRotation; }
};

// ---- LLCharacter (real: llcharacter.h) ----
class LLCharacter
{
public:
    LLJoint* mRootJoint = nullptr;
    U32 mSkeletonSerial = 0;

    LLJoint* getRootJoint() { return mRootJoint; }
    U32 getSkeletonSerialNum() const { return mSkeletonSerial; }
};

// ---- LLOpenXRManager (real: llopenxrmanager.h) ----
class LLOpenXRManager
{
public:
    bool mForwardValid = false;
    bool mUpValid = false;
    LLVector3 mForward;
    LLVector3 mUp;

    static LLOpenXRManager& instance()
    {
        static LLOpenXRManager s_instance;
        return s_instance;
    }
    bool getHmdForward(LLVector3& out_forward)
    {
        out_forward = mForward;
        return mForwardValid;
    }
    bool getHmdUp(LLVector3& out_up)
    {
        out_up = mUp;
        return mUpValid;
    }
};

// ---- LLOpenXRTrace (real: llopenxrtrace.h) ----
class LLOpenXRTrace
{
public:
    static long long nowUs()
    {
        return static_cast<long long>(
            std::chrono::duration_cast<
                std::chrono::microseconds>(
                std::chrono::steady_clock::now()
                    .time_since_epoch())
                .count());
    }
};

// ---- LLViewerCamera (real: llviewercamera.h) ----
class LLViewerCamera
{
public:
    LLVector3 mOrigin;
    LLQuaternion mRotation;

    static LLViewerCamera* getInstance()
    {
        static LLViewerCamera s_instance;
        return &s_instance;
    }
    LLVector3 getOrigin() const { return mOrigin; }
    LLQuaternion getQuaternion() const { return mRotation; }
};

#endif // LL_LLVRBONELOG_TEST_STUBS_H
