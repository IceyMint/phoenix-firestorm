/**
 * @file llheadrotmotion.h
 * @brief Implementation of LLHeadRotMotion class.
 *
 * $LicenseInfo:firstyear=2001&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2010, Linden Research, Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation;
 * version 2.1 of the License only.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Linden Research, Inc., 945 Battery Street, San Francisco, CA  94111  USA
 * $/LicenseInfo$
 */

#ifndef LL_LLHEADROTMOTION_H
#define LL_LLHEADROTMOTION_H

//-----------------------------------------------------------------------------
// Header files
//-----------------------------------------------------------------------------
#include "llmotion.h"
#include "llframetimer.h"

#define MIN_REQUIRED_PIXEL_AREA_HEAD_ROT 500.f;
#define MIN_REQUIRED_PIXEL_AREA_EYE 25000.f;

//-----------------------------------------------------------------------------
// P2-07B DIAG-1 (brief section 2.3, decision D-2 = R-a):
// the headrot capture. LLHeadRotMotion publishes this
// fixed-size, motion-owned struct through the character's
// animation-data slot ("FSVRHeadRotCapture") once per
// onUpdate, at the joint-state write point. Publication is
// unconditional-but-trivial: field stores and a counter
// increment; it allocates nothing after the one-time slot
// store and writes no value the motion consumes.
//
// The state identities are the motion's own mNeckState /
// mHeadState object pointers: IN MEMORY ONLY, never
// serialized. They exist for the glue's same-cycle
// identity comparison against the blend record (brief
// section 2.5); the only identity-derived output anywhere
// is the semantic source:"headrot" marker.
//-----------------------------------------------------------------------------
struct FSVRHeadRotCapture
{
    LLQuaternion neck_state;
    LLQuaternion head_state;
    LLQuaternion last_head_rot_before;
    LLQuaternion last_head_rot_after;
    bool instant = false;
    bool valid = false;
    U64 generation = 0;
    const LLJointState* neck_state_id = nullptr;
    const LLJointState* head_state_id = nullptr;
    // CODE-R1-01: the motion's effective blend mode for
    // the publication cycle -- getPriority() and
    // (getBlendType() == ADDITIVE_BLEND) evaluated at the
    // publication point. Observational metadata for the
    // blend consistency validation (brief section 2.5);
    // never consumed by the motion, never serialized.
    S32 priority = 0;
    bool additive = false;

    // One publication: stores the cycle's ask and the two
    // state identities, bumps the generation, marks valid.
    void publish(const LLQuaternion& neck,
                 const LLQuaternion& head,
                 const LLQuaternion& last_before,
                 const LLQuaternion& last_after,
                 bool instant_path,
                 const LLJointState* neck_id,
                 const LLJointState* head_id,
                 S32 effective_priority,
                 bool effective_additive)
    {
        neck_state = neck;
        head_state = head;
        last_head_rot_before = last_before;
        last_head_rot_after = last_after;
        instant = instant_path;
        neck_state_id = neck_id;
        head_state_id = head_id;
        priority = effective_priority;
        additive = effective_additive;
        ++generation;
        valid = true;
    }

    // Deactivation / destruction: no stale sample and no
    // stale identity survives (brief section 2.3
    // protections 3 and 5).
    void invalidate()
    {
        valid = false;
        neck_state_id = nullptr;
        head_state_id = nullptr;
    }
};

// Consumer-side acceptance (brief section 2.3 protections
// 1, 2 and 5): the consumer remembers the last generation
// it consumed per publishing instance; an instance change
// resets the accepted generation, so a new motion instance
// can never inherit a previous instance's accepted
// generation by numeric coincidence. A capture is accepted
// only when valid and exactly newer than the last consumed
// generation -- an unchanged generation yields no block,
// never a re-emitted stale copy.
struct FSVRHeadRotConsumer
{
    const FSVRHeadRotCapture* mInstance = nullptr;
    U64 mLastGeneration = 0;

    const FSVRHeadRotCapture* accept(const FSVRHeadRotCapture* capture)
    {
        if (!capture)
        {
            return nullptr;
        }
        if (capture != mInstance)
        {
            mInstance = capture;
            mLastGeneration = 0;
        }
        if (!capture->valid || capture->generation == 0 ||
            capture->generation <= mLastGeneration)
        {
            return nullptr;
        }
        mLastGeneration = capture->generation;
        return capture;
    }
};

//-----------------------------------------------------------------------------
// class LLHeadRotMotion
//-----------------------------------------------------------------------------
class LLHeadRotMotion :
    public LLMotion
{
public:
    // Constructor
    LLHeadRotMotion(const LLUUID &id);

    // Destructor
    virtual ~LLHeadRotMotion();

public:
    //-------------------------------------------------------------------------
    // functions to support MotionController and MotionRegistry
    //-------------------------------------------------------------------------

    // static constructor
    // all subclasses must implement such a function and register it
    static LLMotion *create(const LLUUID &id) { return new LLHeadRotMotion(id); }

public:
    //-------------------------------------------------------------------------
    // animation callbacks to be implemented by subclasses
    //-------------------------------------------------------------------------

    // motions must specify whether or not they loop
    virtual bool getLoop() { return true; }

    // motions must report their total duration
    virtual F32 getDuration() { return 0.0; }

    // motions must report their "ease in" duration
    virtual F32 getEaseInDuration() { return 1.f; }

    // motions must report their "ease out" duration.
    virtual F32 getEaseOutDuration() { return 1.f; }

    // called to determine when a motion should be activated/deactivated based on avatar pixel coverage
    virtual F32 getMinPixelArea() { return MIN_REQUIRED_PIXEL_AREA_HEAD_ROT; }

    // motions must report their priority
    virtual LLJoint::JointPriority getPriority();

    virtual LLMotionBlendType getBlendType();

    // run-time (post constructor) initialization,
    // called after parameters have been set
    // must return true to indicate success and be available for activation
    virtual LLMotionInitStatus onInitialize(LLCharacter *character);

    // called when a motion is activated
    // must return true to indicate success, or else
    // it will be deactivated
    virtual bool onActivate();

    // called per time step
    // must return true while it is active, and
    // must return false when the motion is completed.
    virtual bool onUpdate(F32 time, U8* joint_mask);

    // called when a motion is deactivated
    virtual void onDeactivate();

public:
    //-------------------------------------------------------------------------
    // joint states to be animated
    //-------------------------------------------------------------------------
    LLCharacter         *mCharacter;

    LLJoint             *mTorsoJoint;
    LLJoint             *mHeadJoint;
    LLJoint             *mRootJoint;
    LLJoint             *mPelvisJoint;

    LLPointer<LLJointState> mTorsoState;
    LLPointer<LLJointState> mNeckState;
    LLPointer<LLJointState> mHeadState;

    LLQuaternion        mLastHeadRot;

    // P2-07B DIAG-1 (section 2.3): the published capture
    // and its one-time slot registration.
    FSVRHeadRotCapture  mDiagCapture;
    bool                mDiagCaptureRegistered = false;
};

//-----------------------------------------------------------------------------
// class LLEyeMotion
//-----------------------------------------------------------------------------
class LLEyeMotion :
    public LLMotion
{
public:
    // Constructor
    LLEyeMotion(const LLUUID &id);

    // Destructor
    virtual ~LLEyeMotion();

public:
    //-------------------------------------------------------------------------
    // functions to support MotionController and MotionRegistry
    //-------------------------------------------------------------------------

    // static constructor
    // all subclasses must implement such a function and register it
    static LLMotion *create( const LLUUID &id) { return new LLEyeMotion(id); }

public:
    //-------------------------------------------------------------------------
    // animation callbacks to be implemented by subclasses
    //-------------------------------------------------------------------------

    // motions must specify whether or not they loop
    virtual bool getLoop() { return true; }

    // motions must report their total duration
    virtual F32 getDuration() { return 0.0; }

    // motions must report their "ease in" duration
    virtual F32 getEaseInDuration() { return 0.5f; }

    // motions must report their "ease out" duration.
    virtual F32 getEaseOutDuration() { return 0.5f; }

    // called to determine when a motion should be activated/deactivated based on avatar pixel coverage
    virtual F32 getMinPixelArea() { return MIN_REQUIRED_PIXEL_AREA_EYE; }

    // motions must report their priority
    virtual LLJoint::JointPriority getPriority() { return LLJoint::MEDIUM_PRIORITY; }

    virtual LLMotionBlendType getBlendType() { return NORMAL_BLEND; }

    // run-time (post constructor) initialization,
    // called after parameters have been set
    // must return true to indicate success and be available for activation
    virtual LLMotionInitStatus onInitialize(LLCharacter *character);

    // called when a motion is activated
    // must return true to indicate success, or else
    // it will be deactivated
    virtual bool onActivate();

    void adjustEyeTarget(LLVector3* targetPos, LLJointState& left_eye_state, LLJointState& right_eye_state);

    // called per time step
    // must return true while it is active, and
    // must return false when the motion is completed.
    virtual bool onUpdate(F32 time, U8* joint_mask);

    // called when a motion is deactivated
    virtual void onDeactivate();

public:
    //-------------------------------------------------------------------------
    // joint states to be animated
    //-------------------------------------------------------------------------
    LLCharacter         *mCharacter;

    LLJoint             *mHeadJoint;
    LLPointer<LLJointState> mLeftEyeState;
    LLPointer<LLJointState> mRightEyeState;
    LLPointer<LLJointState> mAltLeftEyeState;
    LLPointer<LLJointState> mAltRightEyeState;

    LLFrameTimer        mEyeJitterTimer;
    F32                 mEyeJitterTime;
    F32                 mEyeJitterYaw;
    F32                 mEyeJitterPitch;
    F32                 mEyeLookAwayTime;
    F32                 mEyeLookAwayYaw;
    F32                 mEyeLookAwayPitch;

    // eye blinking
    LLFrameTimer        mEyeBlinkTimer;
    F32                 mEyeBlinkTime;
    bool                mEyesClosed;
};

#endif // LL_LLHEADROTMOTION_H
