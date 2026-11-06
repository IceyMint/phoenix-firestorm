/**
 * @file llpose.h
 * @brief Implementation of LLPose class.
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

#ifndef LL_LLPOSE_H
#define LL_LLPOSE_H

//-----------------------------------------------------------------------------
// Header Files
//-----------------------------------------------------------------------------

#include "lljointstate.h"
#include "lljoint.h"
#include "llpointer.h"

#include <map>
#include <string>


//-----------------------------------------------------------------------------
// class LLPose
//-----------------------------------------------------------------------------
class LLPose
{
    friend class LLPoseBlender;
protected:
    typedef std::map<std::string, LLPointer<LLJointState> > joint_map;
    typedef joint_map::iterator joint_map_iterator;
    typedef joint_map::value_type joint_map_value_type;

    joint_map                   mJointMap;
    F32                         mWeight;
    joint_map_iterator          mListIter;
public:
    // Iterate through jointStates
    LLJointState* getFirstJointState();
    LLJointState* getNextJointState();
    LLJointState* findJointState(LLJoint *joint);
    LLJointState* findJointState(const std::string &name);
public:
    // Constructor
    LLPose() : mWeight(0.f) {}
    // Destructor
    ~LLPose();
    // add a joint state in this pose
    bool addJointState(const LLPointer<LLJointState>& jointState);
    // remove a joint state from this pose
    bool removeJointState(const LLPointer<LLJointState>& jointState);
    // removes all joint states from this pose
    bool removeAllJointStates();
    // set weight for all joint states in this pose
    void setWeight(F32 weight);
    // get weight for this pose
    F32 getWeight() const;
    // returns number of joint states stored in this pose
    S32 getNumJointStates() const;
};

const S32 JSB_NUM_JOINT_STATES = 6;

//-----------------------------------------------------------------------------
// P2-07B DIAG-1 (brief section 2.5): blend observation
// records. Observation-only: the sink copies values the
// blend already computed; it feeds nothing back. The
// per-slot state pointer is TRANSIENT -- it exists only in
// memory, for the glue's same-cycle identity comparison
// against the headrot publication (brief section 2.5,
// FIX1-DIAG1-R2-01), and is never serialized; the only
// identity-derived output anywhere is the semantic
// source_headrot flag stamped at assembly.
//-----------------------------------------------------------------------------
struct FSVRBlendObsSlot
{
    const LLJointState* state = nullptr; // transient, never serialized
    bool            additive = false;
    S32             priority = 0;
    F32             weight = 0.f;
    LLQuaternion    rotation;
    // Non-additive steps: the ROT weight sum after the
    // step. Additive steps never advance that sum; they
    // carry delta (= llmin(1, weight + sum) - sum) instead.
    bool            has_rot_sum = false;
    F32             rot_sum_after = 0.f;
    F32             delta = 0.f;
    LLQuaternion    blended_after;
    LLQuaternion    added_after;
    bool            source_headrot = false; // stamped at assembly, in-memory only
};

struct FSVRBlendObsJoint
{
    bool            written = false;
    U64             generation = 0;
    LLQuaternion    seed_local;
    FSVRBlendObsSlot slots[JSB_NUM_JOINT_STATES];
    S32             slot_count = 0;
    F32             rot_weight_sum = 0.f;
    LLQuaternion    blended_rot;
    LLQuaternion    added_rot;
    LLQuaternion    result_local;
    // CODE-R1-01: assembly-time consistency outcome. Set
    // only by the validation on the glue's stamped copy
    // (fsvrValidateBlendObsSource below), never by the
    // blend itself: true when an identity-attributed
    // slot's recorded values disagree with the published
    // headrot ask / effective mode for the same cycle.
    bool            source_integrity_failure = false;
};

struct FSVRBlendObsRecord
{
    U64             generation = 0;
    FSVRBlendObsJoint neck;
    FSVRBlendObsJoint head;
};

// Assembly-time attribution (brief section 2.5): a slot is
// stamped iff its consumed state pointer IS the published
// headrot state object for that joint. No identity, no
// attribution; value equality never confers it.
inline void fsvrStampBlendObsSource(FSVRBlendObsJoint& entry,
                                    const LLJointState* headrot_state_id)
{
    for (S32 i = 0; i < entry.slot_count; ++i)
    {
        entry.slots[i].source_headrot =
            headrot_state_id != nullptr &&
            entry.slots[i].state == headrot_state_id;
    }
}

// Quaternion equality for the consistency validation
// (brief section 2.5: an attributed slot's rotation
// "must equal" the published ask). The semantics are
// those of the D1-G4 harness's equality: both operands
// normalized, sign-insensitive (unit quaternions
// double-cover rotations, so the absolute dot), float
// tolerance 1e-5 on the dot deviation.
inline bool fsvrQuatConsistent(const LLQuaternion& a,
                               const LLQuaternion& b)
{
    LLQuaternion x = a;
    x.normalize();
    LLQuaternion y = b;
    y.normalize();
    F32 d = x.mQ[0] * y.mQ[0] + x.mQ[1] * y.mQ[1] +
            x.mQ[2] * y.mQ[2] + x.mQ[3] * y.mQ[3];
    if (d < 0.f)
    {
        d = -d;
    }
    return d > 1.f - 1e-5f;
}

// Independent consistency validation (CODE-R1-01; brief
// section 2.5's match-by-value rule, demoted by R3 to an
// assertion independent of attribution): for every
// identity-attributed slot, the recorded rotation must
// equal the cycle's published ask (fsvrQuatConsistent)
// and the recorded additive flag and priority must equal
// the motion's effective mode captured at publication.
// A mismatch sets the entry's record-integrity failure
// flag: the attribution stands (it is never "repaired"
// by re-matching another slot by value) and nothing is
// written back to any joint state, blend, or adapter
// result. Unattributed slots are not checked. Returns
// true iff at least one slot was attributed.
inline bool fsvrValidateBlendObsSource(
    FSVRBlendObsJoint& entry,
    const LLQuaternion& published_rotation,
    S32 published_priority,
    bool published_additive)
{
    bool any_attributed = false;
    for (S32 i = 0; i < entry.slot_count; ++i)
    {
        const FSVRBlendObsSlot& slot = entry.slots[i];
        if (!slot.source_headrot)
        {
            continue;
        }
        any_attributed = true;
        if (!fsvrQuatConsistent(slot.rotation,
                                published_rotation) ||
            slot.additive != published_additive ||
            slot.priority != published_priority)
        {
            entry.source_integrity_failure = true;
        }
    }
    return any_attributed;
}

// Consumer-side freshness for the blend record (brief
// section 2.5 transport/gating): an instance change resets
// the accepted generation, so a new record instance can
// never inherit a previous instance's accepted generation;
// an entry is present only when it was written this pass
// with the record's own generation.
struct FSVRBlendObsConsumer
{
    const FSVRBlendObsRecord* mInstance = nullptr;
    U64 mLastGeneration = 0;

    bool consumePass(const FSVRBlendObsRecord* record)
    {
        if (!record)
        {
            return false;
        }
        if (record != mInstance)
        {
            mInstance = record;
            mLastGeneration = 0;
        }
        if (record->generation == 0 ||
            record->generation <= mLastGeneration)
        {
            return false;
        }
        mLastGeneration = record->generation;
        return true;
    }

    // which: 0 = neck, 1 = head.
    const FSVRBlendObsJoint* entryFor(const FSVRBlendObsRecord& record,
                                      S32 which) const
    {
        const FSVRBlendObsJoint& entry =
            (which == 0) ? record.neck : record.head;
        if (!entry.written || entry.generation != record.generation)
        {
            return nullptr;
        }
        return &entry;
    }
};

LL_ALIGN_PREFIX(16)
class LLJointStateBlender
{
    LL_ALIGN_NEW
protected:
    LLPointer<LLJointState> mJointStates[JSB_NUM_JOINT_STATES];
    S32             mPriorities[JSB_NUM_JOINT_STATES];
    bool            mAdditiveBlends[JSB_NUM_JOINT_STATES];
    // P2-07B DIAG-1: observation sink (null by default; the
    // null check is the only disabled-mode footprint).
    FSVRBlendObsJoint* mObsSink = nullptr;
    U64             mObsGeneration = 0;
public:
    LLJointStateBlender();
    ~LLJointStateBlender();
    void blendJointStates(bool apply_now = true);
    bool addJointState(const LLPointer<LLJointState>& joint_state, S32 priority, bool additive_blend);
    void interpolate(F32 u);
    void clear();
    void resetCachedJoint();
    // P2-07B DIAG-1 (brief section 3 item 5).
    void setObsSink(FSVRBlendObsJoint* sink, U64 generation)
    {
        mObsSink = sink;
        mObsGeneration = generation;
    }
    const LLJoint* targetJoint() const
    {
        return mJointStates[0] ? mJointStates[0]->getJoint() : nullptr;
    }

public:
    LL_ALIGN_16(LLJoint mJointCache);
} LL_ALIGN_POSTFIX(16);

class LLMotion;

class LLPoseBlender
{
protected:
    typedef std::list<LLJointStateBlender*> blender_list_t;
    typedef std::map<LLJoint*,LLJointStateBlender*> blender_map_t;
    blender_map_t mJointStateBlenderPool;
    blender_list_t mActiveBlenders;

    S32         mNextPoseSlot;
    LLPose      mBlendedPose;
    // P2-07B DIAG-1: blend observation record + pass
    // generation (increments once per blendAndApply).
    FSVRBlendObsRecord mBlendObs;
    U64         mBlendObsGeneration = 0;
public:
    // Constructor
    LLPoseBlender();
    // Destructor
    ~LLPoseBlender();

    // request motion joint states to be added to pose blender joint state records
    bool addMotion(LLMotion* motion);

    // blend all joint states and apply to skeleton
    void blendAndApply();

    // P2-07B DIAG-1 (brief section 3 item 6): the blend
    // observation record, owned here with the blender pool
    // and published by the motion controller through the
    // character's animation-data slot.
    FSVRBlendObsRecord& blendObsRecord() { return mBlendObs; }

    // removes all joint state blenders from last time
    void clearBlenders();

    // blend all joint states and cache results
    void blendAndCache(bool reset_cached_joints);

    // interpolate all joints towards cached values
    void interpolate(F32 u);

    LLPose* getBlendedPose() { return &mBlendedPose; }
};

#endif // LL_LLPOSE_H

