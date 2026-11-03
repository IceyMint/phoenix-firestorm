/**
 * @file llvrboneregistry.cpp
 * @brief P2-07a Stage 1: VR tracking bone registry (HEAD only).
 *        P2-07B C3 (CP-5): spine chain, arm chains and finger
 *        chains registered; rollup implemented rule derived
 *        from membership.
 */

#include "llviewerprecompiledheaders.h"

#include "llvrboneregistry.h"

namespace
{
std::vector<FSVRTrackingBoneEntry> buildEntries()
{
    std::vector<FSVRTrackingBoneEntry> result;

    FSVRTrackingBoneEntry neck;
    neck.canonical_id = "mNeck";
    neck.group = FSVRRollup::HEAD;
    neck.expected_joint_name = "mNeck";
    neck.parent_dependency = "mTorso";
    neck.ownership_priority = {"HMD"};
    neck.registration_phase = "P2-07a";
    neck.target_space = "joint_local";
    neck.constraint_adapter_id = ""; // Stage 2, after audits A1-A10.
    neck.audit_status = FSVRTrackingBoneEntry::AuditStatus::DEFERRED;
    neck.direct_tracking_eligible = false; // registry candidate only
    result.push_back(neck);

    FSVRTrackingBoneEntry head;
    head.canonical_id = "mHead";
    head.group = FSVRRollup::HEAD;
    head.expected_joint_name = "mHead";
    head.parent_dependency = "mNeck";
    head.ownership_priority = {"HMD"};
    head.registration_phase = "P2-07a";
    head.target_space = "joint_local";
    head.constraint_adapter_id = "";
    head.audit_status = FSVRTrackingBoneEntry::AuditStatus::DEFERRED;
    head.direct_tracking_eligible = false;
    result.push_back(head);

    // Audit-only: existing keyframe suppression affects mSkull, so it
    // is registered for the audit, but it belongs to no rollup and is
    // never directly driven (design section 2.1, adjudication C).
    FSVRTrackingBoneEntry skull;
    skull.canonical_id = "mSkull";
    skull.group = FSVRRollup::NONE;
    skull.expected_joint_name = "mSkull";
    skull.parent_dependency = "mHead";
    skull.ownership_priority = {};
    skull.registration_phase = "P2-07a";
    skull.target_space = "joint_local";
    skull.constraint_adapter_id = "";
    skull.audit_status = FSVRTrackingBoneEntry::AuditStatus::DEFERRED;
    skull.direct_tracking_eligible = false;
    result.push_back(skull);

    // P2-07B C3 (CP-5): the spine chain, arm chains and finger
    // chains. Field values follow the Stage-1 precedent above
    // with the CP-5 assignments: true skeleton parents
    // (avatar_skeleton.xml nesting -- parent_dependency is audit
    // metadata only, OQ-4); body-estimation ownership RECORDED
    // for the spine chain and elbows [R1 -- C3-01] without
    // granting any rung (C3 plumbs no body-estimation sample, so
    // the arbitrator publishes those bones in the
    // source-unavailable form); collars/shoulders keep empty
    // priority (no tracking source by design, HAND-Q1); wrists
    // carry the hand-tracking ladder; fingers hand-tracking
    // only, each with its XR hand-joint index [R1 -- C3-02].
    // Adapter vocabulary (CP-6 / DP-2): "spine_chain" for the
    // spine chain, "arm_left" / "arm_right" for
    // shoulder/elbow/wrist per side (the side lives in the
    // string; collars and fingers map to NONE).
    auto add_entry = [&result](const char* canonical_id,
                               FSVRRollup group,
                               const char* parent_dependency,
                               std::vector<std::string> ownership_priority,
                               const char* constraint_adapter_id,
                               S32 xr_hand_joint_index) {
        FSVRTrackingBoneEntry entry;
        entry.canonical_id = canonical_id;
        entry.group = group;
        entry.expected_joint_name = canonical_id;
        entry.parent_dependency = parent_dependency;
        entry.ownership_priority = ownership_priority;
        entry.registration_phase = "P2-07b";
        entry.target_space = "joint_local";
        entry.constraint_adapter_id = constraint_adapter_id;
        entry.audit_status = FSVRTrackingBoneEntry::AuditStatus::DEFERRED;
        entry.direct_tracking_eligible = false;
        entry.xr_hand_joint_index = xr_hand_joint_index;
        result.push_back(entry);
    };

    // Spine chain (UPPER_BODY), in skeleton parent order
    // (mPelvis -> mSpine1 -> ... -> mChest).
    add_entry("mSpine1", FSVRRollup::UPPER_BODY, "mPelvis", {"body_estimation"}, "spine_chain", -1);
    add_entry("mSpine2", FSVRRollup::UPPER_BODY, "mSpine1", {"body_estimation"}, "spine_chain", -1);
    add_entry("mTorso", FSVRRollup::UPPER_BODY, "mSpine2", {"body_estimation"}, "spine_chain", -1);
    add_entry("mSpine3", FSVRRollup::UPPER_BODY, "mTorso", {"body_estimation"}, "spine_chain", -1);
    add_entry("mSpine4", FSVRRollup::UPPER_BODY, "mSpine3", {"body_estimation"}, "spine_chain", -1);
    add_entry("mChest", FSVRRollup::UPPER_BODY, "mSpine4", {"body_estimation"}, "spine_chain", -1);

    // Arm chains (ARMS_INCLUDING_HANDS), hanging from mChest.
    add_entry("mCollarLeft", FSVRRollup::ARMS_INCLUDING_HANDS, "mChest", {}, "", -1);
    add_entry("mShoulderLeft", FSVRRollup::ARMS_INCLUDING_HANDS, "mCollarLeft", {}, "arm_left", -1);
    add_entry("mElbowLeft", FSVRRollup::ARMS_INCLUDING_HANDS, "mShoulderLeft", {"body_estimation"}, "arm_left", -1);
    add_entry("mWristLeft", FSVRRollup::ARMS_INCLUDING_HANDS, "mElbowLeft", {"hand_tracking", "animation"}, "arm_left", -1);
    add_entry("mCollarRight", FSVRRollup::ARMS_INCLUDING_HANDS, "mChest", {}, "", -1);
    add_entry("mShoulderRight", FSVRRollup::ARMS_INCLUDING_HANDS, "mCollarRight", {}, "arm_right", -1);
    add_entry("mElbowRight", FSVRRollup::ARMS_INCLUDING_HANDS, "mShoulderRight", {"body_estimation"}, "arm_right", -1);
    add_entry("mWristRight", FSVRRollup::ARMS_INCLUDING_HANDS, "mElbowRight", {"hand_tracking", "animation"}, "arm_right", -1);

    // Finger chains (ARMS_INCLUDING_HANDS, DP-1): 15 per side,
    // the 1-joint parented to the wrist, then chained. XR joint
    // indices per the CP-5 [R1] table; the thumb has no
    // INTERMEDIATE joint in XR (metacarpal/proximal/distal/tip),
    // so Thumb2 takes DISTAL and Thumb3, riding the tip segment,
    // takes TIP.
    struct FingerChainSpec
    {
        const char* stem;
        S32 xr_index[3];
    };
    static const FingerChainSpec kFingerChains[] = {
        {"Thumb", {3, 4, 5}},     // PROXIMAL, DISTAL, TIP
        {"Index", {7, 8, 9}},     // PROXIMAL, INTERMEDIATE, DISTAL
        {"Middle", {12, 13, 14}}, // PROXIMAL, INTERMEDIATE, DISTAL
        {"Ring", {17, 18, 19}},   // PROXIMAL, INTERMEDIATE, DISTAL
        {"Pinky", {22, 23, 24}},  // LITTLE PROXIMAL/INTERMEDIATE/DISTAL
    };
    for (const char* side_name : {"Left", "Right"})
    {
        for (const FingerChainSpec& chain : kFingerChains)
        {
            for (int segment = 0; segment < 3; ++segment)
            {
                const std::string canonical_id = std::string("mHand") +
                    chain.stem + std::to_string(segment + 1) + side_name;
                const std::string parent = (segment == 0)
                    ? std::string("mWrist") + side_name
                    : std::string("mHand") + chain.stem +
                          std::to_string(segment) + side_name;
                add_entry(canonical_id.c_str(),
                          FSVRRollup::ARMS_INCLUDING_HANDS,
                          parent.c_str(), {"hand_tracking"}, "",
                          chain.xr_index[segment]);
            }
        }
    }

    return result;
}
} // namespace

// static
const std::vector<FSVRTrackingBoneEntry>& FSVRBoneRegistry::entries()
{
    static const std::vector<FSVRTrackingBoneEntry> s_entries = buildEntries();
    return s_entries;
}

// static
const FSVRTrackingBoneEntry* FSVRBoneRegistry::find(
    const std::string& canonical_id)
{
    for (const FSVRTrackingBoneEntry& entry : entries())
    {
        if (entry.canonical_id == canonical_id)
        {
            return &entry;
        }
    }
    return nullptr;
}

// static
std::set<std::string> FSVRBoneRegistry::knownIds()
{
    std::set<std::string> ids;
    for (const FSVRTrackingBoneEntry& entry : entries())
    {
        ids.insert(entry.canonical_id);
    }
    return ids;
}

// static
const char* FSVRBoneRegistry::rollupName(FSVRRollup group)
{
    switch (group)
    {
    case FSVRRollup::HEAD:
        return "HEAD";
    case FSVRRollup::UPPER_BODY:
        return "UPPER_BODY";
    case FSVRRollup::LOWER_BODY:
        return "LOWER_BODY";
    case FSVRRollup::ARMS_INCLUDING_HANDS:
        return "ARMS_INCLUDING_HANDS";
    case FSVRRollup::NONE:
    default:
        return "NONE";
    }
}

// static
std::vector<FSVRRollupState> FSVRBoneRegistry::computeRollups(
    const std::set<std::string>& known_disabled_ids)
{
    static const FSVRRollup groups[] = {FSVRRollup::HEAD,
                                        FSVRRollup::UPPER_BODY,
                                        FSVRRollup::LOWER_BODY,
                                        FSVRRollup::ARMS_INCLUDING_HANDS};
    std::vector<FSVRRollupState> result;
    for (const FSVRRollup group : groups)
    {
        FSVRRollupState state;
        state.group = group;
        state.name = rollupName(group);
        for (const FSVRTrackingBoneEntry& entry : entries())
        {
            if (entry.group == group)
            {
                ++state.member_count;
                if (known_disabled_ids.find(entry.canonical_id) !=
                    known_disabled_ids.end())
                {
                    ++state.disabled_count;
                }
            }
        }
        // P2-07B C3 (CP-5): implemented is a membership fact,
        // not a hardcoded group (the P2-07a form was
        // (group == FSVRRollup::HEAD), from when only HEAD had
        // registered members). A group with no members --
        // LOWER_BODY -- keeps reporting not_implemented below.
        state.implemented = (state.member_count > 0);
        if (!state.implemented || state.member_count == 0)
        {
            state.derived_state = "not_implemented";
        }
        else if (state.disabled_count == 0)
        {
            state.derived_state = "all_enabled";
        }
        else if (state.disabled_count < state.member_count)
        {
            state.derived_state = "partially_disabled";
        }
        else
        {
            state.derived_state = "all_disabled";
        }
        result.push_back(state);
    }
    return result;
}
