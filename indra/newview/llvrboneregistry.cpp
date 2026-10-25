/**
 * @file llvrboneregistry.cpp
 * @brief P2-07a Stage 1: VR tracking bone registry (HEAD only).
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
        // Only HEAD has registered members in P2-07a.
        state.implemented = (group == FSVRRollup::HEAD);
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
