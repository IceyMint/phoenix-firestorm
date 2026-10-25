/**
 * @file llvrboneregistry.h
 * @brief P2-07a Stage 1: canonical VR tracking bone registry.
 *
 * Single source of truth for canonical bone identifiers, the four
 * rollup groups, source ownership and audit eligibility (design
 * section 2, implementation spec sections 2.3). Later phases extend
 * the registry; the settings schema never changes.
 */

#ifndef LL_LLVRBONEREGISTRY_H
#define LL_LLVRBONEREGISTRY_H

#include <set>
#include <string>
#include <vector>

enum class FSVRRollup
{
    NONE, // audit-only entry, belongs to no rollup
    HEAD,
    UPPER_BODY,
    LOWER_BODY,
    ARMS_INCLUDING_HANDS
};

struct FSVRTrackingBoneEntry
{
    std::string canonical_id;
    FSVRRollup group = FSVRRollup::NONE;
    std::string expected_joint_name;
    std::string parent_dependency;
    // Eligible sources in ownership-priority order.
    std::vector<std::string> ownership_priority;
    std::string registration_phase;
    std::string target_space;
    std::string constraint_adapter_id;
    enum class AuditStatus
    {
        UNVERIFIED,
        VERIFIED,
        DEFERRED
    };
    AuditStatus audit_status = AuditStatus::UNVERIFIED;
    // Whether this joint may currently receive a direct tracked
    // contribution. False for every entry in Stage 1.
    bool direct_tracking_eligible = false;
};

// Derived (never persisted) rollup state.
struct FSVRRollupState
{
    FSVRRollup group = FSVRRollup::NONE;
    std::string name;
    bool implemented = false;
    S32 member_count = 0;
    S32 disabled_count = 0;
    // "not_implemented", "all_enabled", "partially_disabled",
    // "all_disabled", or "unavailable".
    std::string derived_state;
};

class FSVRBoneRegistry
{
public:
    static const std::vector<FSVRTrackingBoneEntry>& entries();
    static const FSVRTrackingBoneEntry* find(const std::string& canonical_id);
    static std::set<std::string> knownIds();

    // Computes all four rollups against the effective disabled set.
    // HEAD is the only implemented rollup in P2-07a; the others are
    // reported as not implemented rather than pretending to work.
    static std::vector<FSVRRollupState> computeRollups(
        const std::set<std::string>& known_disabled_ids);

    static const char* rollupName(FSVRRollup group);
};

#endif // LL_LLVRBONEREGISTRY_H
