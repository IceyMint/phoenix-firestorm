#include "llviewerprecompiledheaders.h"

#include "fsaiintegration.h"

// Additional includes for time
#include <ctime>

// AI Process Bridge - Hot Restart Capable! 🔥
#include "python_bridge/py_ai_process_bridge.h"

// Viewer systems
#include "llagent.h"
#include "llagentcamera.h"
#include "llworld.h"
#include "llavatarnamecache.h"
#include "llviewerregion.h"
#include "llviewercontrol.h"
#include "llviewerobjectlist.h"
#include "llfloaterreg.h"
#include "llvoavatarself.h"
#include "llpathfindingmanager.h"
#include "llpathinglib.h"
#include "pipeline.h"
#include "llcallbacklist.h"
#include "llstring.h"
#include "llviewermenu.h"
#include "llhudeffectlookat.h"
#include "llselectmgr.h"


// Firestorm specific
#include "fsareasearch.h"
#include "fsfloaternearbychat.h"
#include "fsnearbychathub.h"

// Standard libraries
#include <sstream>
#include <algorithm>
#include <functional>
#include <cmath>
#include <limits>
#include <queue>
#include <map>
#include <memory>
#include <tuple>
#include <chrono>
#include <set>

// Forward declarations
void near_sit_down_point(bool success, void*);

namespace
{
const F32 kDefaultMoveStandoffMeters = 0.9f;
const F32 kMinMoveStandoffMeters = 0.4f;
const F32 kMaxMoveStandoffMeters = 2.5f;
const F32 kMoveClampDistanceMeters = 100.0f;
const F64 kPathProbeRayHeightMeters = 6.0;
const F64 kPersonSnapRadiusMeters = 2.5;
const F32 kMoveAutopilotStopDistanceMeters = 0.5f;
const F32 kMoveAutopilotRotationThreshold = 0.03f;
const F64 kCustomProbeAvatarRadiusMeters = 0.45;
const F64 kCustomProbeFootClearanceMeters = 0.20;
const F64 kCustomProbeChestHeightMeters = 0.75;
const F64 kCustomNodeSpacingMeters = 0.80;
const F64 kCustomEdgeSampleMeters = 1.20;
const F32 kCustomBroadPhaseExpandMeters = 0.60f;
const F32 kCustomBroadPhaseCorridorHalfWidthMeters = 8.0f;
const F32 kCustomBroadPhaseVerticalMarginMeters = 2.5f;
const F64 kObjectApproachStandOffMinMeters = 0.30;
const F64 kObjectApproachStandOffMidMeters = 0.55;
const F64 kObjectApproachStandOffMaxMeters = 0.90;
const F32 kMoveObjectFinalGoalToleranceMeters = 1.25f;
const U32 kMovePathReplanAttempts = 2;
const F32 kNearbyObjectsMaxDistanceMeters = 20.0f;
const F32 kNearbyObjectsMaxVerticalDeltaMeters = 2.5f;
const S32 kNearbyObjectsCandidateLimitMin = 60;
const S32 kNearbyObjectsCandidateLimitMax = 150;
const F64 kNearbyObjectsCandidateCoverageFraction = 0.60;
const S32 kNearbyObjectsMaxOutput = 15;
const S32 kNearbyObjectsReachabilityBudgetPerUpdate = 6;
const F64 kNearbyObjectsReachabilityCacheTtlSeconds = 3.0;
const F64 kNearbyObjectsReachabilityCacheAgentQuantizeScale = 2.0;

struct MoveWaypointCallbackData
{
    U64 plan_id;
    std::vector<LLVector3d> waypoints;
    size_t next_index;
    LLVector3d final_goal;
    F32 final_goal_tolerance;
    U32 replans_remaining;
    LLUUID follow_target_id;
    F32 follow_standoff;
};

struct BroadPhaseObstacle
{
    LLBBox bbox_agent;
    LLVector3 center_agent;
    F32 radius_2d;
    F32 min_z;
    F32 max_z;
};

enum class MovePlanSource
{
    CUSTOM,
    LEGACY,
    FALLBACK_DIRECT
};

struct MovePlannerTelemetry
{
    U64 move_requests_total = 0;
    U64 non_avatar_plan_requests = 0;
    U64 person_follow_requests = 0;
    U64 skipped_close_target = 0;

    U64 custom_attempts = 0;
    U64 custom_success = 0;
    U64 legacy_attempts = 0;
    U64 legacy_success = 0;
    U64 fallback_direct = 0;
    U64 avatar_los_direct_follow = 0;
    U64 avatar_hybrid_path_attempts = 0;
    U64 avatar_hybrid_path_success = 0;
    U64 avatar_hybrid_follow_switch = 0;
    U64 avatar_hybrid_follow_fallback = 0;
    U64 blocked_abort_non_avatar = 0;
    U64 blocked_abort_avatar = 0;

    U64 waypoint_plan_starts = 0;
    U64 waypoint_plan_completions = 0;
    U64 waypoint_leg_failures = 0;
    U64 replans_attempted = 0;
    U64 replans_succeeded = 0;
    U64 replans_failed = 0;

    F64 custom_ms_total = 0.0;
    F64 legacy_ms_total = 0.0;
};

struct ObjectReachabilityCacheEntry
{
    std::tuple<S32, S32, S32> agent_key;
    bool reachable = false;
    std::chrono::steady_clock::time_point stamp;
};

U64 g_move_plan_generation = 0;
MovePlannerTelemetry g_move_planner_telemetry;
std::map<LLUUID, ObjectReachabilityCacheEntry> g_object_reachability_cache;
std::vector<LLUUID> g_nearby_object_round_robin_order;
size_t g_nearby_object_round_robin_cursor = 0;

void gather_broad_phase_obstacles(
    const LLVector3d& start,
    const LLVector3d& goal,
    std::vector<BroadPhaseObstacle>& obstacles_out);
bool is_custom_node_walkable(
    const LLVector3d& node_global,
    const std::vector<BroadPhaseObstacle>& obstacles,
    F64 pelvis_to_foot,
    std::map<std::tuple<S32, S32, S32>, bool>& node_cache);
std::tuple<S32, S32, S32> quantized_point_key(const LLVector3d& point_global, F64 scale = 2.0);

F64 duration_ms(const std::chrono::steady_clock::time_point& start,
                const std::chrono::steady_clock::time_point& end)
{
    return std::chrono::duration_cast<std::chrono::duration<F64, std::milli>>(end - start).count();
}

const char* move_plan_source_to_text(MovePlanSource source)
{
    switch (source)
    {
    case MovePlanSource::CUSTOM:
        return "custom";
    case MovePlanSource::LEGACY:
        return "legacy";
    case MovePlanSource::FALLBACK_DIRECT:
        return "fallback_direct";
    default:
        return "unknown";
    }
}

void maybe_log_move_planner_summary(const char* context)
{
    if ((g_move_planner_telemetry.move_requests_total % 10) != 0)
    {
        return;
    }

    const F64 avg_custom_ms = (g_move_planner_telemetry.custom_attempts > 0)
        ? (g_move_planner_telemetry.custom_ms_total / static_cast<F64>(g_move_planner_telemetry.custom_attempts))
        : 0.0;
    const F64 avg_legacy_ms = (g_move_planner_telemetry.legacy_attempts > 0)
        ? (g_move_planner_telemetry.legacy_ms_total / static_cast<F64>(g_move_planner_telemetry.legacy_attempts))
        : 0.0;

    LL_INFOS("AI") << "[AI] Move planner summary (" << context << "):"
                   << " move_requests=" << g_move_planner_telemetry.move_requests_total
                   << " non_avatar_plan_requests=" << g_move_planner_telemetry.non_avatar_plan_requests
                   << " person_follow_requests=" << g_move_planner_telemetry.person_follow_requests
                   << " skipped_close_target=" << g_move_planner_telemetry.skipped_close_target
                   << " custom=" << g_move_planner_telemetry.custom_success << "/" << g_move_planner_telemetry.custom_attempts
                   << " legacy=" << g_move_planner_telemetry.legacy_success << "/" << g_move_planner_telemetry.legacy_attempts
                   << " fallback_direct=" << g_move_planner_telemetry.fallback_direct
                   << " avatar_los_follow=" << g_move_planner_telemetry.avatar_los_direct_follow
                   << " avatar_path=" << g_move_planner_telemetry.avatar_hybrid_path_success << "/" << g_move_planner_telemetry.avatar_hybrid_path_attempts
                   << " avatar_follow_switch=" << g_move_planner_telemetry.avatar_hybrid_follow_switch
                   << " avatar_follow_fallback=" << g_move_planner_telemetry.avatar_hybrid_follow_fallback
                   << " blocked_abort_non_avatar=" << g_move_planner_telemetry.blocked_abort_non_avatar
                   << " blocked_abort_avatar=" << g_move_planner_telemetry.blocked_abort_avatar
                   << " waypoint_plans=" << g_move_planner_telemetry.waypoint_plan_starts
                   << " completed=" << g_move_planner_telemetry.waypoint_plan_completions
                   << " leg_failures=" << g_move_planner_telemetry.waypoint_leg_failures
                   << " replans=" << g_move_planner_telemetry.replans_succeeded << "/" << g_move_planner_telemetry.replans_attempted
                   << " replans_failed=" << g_move_planner_telemetry.replans_failed
                   << " avg_custom_ms=" << avg_custom_ms
                   << " avg_legacy_ms=" << avg_legacy_ms
                   << LL_ENDL;
}

F32 clamp_move_standoff(F32 raw_standoff)
{
    if (raw_standoff < kMinMoveStandoffMeters)
    {
        return kMinMoveStandoffMeters;
    }
    if (raw_standoff > kMaxMoveStandoffMeters)
    {
        return kMaxMoveStandoffMeters;
    }
    return raw_standoff;
}

bool parse_standoff_parameter(const std::map<std::string, std::string>& parameters, F32& out_standoff)
{
    auto it = parameters.find("standoff");
    if (it == parameters.end())
    {
        return false;
    }
    try
    {
        out_standoff = clamp_move_standoff(std::stof(it->second));
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool resolve_avatar_position(const LLUUID& avatar_id, LLVector3d& avatar_pos)
{
    if (avatar_id.isNull())
    {
        return false;
    }

    uuid_vec_t avatar_ids;
    std::vector<LLVector3d> positions;
    LLWorld::instance().getAvatars(&avatar_ids, &positions, gAgent.getPositionGlobal(), 100.0f);

    for (size_t i = 0; i < avatar_ids.size() && i < positions.size(); ++i)
    {
        if (avatar_ids[i] == avatar_id)
        {
            avatar_pos = positions[i];
            return true;
        }
    }
    return false;
}

bool find_closest_avatar_to_point(const LLVector3d& point, F64 max_distance, LLUUID& avatar_id, LLVector3d& avatar_pos)
{
    avatar_id = LLUUID::null;
    const LLUUID self_id = gAgent.getID();

    uuid_vec_t avatar_ids;
    std::vector<LLVector3d> positions;
    LLWorld::instance().getAvatars(&avatar_ids, &positions, point, static_cast<F32>(max_distance));

    F64 best = max_distance + 1.0;
    for (size_t i = 0; i < avatar_ids.size() && i < positions.size(); ++i)
    {
        if (!self_id.isNull() && avatar_ids[i] == self_id)
        {
            continue;
        }
        const F64 distance = dist_vec(point, positions[i]);
        if (distance < best)
        {
            best = distance;
            avatar_id = avatar_ids[i];
            avatar_pos = positions[i];
        }
    }
    return !avatar_id.isNull();
}

const char* bool_to_text(bool value)
{
    return value ? "yes" : "no";
}

LLVector3 global_to_agent_vec(const LLVector3d& global_pos)
{
    return gAgent.getPosAgentFromGlobal(global_pos);
}

F32 dist_sq_to_segment_2d(const LLVector3& point, const LLVector3& seg_start, const LLVector3& seg_end)
{
    const LLVector3 seg(seg_end.mV[VX] - seg_start.mV[VX], seg_end.mV[VY] - seg_start.mV[VY], 0.0f);
    const LLVector3 from_start(point.mV[VX] - seg_start.mV[VX], point.mV[VY] - seg_start.mV[VY], 0.0f);
    const F32 seg_len_sq = (seg.mV[VX] * seg.mV[VX]) + (seg.mV[VY] * seg.mV[VY]);

    F32 t = 0.0f;
    if (seg_len_sq > 0.0001f)
    {
        t = ((from_start.mV[VX] * seg.mV[VX]) + (from_start.mV[VY] * seg.mV[VY])) / seg_len_sq;
        t = llclamp(t, 0.0f, 1.0f);
    }

    const F32 closest_x = seg_start.mV[VX] + seg.mV[VX] * t;
    const F32 closest_y = seg_start.mV[VY] + seg.mV[VY] * t;
    const F32 dx = point.mV[VX] - closest_x;
    const F32 dy = point.mV[VY] - closest_y;
    return (dx * dx) + (dy * dy);
}

bool raycast_hits_static_world(const LLVector3d& start_global, const LLVector3d& end_global)
{
    LLVector3 start_agent = global_to_agent_vec(start_global);
    LLVector3 end_agent = global_to_agent_vec(end_global);
    LLVector4a start4;
    LLVector4a end4;
    LLVector4a intersection;
    start4.load3(start_agent.mV);
    end4.load3(end_agent.mV);

    S32 face_hit = -1;
    LLViewerObject* hit = gPipeline.lineSegmentIntersectInWorld(
        start4,
        end4,
        true,
        false,
        true,
        false,
        &face_hit,
        nullptr,
        nullptr,
        &intersection,
        nullptr,
        nullptr,
        nullptr);
    if (!hit)
    {
        return false;
    }
    if (hit->isAvatar() || hit->isAttachment())
    {
        return false;
    }
    return true;
}

F64 agent_pelvis_to_foot_meters()
{
    return (gAgentAvatarp != nullptr)
        ? llmax(static_cast<F64>(gAgentAvatarp->getPelvisToFoot()), 0.65)
        : 1.0;
}

bool has_clear_line_of_sight_to_avatar(const LLVector3d& from_global, const LLVector3d& avatar_global)
{
    const F64 pelvis_to_foot = agent_pelvis_to_foot_meters();

    LLVector3d from_foot = from_global;
    LLVector3d to_foot = avatar_global;
    from_foot.mdV[VZ] = from_global.mdV[VZ] - pelvis_to_foot + kCustomProbeFootClearanceMeters;
    to_foot.mdV[VZ] = avatar_global.mdV[VZ] - pelvis_to_foot + kCustomProbeFootClearanceMeters;

    LLVector3d from_chest = from_global;
    LLVector3d to_chest = avatar_global;
    from_chest.mdV[VZ] = from_global.mdV[VZ] + kCustomProbeChestHeightMeters;
    to_chest.mdV[VZ] = avatar_global.mdV[VZ] + kCustomProbeChestHeightMeters;

    const bool foot_blocked = raycast_hits_static_world(from_foot, to_foot);
    const bool chest_blocked = raycast_hits_static_world(from_chest, to_chest);
    return !(foot_blocked || chest_blocked);
}

bool raycast_hits_static_world_except_object(
    const LLVector3d& start_global,
    const LLVector3d& end_global,
    LLViewerObject* target_object)
{
    if (!target_object)
    {
        return raycast_hits_static_world(start_global, end_global);
    }

    LLVector3 start_agent = global_to_agent_vec(start_global);
    LLVector3 end_agent = global_to_agent_vec(end_global);
    LLVector4a start4;
    LLVector4a end4;
    LLVector4a intersection;
    start4.load3(start_agent.mV);
    end4.load3(end_agent.mV);

    S32 face_hit = -1;
    LLViewerObject* hit = gPipeline.lineSegmentIntersectInWorld(
        start4,
        end4,
        true,
        false,
        true,
        false,
        &face_hit,
        nullptr,
        nullptr,
        &intersection,
        nullptr,
        nullptr,
        nullptr);
    if (!hit)
    {
        return false;
    }
    if (hit->isAvatar() || hit->isAttachment())
    {
        return false;
    }

    LLViewerObject* target_root = target_object->getRootEdit();
    LLViewerObject* hit_root = hit->getRootEdit();
    if (hit == target_object || hit == target_root || hit_root == target_object || hit_root == target_root)
    {
        return false;
    }

    return true;
}

bool has_clear_line_of_sight_to_object(const LLVector3d& from_global, LLViewerObject* objectp)
{
    if (!objectp || objectp->isDead())
    {
        return false;
    }

    const F64 pelvis_to_foot = agent_pelvis_to_foot_meters();
    LLVector3d from_foot = from_global;
    from_foot.mdV[VZ] = from_global.mdV[VZ] - pelvis_to_foot + kCustomProbeFootClearanceMeters;

    LLVector3d from_chest = from_global;
    from_chest.mdV[VZ] = from_global.mdV[VZ] + kCustomProbeChestHeightMeters;

    const LLBBox bbox_agent = objectp->getBoundingBoxAgent().getAxisAligned();
    const LLVector3 min_agent = bbox_agent.getMinAgent();
    const LLVector3 max_agent = bbox_agent.getMaxAgent();
    const LLVector3d min_global = gAgent.getPosGlobalFromAgent(min_agent);
    const LLVector3d max_global = gAgent.getPosGlobalFromAgent(max_agent);
    const F64 min_z = llmin(min_global.mdV[VZ], max_global.mdV[VZ]);
    const F64 max_z = llmax(min_global.mdV[VZ], max_global.mdV[VZ]);

    LLVector3d to_center = objectp->getPositionGlobal();
    LLVector3d to_upper = to_center;
    to_center.mdV[VZ] = min_z + 0.20;
    to_upper.mdV[VZ] = llmax(min_z + 0.40, max_z - 0.15);
    if (to_upper.mdV[VZ] < to_center.mdV[VZ])
    {
        to_upper.mdV[VZ] = to_center.mdV[VZ];
    }

    const bool lower_blocked = raycast_hits_static_world_except_object(from_foot, to_center, objectp);
    const bool upper_blocked = raycast_hits_static_world_except_object(from_chest, to_upper, objectp);
    return !(lower_blocked && upper_blocked);
}

std::tuple<S32, S32, S32> nearby_objects_agent_cache_key(const LLVector3d& agent_global)
{
    return quantized_point_key(agent_global, kNearbyObjectsReachabilityCacheAgentQuantizeScale);
}

bool get_cached_object_reachability(
    const LLUUID& object_id,
    const std::tuple<S32, S32, S32>& agent_key,
    bool& reachable_out,
    bool allow_stale = false,
    bool* stale_out = nullptr)
{
    auto it = g_object_reachability_cache.find(object_id);
    if (it == g_object_reachability_cache.end())
    {
        return false;
    }

    const F64 age_seconds = duration_ms(it->second.stamp, std::chrono::steady_clock::now()) / 1000.0;
    const bool stale = (age_seconds > kNearbyObjectsReachabilityCacheTtlSeconds) ||
                       (it->second.agent_key != agent_key);
    if (stale && !allow_stale)
    {
        return false;
    }

    if (stale_out)
    {
        *stale_out = stale;
    }
    reachable_out = it->second.reachable;
    return true;
}

void put_cached_object_reachability(
    const LLUUID& object_id,
    const std::tuple<S32, S32, S32>& agent_key,
    bool reachable)
{
    ObjectReachabilityCacheEntry entry;
    entry.agent_key = agent_key;
    entry.reachable = reachable;
    entry.stamp = std::chrono::steady_clock::now();
    g_object_reachability_cache[object_id] = entry;
}

LLVector3d compute_avatar_approach_goal(const LLVector3d& from_global, const LLVector3d& avatar_global, F32 standoff)
{
    LLVector3d delta = avatar_global - from_global;
    LLVector3d dir2d(delta.mdV[VX], delta.mdV[VY], 0.0);
    const F64 dist2d = dir2d.length();
    if (dist2d < 0.001)
    {
        return avatar_global;
    }

    dir2d.normalize();
    const F64 desired_range = llmax(static_cast<F64>(standoff) + 0.8, 1.25);
    LLVector3d goal(
        avatar_global.mdV[VX] - (dir2d.mdV[VX] * desired_range),
        avatar_global.mdV[VY] - (dir2d.mdV[VY] * desired_range),
        avatar_global.mdV[VZ]
    );
    return goal;
}

bool compute_object_approach_goal(const LLVector3d& from_global, const LLViewerObject* objectp, LLVector3d& goal_out)
{
    if (!objectp || objectp->isDead() || objectp->isAvatar() || objectp->isAttachment())
    {
        return false;
    }

    LLViewerRegion* agent_region = gAgent.getRegion();
    if (!agent_region || objectp->getRegion() != agent_region)
    {
        return false;
    }

    const LLVector3d object_pos = objectp->getPositionGlobal();
    std::vector<BroadPhaseObstacle> obstacles;
    gather_broad_phase_obstacles(from_global, object_pos, obstacles);
    std::map<std::tuple<S32, S32, S32>, bool> node_cache;
    const F64 pelvis_to_foot = agent_pelvis_to_foot_meters();

    const LLBBox bbox_agent = objectp->getBoundingBoxAgent().getAxisAligned();
    const LLVector3 min_agent = bbox_agent.getMinAgent();
    const LLVector3 max_agent = bbox_agent.getMaxAgent();
    const LLVector3 center_agent = (min_agent + max_agent) * 0.5f;
    const F64 half_x = llmax<F64>((max_agent.mV[VX] - min_agent.mV[VX]) * 0.5, 0.10);
    const F64 half_y = llmax<F64>((max_agent.mV[VY] - min_agent.mV[VY]) * 0.5, 0.10);

    LLVector3 from_agent = global_to_agent_vec(from_global);
    LLVector3d preferred_dir(
        static_cast<F64>(from_agent.mV[VX] - center_agent.mV[VX]),
        static_cast<F64>(from_agent.mV[VY] - center_agent.mV[VY]),
        0.0);
    if (preferred_dir.length() < 0.001)
    {
        preferred_dir.setVec(1.0, 0.0, 0.0);
    }
    preferred_dir.normalize();

    static const F64 kPi = 3.14159265358979323846;
    static const F64 kAngleOffsets[] = {
        0.0,
        (20.0 * kPi) / 180.0, -(20.0 * kPi) / 180.0,
        (45.0 * kPi) / 180.0, -(45.0 * kPi) / 180.0,
        (70.0 * kPi) / 180.0, -(70.0 * kPi) / 180.0,
        (110.0 * kPi) / 180.0, -(110.0 * kPi) / 180.0,
        (160.0 * kPi) / 180.0, -(160.0 * kPi) / 180.0
    };
    static const F64 kStandOffs[] = {
        kObjectApproachStandOffMinMeters,
        kObjectApproachStandOffMidMeters,
        kObjectApproachStandOffMaxMeters
    };

    const F64 base_angle = std::atan2(preferred_dir.mdV[VY], preferred_dir.mdV[VX]);

    bool found = false;
    LLVector3d best_goal;
    F64 best_center_distance = std::numeric_limits<F64>::max();
    F64 best_agent_distance = std::numeric_limits<F64>::max();

    for (F64 stand_off : kStandOffs)
    {
        for (F64 angle_offset : kAngleOffsets)
        {
            const F64 theta = base_angle + angle_offset;
            const F64 dir_x = std::cos(theta);
            const F64 dir_y = std::sin(theta);

            const F64 tx = (std::fabs(dir_x) > 0.001) ? (half_x / std::fabs(dir_x)) : std::numeric_limits<F64>::max();
            const F64 ty = (std::fabs(dir_y) > 0.001) ? (half_y / std::fabs(dir_y)) : std::numeric_limits<F64>::max();
            const F64 boundary_t = llmin(tx, ty);
            if (!std::isfinite(boundary_t))
            {
                continue;
            }

            LLVector3 candidate_agent(
                center_agent.mV[VX] + static_cast<F32>((boundary_t + stand_off) * dir_x),
                center_agent.mV[VY] + static_cast<F32>((boundary_t + stand_off) * dir_y),
                center_agent.mV[VZ]);
            LLVector3d candidate_global = gAgent.getPosGlobalFromAgent(candidate_agent);
            candidate_global.mdV[VZ] = object_pos.mdV[VZ];

            if (!is_custom_node_walkable(candidate_global, obstacles, pelvis_to_foot, node_cache))
            {
                continue;
            }

            const F64 center_distance = dist_vec(candidate_global, object_pos);
            const F64 agent_distance = dist_vec(candidate_global, from_global);
            const bool is_better =
                !found ||
                (center_distance < best_center_distance - 0.05) ||
                ((std::fabs(center_distance - best_center_distance) <= 0.05) && (agent_distance < best_agent_distance));
            if (!is_better)
            {
                continue;
            }

            found = true;
            best_goal = candidate_global;
            best_center_distance = center_distance;
            best_agent_distance = agent_distance;
        }
    }

    if (!found)
    {
        return false;
    }

    goal_out = best_goal;
    return true;
}

void gather_broad_phase_obstacles(
    const LLVector3d& start,
    const LLVector3d& goal,
    std::vector<BroadPhaseObstacle>& obstacles_out)
{
    obstacles_out.clear();
    LLViewerRegion* agent_region = gAgent.getRegion();
    if (!agent_region)
    {
        return;
    }

    const LLVector3d plan_center_global = (start + goal) * 0.5;
    const LLVector3 plan_center_agent = global_to_agent_vec(plan_center_global);
    const F32 plan_radius = llmax(static_cast<F32>(dist_vec(start, goal) * 0.5), 12.0f) + 10.0f;
    const F32 plan_min_z = static_cast<F32>(llmin(start.mdV[VZ], goal.mdV[VZ]) - kCustomBroadPhaseVerticalMarginMeters);
    const F32 plan_max_z = static_cast<F32>(llmax(start.mdV[VZ], goal.mdV[VZ]) + kCustomBroadPhaseVerticalMarginMeters);
    const LLVector3 start_agent = global_to_agent_vec(start);
    const LLVector3 goal_agent = global_to_agent_vec(goal);
    const F32 corridor_half_width = kCustomBroadPhaseCorridorHalfWidthMeters;

    obstacles_out.reserve(256);
    const S32 object_count = gObjectList.getNumObjects();
    for (S32 i = 0; i < object_count; ++i)
    {
        LLViewerObject* objectp = gObjectList.getObject(i);
        if (!objectp || objectp->isDead() || objectp->isAvatar() || objectp->isAttachment())
        {
            continue;
        }
        if (objectp->getRegion() != agent_region)
        {
            continue;
        }

        LLBBox bbox_agent = objectp->getBoundingBoxAgent().getAxisAligned();
        bbox_agent.expand(kCustomBroadPhaseExpandMeters);
        const LLVector3 min_agent = bbox_agent.getMinAgent();
        const LLVector3 max_agent = bbox_agent.getMaxAgent();
        const LLVector3 center_agent = (min_agent + max_agent) * 0.5f;
        const F32 half_x = (max_agent.mV[VX] - min_agent.mV[VX]) * 0.5f;
        const F32 half_y = (max_agent.mV[VY] - min_agent.mV[VY]) * 0.5f;
        const F32 radius_2d = std::sqrt((half_x * half_x) + (half_y * half_y));

        const F32 dx = center_agent.mV[VX] - plan_center_agent.mV[VX];
        const F32 dy = center_agent.mV[VY] - plan_center_agent.mV[VY];
        const F32 max_xy = plan_radius + radius_2d;
        if ((dx * dx) + (dy * dy) > (max_xy * max_xy))
        {
            continue;
        }

        const F32 corridor_limit = corridor_half_width + radius_2d;
        const F32 dist_sq_to_path = dist_sq_to_segment_2d(center_agent, start_agent, goal_agent);
        if (dist_sq_to_path > (corridor_limit * corridor_limit))
        {
            continue;
        }

        const LLVector3d min_global = gAgent.getPosGlobalFromAgent(min_agent);
        const LLVector3d max_global = gAgent.getPosGlobalFromAgent(max_agent);
        const F32 min_z = static_cast<F32>(llmin(min_global.mdV[VZ], max_global.mdV[VZ]));
        const F32 max_z = static_cast<F32>(llmax(min_global.mdV[VZ], max_global.mdV[VZ]));
        if (max_z < plan_min_z || min_z > plan_max_z)
        {
            continue;
        }

        BroadPhaseObstacle obstacle;
        obstacle.bbox_agent = bbox_agent;
        obstacle.center_agent = center_agent;
        obstacle.radius_2d = radius_2d;
        obstacle.min_z = min_z;
        obstacle.max_z = max_z;
        obstacles_out.push_back(obstacle);
    }
}

bool is_point_near_broad_phase(const LLVector3d& point_global, const std::vector<BroadPhaseObstacle>& obstacles)
{
    const LLVector3 point_agent = global_to_agent_vec(point_global);
    for (const BroadPhaseObstacle& obstacle : obstacles)
    {
        const F32 dz = static_cast<F32>(point_global.mdV[VZ]) - ((obstacle.min_z + obstacle.max_z) * 0.5f);
        const F32 hz = (obstacle.max_z - obstacle.min_z) * 0.5f + kCustomBroadPhaseVerticalMarginMeters;
        if (std::fabs(dz) > hz)
        {
            continue;
        }
        const F32 dx = point_agent.mV[VX] - obstacle.center_agent.mV[VX];
        const F32 dy = point_agent.mV[VY] - obstacle.center_agent.mV[VY];
        const F32 max_xy = obstacle.radius_2d + 1.0f;
        if ((dx * dx) + (dy * dy) > (max_xy * max_xy))
        {
            continue;
        }
        return true;
    }
    return false;
}

std::tuple<S32, S32, S32> quantized_point_key(const LLVector3d& point_global, F64 scale)
{
    return std::make_tuple(
        static_cast<S32>(std::llround(point_global.mdV[VX] * scale)),
        static_cast<S32>(std::llround(point_global.mdV[VY] * scale)),
        static_cast<S32>(std::llround(point_global.mdV[VZ] * scale)));
}

bool is_custom_node_walkable(
    const LLVector3d& node_global,
    const std::vector<BroadPhaseObstacle>& obstacles,
    F64 pelvis_to_foot,
    std::map<std::tuple<S32, S32, S32>, bool>& node_cache)
{
    const auto key = quantized_point_key(node_global);
    auto cache_it = node_cache.find(key);
    if (cache_it != node_cache.end())
    {
        return cache_it->second;
    }

    bool walkable = true;
    LLViewerRegion* agent_region = gAgent.getRegion();
    if (!agent_region || LLWorld::getInstance()->getRegionFromPosGlobal(node_global) != agent_region)
    {
        walkable = false;
    }

    if (walkable)
    {
        const F64 foot_z = node_global.mdV[VZ] - pelvis_to_foot + kCustomProbeFootClearanceMeters;
        const F64 chest_z = node_global.mdV[VZ] + kCustomProbeChestHeightMeters;

        const F64 land_z = static_cast<F64>(LLWorld::getInstance()->resolveLandHeightGlobal(node_global));
        if ((foot_z - land_z) > 3.5)
        {
            LLVector3d floor_probe_a = node_global;
            LLVector3d floor_probe_b = node_global;
            floor_probe_a.mdV[VZ] = foot_z + 0.20;
            floor_probe_b.mdV[VZ] = foot_z - 3.50;
            if (!raycast_hits_static_world(floor_probe_a, floor_probe_b))
            {
                walkable = false;
            }
        }

        if (walkable)
        {
            static const F64 kTwoPi = 6.28318530717958647692;
            static const int kRingSteps = 8;

            LLVector3d sample = node_global;
            sample.mdV[VZ] = foot_z;
            const bool center_near_obstacle = is_point_near_broad_phase(sample, obstacles);
            if (center_near_obstacle)
            {
                LLVector3d center_foot = sample;
                LLVector3d center_chest = sample;
                center_chest.mdV[VZ] = chest_z;
                if (raycast_hits_static_world(center_foot, center_chest))
                {
                    walkable = false;
                }
            }

            for (int i = 0; walkable && i < kRingSteps; ++i)
            {
                const F64 theta = (kTwoPi * i) / static_cast<F64>(kRingSteps);
                const F64 dx = std::cos(theta) * kCustomProbeAvatarRadiusMeters;
                const F64 dy = std::sin(theta) * kCustomProbeAvatarRadiusMeters;
                LLVector3d sample_point(node_global.mdV[VX] + dx, node_global.mdV[VY] + dy, node_global.mdV[VZ]);
                LLVector3d foot_point = sample_point;
                LLVector3d chest_point = sample_point;
                foot_point.mdV[VZ] = foot_z;
                chest_point.mdV[VZ] = chest_z;

                if (!is_point_near_broad_phase(foot_point, obstacles) &&
                    !is_point_near_broad_phase(chest_point, obstacles))
                {
                    continue;
                }
                if (raycast_hits_static_world(foot_point, chest_point))
                {
                    walkable = false;
                }
            }
        }
    }

    node_cache[key] = walkable;
    return walkable;
}

bool is_custom_segment_walkable(
    const LLVector3d& a,
    const LLVector3d& b,
    const std::vector<BroadPhaseObstacle>& obstacles,
    F64 pelvis_to_foot,
    std::map<std::tuple<S32, S32, S32>, bool>& node_cache)
{
    const F64 length = dist_vec(a, b);
    if (length < 0.05)
    {
        return true;
    }

    LLVector3d dir = b - a;
    LLVector3d dir2d(dir.mdV[VX], dir.mdV[VY], 0.0);
    const F64 dir_len = dir2d.length();
    LLVector3d perp(0.0, 0.0, 0.0);
    if (dir_len > 0.0001)
    {
        dir2d.normalize();
        perp.setVec(-dir2d.mdV[VY], dir2d.mdV[VX], 0.0);
    }

    static const F64 kOffsets[] = {0.0, kCustomProbeAvatarRadiusMeters * 0.9, -kCustomProbeAvatarRadiusMeters * 0.9};
    for (F64 lateral : kOffsets)
    {
        LLVector3d lateral_offset = perp * lateral;
        LLVector3d foot_a = a + lateral_offset;
        LLVector3d foot_b = b + lateral_offset;
        LLVector3d chest_a = a + lateral_offset;
        LLVector3d chest_b = b + lateral_offset;

        foot_a.mdV[VZ] = a.mdV[VZ] - pelvis_to_foot + kCustomProbeFootClearanceMeters;
        foot_b.mdV[VZ] = b.mdV[VZ] - pelvis_to_foot + kCustomProbeFootClearanceMeters;
        chest_a.mdV[VZ] = a.mdV[VZ] + kCustomProbeChestHeightMeters;
        chest_b.mdV[VZ] = b.mdV[VZ] + kCustomProbeChestHeightMeters;

        const bool foot_blocked = raycast_hits_static_world(foot_a, foot_b);
        const bool chest_blocked = raycast_hits_static_world(chest_a, chest_b);
        if (foot_blocked || chest_blocked)
        {
            return false;
        }
    }

    const F64 sample_step = kCustomEdgeSampleMeters;
    const S32 sample_count = llclamp(static_cast<S32>(std::ceil(length / sample_step)), 1, 96);
    for (S32 i = 1; i < sample_count; ++i)
    {
        const F64 t = static_cast<F64>(i) / static_cast<F64>(sample_count);
        const LLVector3d sample = a + ((b - a) * t);
        if (!is_custom_node_walkable(sample, obstacles, pelvis_to_foot, node_cache))
        {
            return false;
        }
    }
    return true;
}

bool has_direct_clearance_route(const LLVector3d& start, const LLVector3d& goal)
{
    LLViewerRegion* agent_region = gAgent.getRegion();
    if (!agent_region)
    {
        return false;
    }

    std::vector<BroadPhaseObstacle> obstacles;
    gather_broad_phase_obstacles(start, goal, obstacles);
    std::map<std::tuple<S32, S32, S32>, bool> node_cache;
    const F64 pelvis_to_foot = agent_pelvis_to_foot_meters();
    return is_custom_segment_walkable(start, goal, obstacles, pelvis_to_foot, node_cache);
}

void log_pathfinding_diagnostics(const char* context)
{
    LLViewerRegion* region = gAgent.getRegion();
    LLPathfindingManager* manager = LLPathfindingManager::getInstance();
    LLPathingLib* pathing = LLPathingLib::getInstance();

    if (!region)
    {
        LL_INFOS("AI") << "[AI] Pathfinding diag (" << context << "): no current region"
                       << " manager=" << bool_to_text(manager != nullptr)
                       << " pathing_lib=" << bool_to_text(pathing != nullptr)
                       << LL_ENDL;
        return;
    }

    const bool caps_received = region->capabilitiesReceived();
    const bool sim_features_received = region->simulatorFeaturesReceived();
    const bool dynamic_enabled = region->dynamicPathfindingEnabled();
    const bool manager_enabled = manager && manager->isPathfindingEnabledForCurrentRegion();
    const bool pathing_functional = pathing && pathing->isFunctional();

    std::string retrieve_navmesh_cap;
    std::string navmesh_status_cap;
    std::string character_props_cap;
    if (caps_received)
    {
        retrieve_navmesh_cap = region->getCapability("RetrieveNavMeshSrc");
        navmesh_status_cap = region->getCapability("NavMeshGenerationStatus");
        character_props_cap = region->getCapability("CharacterProperties");
    }

    std::string reason = "ready";
    if (!manager)
    {
        reason = "missing_llpathfindingmanager";
    }
    else if (!caps_received)
    {
        reason = "region_capabilities_not_received";
    }
    else if (!manager_enabled)
    {
        reason = "manager_reports_not_enabled_for_region";
    }
    else if (!pathing)
    {
        reason = "missing_llpathinglib";
    }
    else if (!pathing_functional)
    {
        reason = "llpathinglib_not_functional";
    }

    LL_INFOS("AI") << "[AI] Pathfinding diag (" << context << "): "
                   << "region=\"" << region->getName() << "\" "
                   << "region_id=" << region->getRegionID().asString() << " "
                   << "caps_received=" << bool_to_text(caps_received) << " "
                   << "sim_features_received=" << bool_to_text(sim_features_received) << " "
                   << "DynamicPathfindingEnabled=" << bool_to_text(dynamic_enabled) << " "
                   << "RetrieveNavMeshSrc_cap=" << bool_to_text(!retrieve_navmesh_cap.empty()) << " "
                   << "NavMeshGenerationStatus_cap=" << bool_to_text(!navmesh_status_cap.empty()) << " "
                   << "CharacterProperties_cap=" << bool_to_text(!character_props_cap.empty()) << " "
                   << "manager_present=" << bool_to_text(manager != nullptr) << " "
                   << "manager_enabled=" << bool_to_text(manager_enabled) << " "
                   << "pathinglib_present=" << bool_to_text(pathing != nullptr) << " "
                   << "pathinglib_functional=" << bool_to_text(pathing_functional) << " "
                   << "reason=" << reason
                   << LL_ENDL;
}

bool is_pathfinding_ready()
{
    LLPathfindingManager* manager = LLPathfindingManager::getInstance();
    const bool manager_enabled = manager && manager->isPathfindingEnabledForCurrentRegion();
    LLPathingLib* pathing = LLPathingLib::getInstance();
    const bool pathing_functional = (pathing != nullptr) && pathing->isFunctional();
    const bool ready = manager_enabled && pathing_functional;

    log_pathfinding_diagnostics("is_pathfinding_ready");

    if (!ready)
    {
        return false;
    }
    return true;
}

bool probe_path_exists(const LLVector3d& from_global, const LLVector3d& to_global)
{
    LLPathingLib* pathing = LLPathingLib::getInstance();
    if (!pathing)
    {
        return false;
    }

    // LLPathingLib does not expose waypoint extraction here; use this as a route-validity probe.
    LLPathingLib::PathingPacket packet;
    LLVector3 from_top((F32)from_global.mdV[0], (F32)from_global.mdV[1], (F32)(from_global.mdV[2] + kPathProbeRayHeightMeters));
    LLVector3 from_bottom((F32)from_global.mdV[0], (F32)from_global.mdV[1], (F32)(from_global.mdV[2] - kPathProbeRayHeightMeters));
    LLVector3 to_top((F32)to_global.mdV[0], (F32)to_global.mdV[1], (F32)(to_global.mdV[2] + kPathProbeRayHeightMeters));
    LLVector3 to_bottom((F32)to_global.mdV[0], (F32)to_global.mdV[1], (F32)(to_global.mdV[2] - kPathProbeRayHeightMeters));
    packet.mStartPointA = from_top;
    packet.mEndPointA = from_bottom;
    packet.mStartPointB = to_top;
    packet.mEndPointB = to_bottom;
    packet.mHasPointA = true;
    packet.mHasPointB = true;
    packet.mCharacterWidth = 1.0;
    packet.mCharacterType = LLPathingLib::LLPL_CHARACTER_TYPE_NONE;
    return pathing->generatePath(packet) == LLPathingLib::LLPL_PATH_GENERATED_OK;
}

void add_unique_node(std::vector<LLVector3d>& nodes, const LLVector3d& candidate, F64 min_spacing = 0.9)
{
    for (const LLVector3d& existing : nodes)
    {
        if (dist_vec(existing, candidate) < min_spacing)
        {
            return;
        }
    }
    nodes.push_back(candidate);
}

std::vector<LLVector3d> build_probe_nodes(const LLVector3d& start, const LLVector3d& goal)
{
    std::vector<LLVector3d> nodes;
    nodes.reserve(96);
    add_unique_node(nodes, start, 0.01);
    add_unique_node(nodes, goal, 0.01);

    static const F64 kTwoPi = 6.28318530717958647692;
    static const F32 kRingRadii[] = {2.0f, 4.0f, 6.5f};
    static const int kRingSteps = 10;

    const LLVector3d centers[2] = {start, goal};
    for (const LLVector3d& center : centers)
    {
        for (F32 radius : kRingRadii)
        {
            for (int i = 0; i < kRingSteps; ++i)
            {
                const F64 theta = (kTwoPi * i) / static_cast<F64>(kRingSteps);
                LLVector3d candidate(
                    center.mdV[0] + std::cos(theta) * radius,
                    center.mdV[1] + std::sin(theta) * radius,
                    center.mdV[2]
                );
                add_unique_node(nodes, candidate);
            }
        }
    }

    // Corridor samples between start and goal with lateral offsets.
    LLVector3d delta = goal - start;
    LLVector3d dir2d(delta.mdV[0], delta.mdV[1], 0.0);
    const F64 dir_mag = dir2d.length();
    if (dir_mag > 0.001)
    {
        dir2d.normalize();
        LLVector3d perp(-dir2d.mdV[1], dir2d.mdV[0], 0.0);
        static const F32 kTValues[] = {0.2f, 0.35f, 0.5f, 0.65f, 0.8f};
        static const F32 kOffsets[] = {-5.0f, -2.5f, 0.0f, 2.5f, 5.0f};
        for (F32 t : kTValues)
        {
            LLVector3d center = start + (delta * t);
            for (F32 offset : kOffsets)
            {
                LLVector3d candidate = center + (perp * offset);
                add_unique_node(nodes, candidate);
            }
        }
    }

    return nodes;
}

std::vector<int> nearest_neighbor_indices(const std::vector<LLVector3d>& nodes, int index, int k)
{
    std::vector<std::pair<F64, int>> distances;
    distances.reserve(nodes.size() > 0 ? nodes.size() - 1 : 0);
    for (int j = 0; j < static_cast<int>(nodes.size()); ++j)
    {
        if (j == index)
        {
            continue;
        }
        distances.emplace_back(dist_vec(nodes[index], nodes[j]), j);
    }
    std::sort(distances.begin(), distances.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

    std::vector<int> out;
    out.reserve(k);
    for (int i = 0; i < static_cast<int>(distances.size()) && i < k; ++i)
    {
        out.push_back(distances[i].second);
    }
    return out;
}

std::vector<int> astar_path(
    const std::vector<std::vector<std::pair<int, F64>>>& adjacency,
    const std::vector<LLVector3d>& nodes,
    int start_idx,
    int goal_idx)
{
    const size_t count = nodes.size();
    if (count == 0)
    {
        return {};
    }

    std::vector<F64> g_score(count, std::numeric_limits<F64>::infinity());
    std::vector<int> came_from(count, -1);
    std::vector<bool> closed(count, false);

    struct OpenNode
    {
        F64 f_score;
        int node;
        bool operator<(const OpenNode& other) const { return f_score > other.f_score; }
    };
    std::priority_queue<OpenNode> open_set;

    g_score[start_idx] = 0.0;
    open_set.push({dist_vec(nodes[start_idx], nodes[goal_idx]), start_idx});

    while (!open_set.empty())
    {
        const int current = open_set.top().node;
        open_set.pop();
        if (closed[current])
        {
            continue;
        }
        if (current == goal_idx)
        {
            std::vector<int> path;
            int node = goal_idx;
            while (node != -1)
            {
                path.push_back(node);
                node = came_from[node];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        closed[current] = true;

        for (const auto& edge : adjacency[current])
        {
            const int neighbor = edge.first;
            const F64 cost = edge.second;
            if (closed[neighbor])
            {
                continue;
            }
            const F64 tentative = g_score[current] + cost;
            if (tentative >= g_score[neighbor])
            {
                continue;
            }
            came_from[neighbor] = current;
            g_score[neighbor] = tentative;
            const F64 f = tentative + dist_vec(nodes[neighbor], nodes[goal_idx]);
            open_set.push({f, neighbor});
        }
    }

    return {};
}

bool plan_probe_route(const LLVector3d& start, const LLVector3d& goal, std::vector<LLVector3d>& waypoints_out, bool& direct_out)
{
    waypoints_out.clear();
    direct_out = false;

    if (probe_path_exists(start, goal))
    {
        direct_out = true;
        waypoints_out.push_back(goal);
        LL_INFOS("AI") << "[AI] Path planner: direct path probe succeeded; single-leg route" << LL_ENDL;
        return true;
    }

    std::vector<LLVector3d> nodes = build_probe_nodes(start, goal);
    if (nodes.size() < 2)
    {
        return false;
    }

    const int start_idx = 0;
    const int goal_idx = 1;
    const int kNeighbors = 10;
    std::vector<std::vector<std::pair<int, F64>>> adjacency(nodes.size());
    std::map<U64, bool> edge_probe_cache;
    size_t edge_probe_calls = 0;
    size_t edge_probe_hits = 0;
    size_t edge_probe_successes = 0;
    auto edge_key = [](int a, int b) -> U64 {
        const U64 lo = static_cast<U64>(std::min(a, b));
        const U64 hi = static_cast<U64>(std::max(a, b));
        return (lo << 32) | hi;
    };
    auto probe_edge = [&](int a, int b) -> bool {
        const U64 key = edge_key(a, b);
        auto it = edge_probe_cache.find(key);
        if (it != edge_probe_cache.end())
        {
            ++edge_probe_hits;
            return it->second;
        }
        ++edge_probe_calls;
        const bool ok = probe_path_exists(nodes[a], nodes[b]);
        edge_probe_cache[key] = ok;
        if (ok)
        {
            ++edge_probe_successes;
        }
        return ok;
    };

    for (int i = 0; i < static_cast<int>(nodes.size()); ++i)
    {
        std::vector<int> neighbors = nearest_neighbor_indices(nodes, i, kNeighbors);
        for (int j : neighbors)
        {
            if (i >= j)
            {
                continue;
            }
            if (!probe_edge(i, j))
            {
                continue;
            }
            const F64 cost = dist_vec(nodes[i], nodes[j]);
            adjacency[i].push_back({j, cost});
            adjacency[j].push_back({i, cost});
        }
    }

    std::vector<int> path = astar_path(adjacency, nodes, start_idx, goal_idx);
    if (path.size() < 2)
    {
        LL_INFOS("AI") << "[AI] Path planner: no A* route. nodes=" << nodes.size()
                       << " edge_probes=" << edge_probe_calls
                       << " edge_cache_hits=" << edge_probe_hits
                       << " edge_successes=" << edge_probe_successes
                       << LL_ENDL;
        return false;
    }

    // Compress path by skipping waypoints when direct probe is valid.
    std::vector<int> compressed;
    compressed.push_back(path.front());
    size_t cursor = 0;
    while (cursor < path.size() - 1)
    {
        size_t best_next = cursor + 1;
        for (size_t j = path.size() - 1; j > cursor; --j)
        {
            if (probe_edge(path[cursor], path[j]))
            {
                best_next = j;
                break;
            }
        }
        compressed.push_back(path[best_next]);
        cursor = best_next;
    }

    for (size_t i = 1; i < compressed.size(); ++i)
    {
        const LLVector3d& wp = nodes[compressed[i]];
        if (!waypoints_out.empty() && dist_vec(waypoints_out.back(), wp) < 0.5)
        {
            continue;
        }
        waypoints_out.push_back(wp);
    }

    if (waypoints_out.empty())
    {
        LL_INFOS("AI") << "[AI] Path planner: compressed route had no usable waypoints"
                       << " path_nodes=" << path.size()
                       << " edge_probes=" << edge_probe_calls
                       << " edge_cache_hits=" << edge_probe_hits
                       << " edge_successes=" << edge_probe_successes
                       << LL_ENDL;
        return false;
    }
    if (dist_vec(waypoints_out.back(), goal) > 0.5)
    {
        waypoints_out.push_back(goal);
    }

    LL_INFOS("AI") << "[AI] Path planner: route ready"
                   << " nodes=" << nodes.size()
                   << " path_nodes=" << path.size()
                   << " waypoints=" << waypoints_out.size()
                   << " edge_probes=" << edge_probe_calls
                   << " edge_cache_hits=" << edge_probe_hits
                   << " edge_successes=" << edge_probe_successes
                   << LL_ENDL;

    return true;
}

bool plan_custom_clearance_route(
    const LLVector3d& start,
    const LLVector3d& goal,
    std::vector<LLVector3d>& waypoints_out,
    bool& direct_out,
    bool log_diagnostics = true)
{
    waypoints_out.clear();
    direct_out = false;

    LLViewerRegion* agent_region = gAgent.getRegion();
    if (!agent_region)
    {
        return false;
    }

    const F64 pelvis_to_foot = (gAgentAvatarp != nullptr)
        ? llmax(static_cast<F64>(gAgentAvatarp->getPelvisToFoot()), 0.65)
        : 1.0;

    std::vector<BroadPhaseObstacle> obstacles;
    gather_broad_phase_obstacles(start, goal, obstacles);

    std::map<std::tuple<S32, S32, S32>, bool> node_cache;
    auto node_walkable = [&](const LLVector3d& node) -> bool {
        return is_custom_node_walkable(node, obstacles, pelvis_to_foot, node_cache);
    };
    auto segment_walkable = [&](const LLVector3d& a, const LLVector3d& b) -> bool {
        return is_custom_segment_walkable(a, b, obstacles, pelvis_to_foot, node_cache);
    };

    if (segment_walkable(start, goal))
    {
        direct_out = true;
        waypoints_out.push_back(goal);
        if (log_diagnostics)
        {
            LL_INFOS("AI") << "[AI] Custom planner: direct clearance route succeeded; single-leg route"
                           << " obstacles=" << obstacles.size()
                           << LL_ENDL;
        }
        return true;
    }

    std::vector<LLVector3d> nodes = build_probe_nodes(start, goal);

    // Add denser corridor samples so hallways survive coarse mesh/object metadata.
    LLVector3d delta = goal - start;
    LLVector3d dir2d(delta.mdV[VX], delta.mdV[VY], 0.0);
    const F64 dir_mag = dir2d.length();
    if (dir_mag > 0.001)
    {
        dir2d.normalize();
        LLVector3d perp(-dir2d.mdV[VY], dir2d.mdV[VX], 0.0);
        static const F32 kTValues[] = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f};
        static const F32 kOffsets[] = {-8.0f, -6.0f, -4.0f, -2.0f, 0.0f, 2.0f, 4.0f, 6.0f, 8.0f};
        for (F32 t : kTValues)
        {
            LLVector3d center = start + (delta * t);
            for (F32 offset : kOffsets)
            {
                LLVector3d candidate = center + (perp * offset);
                add_unique_node(nodes, candidate, kCustomNodeSpacingMeters);
            }
        }
    }

    std::vector<LLVector3d> walkable_nodes;
    walkable_nodes.reserve(nodes.size());
    size_t rejected_nodes = 0;
    for (size_t i = 0; i < nodes.size(); ++i)
    {
        const LLVector3d& candidate = nodes[i];
        if ((i < 2) || node_walkable(candidate))
        {
            add_unique_node(walkable_nodes, candidate, (i < 2) ? 0.01 : kCustomNodeSpacingMeters);
        }
        else
        {
            ++rejected_nodes;
        }
    }

    if (walkable_nodes.size() < 2)
    {
        if (log_diagnostics)
        {
            LL_INFOS("AI") << "[AI] Custom planner: no walkable nodes after clearance filtering"
                           << " candidates=" << nodes.size()
                           << " rejected=" << rejected_nodes
                           << LL_ENDL;
        }
        return false;
    }

    const int start_idx = 0;
    const int goal_idx = 1;
    const int kNeighbors = 12;
    std::vector<std::vector<std::pair<int, F64>>> adjacency(walkable_nodes.size());
    std::map<U64, bool> edge_cache;
    size_t edge_tests = 0;
    size_t edge_hits = 0;
    size_t edge_successes = 0;
    auto edge_key = [](int a, int b) -> U64 {
        const U64 lo = static_cast<U64>(std::min(a, b));
        const U64 hi = static_cast<U64>(std::max(a, b));
        return (lo << 32) | hi;
    };
    auto probe_edge = [&](int a, int b) -> bool {
        const U64 key = edge_key(a, b);
        auto it = edge_cache.find(key);
        if (it != edge_cache.end())
        {
            ++edge_hits;
            return it->second;
        }
        ++edge_tests;
        const bool ok = segment_walkable(walkable_nodes[a], walkable_nodes[b]);
        edge_cache[key] = ok;
        if (ok)
        {
            ++edge_successes;
        }
        return ok;
    };

    for (int i = 0; i < static_cast<int>(walkable_nodes.size()); ++i)
    {
        std::vector<int> neighbors = nearest_neighbor_indices(walkable_nodes, i, kNeighbors);
        for (int j : neighbors)
        {
            if (i >= j)
            {
                continue;
            }
            if (!probe_edge(i, j))
            {
                continue;
            }
            const F64 cost = dist_vec(walkable_nodes[i], walkable_nodes[j]);
            adjacency[i].push_back({j, cost});
            adjacency[j].push_back({i, cost});
        }
    }

    std::vector<int> path = astar_path(adjacency, walkable_nodes, start_idx, goal_idx);
    if (path.size() < 2)
    {
        if (log_diagnostics)
        {
            LL_INFOS("AI") << "[AI] Custom planner: no A* route"
                           << " nodes=" << walkable_nodes.size()
                           << " obstacles=" << obstacles.size()
                           << " rejected_nodes=" << rejected_nodes
                           << " edge_tests=" << edge_tests
                           << " edge_cache_hits=" << edge_hits
                           << " edge_successes=" << edge_successes
                           << LL_ENDL;
        }
        return false;
    }

    std::vector<int> compressed;
    compressed.push_back(path.front());
    size_t cursor = 0;
    while (cursor < path.size() - 1)
    {
        size_t best_next = cursor + 1;
        for (size_t j = path.size() - 1; j > cursor; --j)
        {
            if (probe_edge(path[cursor], path[j]))
            {
                best_next = j;
                break;
            }
        }
        compressed.push_back(path[best_next]);
        cursor = best_next;
    }

    for (size_t i = 1; i < compressed.size(); ++i)
    {
        const LLVector3d& wp = walkable_nodes[compressed[i]];
        if (!waypoints_out.empty() && dist_vec(waypoints_out.back(), wp) < 0.5)
        {
            continue;
        }
        waypoints_out.push_back(wp);
    }

    if (waypoints_out.empty())
    {
        if (log_diagnostics)
        {
            LL_INFOS("AI") << "[AI] Custom planner: compressed route had no waypoints"
                           << " path_nodes=" << path.size()
                           << LL_ENDL;
        }
        return false;
    }
    if (dist_vec(waypoints_out.back(), goal) > 0.5)
    {
        waypoints_out.push_back(goal);
    }

    if (log_diagnostics)
    {
        LL_INFOS("AI") << "[AI] Custom planner: route ready"
                       << " obstacles=" << obstacles.size()
                       << " nodes=" << walkable_nodes.size()
                       << " rejected_nodes=" << rejected_nodes
                       << " path_nodes=" << path.size()
                       << " waypoints=" << waypoints_out.size()
                       << " edge_tests=" << edge_tests
                       << " edge_cache_hits=" << edge_hits
                       << " edge_successes=" << edge_successes
                       << LL_ENDL;
    }
    return true;
}

void ai_move_waypoint_callback(bool success, void* user_data);

void start_move_path_plan(
    U64 plan_id,
    const std::vector<LLVector3d>& waypoints,
    U32 replans_remaining = kMovePathReplanAttempts,
    const LLVector3d* final_goal_override = nullptr,
    const LLUUID* follow_target_id_override = nullptr,
    F32 follow_standoff = kDefaultMoveStandoffMeters,
    F32 final_goal_tolerance = kMoveAutopilotStopDistanceMeters)
{
    if (waypoints.empty())
    {
        return;
    }
    ++g_move_planner_telemetry.waypoint_plan_starts;
    MoveWaypointCallbackData* cb_data = new MoveWaypointCallbackData();
    cb_data->plan_id = plan_id;
    cb_data->waypoints = waypoints;
    cb_data->next_index = 1;
    cb_data->final_goal = final_goal_override ? *final_goal_override : waypoints.back();
    cb_data->final_goal_tolerance = llmax(final_goal_tolerance, kMoveAutopilotStopDistanceMeters);
    cb_data->replans_remaining = replans_remaining;
    cb_data->follow_target_id = follow_target_id_override ? *follow_target_id_override : LLUUID::null;
    cb_data->follow_standoff = clamp_move_standoff(follow_standoff);
    gAgent.startAutoPilotGlobal(
        waypoints[0],
        "AI Movement Waypoint",
        NULL,
        ai_move_waypoint_callback,
        cb_data,
        kMoveAutopilotStopDistanceMeters,
        kMoveAutopilotRotationThreshold,
        false
    );
}

void ai_move_waypoint_callback(bool success, void* user_data)
{
    MoveWaypointCallbackData* cb_data = static_cast<MoveWaypointCallbackData*>(user_data);
    if (!cb_data)
    {
        return;
    }

    const U64 callback_plan_id = cb_data->plan_id;
    const std::vector<LLVector3d> waypoints = cb_data->waypoints;
    const size_t next_index = cb_data->next_index;
    const LLVector3d final_goal = cb_data->final_goal;
    const F32 final_goal_tolerance = cb_data->final_goal_tolerance;
    const U32 replans_remaining = cb_data->replans_remaining;
    const LLUUID follow_target_id = cb_data->follow_target_id;
    const F32 follow_standoff = cb_data->follow_standoff;
    delete cb_data;

    if (callback_plan_id != g_move_plan_generation)
    {
        return; // stale plan
    }

    // Avatar-targeted plans switch to follow-pilot as soon as LoS (or standoff) is good enough.
    if (!follow_target_id.isNull())
    {
        LLVector3d avatar_pos;
        if (resolve_avatar_position(follow_target_id, avatar_pos))
        {
            const LLVector3d current_pos = gAgent.getPositionGlobal();
            const F64 avatar_distance = dist_vec(current_pos, avatar_pos);
            const bool los_clear = has_clear_line_of_sight_to_avatar(current_pos, avatar_pos);
            if (avatar_distance <= follow_standoff || los_clear)
            {
                ++g_move_planner_telemetry.person_follow_requests;
                ++g_move_planner_telemetry.avatar_hybrid_follow_switch;
                gAgent.stopAutoPilot();
                const U64 follow_plan_id = callback_plan_id;
                doOnIdleOneTime([follow_target_id, follow_standoff, follow_plan_id]()
                {
                    if (follow_plan_id != g_move_plan_generation)
                    {
                        return;
                    }
                    gAgent.startFollowPilot(follow_target_id, false, follow_standoff);
                });
                LL_INFOS("AI") << "[AI] Avatar LoS/standoff reached during waypoint plan; switching to follow pilot target="
                               << follow_target_id.asString()
                               << " dist=" << avatar_distance
                               << " los=" << (los_clear ? "yes" : "no")
                               << LL_ENDL;
                maybe_log_move_planner_summary("ai_move_waypoint_callback");
                return;
            }
        }
    }

    if (!success)
    {
        ++g_move_planner_telemetry.waypoint_leg_failures;
        const LLVector3d current_pos = gAgent.getPositionGlobal();
        const F64 dist_to_final = dist_vec(current_pos, final_goal);
        if (follow_target_id.isNull() && dist_to_final <= final_goal_tolerance)
        {
            ++g_move_planner_telemetry.waypoint_plan_completions;
            LL_INFOS("AI") << "[AI] Waypoint leg reported failure, but final goal tolerance met; accepting completion"
                           << " dist_to_final=" << dist_to_final
                           << " tolerance=" << final_goal_tolerance
                           << LL_ENDL;
            maybe_log_move_planner_summary("ai_move_waypoint_callback");
            return;
        }

        if (replans_remaining > 0)
        {
            ++g_move_planner_telemetry.replans_attempted;
            std::vector<LLVector3d> replanned_waypoints;
            bool direct_path = false;
            LLVector3d replan_goal = final_goal;
            if (!follow_target_id.isNull())
            {
                LLVector3d avatar_pos;
                if (resolve_avatar_position(follow_target_id, avatar_pos))
                {
                    replan_goal = compute_avatar_approach_goal(current_pos, avatar_pos, follow_standoff);
                }
            }
            bool replanned = plan_custom_clearance_route(current_pos, replan_goal, replanned_waypoints, direct_path);
            if (!replanned && is_pathfinding_ready())
            {
                replanned = plan_probe_route(current_pos, replan_goal, replanned_waypoints, direct_path);
            }

            if (replanned && !replanned_waypoints.empty())
            {
                ++g_move_planner_telemetry.replans_succeeded;
                const U64 new_plan_id = ++g_move_plan_generation;
                const std::vector<LLVector3d> replanned_copy = replanned_waypoints;
                const LLVector3d replan_goal_copy = replan_goal;
                const LLUUID follow_target_copy = follow_target_id;
                doOnIdleOneTime([new_plan_id, replanned_copy, replans_remaining, replan_goal_copy, follow_target_copy, follow_standoff, final_goal_tolerance]()
                {
                    if (new_plan_id != g_move_plan_generation)
                    {
                        return;
                    }
                    start_move_path_plan(
                        new_plan_id,
                        replanned_copy,
                        replans_remaining - 1,
                        &replan_goal_copy,
                        follow_target_copy.isNull() ? nullptr : &follow_target_copy,
                        follow_standoff,
                        final_goal_tolerance);
                });
                LL_INFOS("AI") << "[AI] Waypoint leg failed; replanned route to goal ("
                               << replan_goal.mdV[0] << ", " << replan_goal.mdV[1] << ", " << replan_goal.mdV[2]
                               << ") remaining_replans=" << (replans_remaining - 1)
                               << LL_ENDL;
                return;
            }
            ++g_move_planner_telemetry.replans_failed;
        }

        if (!follow_target_id.isNull())
        {
            LLVector3d avatar_pos;
            const LLVector3d current_pos = gAgent.getPositionGlobal();
            const bool can_resolve = resolve_avatar_position(follow_target_id, avatar_pos);
            const bool los_clear = can_resolve && has_clear_line_of_sight_to_avatar(current_pos, avatar_pos);
            const F64 avatar_distance = can_resolve ? dist_vec(current_pos, avatar_pos) : std::numeric_limits<F64>::max();

            if (can_resolve && (avatar_distance <= follow_standoff || los_clear))
            {
                ++g_move_planner_telemetry.person_follow_requests;
                ++g_move_planner_telemetry.avatar_hybrid_follow_fallback;
                gAgent.stopAutoPilot();
                const U64 follow_plan_id = callback_plan_id;
                doOnIdleOneTime([follow_target_id, follow_standoff, follow_plan_id]()
                {
                    if (follow_plan_id != g_move_plan_generation)
                    {
                        return;
                    }
                    gAgent.startFollowPilot(follow_target_id, false, follow_standoff);
                });
                LL_WARNS("AI") << "[AI] Waypoint leg failed with no replan, but direct avatar follow is clear; switching to follow pilot target="
                               << follow_target_id.asString()
                               << " dist=" << avatar_distance
                               << " los=" << (los_clear ? "yes" : "no")
                               << LL_ENDL;
                maybe_log_move_planner_summary("ai_move_waypoint_callback");
                return;
            }

            ++g_move_planner_telemetry.blocked_abort_avatar;
            gAgent.stopAutoPilot();
            LL_WARNS("AI") << "[AI] Waypoint leg failed with no replan and avatar LoS blocked; aborting MOVE target="
                           << follow_target_id.asString()
                           << " can_resolve=" << (can_resolve ? "yes" : "no")
                           << " dist=" << avatar_distance
                           << " los=" << (los_clear ? "yes" : "no")
                           << LL_ENDL;
            maybe_log_move_planner_summary("ai_move_waypoint_callback");
            return;
        }

        LL_WARNS("AI") << "[AI] Waypoint leg did not complete; no replan available" << LL_ENDL;
        maybe_log_move_planner_summary("ai_move_waypoint_callback");
        return;
    }

    if (next_index >= waypoints.size())
    {
        ++g_move_planner_telemetry.waypoint_plan_completions;
        if (!follow_target_id.isNull())
        {
            LLVector3d current_pos = gAgent.getPositionGlobal();
            LLVector3d avatar_pos;
            bool can_resolve = resolve_avatar_position(follow_target_id, avatar_pos);
            bool los_clear = can_resolve && has_clear_line_of_sight_to_avatar(current_pos, avatar_pos);
            const F64 avatar_distance = can_resolve ? dist_vec(current_pos, avatar_pos) : std::numeric_limits<F64>::max();

            if (can_resolve && !los_clear && (avatar_distance > follow_standoff) && (replans_remaining > 0))
            {
                ++g_move_planner_telemetry.replans_attempted;
                const LLVector3d replan_goal = compute_avatar_approach_goal(current_pos, avatar_pos, follow_standoff);
                std::vector<LLVector3d> replanned_waypoints;
                bool direct_path = false;
                bool replanned = plan_custom_clearance_route(current_pos, replan_goal, replanned_waypoints, direct_path);
                if (!replanned && is_pathfinding_ready())
                {
                    replanned = plan_probe_route(current_pos, replan_goal, replanned_waypoints, direct_path);
                }
                if (replanned && !replanned_waypoints.empty())
                {
                    ++g_move_planner_telemetry.replans_succeeded;
                    const U64 new_plan_id = ++g_move_plan_generation;
                    const std::vector<LLVector3d> replanned_copy = replanned_waypoints;
                    const LLVector3d replan_goal_copy = replan_goal;
                    const LLUUID follow_target_copy = follow_target_id;
                    doOnIdleOneTime([new_plan_id, replanned_copy, replans_remaining, replan_goal_copy, follow_target_copy, follow_standoff, final_goal_tolerance]()
                    {
                        if (new_plan_id != g_move_plan_generation)
                        {
                            return;
                        }
                        start_move_path_plan(
                            new_plan_id,
                            replanned_copy,
                            replans_remaining - 1,
                            &replan_goal_copy,
                            &follow_target_copy,
                            follow_standoff,
                            final_goal_tolerance);
                    });
                    LL_INFOS("AI") << "[AI] Waypoint plan completed but avatar LoS still blocked; replanning toward updated approach goal"
                                   << " remaining_replans=" << (replans_remaining - 1)
                                   << LL_ENDL;
                    return;
                }
                ++g_move_planner_telemetry.replans_failed;
            }

            if (can_resolve && (avatar_distance <= follow_standoff || los_clear))
            {
                ++g_move_planner_telemetry.person_follow_requests;
                ++g_move_planner_telemetry.avatar_hybrid_follow_switch;
                gAgent.stopAutoPilot();
                const U64 follow_plan_id = callback_plan_id;
                doOnIdleOneTime([follow_target_id, follow_standoff, follow_plan_id]()
                {
                    if (follow_plan_id != g_move_plan_generation)
                    {
                        return;
                    }
                    gAgent.startFollowPilot(follow_target_id, false, follow_standoff);
                });
                LL_INFOS("AI") << "[AI] Avatar waypoint approach complete; switching to follow pilot target="
                               << follow_target_id.asString()
                               << " dist=" << avatar_distance
                               << " los=" << (los_clear ? "yes" : "no")
                               << LL_ENDL;
                maybe_log_move_planner_summary("ai_move_waypoint_callback");
                return;
            }

            ++g_move_planner_telemetry.blocked_abort_avatar;
            gAgent.stopAutoPilot();
            LL_WARNS("AI") << "[AI] Avatar waypoint approach complete but follow conditions are not met; aborting MOVE target="
                           << follow_target_id.asString()
                           << " can_resolve=" << (can_resolve ? "yes" : "no")
                           << " dist=" << avatar_distance
                           << " los=" << (los_clear ? "yes" : "no")
                           << LL_ENDL;
            maybe_log_move_planner_summary("ai_move_waypoint_callback");
            return;
        }
        LL_INFOS("AI") << "[AI] Multi-waypoint movement plan completed" << LL_ENDL;
        maybe_log_move_planner_summary("ai_move_waypoint_callback");
        return;
    }

    MoveWaypointCallbackData* next_cb = new MoveWaypointCallbackData();
    next_cb->plan_id = callback_plan_id;
    next_cb->waypoints = waypoints;
    next_cb->next_index = next_index + 1;
    next_cb->final_goal = final_goal;
    next_cb->final_goal_tolerance = final_goal_tolerance;
    next_cb->replans_remaining = replans_remaining;
    next_cb->follow_target_id = follow_target_id;
    next_cb->follow_standoff = follow_standoff;

    const LLVector3d next_target = waypoints[next_index];
    const size_t leg_index = next_index + 1;
    const size_t total_legs = waypoints.size();
    doOnIdleOneTime([next_cb, next_target, leg_index, total_legs]()
    {
        if (!next_cb)
        {
            return;
        }
        if (next_cb->plan_id != g_move_plan_generation)
        {
            delete next_cb;
            return;
        }

        gAgent.startAutoPilotGlobal(
            next_target,
            "AI Movement Waypoint",
            NULL,
            ai_move_waypoint_callback,
            next_cb,
            kMoveAutopilotStopDistanceMeters,
            kMoveAutopilotRotationThreshold,
            false
        );

        LL_INFOS("AI") << "[AI] Waypoint leg " << leg_index << "/" << total_legs
                       << " started to (" << next_target.mdV[0] << ", " << next_target.mdV[1] << ", " << next_target.mdV[2] << ")"
                       << LL_ENDL;
    });
}
}

FSAIIntegration::FSAIIntegration()
    : mAIBridge(std::make_unique<AIProcessBridge>())
    , mMonitoringEnabled(false)
    , mCommandHandlersInitialized(false)
    , mUserIdSet(false)
    , mCurrentUserId("")
    , mPersonaSet(false)
    , mCurrentPersona("")
    , mEnvironmentUpdateCount(0)
    , mMessageProcessCount(0)
    , mLastAIEnabledState(false)
    , mWaitingForReconnection(false)
    , mLLMChatEnabled(true)
{
    LL_INFOS("AI") << "🔥 FSAIIntegration created with HOT RESTART capability!" << LL_ENDL;
    
    // Initialize state tracking
    mLastAIEnabledState = isAIEnabled();
    
    // Initialize the process bridge
    if (mAIBridge->initialize()) {
        LL_INFOS("AI") << "✅ AI Process Bridge initialized successfully" << LL_ENDL;
        
        // Initialize command handlers immediately
        initializeCommandHandlers();
    } else {
        LL_ERRS("AI") << "💥 Failed to initialize AI Process Bridge" << LL_ENDL;
    }
}

FSAIIntegration::~FSAIIntegration()
{
    LL_INFOS("AI") << "FSAIIntegration destroyed" << LL_ENDL;
    
    if (mAIBridge) {
        mAIBridge->shutdown();
    }
}

void FSAIIntegration::processMessage(const LLChat& chat)
{
    if (!isAIEnabled()) {
        return;
    }

    try {
        // Skip processing if persona not set up yet (happens during login-time setup)
        if (!mPersonaSet) {
            LL_INFOS("AI") << "[AI] Persona not yet set up, skipping message processing" << LL_ENDL;
            return;
        }
        
        // Get logged in user name
        std::string logged_in_agent;
        if (LLAvatarName av_name; LLAvatarNameCache::get(gAgent.getID(), &av_name)) {
            logged_in_agent = av_name.getCompleteName();
        }

        // Process through AI controller bridge
        std::string from_id_str = chat.mFromID.asString();
        
        LL_INFOS("AI") << "[AI] Processing message: from=" << chat.mFromName 
                       << " id=" << from_id_str << " text=" << chat.mText << LL_ENDL;
        
        // Use new process bridge for hot-restart capability
        if (mAIBridge && mAIBridge->isInitialized()) {
            std::string spatial_context = ""; // Empty spatial context - handled by bridge environment monitoring
            mAIBridge->sendChatMessage(
                chat.mText,
                chat.mFromName,
                from_id_str,
                spatial_context,
                logged_in_agent
            );
        } else {
            LL_WARNS("AI") << "[AI] Process bridge not initialized, cannot process message" << LL_ENDL;
        }

        mMessageProcessCount++;

    } catch (const std::exception& e) {
        LL_WARNS("AI") << "[AI] Error processing message: " << e.what() << LL_ENDL;
    }
}

void FSAIIntegration::updateEnvironmentMonitoring()
{
    // Handle reconnection waiting (non-blocking)
    if (mWaitingForReconnection) {
        // Try non-blocking reconnection check every frame
        if (checkForReconnection()) {
            LL_INFOS("AI") << "[AI] ✅ AI server reconnected successfully!" << LL_ENDL;
            mWaitingForReconnection = false;
        } else {
            // Still waiting - don't proceed with normal monitoring until reconnected
            return;
        }
    }
    
    if (!isAIEnabled()) {
        mMonitoringEnabled = false;
        return;
    }

    // Initialize monitoring if needed
    if (!mMonitoringEnabled) {
        mMonitoringEnabled = true;
        mEnvironmentUpdateTimer.reset();
        LL_INFOS("AI") << "[AI] Environment monitoring started" << LL_ENDL;
    }

    // Check agent login and set up user ID if not already done (non-blocking check only)
    if (!mUserIdSet && isAgentAvatarValid()) {
        LLUUID agent_id = gAgent.getID();
        if (!agent_id.isNull()) {
            std::string user_id = agent_id.asString();
            if (mCurrentUserId != user_id) {
                mCurrentUserId = user_id;
                mUserIdSet = true;
                LL_INFOS("AI") << "[AI] User ID set (non-blocking): " << user_id << LL_ENDL;
            }
        }
    }

    // Retry persona setup if it wasn't completed during login (avatar name might not have been available)
    if (!mPersonaSet && mUserIdSet) {
        LL_INFOS("AI") << "[AI] Attempting persona setup - mPersonaSet: " << mPersonaSet << ", mUserIdSet: " << mUserIdSet << LL_ENDL;
        LLAvatarName av_name;
        if (LLAvatarNameCache::get(gAgent.getID(), &av_name)) {
            std::string complete_name = av_name.getCompleteName();
            if (!complete_name.empty()) {
                // Extract persona name (same logic as performLoginTimeSetup)
                std::string persona_name = complete_name;
                size_t paren_pos = persona_name.find('(');
                if (paren_pos != std::string::npos) {
                    persona_name = persona_name.substr(0, paren_pos);
                    while (!persona_name.empty() && std::isspace(persona_name.back())) {
                        persona_name.pop_back();
                    }
                }
                mCurrentPersona = persona_name;
                mPersonaSet = true;
                
                // Persona is fixed per session; no need to send a switch message.
                if (!mAIBridge || !mAIBridge->isInitialized()) {
                    LL_WARNS("AI") << "[AI] Process bridge not initialized during env update, cannot store persona: " << persona_name << LL_ENDL;
                }
                
                LL_INFOS("AI") << "[AI] Persona name set during environment update: " << persona_name << LL_ENDL;
            }
        }
    }

    // Update every 2 seconds (only if we have a user ID)
    if (mUserIdSet && mEnvironmentUpdateTimer.getElapsedTimeF32() > 2.0f) {
        gatherAndSendEnvironmentUpdate();
        mEnvironmentUpdateTimer.reset();
    }
    
    // Process AI messages from the process bridge (every frame for responsiveness)
    try {
        if (mAIBridge && mAIBridge->isInitialized()) {
            // This processes all queued incoming messages from the Python AI
            mAIBridge->processIncomingMessages();
        }
    } catch (const std::exception& e) {
        LL_WARNS("AI") << "[AI] Error processing AI messages: " << e.what() << LL_ENDL;
    }
}

bool FSAIIntegration::isAIEnabled() const
{
    return gSavedSettings.getBOOL("EnablePythonAI");
}

bool FSAIIntegration::isLLMChatEnabled() const
{
    return mLLMChatEnabled;
}

bool FSAIIntegration::setLLMChatEnabled(bool enabled)
{
    mLLMChatEnabled = enabled;
    if (mAIBridge && mAIBridge->isInitialized()) {
        std::string data = std::string("{\"enabled\":") + (enabled ? "true" : "false") + "}";
        AIMessage message("set_llm_chat_enabled", data);
        mAIBridge->sendMessage(message);
    } else {
        LL_WARNS("AI") << "[AI] AI Bridge not available, cannot send chat output toggle" << LL_ENDL;
    }
    return mLLMChatEnabled;
}

bool FSAIIntegration::toggleLLMChatEnabled()
{
    return setLLMChatEnabled(!mLLMChatEnabled);
}

void FSAIIntegration::emergencyStop()
{
    LL_WARNS("AI") << "[AI] Emergency stop requested" << LL_ENDL;
    if (mAIBridge) {
        mAIBridge->stopAIProcess();
    }
    mMonitoringEnabled = false;
}

void FSAIIntegration::pauseAutonomousBehavior()
{
    LL_INFOS("AI") << "[AI] Pausing autonomous behavior" << LL_ENDL;
    // TODO: Implement pause functionality in process bridge
    LL_WARNS("AI") << "[AI] Pause functionality not yet implemented in process bridge" << LL_ENDL;
}

void FSAIIntegration::resumeAutonomousBehavior()
{
    LL_INFOS("AI") << "[AI] Resuming autonomous behavior" << LL_ENDL;
    // TODO: Implement resume functionality in process bridge
    LL_WARNS("AI") << "[AI] Resume functionality not yet implemented in process bridge" << LL_ENDL;
}

std::map<std::string, std::string> FSAIIntegration::getAIStatus()
{
    std::map<std::string, std::string> status;
    
    status["enabled"] = isAIEnabled() ? "true" : "false";
    status["monitoring"] = mMonitoringEnabled ? "true" : "false";
    status["handlers_initialized"] = mCommandHandlersInitialized ? "true" : "false";
    status["environment_updates"] = std::to_string(mEnvironmentUpdateCount);
    status["messages_processed"] = std::to_string(mMessageProcessCount);

    // Get bridge status
    if (mAIBridge) {
        status["bridge_status"] = mAIBridge->getProcessStatus();
        status["process_running"] = mAIBridge->isProcessRunning() ? "true" : "false";
    } else {
        status["bridge_status"] = "Bridge not initialized";
        status["process_running"] = "false";
    }

    return status;
}

// 🔥 HOT RESTART METHODS - No viewer restart needed!

bool FSAIIntegration::hotRestartAI()
{
    LL_INFOS("AI") << "🔥 Reconnecting to AI server - No viewer restart needed!" << LL_ENDL;
    
    if (!mAIBridge) {
        LL_WARNS("AI") << "[AI] Cannot hot restart - bridge not initialized" << LL_ENDL;
        return false;
    }
    
    return mAIBridge->restartAIProcess();
}

bool FSAIIntegration::startAI(const std::string& script_path)
{
    LL_INFOS("AI") << "[AI] Connecting to AI server (script ignored): " << script_path << LL_ENDL;
    
    if (!mAIBridge) {
        LL_WARNS("AI") << "[AI] Cannot start AI - bridge not initialized" << LL_ENDL;
        return false;
    }
    
    return mAIBridge->startAIProcess(script_path);
}

void FSAIIntegration::stopAI()
{
    LL_INFOS("AI") << "[AI] Disconnecting from AI server" << LL_ENDL;
    
    if (mAIBridge) {
        mAIBridge->stopAIProcess();
    }
}

void FSAIIntegration::processAIMessages()
{
    if (mAIBridge && mAIBridge->isInitialized()) {
        mAIBridge->processIncomingMessages();
    }
}

void FSAIIntegration::setAutoRestart(bool enable)
{
    LL_INFOS("AI") << "[AI] Setting auto-restart to: " << (enable ? "enabled" : "disabled") << LL_ENDL;
    
    if (mAIBridge) {
        mAIBridge->enableAutoRestart(enable);
    }
}

bool FSAIIntegration::isConnectionActive() const
{
    if (mAIBridge && mAIBridge->isInitialized()) {
        return mAIBridge->isConnectionActive();
    }
    return false;
}

bool FSAIIntegration::waitForReconnection()
{
    LL_INFOS("AI") << "[AI] Waiting for AI server to reconnect..." << LL_ENDL;
    
    if (!mAIBridge || !mAIBridge->isInitialized()) {
        LL_WARNS("AI") << "[AI] AI Bridge not initialized, cannot wait for reconnection" << LL_ENDL;
        return false;
    }
    
    return mAIBridge->waitForReconnection(30);
}

bool FSAIIntegration::checkForReconnection()
{
    if (!mAIBridge || !mAIBridge->isInitialized()) {
        return false;
    }
    
    return mAIBridge->checkForReconnection();
}

void FSAIIntegration::onAIDisabled()
{
    LL_INFOS("AI") << "[AI] AI disabled - preparing for reconnection when re-enabled" << LL_ENDL;
    
    // Reset connection state to prepare for reconnection
    if (mAIBridge && mAIBridge->isInitialized()) {
        mAIBridge->stopAIProcess();
    }
    
    mWaitingForReconnection = false;
    mLastAIEnabledState = false;
    mMonitoringEnabled = false;
    
    LL_INFOS("AI") << "[AI] AI disabled - ready for reconnection" << LL_ENDL;
}

void FSAIIntegration::onAIEnabled()
{
    LL_INFOS("AI") << "[AI] AI enabled - checking for reconnection" << LL_ENDL;
    
    mLastAIEnabledState = true;
    
    if (!isConnectionActive()) {
        LL_INFOS("AI") << "[AI] No active connection - will wait for reconnection" << LL_ENDL;
        mWaitingForReconnection = true;
    } else {
        LL_INFOS("AI") << "[AI] Connection already active" << LL_ENDL;
        mWaitingForReconnection = false;
    }
}

void FSAIIntegration::gatherAndSendEnvironmentUpdate()
{
    try {
        // Gather nearby agents
        std::vector<std::pair<std::string, std::string>> agents_str;
        gatherNearbyAgents(agents_str);

        // Gather nearby objects
        std::vector<std::pair<std::string, std::string>> objects_str;
        gatherNearbyObjects(objects_str);

        // Get current location
        std::string current_location = getCurrentLocation();

        // Get is_sitting status
        bool is_sitting = gAgent.isSitting();

        mEnvironmentUpdateCount++;

        // Send data to python through TCP via AI process bridge
        if (mAIBridge && mAIBridge->isInitialized()) {
            // Create JSON message with environment data
            std::string agents_json = "[";
            for (size_t i = 0; i < agents_str.size(); ++i) {
                if (i > 0) agents_json += ",";
                
                // Parse name and UUID from combined string (format: "name|uuid")
                std::string name_and_uuid = agents_str[i].first;
                size_t delimiter_pos = name_and_uuid.find('|');
                std::string name = name_and_uuid.substr(0, delimiter_pos);
                std::string target_uuid = (delimiter_pos != std::string::npos) ? 
                    name_and_uuid.substr(delimiter_pos + 1) : "";
                
                agents_json += "{\"name\":\"" + name + "\",\"position\":\"" + agents_str[i].second + 
                              "\",\"target_uuid\":\"" + target_uuid + "\"}";
            }
            agents_json += "]";

            std::string objects_json = "[";
            for (size_t i = 0; i < objects_str.size(); ++i) {
                if (i > 0) objects_json += ",";
                objects_json += "{\"name\":\"" + objects_str[i].first + "\",\"position\":\"" + objects_str[i].second + "\"}";
            }
            objects_json += "]";

            // Get avatar position
            LLVector3d avatar_pos = gAgent.getPositionGlobal();
            std::string avatar_position = "(" + std::to_string(avatar_pos.mdV[0]) + ", " + 
                                        std::to_string(avatar_pos.mdV[1]) + ", " +
                                        std::to_string(avatar_pos.mdV[2]) + ")";

            // Create environment update message
            std::string env_data = "{"
                "\"agents\":" + agents_json + ","
                "\"objects\":" + objects_json + ","
                "\"location\":\"" + current_location + "\","
                "\"avatar_position\":\"" + avatar_position + "\","
                "\"is_sitting\":" + (is_sitting ? "true" : "false") + ","
                "\"timestamp\":" + std::to_string(std::time(nullptr)) +
                "}";

            // Send environment update via TCP
            AIMessage env_message("environment_update", env_data);
            mAIBridge->sendMessage(env_message);

            // LL_INFOS("AI") << "[AI] Environment update sent via TCP: " << agents_str.size()
            //                << " agents, " << objects_str.size() << " objects in "
            //                << current_location << LL_ENDL;
        } else {
            LL_WARNS("AI") << "[AI] AI Bridge not available, cannot send environment update" << LL_ENDL;
        }
                       
    } catch (const std::exception& e) {
        LL_WARNS("AI") << "[AI] Error in environment monitoring: " << e.what() << LL_ENDL;
    }
}

void FSAIIntegration::gatherNearbyAgents(std::vector<std::pair<std::string, std::string>>& agents_str)
{
    uuid_vec_t avatar_ids;
    std::vector<LLVector3d> positions;
    LLWorld::instance().getAvatars(&avatar_ids, &positions, gAgent.getPositionGlobal(), 100.0f);
    const LLUUID self_id = gAgent.getID();

    for (size_t i = 0; i < avatar_ids.size() && i < positions.size(); ++i) {
        if (!self_id.isNull() && avatar_ids[i] == self_id) {
            continue; // Do not include the AI's own avatar in nearby agents.
        }
        LLAvatarName av_name;
        if (LLAvatarNameCache::get(avatar_ids[i], &av_name)) {
            std::string name = av_name.getCompleteName();
            std::string pos_str = "(" + std::to_string(positions[i].mdV[0]) + ", " + 
                                 std::to_string(positions[i].mdV[1]) + ", " +
                                 std::to_string(positions[i].mdV[2]) + ")";
            std::string uuid_str = avatar_ids[i].asString();
            agents_str.emplace_back(name + "|" + uuid_str, pos_str);
        }
    }
}

void FSAIIntegration::gatherNearbyObjects(std::vector<std::pair<std::string, std::string>>& objects_str)
{
    objects_str.clear();

    // Get nearby objects from area search if available
    FSAreaSearch* areaSearch = LLFloaterReg::findTypedInstance<FSAreaSearch>("area_search");
    if (!areaSearch || areaSearch->mObjectDetails.empty())
    {
        return;
    }

    const LLVector3d agentPos = gAgent.getPositionGlobal();
    const auto agent_key = nearby_objects_agent_cache_key(agentPos);

    // Keep cache bounded if object UUID churn is high.
    if (g_object_reachability_cache.size() > 2048)
    {
        const auto now = std::chrono::steady_clock::now();
        for (auto it = g_object_reachability_cache.begin(); it != g_object_reachability_cache.end();)
        {
            const F64 age_seconds = duration_ms(it->second.stamp, now) / 1000.0;
            if (age_seconds > (kNearbyObjectsReachabilityCacheTtlSeconds * 4.0))
            {
                it = g_object_reachability_cache.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    struct NearbyObjectCandidate
    {
        LLUUID object_id;
        std::string name;
        LLVector3d pos;
        F32 distance = 0.0f;
    };

    std::vector<NearbyObjectCandidate> candidates;
    candidates.reserve(128);

    S32 scanned = 0;
    S32 in_distance = 0;
    S32 same_floor = 0;
    S32 in_los = 0;

    for (const auto& [objectId, props] : areaSearch->mObjectDetails)
    {
        ++scanned;
        if (!props.listed)
        {
            continue;
        }

        LLViewerObject* objectp = gObjectList.findObject(objectId);
        if (!objectp || objectp->isDead() || objectp->getRegion() != gAgent.getRegion() || objectp->isAttachment())
        {
            continue;
        }

        LLVector3d objectPos = objectp->getPositionGlobal();
        const F32 distance = static_cast<F32>(dist_vec(agentPos, objectPos));
        if (distance > kNearbyObjectsMaxDistanceMeters)
        {
            continue;
        }
        ++in_distance;

        if (std::fabs(objectPos.mdV[VZ] - agentPos.mdV[VZ]) > kNearbyObjectsMaxVerticalDeltaMeters)
        {
            continue;
        }
        ++same_floor;

        if (!has_clear_line_of_sight_to_object(agentPos, objectp))
        {
            continue;
        }
        ++in_los;

        NearbyObjectCandidate candidate;
        candidate.object_id = objectId;
        candidate.name = props.name.empty() ? "Unknown Object" : props.name;
        candidate.pos = objectPos;
        candidate.distance = distance;
        candidates.push_back(candidate);
    }

    std::sort(candidates.begin(), candidates.end(),
        [](const NearbyObjectCandidate& a, const NearbyObjectCandidate& b)
        {
            if (a.distance != b.distance)
            {
                return a.distance < b.distance;
            }
            return a.object_id < b.object_id;
        });

    const S32 adaptive_candidate_limit = llclamp(
        llmax(
            kNearbyObjectsCandidateLimitMin,
            static_cast<S32>(std::ceil(static_cast<F64>(candidates.size()) * kNearbyObjectsCandidateCoverageFraction))),
        kNearbyObjectsCandidateLimitMin,
        kNearbyObjectsCandidateLimitMax);
    if (static_cast<S32>(candidates.size()) > adaptive_candidate_limit)
    {
        candidates.resize(adaptive_candidate_limit);
    }

    std::map<LLUUID, size_t> candidate_index_by_id;
    for (size_t i = 0; i < candidates.size(); ++i)
    {
        candidate_index_by_id[candidates[i].object_id] = i;
    }

    std::vector<LLUUID> updated_rr_order;
    updated_rr_order.reserve(candidates.size());
    std::set<LLUUID> rr_members;
    for (const LLUUID& object_id : g_nearby_object_round_robin_order)
    {
        if (candidate_index_by_id.find(object_id) == candidate_index_by_id.end())
        {
            continue;
        }
        updated_rr_order.push_back(object_id);
        rr_members.insert(object_id);
    }
    for (const NearbyObjectCandidate& candidate : candidates)
    {
        if (rr_members.insert(candidate.object_id).second)
        {
            updated_rr_order.push_back(candidate.object_id);
        }
    }
    g_nearby_object_round_robin_order.swap(updated_rr_order);
    if (g_nearby_object_round_robin_order.empty())
    {
        g_nearby_object_round_robin_cursor = 0;
    }
    else if (g_nearby_object_round_robin_cursor >= g_nearby_object_round_robin_order.size())
    {
        g_nearby_object_round_robin_cursor %= g_nearby_object_round_robin_order.size();
    }

    std::map<LLUUID, bool> reachable_by_id;
    std::set<LLUUID> uncached_ids;
    S32 cache_hits = 0;
    for (const NearbyObjectCandidate& candidate : candidates)
    {
        bool reachable = false;
        bool stale_cached = false;
        if (get_cached_object_reachability(candidate.object_id, agent_key, reachable, true, &stale_cached))
        {
            ++cache_hits;
            reachable_by_id[candidate.object_id] = reachable;
            if (stale_cached)
            {
                // Keep display stable while stale entries are refreshed by the round-robin budget.
                uncached_ids.insert(candidate.object_id);
            }
        }
        else
        {
            uncached_ids.insert(candidate.object_id);
        }
    }

    S32 path_checks = 0;
    if (!uncached_ids.empty() && !g_nearby_object_round_robin_order.empty())
    {
        const size_t rr_count = g_nearby_object_round_robin_order.size();
        size_t visited = 0;
        while (visited < rr_count && path_checks < kNearbyObjectsReachabilityBudgetPerUpdate)
        {
            const size_t rr_idx = (g_nearby_object_round_robin_cursor + visited) % rr_count;
            const LLUUID object_id = g_nearby_object_round_robin_order[rr_idx];
            ++visited;
            if (uncached_ids.find(object_id) == uncached_ids.end())
            {
                continue;
            }

            auto candidate_it = candidate_index_by_id.find(object_id);
            if (candidate_it == candidate_index_by_id.end())
            {
                continue;
            }

            const NearbyObjectCandidate& candidate = candidates[candidate_it->second];
            bool reachable = false;
            LLViewerObject* objectp = gObjectList.findObject(candidate.object_id);
            if (objectp && !objectp->isDead() && objectp->getRegion() == gAgent.getRegion())
            {
                LLVector3d approach_goal = candidate.pos;
                if (!compute_object_approach_goal(agentPos, objectp, approach_goal))
                {
                    approach_goal = candidate.pos;
                }

                reachable = has_direct_clearance_route(agentPos, approach_goal);
                if (!reachable)
                {
                    std::vector<LLVector3d> waypoints;
                    bool direct_out = false;
                    reachable = plan_custom_clearance_route(agentPos, approach_goal, waypoints, direct_out, false);
                }
            }

            put_cached_object_reachability(candidate.object_id, agent_key, reachable);
            reachable_by_id[candidate.object_id] = reachable;
            uncached_ids.erase(candidate.object_id);
            ++path_checks;
        }

        if (rr_count > 0)
        {
            g_nearby_object_round_robin_cursor = (g_nearby_object_round_robin_cursor + visited) % rr_count;
        }
    }

    S32 reachable_count = 0;
    for (const NearbyObjectCandidate& candidate : candidates)
    {
        auto reachable_it = reachable_by_id.find(candidate.object_id);
        if (reachable_it == reachable_by_id.end() || !reachable_it->second)
        {
            continue;
        }

        ++reachable_count;
        std::string pos_str = "(" + std::to_string(candidate.pos.mdV[0]) + ", " +
                              std::to_string(candidate.pos.mdV[1]) + ", " +
                              std::to_string(candidate.pos.mdV[2]) + ")";
        std::string uuid_str = candidate.object_id.asString();
        objects_str.emplace_back(candidate.name + "|" + uuid_str, pos_str);
        if (static_cast<S32>(objects_str.size()) >= kNearbyObjectsMaxOutput)
        {
            break;
        }
    }

    LL_DEBUGS("AI") << "[AI] Nearby object filter: scanned=" << scanned
                    << " in_distance=" << in_distance
                    << " same_floor=" << same_floor
                    << " in_los=" << in_los
                    << " candidates=" << candidates.size()
                    << " candidate_limit=" << adaptive_candidate_limit
                    << " rr_queue=" << g_nearby_object_round_robin_order.size()
                    << " rr_cursor=" << g_nearby_object_round_robin_cursor
                    << " path_checks=" << path_checks
                    << " cache_hits=" << cache_hits
                    << " remaining_uncached=" << uncached_ids.size()
                    << " reachable=" << reachable_count
                    << " output=" << objects_str.size()
                    << LL_ENDL;
}

std::string FSAIIntegration::getCurrentLocation()
{
    if (LLViewerRegion* region = gAgent.getRegion()) {
        return region->getName();
    }
    return "Unknown Location";
}

void FSAIIntegration::initializeCommandHandlers()
{
    if (mCommandHandlersInitialized) {
        return;
    }

    LL_INFOS("AI") << "[AI] Initializing command handlers" << LL_ENDL;

    // Register command handlers with the AI process bridge
    if (mAIBridge && mAIBridge->isInitialized()) {
        LL_INFOS("AI") << "[AI] Registering command handlers with process bridge" << LL_ENDL;
        
        mAIBridge->registerCommandHandler("CHAT", [this](const AICommandData& cmd) {
            S32 channel = 0;
            auto it = cmd.parameters.find("channel");
            if (it != cmd.parameters.end()) {
                try {
                    channel = static_cast<S32>(std::stoi(it->second));
                } catch (...) {
                    channel = 0;
                }
            }
            handleAIChatOutput(cmd.content.c_str(), channel);
        });
        
        mAIBridge->registerCommandHandler("EMOTE", [this](const AICommandData& cmd) {
            S32 channel = 0;
            auto it = cmd.parameters.find("channel");
            if (it != cmd.parameters.end()) {
                try {
                    channel = static_cast<S32>(std::stoi(it->second));
                } catch (...) {
                    channel = 0;
                }
            }
            handleAIEmoteOutput(cmd.content.c_str(), channel);
        });
        
        mAIBridge->registerCommandHandler("MOVE", [this](const AICommandData& cmd) {
            F32 desired_standoff = kDefaultMoveStandoffMeters;
            parse_standoff_parameter(cmd.parameters, desired_standoff);
            handleAIMovementOutput(cmd.x, cmd.y, cmd.z, cmd.target_uuid.c_str(), desired_standoff);
        });
        
        mAIBridge->registerCommandHandler("TOUCH", [this](const AICommandData& cmd) {
            handleAITouchOutput(cmd.target_uuid.c_str());
        });
        
        mAIBridge->registerCommandHandler("SIT", [this](const AICommandData& cmd) {
            handleAISitOutput(cmd.target_uuid.c_str());
        });
        
        mAIBridge->registerCommandHandler("STAND", [this](const AICommandData& cmd) {
            handleAIStandOutput();
        });
        
        mAIBridge->registerCommandHandler("LOOK_AT", [this](const AICommandData& cmd) {
            handleAILookAtOutput(cmd.x, cmd.y, cmd.z);
        });
        
        mAIBridge->registerCommandHandler("FOLLOW", [this](const AICommandData& cmd) {
            handle_AIFollowOutput(cmd.target_uuid.c_str());
        });
        
        LL_INFOS("AI") << "[AI] Command handlers registered with process bridge" << LL_ENDL;
    }


    mCommandHandlersInitialized = true;
    LL_INFOS("AI") << "[AI] Command handlers initialized successfully" << LL_ENDL;
}

// Static command handler implementations
void FSAIIntegration::handleAIChatOutput(const char* content, S32 channel)
{
    if (!content) return;
    
    LL_INFOS("AI") << "[AI] Chat output (channel " << channel << "): " << content << LL_ENDL;
    
    // Use static method from FSNearbyChat
    LLWString wtext = utf8str_to_wstring(std::string(content));
    FSNearbyChat::sendChatFromViewer(wtext, wtext, CHAT_TYPE_NORMAL, true, channel);
}

void FSAIIntegration::handleAIEmoteOutput(const char* content, S32 channel)
{
    if (!content) return;
    
    std::string emote(content);
    
    // Format as emote if not already formatted
    if (!emote.empty() && emote.find("/me ") != 0) {
        emote = "/me " + emote;
    }
    
    LL_INFOS("AI") << "[AI] Emote output (channel " << channel << "): " << emote << LL_ENDL;
    
    // Use static method from FSNearbyChat
    LLWString wtext = utf8str_to_wstring(emote);
    FSNearbyChat::sendChatFromViewer(wtext, wtext, CHAT_TYPE_NORMAL, true, channel);
}

void FSAIIntegration::handleAIMovementOutput(float x, float y, float z, const char* target_uuid, float desired_standoff)
{
    ++g_move_planner_telemetry.move_requests_total;
    const F32 standoff = clamp_move_standoff(desired_standoff);
    LL_INFOS("AI") << "[AI] Movement command: (" << x << ", " << y << ", " << z << ")"
                   << " target_uuid=" << (target_uuid ? target_uuid : "")
                   << " standoff=" << standoff << LL_ENDL;
    
    // Validate agent avatar
    if (!isAgentAvatarValid()) {
        LL_WARNS("AI") << "[AI] Cannot move - agent avatar not valid" << LL_ENDL;
        return;
    }
    
    // Convert local region coordinates to global coordinates
    LLViewerRegion* region = gAgent.getRegion();
    if (!region) {
        LL_WARNS("AI") << "[AI] No current region for movement calculation" << LL_ENDL;
        return;
    }
    LLVector3d target_pos(x, y, z);
    // If AI sends local region coordinates (0-256), convert to global
    if (x >= 0.f && y >= 0.f && x < 256.f && y < 256.f)
    {
        LLVector3 region_pos(x, y, z);
        target_pos = region->getPosGlobalFromRegion(region_pos);
    
        LL_INFOS("AI") << "[AI] Local coords (" << x << ", " << y << ", " << z 
                       << ") converted to global (" << target_pos.mdV[0] << ", " 
                       << target_pos.mdV[1] << ", " << target_pos.mdV[2] << ")" << LL_ENDL;
        
    }
    
    LLVector3d agent_pos = gAgent.getPositionGlobal();
    F64 distance = dist_vec(agent_pos, target_pos);
    
    if (distance > 256.0) { // Maximum reasonable movement distance in meters
        LL_WARNS("AI") << "[AI] Movement target too far: " << distance << "m, clamping" << LL_ENDL;
        // Clamp to maximum distance
        LLVector3d direction = target_pos - agent_pos;
        direction.normalize();
        target_pos = agent_pos + (direction * kMoveClampDistanceMeters);
    }

    // Every MOVE invalidates any prior multi-waypoint callback plan.
    const U64 plan_id = ++g_move_plan_generation;

    // Person-aware movement: use follow pilot only when LoS/standoff is clear, otherwise path to an approach point.
    LLUUID person_target_id;
    LLVector3d person_target_pos;
    bool has_person_target = false;

    if (target_uuid && *target_uuid)
    {
        LLUUID requested_uuid(target_uuid);
        if (!requested_uuid.isNull() && resolve_avatar_position(requested_uuid, person_target_pos))
        {
            person_target_id = requested_uuid;
            has_person_target = true;
        }
    }

    if (!has_person_target)
    {
        has_person_target = find_closest_avatar_to_point(target_pos, kPersonSnapRadiusMeters, person_target_id, person_target_pos);
    }

    if (has_person_target)
    {
        const F64 avatar_distance = dist_vec(agent_pos, person_target_pos);
        const bool los_clear = has_clear_line_of_sight_to_avatar(agent_pos, person_target_pos);

        if (avatar_distance <= standoff || los_clear)
        {
            ++g_move_planner_telemetry.person_follow_requests;
            ++g_move_planner_telemetry.avatar_los_direct_follow;
            gAgent.stopAutoPilot();
            gAgent.startFollowPilot(person_target_id, false, standoff);
            LL_INFOS("AI") << "[AI] Avatar target has direct follow conditions; following target="
                           << person_target_id.asString()
                           << " dist=" << avatar_distance
                           << " los=" << (los_clear ? "yes" : "no")
                           << LL_ENDL;
            maybe_log_move_planner_summary("handleAIMovementOutput");
            return;
        }

        ++g_move_planner_telemetry.avatar_hybrid_path_attempts;

        LLVector3d avatar_approach_goal = compute_avatar_approach_goal(agent_pos, person_target_pos, standoff);
        if (dist_vec(agent_pos, avatar_approach_goal) <= kMoveAutopilotStopDistanceMeters)
        {
            LLVector3d current_avatar_pos;
            const bool can_resolve = resolve_avatar_position(person_target_id, current_avatar_pos);
            const F64 current_distance = can_resolve ? dist_vec(agent_pos, current_avatar_pos) : std::numeric_limits<F64>::max();
            const bool current_los = can_resolve && has_clear_line_of_sight_to_avatar(agent_pos, current_avatar_pos);
            if (can_resolve && (current_distance <= standoff || current_los))
            {
                ++g_move_planner_telemetry.person_follow_requests;
                ++g_move_planner_telemetry.avatar_hybrid_follow_fallback;
                gAgent.stopAutoPilot();
                gAgent.startFollowPilot(person_target_id, false, standoff);
                LL_INFOS("AI") << "[AI] Avatar hybrid approach point already close and follow conditions are clear; using follow pilot target="
                               << person_target_id.asString()
                               << " dist=" << current_distance
                               << " los=" << (current_los ? "yes" : "no")
                               << LL_ENDL;
                maybe_log_move_planner_summary("handleAIMovementOutput");
                return;
            }

            ++g_move_planner_telemetry.blocked_abort_avatar;
            gAgent.stopAutoPilot();
            LL_WARNS("AI") << "[AI] Avatar approach point is close but LoS is blocked; aborting MOVE target="
                           << person_target_id.asString()
                           << " can_resolve=" << (can_resolve ? "yes" : "no")
                           << " dist=" << current_distance
                           << " los=" << (current_los ? "yes" : "no")
                           << LL_ENDL;
            maybe_log_move_planner_summary("handleAIMovementOutput");
            return;
        }

        bool avatar_direct_path = false;
        std::vector<LLVector3d> avatar_waypoints;
        MovePlanSource avatar_source = MovePlanSource::FALLBACK_DIRECT;
        F64 custom_ms = 0.0;
        F64 legacy_ms = 0.0;

        ++g_move_planner_telemetry.custom_attempts;
        auto custom_start = std::chrono::steady_clock::now();
        bool planned_avatar_path = plan_custom_clearance_route(agent_pos, avatar_approach_goal, avatar_waypoints, avatar_direct_path);
        auto custom_end = std::chrono::steady_clock::now();
        custom_ms = duration_ms(custom_start, custom_end);
        g_move_planner_telemetry.custom_ms_total += custom_ms;
        if (planned_avatar_path)
        {
            ++g_move_planner_telemetry.custom_success;
            avatar_source = MovePlanSource::CUSTOM;
        }
        else
        {
            const bool pathfinding_ready = is_pathfinding_ready();
            if (pathfinding_ready)
            {
                ++g_move_planner_telemetry.legacy_attempts;
                auto legacy_start = std::chrono::steady_clock::now();
                planned_avatar_path = plan_probe_route(agent_pos, avatar_approach_goal, avatar_waypoints, avatar_direct_path);
                auto legacy_end = std::chrono::steady_clock::now();
                legacy_ms = duration_ms(legacy_start, legacy_end);
                g_move_planner_telemetry.legacy_ms_total += legacy_ms;
                if (planned_avatar_path)
                {
                    ++g_move_planner_telemetry.legacy_success;
                    avatar_source = MovePlanSource::LEGACY;
                }
            }
        }

        LL_INFOS("AI") << "[AI] Avatar hybrid planner timing: source=" << move_plan_source_to_text(avatar_source)
                       << " custom_ms=" << custom_ms
                       << " legacy_ms=" << legacy_ms
                       << " waypoints=" << avatar_waypoints.size()
                       << LL_ENDL;

        if (planned_avatar_path && !avatar_waypoints.empty())
        {
            ++g_move_planner_telemetry.avatar_hybrid_path_success;
            gAgent.stopAutoPilot();
            start_move_path_plan(
                plan_id,
                avatar_waypoints,
                kMovePathReplanAttempts,
                &avatar_approach_goal,
                &person_target_id,
                standoff);
            LL_INFOS("AI") << "[AI] Avatar hybrid movement started with " << avatar_waypoints.size()
                           << " leg(s). Approach goal: ("
                           << avatar_approach_goal.mdV[0] << ", " << avatar_approach_goal.mdV[1] << ", " << avatar_approach_goal.mdV[2]
                           << ") target=" << person_target_id.asString()
                           << LL_ENDL;
            maybe_log_move_planner_summary("handleAIMovementOutput");
            return;
        }

        LLVector3d current_avatar_pos;
        const bool can_resolve = resolve_avatar_position(person_target_id, current_avatar_pos);
        const F64 current_distance = can_resolve ? dist_vec(agent_pos, current_avatar_pos) : std::numeric_limits<F64>::max();
        const bool current_los = can_resolve && has_clear_line_of_sight_to_avatar(agent_pos, current_avatar_pos);
        if (can_resolve && (current_distance <= standoff || current_los))
        {
            ++g_move_planner_telemetry.person_follow_requests;
            ++g_move_planner_telemetry.avatar_hybrid_follow_fallback;
            gAgent.stopAutoPilot();
            gAgent.startFollowPilot(person_target_id, false, standoff);
            LL_INFOS("AI") << "[AI] Avatar hybrid path unavailable, but follow conditions became clear; using follow pilot target="
                           << person_target_id.asString()
                           << " dist=" << current_distance
                           << " los=" << (current_los ? "yes" : "no")
                           << LL_ENDL;
            maybe_log_move_planner_summary("handleAIMovementOutput");
            return;
        }

        ++g_move_planner_telemetry.blocked_abort_avatar;
        gAgent.stopAutoPilot();
        LL_WARNS("AI") << "[AI] Avatar hybrid path unavailable and LoS is blocked; aborting MOVE target="
                       << person_target_id.asString()
                       << " can_resolve=" << (can_resolve ? "yes" : "no")
                       << " dist=" << current_distance
                       << " los=" << (current_los ? "yes" : "no")
                       << LL_ENDL;
        maybe_log_move_planner_summary("handleAIMovementOutput");
        return;
    }

    F32 final_goal_tolerance = kMoveAutopilotStopDistanceMeters;

    // For object UUID targets, move to the closest walkable approach point near object bounds
    // rather than aiming at object center (which is often obstructed).
    if (target_uuid && *target_uuid)
    {
        const LLUUID requested_uuid(target_uuid);
        if (!requested_uuid.isNull())
        {
            LLViewerObject* object_target = gObjectList.findObject(requested_uuid);
            if (object_target && !object_target->isDead() && !object_target->isAvatar() && !object_target->isAttachment())
            {
                final_goal_tolerance = kMoveObjectFinalGoalToleranceMeters;
                const LLVector3d object_center = object_target->getPositionGlobal();
                LLVector3d object_approach_goal;
                if (compute_object_approach_goal(agent_pos, object_target, object_approach_goal))
                {
                    target_pos = object_approach_goal;
                    distance = dist_vec(agent_pos, target_pos);
                    LL_INFOS("AI") << "[AI] Object approach goal selected near target="
                                   << requested_uuid.asString()
                                   << " center=(" << object_center.mdV[0] << ", " << object_center.mdV[1] << ", " << object_center.mdV[2] << ")"
                                   << " approach=(" << target_pos.mdV[0] << ", " << target_pos.mdV[1] << ", " << target_pos.mdV[2] << ")"
                                   << LL_ENDL;
                }
                else
                {
                    target_pos = object_center;
                    distance = dist_vec(agent_pos, target_pos);
                    LL_INFOS("AI") << "[AI] Object approach goal unavailable; using object center target="
                                   << requested_uuid.asString()
                                   << LL_ENDL;
                }
            }
        }
    }

    // Non-avatar movement target.
    if (distance <= kMoveAutopilotStopDistanceMeters)
    {
        ++g_move_planner_telemetry.skipped_close_target;
        gAgent.stopAutoPilot();
        LL_INFOS("AI") << "[AI] Target already within " << kMoveAutopilotStopDistanceMeters << "m; no MOVE needed" << LL_ENDL;
        maybe_log_move_planner_summary("handleAIMovementOutput");
        return;
    }

    ++g_move_planner_telemetry.non_avatar_plan_requests;
    bool direct_path_available = false;
    bool direct_fallback_allowed = false;
    std::vector<LLVector3d> planned_waypoints;
    MovePlanSource selected_source = MovePlanSource::FALLBACK_DIRECT;
    F64 custom_ms = 0.0;
    F64 legacy_ms = 0.0;

    ++g_move_planner_telemetry.custom_attempts;
    auto custom_start = std::chrono::steady_clock::now();
    bool planned = plan_custom_clearance_route(agent_pos, target_pos, planned_waypoints, direct_path_available);
    auto custom_end = std::chrono::steady_clock::now();
    custom_ms = duration_ms(custom_start, custom_end);
    g_move_planner_telemetry.custom_ms_total += custom_ms;
    if (planned)
    {
        ++g_move_planner_telemetry.custom_success;
        selected_source = MovePlanSource::CUSTOM;
    }
    if (!planned)
    {
        const bool pathfinding_ready = is_pathfinding_ready();
        if (pathfinding_ready)
        {
            ++g_move_planner_telemetry.legacy_attempts;
            auto legacy_start = std::chrono::steady_clock::now();
            planned = plan_probe_route(agent_pos, target_pos, planned_waypoints, direct_path_available);
            auto legacy_end = std::chrono::steady_clock::now();
            legacy_ms = duration_ms(legacy_start, legacy_end);
            g_move_planner_telemetry.legacy_ms_total += legacy_ms;
            if (planned)
            {
                ++g_move_planner_telemetry.legacy_success;
                selected_source = MovePlanSource::LEGACY;
            }
            LL_INFOS("AI") << "[AI] Legacy path probe result: direct=" << (direct_path_available ? "yes" : "no")
                           << " planned_waypoints=" << planned_waypoints.size()
                           << " planned=" << (planned ? "yes" : "no") << LL_ENDL;
        }
        else
        {
            LL_INFOS("AI") << "[AI] Legacy pathfinding unavailable; relying on custom planner and strict direct-clearance fallback" << LL_ENDL;
        }
    }

    LL_INFOS("AI") << "[AI] Move planner timing: source=" << move_plan_source_to_text(selected_source)
                   << " custom_ms=" << custom_ms
                   << " legacy_ms=" << legacy_ms
                   << " waypoints=" << planned_waypoints.size()
                   << LL_ENDL;

    if (!planned_waypoints.empty())
    {
        std::ostringstream route_stream;
        route_stream << "[AI] Planned waypoint list:";
        for (size_t i = 0; i < planned_waypoints.size(); ++i)
        {
            const LLVector3d& wp = planned_waypoints[i];
            route_stream << " [" << (i + 1) << "]("
                         << wp.mdV[0] << "," << wp.mdV[1] << "," << wp.mdV[2] << ")";
        }
        LL_INFOS("AI") << route_stream.str() << LL_ENDL;
    }
    gAgent.stopAutoPilot();

    if (!planned_waypoints.empty())
    {
        start_move_path_plan(
            plan_id,
            planned_waypoints,
            kMovePathReplanAttempts,
            &target_pos,
            nullptr,
            kDefaultMoveStandoffMeters,
            final_goal_tolerance);
        const LLVector3d& first_leg = planned_waypoints.front();
        LL_INFOS("AI") << "[AI] Multi-waypoint movement started with " << planned_waypoints.size()
                       << " leg(s). First leg target: ("
                       << first_leg.mdV[0] << ", " << first_leg.mdV[1] << ", " << first_leg.mdV[2] << ")" << LL_ENDL;
        maybe_log_move_planner_summary("handleAIMovementOutput");
        return;
    }

    direct_fallback_allowed = has_direct_clearance_route(agent_pos, target_pos);
    if (!direct_fallback_allowed)
    {
        ++g_move_planner_telemetry.blocked_abort_non_avatar;
        LL_WARNS("AI") << "[AI] No waypoint route and direct clearance is blocked; aborting MOVE to ("
                       << target_pos.mdV[0] << ", " << target_pos.mdV[1] << ", " << target_pos.mdV[2] << ")"
                       << LL_ENDL;
        maybe_log_move_planner_summary("handleAIMovementOutput");
        return;
    }
    direct_path_available = true;
    ++g_move_planner_telemetry.fallback_direct;
    LL_INFOS("AI") << "[AI] No waypoint route found, but direct clearance is open; using direct autopilot" << LL_ENDL;

    gAgent.startAutoPilotGlobal(
        target_pos,
        "AI Movement",
        NULL,
        NULL,
        NULL,
        kMoveAutopilotStopDistanceMeters,
        kMoveAutopilotRotationThreshold,
        false
    );

    LL_INFOS("AI") << "[AI] Movement initiated to (" << target_pos.mdV[0] << ", "
                   << target_pos.mdV[1] << ", " << target_pos.mdV[2] << ")"
                   << (direct_path_available ? " [direct-path ok]" : "") << LL_ENDL;

    maybe_log_move_planner_summary("handleAIMovementOutput");
}

void FSAIIntegration::handleAITouchOutput(const char* object_uuid)
{
    if (!object_uuid) return;
    
    LLUUID target_uuid(object_uuid);
    if (target_uuid.isNull()) {
        LL_WARNS("AI") << "[AI] Invalid object UUID for touch command" << LL_ENDL;
        return;
    }
    
    LL_INFOS("AI") << "[AI] Touch command for object: " << target_uuid.asString() << LL_ENDL;
    
    // Validate agent avatar
    if (!isAgentAvatarValid()) {
        LL_WARNS("AI") << "[AI] Cannot touch - agent avatar not valid" << LL_ENDL;
        return;
    }
    
    // Find the target object
    LLViewerObject* object = gObjectList.findObject(target_uuid);
    if (!object) {
        LL_WARNS("AI") << "[AI] Object not found: " << target_uuid.asString() << LL_ENDL;
        return;
    }
    
    // Check if object is in range (approximate check)
    LLVector3d agent_pos = gAgent.getPositionGlobal();
    LLVector3d object_pos = object->getPositionGlobal();
    F64 distance = dist_vec(agent_pos, object_pos);
    
    if (distance > 10.0) { // Maximum touch distance in meters
        LL_WARNS("AI") << "[AI] Object too far to touch: " << distance << "m" << LL_ENDL;
        return;
    }
    
    // Select the object temporarily for the touch operation
    LLSelectMgr::getInstance()->deselectAll();
    LLSelectMgr::getInstance()->selectObjectAndFamily(object);
    
    // Perform the touch operation using the standard viewer function
    handle_object_touch();
    
    // Deselect after touch
    LLSelectMgr::getInstance()->deselectAll();
    
    LL_INFOS("AI") << "[AI] Touch completed for object: " << target_uuid.asString() << LL_ENDL;
}

void FSAIIntegration::handleAISitOutput(const char* object_uuid)
{
    if (!object_uuid) return;
    
    LLUUID target_uuid(object_uuid);
    if (target_uuid.isNull()) {
        LL_WARNS("AI") << "[AI] Invalid object UUID for sit command" << LL_ENDL;
        return;
    }
    
    LL_INFOS("AI") << "[AI] Sit command for object: " << target_uuid.asString() << LL_ENDL;
    
    // Validate agent avatar
    if (!isAgentAvatarValid()) {
        LL_WARNS("AI") << "[AI] Cannot sit - agent avatar not valid" << LL_ENDL;
        return;
    }
    
    // Check if already sitting
    if (gAgentAvatarp->isSitting()) {
        LL_INFOS("AI") << "[AI] Agent is already sitting, standing up first" << LL_ENDL;
        gAgent.standUp();
    }
    
    // Find the target object
    LLViewerObject* object = gObjectList.findObject(target_uuid);
    if (!object) {
        LL_WARNS("AI") << "[AI] Sit target object not found: " << target_uuid.asString() << LL_ENDL;
        return;
    }
    
    // Check if object is in reasonable range for sitting
    LLVector3d agent_pos = gAgent.getPositionGlobal();
    LLVector3d object_pos = object->getPositionGlobal();
    F64 distance = dist_vec(agent_pos, object_pos);
    
    if (distance > 30.0) { // Maximum reasonable sit distance in meters
        LL_WARNS("AI") << "[AI] Sit target too far: " << distance << "m" << LL_ENDL;
        return;
    }
    
    // Use autopilot to approach the object, then sit on it using proper object sitting
    // This allows the LLM to say "SIT chair" without needing to understand movement first
    LLQuaternion target_rot;  // Default rotation
    handle_object_sit(target_uuid);
    
    LL_INFOS("AI") << "[AI] Sit initiated for object: " << target_uuid.asString() << LL_ENDL;
}

void FSAIIntegration::handleAIStandOutput()
{
    LL_INFOS("AI") << "[AI] Stand command" << LL_ENDL;
    
    // Validate agent avatar
    if (!isAgentAvatarValid()) {
        LL_WARNS("AI") << "[AI] Cannot stand - agent avatar not valid" << LL_ENDL;
        return;
    }
    
    // Check if currently sitting
    if (gAgentAvatarp->isSitting()) {
        // Stop any current autopilot movement first
        gAgent.stopAutoPilot();
        
        // Stand up
        gAgent.standUp();
        
        LL_INFOS("AI") << "[AI] Standing up from sitting position" << LL_ENDL;
    } else {
        LL_INFOS("AI") << "[AI] Agent is already standing" << LL_ENDL;
    }
}

void FSAIIntegration::handleAILookAtOutput(float x, float y, float z)
{
    LL_INFOS("AI") << "[AI] Look At command: (" << x << ", " << y << ", " << z << ")" << LL_ENDL;
    
    // Validate agent avatar
    if (!isAgentAvatarValid()) {
        LL_WARNS("AI") << "[AI] Cannot look - agent avatar not valid" << LL_ENDL;
        return;
    }
    
    // Convert coordinates to LLVector3d for global positioning
    LLVector3d look_at_global(x, y, z);
    
    // Validate look at position (basic range check)
    LLVector3d agent_pos = gAgent.getPositionGlobal();
    F64 distance = dist_vec(agent_pos, look_at_global);
    
    if (distance > 1000.0) { // Maximum reasonable look-at distance
        LL_WARNS("AI") << "[AI] Look at target very far: " << distance << "m" << LL_ENDL;
    }
    
    // Convert global position to region-relative position for lookAtObject
    LLViewerRegion* region = gAgent.getRegion();
    if (!region) {
        LL_WARNS("AI") << "[AI] No current region for look at operation" << LL_ENDL;
        return;
    }
    
    // Convert global to region coordinates
    LLVector3 region_pos = region->getPosRegionFromGlobal(look_at_global);
    
    // Use the HUD effect system to create a look-at effect
    // This creates the visual beam and avatar head turning
    gAgentCamera.setLookAt(LOOKAT_TARGET_FOCUS, NULL, region_pos);
    
    LL_INFOS("AI") << "[AI] Look At initiated towards (" << look_at_global.mdV[0] << ", " 
                   << look_at_global.mdV[1] << ", " << look_at_global.mdV[2] << ")" << LL_ENDL;
}

void FSAIIntegration::handle_AIFollowOutput(const char* target_uuid)
{
    if (!target_uuid)
        return;

    LLUUID target_id(target_uuid);
    if (target_id.isNull())
    {
        LL_WARNS("AI") << "[AI] Invalid target UUID for follow command" << LL_ENDL;
        return;
    }

    LL_INFOS("AI") << "[AI] Follow command for target: " << target_id.asString() << LL_ENDL;

    // Validate agent avatar
    if (!isAgentAvatarValid())
    {
        LL_WARNS("AI") << "[AI] Cannot follow - agent avatar not valid" << LL_ENDL;
        return;
    }

    // Find the target object
    LLViewerObject* target_object = gObjectList.findObject(target_id);
    if (!target_object)
    {
        LL_WARNS("AI") << "[AI] Follow target object not found: " << target_id.asString() << LL_ENDL;
        return;
    }

    // Check if the target is in range
    LLVector3d agent_pos  = gAgent.getPositionGlobal();
    LLVector3d target_pos = target_object->getPositionGlobal();
    F64        distance   = dist_vec(agent_pos, target_pos);

    if (distance > 50.0)
    { // Maximum follow distance in meters
        LL_WARNS("AI") << "[AI] Follow target too far: " << distance << "m" << LL_ENDL;
        return;
    }

    // Start autopilot to follow the object
    gAgent.startFollowPilot(target_id, false, 0.5f);

    LL_INFOS("AI") << "[AI] Follow initiated for object: " << target_id.asString() << LL_ENDL;
}

void FSAIIntegration::checkAndSetupAgentLogin()
{
    try {
        LL_INFOS("AI") << "[AI] checkAndSetupAgentLogin() called" << LL_ENDL;
        // Check if agent avatar is valid
        if (!isAgentAvatarValid()) {
            LL_INFOS("AI") << "[AI] Agent avatar not valid yet, will retry" << LL_ENDL;
            return; // Not logged in yet, try again later
        }

        // Get agent ID (this is the permanent user ID)
        LLUUID agent_id = gAgent.getID();
        if (agent_id.isNull()) {
            return; // Invalid agent ID, try again later
        }

        std::string user_id = agent_id.asString();
        
        // Check if this is a new user ID
        if (mCurrentUserId != user_id) {
            mCurrentUserId = user_id;
            
            LL_INFOS("AI") << "[AI] Agent logged in - User ID: " << user_id << LL_ENDL;
            
            // Set up user ID in AI process bridge for ChromaDB collection
            if (mAIBridge && mAIBridge->isInitialized()) {
                mAIBridge->setUserIdAsync(user_id);
            }
            
            LL_INFOS("AI") << "[AI] ChromaDB collection setup initiated for user: " << user_id << LL_ENDL;
        }
        
        // Set up persona once we have the agent name (only if not already set)
        if (!mPersonaSet) {
            LLAvatarName av_name;
            if (LLAvatarNameCache::get(agent_id, &av_name)) {
                std::string complete_name = av_name.getCompleteName();
                if (!complete_name.empty()) {
                    setupPersonaOnLogin(complete_name);
                    mPersonaSet = true;
                    LL_INFOS("AI") << "[AI] Persona setup completed for: " << complete_name << LL_ENDL;
                }
            }
        }
        
        // Mark user ID as set, but keep checking until persona is also set
        mUserIdSet = true;
        
    } catch (const std::exception& e) {
        LL_WARNS("AI") << "[AI] Error checking agent login: " << e.what() << LL_ENDL;
    }
}

void FSAIIntegration::setupPersonaOnLogin(const std::string& agent_name)
{
    try {
        LL_INFOS("AI") << "[AI] Setting up persona for agent: " << agent_name << LL_ENDL;
        
        // Check if AI bridge is ready
        if (!mAIBridge || !mAIBridge->isInitialized()) {
            LL_WARNS("AI") << "[AI] AI Process Bridge not initialized, cannot setup persona" << LL_ENDL;
            return;
        }
        
        // Extract persona name from agent display name (same logic as processMessage)
        std::string persona_name = agent_name;
        size_t paren_pos = persona_name.find('(');
        if (paren_pos != std::string::npos) {
            persona_name = persona_name.substr(0, paren_pos);
            // Trim whitespace
            while (!persona_name.empty() && std::isspace(persona_name.back())) {
                persona_name.pop_back();
            }
        }
        
        // Store persona name safely without initializing controller (prevents deadlock)
        mCurrentPersona = persona_name;
        mPersonaSet = true;
        LL_INFOS("AI") << "[AI] Persona name stored for safe initialization: " << persona_name << LL_ENDL;
        
    } catch (const std::exception& e) {
        LL_WARNS("AI") << "[AI] Error setting up persona: " << e.what() << LL_ENDL;
    }
}

// Static method called when login completes
void FSAIIntegration::onLoginCompleted() {
    // Only initialize if AI is enabled
    if (gSavedSettings.getBOOL("EnablePythonAI")) {
        LL_INFOS("AI") << "[AI] Login completed, setting up AI system..." << LL_ENDL;
        try {
            // Create singleton instance if needed and perform setup
            FSAIIntegration::getInstance()->performLoginTimeSetup();
        } catch (const std::exception& e) {
            LL_WARNS("AI") << "[AI] Error during login-time AI setup: " << e.what() << LL_ENDL;
        }
    }
}

void FSAIIntegration::performLoginTimeSetup() {
    try {
        LL_INFOS("AI") << "[AI] Performing login-time AI setup..." << LL_ENDL;
        
        // Get user ID safely
        LLUUID agent_id = gAgent.getID();
        if (!agent_id.isNull()) {
            std::string user_id = agent_id.asString();
            
            // Set up user ID if different from current
            if (mCurrentUserId != user_id) {
                mCurrentUserId = user_id;
                mUserIdSet = true;
                LL_INFOS("AI") << "[AI] User ID set: " << user_id << LL_ENDL;
                
                // Set up user ID in AI process bridge for ChromaDB collection
                if (mAIBridge && mAIBridge->isInitialized()) {
                    // This is a safe operation that doesn't cause deadlock
                    mAIBridge->setUserIdAsync(user_id);
                }
            }
            
            // Get persona name safely  
            LLAvatarName av_name;
            if (LLAvatarNameCache::get(agent_id, &av_name)) {
                std::string complete_name = av_name.getCompleteName();
                if (!complete_name.empty()) {
                    // Extract persona name (same logic as extractPersonaFromLoggedInAgent)
                    std::string persona_name = complete_name;
                    size_t paren_pos = persona_name.find('(');
                    if (paren_pos != std::string::npos) {
                        persona_name = persona_name.substr(0, paren_pos);
                        // Trim whitespace
                        while (!persona_name.empty() && std::isspace(persona_name.back())) {
                            persona_name.pop_back();
                        }
                    }
                    
                    // Store persona name WITHOUT initializing controller (prevents deadlock)
                    mCurrentPersona = persona_name;
                    mPersonaSet = true;
                    
                    // Also set the persona in the process bridge so it knows what persona to use
            if (mAIBridge && mAIBridge->isInitialized()) {
                LL_INFOS("AI") << "[AI] Persona stored for session (server ignores switch): " << persona_name << LL_ENDL;
            }
                    
                    LL_INFOS("AI") << "[AI] Persona name stored for safe initialization: " << persona_name << LL_ENDL;
                    
                    // 🚀 AUTO-START AI PROCESS! 🚀
                    if (!startAI("")) {
                        LL_WARNS("AI") << "[AI] Failed to auto-connect AI server during login setup" << LL_ENDL;
                    } else {
                        LL_INFOS("AI") << "[AI] ✅ AI server auto-connected successfully!" << LL_ENDL;
                    }
                }
            } else {
                LL_INFOS("AI") << "[AI] Avatar name not available yet, will retry during update loop" << LL_ENDL;
            }
        }
        
    } catch (const std::exception& e) {
        LL_WARNS("AI") << "[AI] Error in performLoginTimeSetup: " << e.what() << LL_ENDL;
    }
}

void FSAIIntegration::requestEnvironmentUpdate()
{
    LL_INFOS("AI") << "[AI] Environment update requested via public method" << LL_ENDL;
    
    if (!isAIEnabled()) {
        LL_WARNS("AI") << "[AI] Cannot provide environment update - AI is disabled" << LL_ENDL;
        return;
    }
    
    try {
        // Call the private method to gather and send environment data
        gatherAndSendEnvironmentUpdate();
    } catch (const std::exception& e) {
        LL_WARNS("AI") << "[AI] Error in requestEnvironmentUpdate: " << e.what() << LL_ENDL;
    }
}
