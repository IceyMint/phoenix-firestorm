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
#include <map>
#include <memory>

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

struct MoveWaypointCallbackData
{
    U64 plan_id;
    LLVector3d final_target;
};

U64 g_move_plan_generation = 0;

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

bool is_pathfinding_ready()
{
    LLPathfindingManager* manager = LLPathfindingManager::getInstance();
    if (!manager || !manager->isPathfindingEnabledForCurrentRegion())
    {
        return false;
    }
    LLPathingLib* pathing = LLPathingLib::getInstance();
    return (pathing != nullptr) && pathing->isFunctional();
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

bool find_probe_waypoint(const LLVector3d& start, const LLVector3d& goal, LLVector3d& waypoint_out)
{
    static const F32 kRadii[] = {2.0f, 3.5f, 5.0f, 7.0f};
    static const int kSteps = 12;
    static const F64 kTwoPi = 6.28318530717958647692;

    F64 best_score = std::numeric_limits<F64>::max();
    bool found = false;

    for (F32 radius : kRadii)
    {
        for (int i = 0; i < kSteps; ++i)
        {
            const F64 theta = (kTwoPi * i) / static_cast<F64>(kSteps);
            LLVector3d candidate(
                goal.mdV[0] + std::cos(theta) * radius,
                goal.mdV[1] + std::sin(theta) * radius,
                goal.mdV[2]
            );

            if (!probe_path_exists(start, candidate))
            {
                continue;
            }
            if (!probe_path_exists(candidate, goal))
            {
                continue;
            }

            const F64 score = dist_vec(start, candidate) + dist_vec(candidate, goal);
            if (score < best_score)
            {
                best_score = score;
                waypoint_out = candidate;
                found = true;
            }
        }
    }
    return found;
}

void ai_move_waypoint_callback(bool success, void* user_data)
{
    MoveWaypointCallbackData* cb_data = static_cast<MoveWaypointCallbackData*>(user_data);
    if (!cb_data)
    {
        return;
    }

    const U64 callback_plan_id = cb_data->plan_id;
    const LLVector3d final_target = cb_data->final_target;
    delete cb_data;

    if (callback_plan_id != g_move_plan_generation)
    {
        return; // stale plan
    }

    if (!success)
    {
        LL_WARNS("AI") << "[AI] Waypoint leg did not complete; skipping final leg" << LL_ENDL;
        return;
    }

    gAgent.startAutoPilotGlobal(
        final_target,
        "AI Movement Final",
        NULL,
        NULL,
        NULL,
        kMoveAutopilotStopDistanceMeters,
        kMoveAutopilotRotationThreshold,
        false
    );

    LL_INFOS("AI") << "[AI] Waypoint reached; final leg started to ("
                   << final_target.mdV[0] << ", " << final_target.mdV[1] << ", " << final_target.mdV[2] << ")"
                   << LL_ENDL;
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

            LL_INFOS("AI") << "[AI] Environment update sent via TCP: " << agents_str.size() 
                           << " agents, " << objects_str.size() << " objects in " 
                           << current_location << LL_ENDL;
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
    // Get nearby objects from area search if available
    FSAreaSearch* areaSearch = LLFloaterReg::findTypedInstance<FSAreaSearch>("area_search");
    if (!areaSearch || areaSearch->mObjectDetails.empty())
    {
        return;
    }

    LLVector3d agentPos    = gAgent.getPositionGlobal();
    const F32  maxDistance = 10.0f;

    for (const auto& [objectId, props] : areaSearch->mObjectDetails)
    {
        if (!props.listed)
            continue;

        LLViewerObject* objectp = gObjectList.findObject(objectId);
        if (!objectp || objectp->isDead() || objectp->getRegion() != gAgent.getRegion() || objectp->isAttachment())
            continue;

        LLVector3d objectPos = objectp->getPositionGlobal();
        F32        distance  = (F32)dist_vec(agentPos, objectPos);

        // LL_INFOS("AI") << "[AI] Found object: " << props.name << " at distance: " << distance << "m" << LL_ENDL;

        if (distance <= maxDistance)
        {
            std::string objectName = props.name.empty() ? "Unknown Object" : props.name;
            std::string pos_str    = "(" + std::to_string(objectPos.mdV[0]) + ", " + std::to_string(objectPos.mdV[1]) + ", " +
                                  std::to_string(objectPos.mdV[2]) + ")";
            std::string uuid_str = objectId.asString();                     // Add UUID like agents do
            objects_str.emplace_back(objectName + "|" + uuid_str, pos_str); // Include UUID in name field
        }
    }

    // Sort by distance (closest first)
    LLVector3d agentPos_final = agentPos; // Capture for lambda
    std::sort(objects_str.begin(), objects_str.end(), 
        [agentPos_final](const auto& a, const auto& b) {
            // Simple distance comparison by parsing position strings
            // This is approximate sorting - exact distances would require parsing
            return a.first < b.first; // Fallback to name sorting
        });
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

    // Every MOVE invalidates any prior two-hop callback plan.
    const U64 plan_id = ++g_move_plan_generation;

    // Person-aware movement: use follow pilot with standoff to avoid bumping avatars.
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
        if (avatar_distance <= standoff)
        {
            gAgent.stopAutoPilot();
            LL_INFOS("AI") << "[AI] Already within standoff of avatar target; no MOVE needed" << LL_ENDL;
            return;
        }
        gAgent.stopAutoPilot();
        gAgent.startFollowPilot(person_target_id, false, standoff);
        LL_INFOS("AI") << "[AI] Person-aware movement initiated via follow pilot to " << person_target_id.asString()
                       << " with standoff " << standoff << "m" << LL_ENDL;
        return;
    }

    // Non-avatar movement target.
    if (distance <= kMoveAutopilotStopDistanceMeters)
    {
        gAgent.stopAutoPilot();
        LL_INFOS("AI") << "[AI] Target already within " << kMoveAutopilotStopDistanceMeters << "m; no MOVE needed" << LL_ENDL;
        return;
    }

    bool direct_path_available = false;
    bool waypoint_planned = false;
    LLVector3d waypoint_target;
    const bool pathfinding_ready = is_pathfinding_ready();
    if (pathfinding_ready)
    {
        direct_path_available = probe_path_exists(agent_pos, target_pos);
        if (!direct_path_available)
        {
            waypoint_planned = find_probe_waypoint(agent_pos, target_pos, waypoint_target);
        }
        LL_INFOS("AI") << "[AI] Path probe result: direct=" << (direct_path_available ? "yes" : "no")
                       << " waypoint=" << (waypoint_planned ? "yes" : "no") << LL_ENDL;
    }
    else
    {
        LL_INFOS("AI") << "[AI] Pathfinding unavailable for current region; using direct autopilot fallback" << LL_ENDL;
    }

    gAgent.stopAutoPilot();

    if (waypoint_planned)
    {
        MoveWaypointCallbackData* cb_data = new MoveWaypointCallbackData();
        cb_data->plan_id = plan_id;
        cb_data->final_target = target_pos;
        gAgent.startAutoPilotGlobal(
            waypoint_target,
            "AI Movement Waypoint",
            NULL,
            ai_move_waypoint_callback,
            cb_data,
            kMoveAutopilotStopDistanceMeters,
            kMoveAutopilotRotationThreshold,
            false
        );

        LL_INFOS("AI") << "[AI] Two-hop movement: waypoint (" << waypoint_target.mdV[0] << ", "
                       << waypoint_target.mdV[1] << ", " << waypoint_target.mdV[2] << ")"
                       << " then goal (" << target_pos.mdV[0] << ", " << target_pos.mdV[1] << ", " << target_pos.mdV[2] << ")"
                       << LL_ENDL;
        return;
    }

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
