/**
 * Example integration of the new AI Controller Bridge system
 * This shows how to integrate the command-based AI system into the viewer
 */

#include "py_ai_controller_bridge.h"
#include "llviewercontrol.h"
#include "llviewerregion.h"
#include "llagent.h"
#include "llavatarnamecache.h"
#include "llchatbar.h"
#include "llnotificationsutil.h"

using namespace python_bridge;

// Example chat output handler
void handleAIChatOutput(const char* content) {
    if (!content) return;
    
    std::string message(content);
    
    // Send to local chat
    LLViewerRegion* region = gAgent.getRegion();
    if (region) {
        // This would be the actual viewer chat sending mechanism
        // gAgent.sendChat(message, CHAT_TYPE_NORMAL);
        
        // For now, just log it
        LL_INFOS("AI") << "AI Chat: " << message << LL_ENDL;
    }
}

// Example emote handler
void handleAIEmote(const char* content) {
    if (!content) return;
    
    std::string emote(content);
    
    // Send emote to chat
    // This would typically be formatted as "/me <action>"
    if (!emote.empty()) {
        if (emote.find("/me ") != 0) {
            emote = "/me " + emote;
        }
        
        // Send the emote
        // gAgent.sendChat(emote, CHAT_TYPE_NORMAL);
        
        LL_INFOS("AI") << "AI Emote: " << emote << LL_ENDL;
    }
}

// Example movement handler
void handleAIMovement(float x, float y, float z) {
    LL_INFOS("AI") << "AI Movement command: (" << x << ", " << y << ", " << z << ")" << LL_ENDL;
    
    // This would implement actual avatar movement
    LLVector3 target_pos(x, y, z);
    
    // Example: Set autopilot to target position
    // gAgent.startAutoPilotGlobal(target_pos, "AI Movement", NULL, NULL, NULL, 0.5f, false);
    
    // Or for immediate teleport (if allowed):
    // gAgent.teleportViaLocationLookAt(target_pos);
}

// Example object touch handler
void handleAITouch(const char* object_uuid) {
    if (!object_uuid) return;
    
    LLUUID target_uuid(object_uuid);
    if (target_uuid.isNull()) return;
    
    LL_INFOS("AI") << "AI Touch command for object: " << target_uuid.asString() << LL_ENDL;
    
    // Find the object in the world
    LLViewerObject* object = gObjectList.findObject(target_uuid);
    if (object) {
        // Simulate a touch interaction
        // This would typically send a ObjectGrab message to the simulator
        LL_INFOS("AI") << "Touching object: " << object->getID().asString() << LL_ENDL;
        
        // Example touch implementation:
        // LLPickInfo pick_info;
        // pick_info.mObjectID = target_uuid;
        // pick_info.mPickType = LLPickInfo::PICK_OBJECT;
        // handle_object_touch(pick_info);
    } else {
        LL_WARNS("AI") << "Object not found for touch: " << target_uuid.asString() << LL_ENDL;
    }
}

// Example sit handler
void handleAISit(const char* object_uuid) {
    if (!object_uuid) return;
    
    LLUUID target_uuid(object_uuid);
    if (target_uuid.isNull()) return;
    
    LL_INFOS("AI") << "AI Sit command for object: " << target_uuid.asString() << LL_ENDL;
    
    // Find the object and attempt to sit
    LLViewerObject* object = gObjectList.findObject(target_uuid);
    if (object) {
        // This would implement sitting on the object
        // gAgent.sitOnObject(object);
        LL_INFOS("AI") << "Sitting on object: " << object->getID().asString() << LL_ENDL;
    } else {
        LL_WARNS("AI") << "Object not found for sitting: " << target_uuid.asString() << LL_ENDL;
    }
}

// Example stand handler
void handleAIStand() {
    LL_INFOS("AI") << "AI Stand command" << LL_ENDL;
    
    // Stand up if currently sitting
    // if (gAgent.getAvatarObject() && gAgent.getAvatarObject()->isSitting()) {
    //     gAgent.standUp();
    // }
}

// Example look at handler
void handleAILookAt(float x, float y, float z) {
    LL_INFOS("AI") << "AI Look At command: (" << x << ", " << y << ", " << z << ")" << LL_ENDL;
    
    LLVector3 look_at_pos(x, y, z);
    
    // Set the avatar to look at the target position
    // gAgent.lookAtObject(NULL, look_at_pos, LOOKAT_TARGET_FOCUS);
}

// Integration initialization function
bool initializeAISystem() {
    LL_INFOS("AI") << "Initializing AI Controller Bridge system" << LL_ENDL;
    
    // Initialize the bridge
    if (!ai_controller_init()) {
        LL_ERRS("AI") << "Failed to initialize AI Controller Bridge" << LL_ENDL;
        return false;
    }
    
    // Register command handlers
    ai_controller_register_chat_handler(handleAIChatOutput);
    ai_controller_register_emote_handler(handleAIEmote);
    ai_controller_register_move_handler(handleAIMovement);
    ai_controller_register_touch_handler(handleAITouch);
    ai_controller_register_sit_handler(handleAISit);
    ai_controller_register_stand_handler(handleAIStand);
    ai_controller_register_look_at_handler(handleAILookAt);
    
    // Start the AI controller with default persona
    if (!ai_controller_start("Tiffany")) {
        LL_ERRS("AI") << "Failed to start AI controller" << LL_ENDL;
        return false;
    }
    
    LL_INFOS("AI") << "AI Controller Bridge system initialized successfully" << LL_ENDL;
    return true;
}

// Shutdown function
void shutdownAISystem() {
    LL_INFOS("AI") << "Shutting down AI system" << LL_ENDL;
    ai_controller_stop();
    ai_controller_shutdown();
}

// Function to process incoming chat messages
void processIncomingMessage(const std::string& text, 
                          const std::string& from_name,
                          const std::string& from_id,
                          const LLVector3& speaker_pos,
                          const std::string& logged_in_agent) {
    
    // Build spatial context
    std::string spatial_context = buildSpatialContext(speaker_pos);
    
    // Process through AI controller
    ai_controller_process_message(
        text.c_str(),
        from_name.c_str(),
        from_id.c_str(),
        spatial_context.c_str(),
        logged_in_agent.c_str()
    );
}

// Helper function to build spatial context
std::string buildSpatialContext(const LLVector3& speaker_pos) {
    std::stringstream spatial_context;
    
    // Get nearby agents
    spatial_context << "AGENTS:";
    // This would iterate through nearby avatars
    // for (auto& avatar : nearby_avatars) {
    //     spatial_context << " [" << avatar.getName() << " @ (" 
    //                    << avatar.getPosition().mV[VX] << ", "
    //                    << avatar.getPosition().mV[VY] << ", "
    //                    << avatar.getPosition().mV[VZ] << ")]";
    // }
    
    spatial_context << "\nOBJECTS:";
    // This would iterate through nearby objects
    // for (auto& object : nearby_objects) {
    //     spatial_context << " [" << object.getName() << " @ (" 
    //                    << object.getPosition().mV[VX] << ", "
    //                    << object.getPosition().mV[VY] << ", "
    //                    << object.getPosition().mV[VZ] << ")]";
    // }
    
    return spatial_context.str();
}

// Emergency stop function (can be called from UI)
void emergencyStopAI() {
    LL_WARNS("AI") << "Emergency stop requested by user" << LL_ENDL;
    ai_controller_emergency_stop();
    
    // Show notification to user
    LLNotificationsUtil::add("AIEmergencyStop");
}

// Pause/Resume functions for user control
void pauseAI() {
    ai_controller_pause();
    LLNotificationsUtil::add("AIPaused");
}

void resumeAI() {
    ai_controller_resume();
    LLNotificationsUtil::add("AIResumed");
}

// Get AI status for UI display
std::string getAIStatus() {
    const char* status_json = ai_controller_get_status();
    return status_json ? std::string(status_json) : "{}";
}

// Example of how to integrate into the viewer's chat processing
class LLAIChatHandler {
public:
    static void processChatMessage(const LLChat& chat) {
        // Extract message details
        std::string text = chat.mText;
        std::string from_name = chat.mFromName;
        std::string from_id = chat.mFromID.asString();
        std::string logged_in_agent = gAgent.getID().asString();
        
        // Get speaker position
        LLVector3 speaker_pos = chat.mPosAgent;
        
        // Process through AI system
        processIncomingMessage(text, from_name, from_id, speaker_pos, logged_in_agent);
    }
    
    static void updateEnvironmentData() {
        // This would be called periodically to update the AI with environment changes
        // Build current environment data and send to AI
        // ai_controller_update_environment(...);
    }
};

// Integration points for viewer initialization
// This would be called during viewer startup
void LLAppViewer::initAI() {
    if (!initializeAISystem()) {
        LL_WARNS("AI") << "AI system initialization failed, continuing without AI" << LL_ENDL;
    }
}

// This would be called during viewer shutdown
void LLAppViewer::cleanupAI() {
    shutdownAISystem();
}