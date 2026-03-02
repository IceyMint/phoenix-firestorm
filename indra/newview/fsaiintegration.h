#ifndef FS_AI_INTEGRATION_H
#define FS_AI_INTEGRATION_H

#include "llsingleton.h"
#include "lltimer.h"
#include "llchat.h"
#include <vector>
#include <string>
#include <memory>

// Forward declarations
class FSAreaSearch;
class AIProcessBridge;

/**
 * @brief Handles all AI integration functionality for the Firestorm viewer
 * 
 * This class manages the interface between the viewer and the Python AI system,
 * including environment monitoring, message processing, and command handling.
 * It keeps AI-related code isolated from core viewer components.
 */
class FSAIIntegration : public LLSingleton<FSAIIntegration>
{
    LLSINGLETON(FSAIIntegration);
    ~FSAIIntegration();

public:
    /**
     * @brief Called when viewer login completes to initialize AI system
     */
    static void onLoginCompleted();

    /**
     * @brief Process an incoming chat message through the AI system
     * @param chat The chat message to process
     */
    void processMessage(const LLChat& chat);

    /**
     * @brief Update environment monitoring (call from main update loop)
     * 
     * This performs periodic environment monitoring and sends updates
     * to the Python AI system when significant changes are detected.
     */
    void updateEnvironmentMonitoring();

    /**
     * @brief Check if AI integration is enabled and ready
     * @return true if AI is enabled and operational
     */
    bool isAIEnabled() const;

    /**
     * @brief Emergency stop all AI operations
     */
    void emergencyStop();

    /**
     * @brief Pause autonomous AI behavior
     */
    void pauseAutonomousBehavior();

    /**
     * @brief Resume autonomous AI behavior
     */
    void resumeAutonomousBehavior();

    /**
     * @brief Get AI system status information
     * @return Status information as key-value pairs
     */
    std::map<std::string, std::string> getAIStatus();
    
    // 🔥 HOT RESTART CAPABILITIES! 🔥
    
    /**
     * @brief Hot restart the AI system (no viewer restart needed!)
     * @return true if restart was successful
     */
    bool hotRestartAI();
    
    /**
     * @brief Stop the AI process 
     */
    void stopAI();
    
    /**
     * @brief Start the AI process
     * @param script_path Path to the Python AI script
     * @return true if started successfully
     */
    bool startAI(const std::string& script_path = "ai_process.py");
    
    /**
     * @brief Process incoming AI messages (call from game loop!)
     * This replaces the old threading callback system
     */
    void processAIMessages();
    
    /**
     * @brief Enable/disable auto-restart on AI crashes
     * @param enable true to enable auto-restart
     */
    void setAutoRestart(bool enable);
    
    /**
     * @brief Check if AI connection is active
     * @return true if Python process is connected
     */
    bool isConnectionActive() const;
    
    /**
     * @brief Wait for Python process to reconnect (for enable/disable toggle)
     * @return true if reconnection successful
     */
    bool waitForReconnection();
    
    /**
     * @brief Non-blocking check for Python process reconnection
     * @return true if connected or reconnected
     */
    bool checkForReconnection();
    
    /**
     * @brief Called when AI is disabled via setting toggle
     */
    void onAIDisabled();
    
    /**
     * @brief Called when AI is enabled via setting toggle
     */
    void onAIEnabled();

    bool isLLMChatEnabled() const;
    bool setLLMChatEnabled(bool enabled);
    bool toggleLLMChatEnabled();

    /**
     * @brief Public method to request environment update (called by Python AI via TCP)
     */
    void requestEnvironmentUpdate();

private:
    /**
     * @brief Gather current environment state and send to AI bridge
     */
    void gatherAndSendEnvironmentUpdate();

    /**
     * @brief Check if agent is logged in and set up user ID for AI system
     */
    void checkAndSetupAgentLogin();
    
    /**
     * @brief Set up AI persona once during login
     */
    void setupPersonaOnLogin(const std::string& agent_name);
    
    /**
     * @brief Perform safe login-time AI setup
     */
    void performLoginTimeSetup();

    /**
     * @brief Collect nearby agent information
     * @param agents_str Output vector of (name, position) pairs
     */
    void gatherNearbyAgents(std::vector<std::pair<std::string, std::string>>& agents_str);

    /**
     * @brief Collect nearby object information
     * @param objects_str Output vector of (name, position) pairs
     */
    void gatherNearbyObjects(std::vector<std::pair<std::string, std::string>>& objects_str);

    /**
     * @brief Get current location information
     * @return Region name or location string
     */
    std::string getCurrentLocation();

    /**
     * @brief Initialize command handlers for AI bridge
     */
    void initializeCommandHandlers();

    // Static command handler functions (called by C bridge)
    static void handleAIChatOutput(const char* content, S32 channel = 0);
    static void handleAIEmoteOutput(const char* content, S32 channel = 0);
    static void handleAIMovementOutput(float x, float y, float z, const char* target_uuid = nullptr, float desired_standoff = 0.9f);
    static void handleAITouchOutput(const char* object_uuid);
    static void handleAISitOutput(const char* object_uuid);
    static void handleAIStandOutput();
    static void handleAILookAtOutput(float x, float y, float z);
    static void handleAIFaceTargetOutput(const char* target_uuid, float x, float y, float z);
    static void handle_AIFollowOutput(const char* target_uuid);

private:
    // AI Process Bridge (Hot Restart Capable! 🔥)
    std::unique_ptr<AIProcessBridge> mAIBridge;
    
    // Environment monitoring state
    LLTimer mEnvironmentUpdateTimer;
    bool mMonitoringEnabled;
    bool mCommandHandlersInitialized;

    // Agent login detection
    bool mUserIdSet;
    std::string mCurrentUserId;
    bool mPersonaSet;
    std::string mCurrentPersona;

    // Performance tracking
    U32 mEnvironmentUpdateCount;
    U32 mMessageProcessCount;
    
    // Enable/disable state tracking
    bool mLastAIEnabledState;
    bool mWaitingForReconnection;
    bool mLLMChatEnabled;
};

#endif // FS_AI_INTEGRATION_H
