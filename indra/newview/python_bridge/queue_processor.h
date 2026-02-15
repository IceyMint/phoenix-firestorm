#ifndef QUEUE_PROCESSOR_H
#define QUEUE_PROCESSOR_H

#include "ai_message_queue.h"
#include <functional>
#include <string>
#include <map>
#include <atomic>
#include <mutex>
#include <chrono>

namespace python_bridge {

/**
 * Queue Processor for AI Commands
 * 
 * This class integrates with the game loop to process commands from the 
 * message queue without blocking the main thread. It replaces the fragile
 * callback system with a robust, buffered command processing architecture.
 * 
 * FEATURES:
 * - Non-blocking queue processing in game loop
 * - Command validation and error handling
 * - Performance monitoring and statistics
 * - Graceful degradation when queue is full
 * - Easy integration with existing AI systems
 * 
 * THREAD SAFETY:
 * - All operations are thread-safe
 * - Can be called from main game loop thread
 * - Message queue handles cross-thread communication
 * 
 * RELIABILITY:
 * - Commands survive process restarts (with persistent queue)
 * - No silent failures like callback system
 * - Comprehensive error reporting
 * - Queue overflow handling
 */
class QueueProcessor {
public:
    // Command handler function types
    using ChatHandler = std::function<void(const std::string& content)>;
    using EmoteHandler = std::function<void(const std::string& content)>;
    using MoveHandler = std::function<void(float x, float y, float z)>;
    using TouchHandler = std::function<void(const std::string& object_uuid)>;
    using SitHandler = std::function<void(const std::string& object_uuid)>;
    using StandHandler = std::function<void()>;
    using LookAtHandler = std::function<void(float x, float y, float z)>;
    
    // Performance statistics
    struct ProcessorStats {
        uint64_t commands_processed = 0;
        uint64_t commands_failed = 0;
        uint64_t validation_errors = 0;
        uint64_t handler_errors = 0;
        uint64_t json_parse_errors = 0;
        double avg_processing_time_ms = 0.0;
        double max_processing_time_ms = 0.0;
        std::chrono::steady_clock::time_point start_time;
        
        ProcessorStats() : start_time(std::chrono::steady_clock::now()) {}
        
        double getUptimeSeconds() const {
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
            return duration.count() / 1000.0;
        }
        
        double getProcessingRate() const {
            double uptime = getUptimeSeconds();
            return uptime > 0.0 ? commands_processed / uptime : 0.0;
        }
    };

public:
    QueueProcessor();
    ~QueueProcessor();
    
    // Lifecycle management
    bool initialize(MessageQueueInterface* queue);
    void shutdown();
    bool isInitialized() const { return initialized_; }
    
    // Handler registration
    void registerChatHandler(ChatHandler handler);
    void registerEmoteHandler(EmoteHandler handler);
    void registerMoveHandler(MoveHandler handler);
    void registerTouchHandler(TouchHandler handler);
    void registerSitHandler(SitHandler handler);
    void registerStandHandler(StandHandler handler);
    void registerLookAtHandler(LookAtHandler handler);
    
    // Main processing method (call from game loop)
    void processCommands(int max_commands_per_frame = 10);
    
    // Statistics and monitoring
    ProcessorStats getStatistics() const;
    void resetStatistics();
    
    // Configuration
    void setMaxCommandsPerFrame(int max_commands) {
        max_commands_per_frame_ = max_commands;
    }
    
    void setEnableDetailedLogging(bool enabled) {
        detailed_logging_ = enabled;
    }

private:
    // Command processing
    bool processCommand(const std::string& json_command);
    bool parseAndExecuteCommand(const std::string& json_command);
    
    // Command handlers
    bool executeChatCommand(const std::map<std::string, std::string>& parameters);
    bool executeEmoteCommand(const std::map<std::string, std::string>& parameters);
    bool executeMoveCommand(const std::map<std::string, std::string>& parameters);
    bool executeTouchCommand(const std::map<std::string, std::string>& parameters);
    bool executeSitCommand(const std::map<std::string, std::string>& parameters);
    bool executeStandCommand(const std::map<std::string, std::string>& parameters);
    bool executeLookAtCommand(const std::map<std::string, std::string>& parameters);
    
    // Validation helpers
    bool validateChatParameters(const std::map<std::string, std::string>& params);
    bool validateEmoteParameters(const std::map<std::string, std::string>& params);
    bool validateMoveParameters(const std::map<std::string, std::string>& params);
    bool validateTouchParameters(const std::map<std::string, std::string>& params);
    bool validateSitParameters(const std::map<std::string, std::string>& params);
    bool validateLookAtParameters(const std::map<std::string, std::string>& params);
    
    // Utility functions
    bool parseFloat(const std::string& str, float& result);
    void recordProcessingTime(double time_ms);
    void logCommandExecution(const std::string& command_type, bool success, 
                           const std::string& details = "");

private:
    // State
    std::atomic<bool> initialized_{false};
    MessageQueueInterface* message_queue_;
    int max_commands_per_frame_;
    bool detailed_logging_;
    
    // Thread safety
    mutable std::mutex stats_mutex_;
    mutable std::mutex handlers_mutex_;
    
    // Command handlers
    ChatHandler chat_handler_;
    EmoteHandler emote_handler_;
    MoveHandler move_handler_;
    TouchHandler touch_handler_;
    SitHandler sit_handler_;
    StandHandler stand_handler_;
    LookAtHandler look_at_handler_;
    
    // Statistics
    mutable ProcessorStats stats_;
    mutable std::vector<double> processing_times_;  // For calculating averages
};

/**
 * Integration helper for existing AI systems
 * 
 * This provides a simple interface for integrating the queue processor
 * with existing AI bridge code.
 */
class QueueProcessorIntegration {
public:
    static QueueProcessor& getInstance();
    
    // Easy integration with existing handler registration
    static void initializeWithHandlers(
        MessageQueueInterface* queue,
        std::function<void(const char*)> chat_handler,
        std::function<void(const char*)> emote_handler,
        std::function<void(float, float, float)> move_handler,
        std::function<void(const char*)> touch_handler,
        std::function<void(const char*)> sit_handler,
        std::function<void()> stand_handler,
        std::function<void(float, float, float)> look_at_handler
    );
    
    // Game loop integration
    static void processCommandsInGameLoop(int max_commands_per_frame = 10);
    
    // Statistics access
    static QueueProcessor::ProcessorStats getProcessingStatistics();

private:
    static QueueProcessor instance_;
    static std::mutex instance_mutex_;
};

// Convenience macros for game loop integration
#define AI_QUEUE_PROCESS_COMMANDS() \
    QueueProcessorIntegration::processCommandsInGameLoop()

#define AI_QUEUE_PROCESS_COMMANDS_LIMITED(max_commands) \
    QueueProcessorIntegration::processCommandsInGameLoop(max_commands)

} // namespace python_bridge

#endif // QUEUE_PROCESSOR_H