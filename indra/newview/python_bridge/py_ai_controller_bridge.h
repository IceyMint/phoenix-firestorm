/*
 * Thread-Safe Python-C++ Bridge for Genesis AI Integration
 * 
 * CRITICAL THREAD SAFETY DESIGN:
 * 
 * This implementation addresses critical race conditions and thread safety issues
 * identified in the original bridge layer. The architecture provides:
 * 
 * 1. RAII GIL MANAGEMENT:
 *    - GILGuard class automatically manages Python GIL acquisition/release
 *    - Prevents GIL state corruption in error paths
 *    - Exception-safe resource management
 * 
 * 2. THREAD-SAFE PYTHON OBJECT STORAGE:
 *    - SafePythonObject wrapper protects Python object access
 *    - Mutex-protected operations prevent race conditions
 *    - Proper reference counting across thread boundaries
 * 
 * 3. TASK-BASED PYTHON EXECUTION:
 *    - All Python operations queued through thread-safe task system
 *    - Background thread processes Python tasks with proper GIL management
 *    - Async/sync boundary clearly defined and controlled
 * 
 * 4. ATOMIC STATE MANAGEMENT:
 *    - std::atomic flags for thread-safe initialization/shutdown
 *    - Mutex-protected command handlers and callbacks
 *    - No data races on shared state
 * 
 * 5. PROPER LIFECYCLE MANAGEMENT:
 *    - Clean shutdown prevents deadlocks and resource leaks
 *    - Background thread lifecycle properly managed
 *    - Graceful error handling and recovery
 * 
 * THREADING MODEL:
 * - Main Thread: C++ game thread, calls bridge methods (non-blocking)
 * - Python Thread: Dedicated background thread for all Python operations
 * - Task Queue: Thread-safe communication between main and Python threads
 * 
 * USAGE CONSTRAINTS:
 * - ALL Python operations must go through the task system
 * - Never call Python API directly from main thread
 * - Always use GILGuard when accessing Python objects
 * - Shutdown must be called before destruction
 */

#ifndef PY_AI_CONTROLLER_BRIDGE_H
#define PY_AI_CONTROLLER_BRIDGE_H

// Python.h must be included first to avoid macro conflicts
#include <Python.h>

#include <string>
#include <functional>
#include <vector>
#include <map>
#include <memory>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <future>
#include "ai_message_queue.h"
#include "queue_processor.h"

// Forward declarations for pybind11 types
namespace pybind11 {
    class object;
    class args;
    class dict;
}

namespace python_bridge {

    // RAII GIL management helper
    class GILGuard {
    public:
        GILGuard() : gstate_(PyGILState_Ensure()) {}
        ~GILGuard() { PyGILState_Release(gstate_); }
        GILGuard(const GILGuard&) = delete;
        GILGuard& operator=(const GILGuard&) = delete;
    private:
        PyGILState_STATE gstate_;
    };

    // Thread-safe Python object wrapper
    class SafePythonObject {
    public:
        SafePythonObject() = default;
        explicit SafePythonObject(pybind11::object obj);
        SafePythonObject(const SafePythonObject& other);
        SafePythonObject& operator=(const SafePythonObject& other);
        SafePythonObject(SafePythonObject&& other) noexcept;
        SafePythonObject& operator=(SafePythonObject&& other) noexcept;
        ~SafePythonObject();

        pybind11::object get() const;
        bool is_none() const;
        void reset();
        void reset(pybind11::object obj);

    private:
        mutable std::mutex mutex_;
        pybind11::object* obj_ptr_ = nullptr;
    };

    // Async task for Python operations
    struct PythonTask {
        std::function<void()> task;
        std::promise<void> promise;
        bool is_shutdown = false;
    };

    // Command types for AI controller communication
    enum class AICommand {
        CHAT,           // Send message to viewer
        EMOTE,          // Send emote action
        MOVE,           // Move agent to position
        TOUCH,          // Touch object with UUID
        SIT,            // Sit on object
        STAND,          // Stand up
        LOOK_AT,        // Look at target
        WALK_TO,        // Walk to specific location
        TURN_TO,        // Turn to face direction
        GESTURE,        // Play gesture animation
        STATUS_UPDATE   // Update internal status
    };

    // Command data structure
    struct AICommandData {
        AICommand command_type;
        std::string target_uuid;        // For TOUCH, SIT commands
        std::string content;            // For CHAT, EMOTE commands
        float x = 0.0f, y = 0.0f, z = 0.0f;  // For MOVE, LOOK_AT commands
        std::map<std::string, std::string> parameters;  // Additional parameters
        
        AICommandData(AICommand type) : command_type(type) {}
    };

    // Result structure for command execution
    struct AICommandResult {
        bool success = false;
        std::string error_message;
        std::map<std::string, std::string> result_data;
    };

    // Callback types
    using CommandCallback = std::function<void(const AICommandData&)>;
    using StatusCallback = std::function<void(const std::string& status, const std::map<std::string, std::string>& data)>;
    using ResultCallback = std::function<void(const AICommandResult& result)>;

    class AIControllerBridge {
    private:
        static AIControllerBridge* instance_;
        static std::mutex instance_mutex_;
        
        // Thread-safe state management
        mutable std::mutex state_mutex_;
        std::map<AICommand, CommandCallback> command_handlers_;
        StatusCallback status_callback_;
        std::atomic<bool> initialized_;
        std::string current_persona_;
        
        // Message queue integration
        std::unique_ptr<MessageQueueInterface> message_queue_;
        std::unique_ptr<QueueProcessor> queue_processor_;
        bool queue_enabled_;
        
        // Python thread management
        std::atomic<bool> shutdown_requested_;
        std::atomic<bool> python_thread_running_;
        std::thread python_thread_;
        
        // Thread-safe task queue for Python operations
        std::queue<PythonTask> python_task_queue_;
        std::mutex task_queue_mutex_;
        std::condition_variable task_condition_;
        
        // Thread-safe Python object storage
        SafePythonObject ai_controller_module_;
        SafePythonObject controller_instance_;
        SafePythonObject asyncio_loop_;
        
        // Friend declaration to allow access to private members from C API
        friend void ai_controller_set_user_id_impl(const char* user_id);

    public:
        static AIControllerBridge& getInstance();
        
        // Core lifecycle
        bool initialize();
        void shutdown();
        bool isInitialized() const { return initialized_.load(); }
        
        // Command handler registration
        void registerCommandHandler(AICommand command, CommandCallback handler);
        void setStatusCallback(StatusCallback callback);
        
        // AI Controller Management
        bool startAIController(const std::string& persona_name = "");
        bool stopAIController();
        bool switchPersona(const std::string& persona_name);
        std::string getCurrentPersona() const { return current_persona_; }
        void setCurrentPersona(const std::string& persona_name) { current_persona_ = persona_name; }
        
        // Message processing (replaces the old addMessage function)
        bool processMessage(
            const std::string& text,
            const std::string& from_name,
            const std::string& from_id,
            const std::string& spatial_context,
            const std::string& logged_in_agent
        );
        
        // Command execution (called by Python AI)
        void executeCommand(const AICommandData& command_data);
        
        // Status and feedback
        void sendCommandResult(const std::string& command_id, const AICommandResult& result);
        void requestStatus();
        std::map<std::string, std::string> getSystemStatus();
        
        // Environment updates
        void updateEnvironment(
            const std::vector<std::pair<std::string, std::string>>& agents,
            const std::vector<std::pair<std::string, std::string>>& objects,
            const std::string& current_location = ""
        );
        
        // Message queue management
        bool initializeMessageQueue(MessageQueueInterface::QueueType queue_type = MessageQueueInterface::QueueType::IN_MEMORY);
        void shutdownMessageQueue();
        void processMessageQueue(int max_commands_per_frame = 10);
        MessageQueueInterface* getMessageQueue() { return message_queue_.get(); }
        QueueProcessor* getQueueProcessor() { return queue_processor_.get(); }
        
        // Queue statistics and monitoring
        std::map<std::string, std::string> getQueueStatistics();
        bool isMessageQueueEnabled() const { return queue_enabled_; }
        
        // Emergency controls
        void emergencyStop();
        void pauseAutonomousBehavior();
        void resumeAutonomousBehavior();
        
        // Public async wrappers for external access (fire-and-forget)
        void setUserIdAsync(const std::string& user_id);
        void processMessageAsync(const std::string& text, const std::string& from_name, 
                               const std::string& from_id, const std::string& spatial_context, 
                               const std::string& logged_in_agent);
        
        // Configuration updates
        void updatePersonaConfig(const std::map<std::string, std::string>& config_updates);
        
        // Utility functions
        static std::string commandToString(AICommand command);
        static AICommand stringToCommand(const std::string& command_str);
        
    private:
        AIControllerBridge();
        ~AIControllerBridge();
        
        // Prevent copy/assignment
        AIControllerBridge(const AIControllerBridge&) = delete;
        AIControllerBridge& operator=(const AIControllerBridge&) = delete;
        
        // Internal helper methods
        void callPythonFunction(const std::string& function_name, const std::map<std::string, std::string>& parameters);
        bool ensurePythonInterpreter();
        void handlePythonError(const std::string& context);
        std::string extractPersonaFromLoggedInAgent(const std::string& logged_in_agent);
        
        // Thread-safe Python task execution
        std::future<void> enqueuePythonTask(std::function<void()> task);
        void executePythonTaskSync(std::function<void()> task);
        
        // Legacy compatibility
        pybind11::object runAsyncFunction(pybind11::object func, pybind11::args args);
        void handlePythonCommand(const std::string& command_str, const pybind11::dict& data);
        
        // Background thread management
        void pythonThreadLoop();
        void shutdownPythonThread();
        
        // Internal implementation for C API functions
        void setUserIdInternal(const char* user_id);
        
        // Background message processing
        void processMessageInBackground(const std::string& text, const std::string& from_name, 
                                      const std::string& from_id, const std::string& spatial_context, 
                                      const std::string& logged_in_agent);
    };


    // Helper macros for command registration
    #define REGISTER_AI_COMMAND(command_type, handler_func) \
        AIControllerBridge::getInstance().registerCommandHandler(command_type, handler_func)

    #define REGISTER_AI_STATUS_CALLBACK(callback_func) \
        AIControllerBridge::getInstance().setStatusCallback(callback_func)

} // namespace python_bridge

// Global functions for easier C integration
extern "C" {
    // Initialize the AI controller bridge
    bool ai_controller_init();
    
    // Shutdown the AI controller bridge
    void ai_controller_shutdown();
    
    // Start AI controller with persona
    bool ai_controller_start(const char* persona_name);
    
    // Stop AI controller
    bool ai_controller_stop();
    
    // Set user ID for ChromaDB collection setup
    void ai_controller_set_user_id(const char* user_id);
    
    // Get current display name for persona
    const char* ai_controller_get_display_name();
    
    // Process incoming message
    bool ai_controller_process_message(
        const char* text,
        const char* from_name,
        const char* from_id,
        const char* spatial_context,
        const char* logged_in_agent
    );
    
    // Register command handlers
    void ai_controller_register_chat_handler(void (*handler)(const char* content));
    void ai_controller_register_emote_handler(void (*handler)(const char* content));
    void ai_controller_register_move_handler(void (*handler)(float x, float y, float z));
    void ai_controller_register_touch_handler(void (*handler)(const char* object_uuid));
    void ai_controller_register_sit_handler(void (*handler)(const char* object_uuid));
    void ai_controller_register_stand_handler(void (*handler)());
    void ai_controller_register_look_at_handler(void (*handler)(float x, float y, float z));
    
    // Update environment
    void ai_controller_update_environment(
        const char* agents_data,    // JSON string with agent positions
        const char* objects_data,   // JSON string with object positions
        const char* location_data   // Current location info
    );
    
    // Emergency controls
    void ai_controller_emergency_stop();
    void ai_controller_pause();
    void ai_controller_resume();
    
    // Get system status (returns JSON string)
    const char* ai_controller_get_status();
    
    // Send command execution result back to Python
    void ai_controller_send_result(const char* command_id, bool success, const char* error_message);
}

#endif // PY_AI_CONTROLLER_BRIDGE_H