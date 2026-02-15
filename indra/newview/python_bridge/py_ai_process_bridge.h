/**
 * @file py_ai_process_bridge.h
 * @brief Process-based Python AI Bridge for Firestorm
 * 
 * Replaces the threading-based bridge with a process-based architecture for:
 * - Hot restart capability (no viewer restart needed!)
 * - Crash isolation (Python crashes don't affect game)
 * - Cross-platform compatibility (Windows/Linux)
 * - No GIL management complexity
 * 
 * @author Claude AI Assistant
 * @date August 2025
 */

#ifndef PY_AI_PROCESS_BRIDGE_H
#define PY_AI_PROCESS_BRIDGE_H

#include <string>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <chrono>
#include <memory>
#include <map>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    #pragma comment(lib, "ws2_32.lib")
    #define SOCKET_ERROR_CODE WSAGetLastError()
    #define CLOSE_SOCKET closesocket
    typedef SOCKET socket_t;
    typedef HANDLE process_handle_t;
#else
    #include <sys/socket.h>
    #include <sys/select.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <sys/wait.h>
    #include <fcntl.h>
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define SOCKET_ERROR_CODE errno
    #define CLOSE_SOCKET close
    typedef int socket_t;
    typedef pid_t process_handle_t;
#endif

// Forward declarations
class AIMessage;

/**
 * @brief AI command data structure for processing commands
 */
class AICommandData {
public:
    std::string command_type;           // Command type (CHAT, EMOTE, MOVE, etc.)
    std::string content;               // For CHAT, EMOTE commands
    std::string target_uuid;           // For TOUCH, SIT commands
    float x = 0.0f, y = 0.0f, z = 0.0f; // For MOVE, LOOK_AT commands
    std::map<std::string, std::string> parameters; // Additional parameters
    
    AICommandData() = default;
    AICommandData(const std::string& type) : command_type(type) {}
    AICommandData(const std::string& type, const std::string& cmd_content) 
        : command_type(type), content(cmd_content) {}
};

// Callback types
using CommandCallback = std::function<void(const AICommandData&)>;
using MessageCallback = std::function<void(const std::string&)>;

/**
 * @brief AI message for communication between C++ and Python
 */
class AIMessage {
public:
    std::string type;
    std::string id;
    std::string data;
    std::chrono::system_clock::time_point timestamp;
    
    AIMessage() : timestamp(std::chrono::system_clock::now()) {}
    AIMessage(const std::string& t, const std::string& d) 
        : type(t), data(d), timestamp(std::chrono::system_clock::now()) {}
    
    std::string toJSON() const;
    static AIMessage fromJSON(const std::string& json);
};

/**
 * @brief TCP client bridge for AI server communication
 *
 * Connects the viewer to an external AI server over TCP (NDJSON).
 */
class AIProcessBridge {
public:
    AIProcessBridge();
    ~AIProcessBridge();
    
    // Core lifecycle
    bool initialize(int port = 9999);
    void shutdown();
    bool isInitialized() const { return initialized_.load(); }
    
    // Connection management (server-side AI)
    bool startAIProcess(const std::string& script_path = "ai_process.py");
    void stopAIProcess();
    bool restartAIProcess();
    bool isProcessRunning() const { return process_running_.load(); }
    
    // Communication
    bool sendMessage(const AIMessage& message);
    bool sendChatMessage(const std::string& text, const std::string& from_name, 
                        const std::string& from_id, const std::string& spatial_context,
                        const std::string& logged_in_agent);
    
    // Message processing (called from game loop)
    void processIncomingMessages();
    bool hasIncomingMessages() const;
    
    // Command handler registration (same as before!)
    void registerCommandHandler(const std::string& command_type, CommandCallback callback);
    void executeCommand(const AICommandData& command_data);
    
    // Development features
    void enableAutoRestart(bool enable) { auto_restart_enabled_ = enable; }
    void enableFileWatching(bool enable) { file_watching_enabled_ = enable; }
    std::string getProcessStatus() const;
    
    // Connection management for enable/disable toggle
    bool isConnectionActive() const { return connection_active_.load(); }
    bool waitForReconnection(int timeout_seconds = 30);
    bool checkForReconnection(); // Non-blocking check
    bool sendTestMessage(); // Send test message to verify connection
    
    // Persona management (same interface as before)
    bool switchPersona(const std::string& persona_name);
    void setUserIdAsync(const std::string& user_id);
    
private:
    // Network management
    bool initializeNetwork();
    void cleanupNetwork();
    bool connectToServer(int timeout_seconds = 10);
    void disconnectFromServer();
    bool acceptConnection();
    
    // Legacy process hooks (now unused for client mode)
    bool launchPythonProcess(const std::string& script_path);
    void killPythonProcess();
    bool waitForConnection(int timeout_seconds = 30);
    
    // Communication threads
    void messageReaderThread();
    void messageWriterThread();
    void processMonitorThread();
    
    // Message handling
    void handleIncomingMessage(const AIMessage& message);
    void queueOutgoingMessage(const AIMessage& message);
    
    // Utilities
    std::string generateMessageId();
    void logStatus(const std::string& message);
    
private:
    // State
    std::atomic<bool> initialized_;
    std::atomic<bool> process_running_;
    std::atomic<bool> connection_active_;
    std::atomic<bool> connection_pending_;
    std::atomic<bool> shutdown_requested_;
    
    // Network
    socket_t server_socket_;
    socket_t client_socket_;
    int tcp_port_;
    std::string server_host_;
    
    // Process
    process_handle_t python_process_;
    std::string ai_script_path_;
    std::string python_executable_;
    
    // Threading
    std::thread message_reader_thread_;
    std::thread message_writer_thread_;
    std::thread process_monitor_thread_;
    
    // Message queues
    std::queue<AIMessage> incoming_messages_;
    std::queue<AIMessage> outgoing_messages_;
    mutable std::mutex incoming_mutex_;  // mutable for const methods
    mutable std::mutex outgoing_mutex_;  // mutable for const methods
    std::condition_variable outgoing_cv_;
    
    // Command handlers (same as before!)
    std::map<std::string, CommandCallback> command_handlers_;
    mutable std::mutex handlers_mutex_;  // mutable for const methods
    
    // Configuration
    bool auto_restart_enabled_;
    bool file_watching_enabled_;
    std::string current_persona_;
    std::string current_user_id_;
    
    // Message ID generation
    std::atomic<int> message_counter_;
};

// Global functions for C interface (same as before)
extern "C" {
    bool ai_process_bridge_initialize(int port = 9999);
    void ai_process_bridge_shutdown();
    bool ai_process_bridge_start_ai(const char* script_path);
    void ai_process_bridge_stop_ai();
    bool ai_process_bridge_restart_ai();
    bool ai_process_bridge_send_chat(const char* text, const char* from_name, 
                                   const char* from_id, const char* spatial_context, 
                                   const char* logged_in_agent);
    void ai_process_bridge_process_messages();
    const char* ai_process_bridge_get_status();
    bool ai_process_bridge_wait_for_reconnection();
    bool ai_process_bridge_is_connection_active();
    bool ai_process_bridge_send_test_message();
}

#endif // PY_AI_PROCESS_BRIDGE_H
