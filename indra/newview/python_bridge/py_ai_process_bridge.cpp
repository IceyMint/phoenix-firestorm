/**
 * @file py_ai_process_bridge.cpp
 * @brief Process-based Python AI Bridge Implementation
 * 
 * 🔥 HOT RESTART CAPABLE AI BRIDGE! 🔥
 * 
 * This implementation replaces the complex threading + GIL management
 * with a clean process-based architecture that allows:
 * 
 * - Hot restart: Edit Python → restart AI → test (2 seconds!)
 * - Crash isolation: Python crashes don't affect the game
 * - Cross-platform: Same code on Windows and Linux
 * - Easy debugging: Debug Python in separate process
 * 
 * @author Claude AI Assistant
 * @date August 2025
 */

#include "py_ai_process_bridge.h"
#include "../llviewercontrol.h"
#include "../fsaiintegration.h"
// JSON parsing - using nlohmann JSON
#include "./include/nlohmann/json.hpp"

using json = nlohmann::json;

#include <sstream>
#include <iomanip>
#include <algorithm>
#include <random>

// Global instance for C interface
static std::unique_ptr<AIProcessBridge> g_ai_bridge = nullptr;

// ============================================================================
// AIMessage Implementation
// ============================================================================

std::string AIMessage::toJSON() const {
    json message;
    message["type"] = type;
    message["id"] = id;
    message["data"] = data;
    message["timestamp"] = std::chrono::duration_cast<std::chrono::seconds>(
        timestamp.time_since_epoch()).count();
    
    std::string json_result = message.dump();
    
    LL_DEBUGS("AIProcessBridge") << "🔍 Serialized message to JSON (length " << json_result.length() 
             << "): " << json_result << LL_ENDL;
    
    return json_result;
}

AIMessage AIMessage::fromJSON(const std::string& json_str) {
    AIMessage msg;
    
    try {
        // Log the raw JSON for debugging
        LL_INFOS("AIProcessBridge") << "Parsing JSON: " << json_str << LL_ENDL;
        
        json parsed = json::parse(json_str);
        
        // Log the parsed JSON structure for debugging
        LL_INFOS("AIProcessBridge") << "Parsed JSON structure: " << parsed.dump() << LL_ENDL;
        
        // Check and parse type field with detailed logging
        if (parsed.contains("type")) {
            if (parsed["type"].is_null()) {
                LL_WARNS("AIProcessBridge") << "JSON message has null 'type' field in message: " << json_str << LL_ENDL;
                msg.type = "unknown";
            } else if (parsed["type"].is_string()) {
                msg.type = parsed["type"].get<std::string>();
                LL_INFOS("AIProcessBridge") << "Message type: " << msg.type << LL_ENDL;
            } else {
                LL_WARNS("AIProcessBridge") << "JSON message 'type' field is not a string, type is: " << parsed["type"].type_name() << LL_ENDL;
                msg.type = "unknown";
            }
        } else {
            LL_WARNS("AIProcessBridge") << "JSON message missing 'type' field in message: " << json_str << LL_ENDL;
            msg.type = "unknown";
        }
        
        // Check and parse id field with null safety
        if (parsed.contains("id") && !parsed["id"].is_null()) {
            msg.id = parsed["id"].get<std::string>();
        } else {
            msg.id = "";
        }
        
        // Check and parse data field with null safety
        if (parsed.contains("data") && !parsed["data"].is_null()) {
            if (parsed["data"].is_string()) {
                msg.data = parsed["data"].get<std::string>();
            } else {
                // If data is not a string, serialize it as JSON
                msg.data = parsed["data"].dump();
            }
        } else {
            msg.data = "";
        }
        
        // Parse timestamp
        if (parsed.contains("timestamp") && !parsed["timestamp"].is_null()) {
            auto seconds = std::chrono::seconds(parsed["timestamp"].get<int64_t>());
            msg.timestamp = std::chrono::system_clock::time_point(seconds);
        }
    } catch (const std::exception& e) {
        LL_WARNS("AIProcessBridge") << "Failed to parse JSON message: " << e.what() << LL_ENDL;
        // Set default values for failed parsing
        msg.type = "error";
        msg.id = "";
        msg.data = "";
    }
    
    return msg;
}

// ============================================================================
// AIProcessBridge Implementation
// ============================================================================

AIProcessBridge::AIProcessBridge()
    : initialized_(false)
    , process_running_(false)
    , connection_active_(false)
    , connection_pending_(false)
    , shutdown_requested_(false)
    , server_socket_(INVALID_SOCKET)
    , client_socket_(INVALID_SOCKET)
    , tcp_port_(9999)
    , server_host_("127.0.0.1")
    , python_process_(0)
    , auto_restart_enabled_(true)
    , file_watching_enabled_(false)
    , message_counter_(0)
{
    LL_INFOS("AIProcessBridge") << "🚀 Creating AIProcessBridge - TCP client mode" << LL_ENDL;
    
    // Set default Python executable
#ifdef _WIN32
    python_executable_ = "python";  // Should find Python in PATH
#else
    python_executable_ = "python3";
#endif
}

AIProcessBridge::~AIProcessBridge() {
    shutdown();
    LL_INFOS("AIProcessBridge") << "AIProcessBridge destroyed" << LL_ENDL;
}

bool AIProcessBridge::initialize(int port) {
    if (initialized_.load()) {
        LL_WARNS("AIProcessBridge") << "Already initialized" << LL_ENDL;
        return true;
    }
    
    LL_INFOS("AIProcessBridge") << "🔧 Initializing AI Process Bridge (client) for "
                                << server_host_ << ":" << port << LL_ENDL;
    
    tcp_port_ = port;
    
    // Initialize network
    if (!initializeNetwork()) {
        LL_ERRS("AIProcessBridge") << "Failed to initialize network" << LL_ENDL;
        return false;
    }
    
    initialized_.store(true);
    LL_INFOS("AIProcessBridge") << "✅ AI Process Bridge initialized successfully!" << LL_ENDL;
    
    return true;
}

void AIProcessBridge::shutdown() {
    if (!initialized_.load()) {
        return;
    }
    
    LL_INFOS("AIProcessBridge") << "🔄 Shutting down AI Process Bridge..." << LL_ENDL;
    shutdown_requested_.store(true);
    
    // Stop AI connection first
    stopAIProcess();
    
    // Stop networking
    disconnectFromServer();
    cleanupNetwork();
    
    // Wait for threads to finish
    if (message_reader_thread_.joinable()) {
        message_reader_thread_.join();
    }
    if (message_writer_thread_.joinable()) {
        message_writer_thread_.join();
    }
    if (process_monitor_thread_.joinable()) {
        process_monitor_thread_.join();
    }
    
    initialized_.store(false);
    LL_INFOS("AIProcessBridge") << "🏁 AI Process Bridge shutdown complete" << LL_ENDL;
}

// ============================================================================
// Network Management
// ============================================================================

bool AIProcessBridge::initializeNetwork() {
#ifdef _WIN32
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != 0) {
        LL_ERRS("AIProcessBridge") << "WSAStartup failed: " << result << LL_ENDL;
        return false;
    }
    LL_INFOS("AIProcessBridge") << "Winsock initialized" << LL_ENDL;
#endif
    return true;
}

void AIProcessBridge::cleanupNetwork() {
#ifdef _WIN32
    WSACleanup();
#endif
}

bool AIProcessBridge::connectToServer(int timeout_seconds) {
    connection_active_.store(false);
    auto set_blocking = [this]() {
#ifdef _WIN32
        u_long mode = 0;
        ioctlsocket(client_socket_, FIONBIO, &mode);
#else
        int flags_blocking = fcntl(client_socket_, F_GETFL, 0);
        fcntl(client_socket_, F_SETFL, flags_blocking & ~O_NONBLOCK);
#endif
    };

    if (connection_pending_.load()) {
        if (client_socket_ == INVALID_SOCKET) {
            connection_pending_.store(false);
        } else {
            fd_set write_fds;
            FD_ZERO(&write_fds);
            FD_SET(client_socket_, &write_fds);

            struct timeval tv;
            tv.tv_sec = timeout_seconds > 0 ? timeout_seconds : 0;
            tv.tv_usec = 0;

            int sel = select(static_cast<int>(client_socket_) + 1, nullptr, &write_fds, nullptr, &tv);
            if (sel == 0 && timeout_seconds <= 0) {
                return false;
            }
            if (sel <= 0) {
                CLOSE_SOCKET(client_socket_);
                client_socket_ = INVALID_SOCKET;
                connection_pending_.store(false);
                return false;
            }

            int so_error = 0;
            socklen_t len = sizeof(so_error);
            getsockopt(client_socket_, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&so_error), &len);
            if (so_error != 0) {
                CLOSE_SOCKET(client_socket_);
                client_socket_ = INVALID_SOCKET;
                connection_pending_.store(false);
                return false;
            }

            connection_pending_.store(false);
            connection_active_.store(true);
            set_blocking();
            LL_INFOS("AIProcessBridge") << "🔗 Connected to AI server at " << server_host_ << ":" << tcp_port_ << LL_ENDL;
            return true;
        }
    }

    if (client_socket_ != INVALID_SOCKET) {
        CLOSE_SOCKET(client_socket_);
        client_socket_ = INVALID_SOCKET;
    }

    client_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket_ == INVALID_SOCKET) {
        LL_ERRS("AIProcessBridge") << "Failed to create client socket: " << SOCKET_ERROR_CODE << LL_ENDL;
        return false;
    }

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(server_host_.c_str());
    address.sin_port = htons(tcp_port_);

    if (address.sin_addr.s_addr == INADDR_NONE) {
        LL_ERRS("AIProcessBridge") << "Invalid server host: " << server_host_ << LL_ENDL;
        CLOSE_SOCKET(client_socket_);
        client_socket_ = INVALID_SOCKET;
        return false;
    }

#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(client_socket_, FIONBIO, &mode);
#else
    int flags = fcntl(client_socket_, F_GETFL, 0);
    fcntl(client_socket_, F_SETFL, flags | O_NONBLOCK);
#endif

    int result = connect(client_socket_, (struct sockaddr*)&address, sizeof(address));
    bool in_progress = false;
    if (result == 0) {
        connection_active_.store(true);
    } else {
#ifdef _WIN32
        int error = WSAGetLastError();
        in_progress = (error == WSAEWOULDBLOCK || error == WSAEINPROGRESS || error == WSAEALREADY);
        if (!in_progress) {
            LL_WARNS("AIProcessBridge") << "Connect failed: " << error << LL_ENDL;
        }
#else
        in_progress = (errno == EINPROGRESS || errno == EWOULDBLOCK || errno == EAGAIN);
        if (!in_progress) {
            LL_WARNS("AIProcessBridge") << "Connect failed: " << strerror(errno) << LL_ENDL;
        }
#endif
    }

    if (!connection_active_.load()) {
        if (!in_progress) {
            CLOSE_SOCKET(client_socket_);
            client_socket_ = INVALID_SOCKET;
            return false;
        }
        if (timeout_seconds <= 0) {
            connection_pending_.store(true);
            return false;
        }

        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(client_socket_, &write_fds);

        struct timeval tv;
        tv.tv_sec = timeout_seconds;
        tv.tv_usec = 0;

        int sel = select(static_cast<int>(client_socket_) + 1, nullptr, &write_fds, nullptr, &tv);
        if (sel <= 0) {
            CLOSE_SOCKET(client_socket_);
            client_socket_ = INVALID_SOCKET;
            return false;
        }

        int so_error = 0;
        socklen_t len = sizeof(so_error);
        getsockopt(client_socket_, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&so_error), &len);
        if (so_error != 0) {
            CLOSE_SOCKET(client_socket_);
            client_socket_ = INVALID_SOCKET;
            return false;
        }

        connection_active_.store(true);
    }

    connection_pending_.store(false);
    set_blocking();

    LL_INFOS("AIProcessBridge") << "🔗 Connected to AI server at " << server_host_ << ":" << tcp_port_ << LL_ENDL;
    return true;
}

void AIProcessBridge::disconnectFromServer() {
    connection_active_.store(false);
    connection_pending_.store(false);

    if (client_socket_ != INVALID_SOCKET) {
#ifdef _WIN32
        ::shutdown(client_socket_, SD_BOTH);
#else
        ::shutdown(client_socket_, SHUT_RDWR);
#endif
        CLOSE_SOCKET(client_socket_);
        client_socket_ = INVALID_SOCKET;
    }

    LL_INFOS("AIProcessBridge") << "TCP client disconnected" << LL_ENDL;
}

// ============================================================================
// 🔥 HOT RESTART PROCESS MANAGEMENT 🔥
// ============================================================================

bool AIProcessBridge::startAIProcess(const std::string& script_path) {
    if (process_running_.load()) {
        LL_WARNS("AIProcessBridge") << "AI connection already active" << LL_ENDL;
        return true;
    }
    
    if (!initialized_.load()) {
        LL_ERRS("AIProcessBridge") << "Bridge not initialized - cannot start AI process" << LL_ENDL;
        return false;
    }
    
    LL_INFOS("AIProcessBridge") << "🚀 Connecting to AI server (script ignored): " << script_path << LL_ENDL;
    ai_script_path_ = script_path;

    if (!connectToServer(10)) {
        LL_ERRS("AIProcessBridge") << "Failed to connect to AI server" << LL_ENDL;
        return false;
    }
    
    // Start communication threads
    message_reader_thread_ = std::thread(&AIProcessBridge::messageReaderThread, this);
    message_writer_thread_ = std::thread(&AIProcessBridge::messageWriterThread, this);
    process_running_.store(true);
    LL_INFOS("AIProcessBridge") << "✅ AI server connection established!" << LL_ENDL;
    
    return true;
}

void AIProcessBridge::stopAIProcess() {
    if (!process_running_.load()) {
        return;
    }
    
    LL_INFOS("AIProcessBridge") << "🛑 Disconnecting from AI server..." << LL_ENDL;
    process_running_.store(false);

    connection_active_.store(false);

    {
        std::lock_guard<std::mutex> lock(outgoing_mutex_);
        outgoing_cv_.notify_all();
    }

    disconnectFromServer();

    // Wait for threads
    if (message_reader_thread_.joinable()) {
        message_reader_thread_.join();
    }
    if (message_writer_thread_.joinable()) {
        message_writer_thread_.join();
    }

    LL_INFOS("AIProcessBridge") << "🏁 AI server connection closed" << LL_ENDL;
}

bool AIProcessBridge::restartAIProcess() {
    LL_INFOS("AIProcessBridge") << "🔄 Reconnecting to AI server" << LL_ENDL;
    
    std::string script_path = ai_script_path_;  // Save current path
    
    // Stop current process
    stopAIProcess();
    
    // Small delay to ensure clean shutdown
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Start new connection
    bool success = startAIProcess(script_path);
    
    if (success) {
        LL_INFOS("AIProcessBridge") << "🎉 Reconnected to AI server" << LL_ENDL;
    } else {
        LL_ERRS("AIProcessBridge") << "💥 Reconnect failed" << LL_ENDL;
    }
    
    return success;
}

bool AIProcessBridge::launchPythonProcess(const std::string& script_path) {
    LL_WARNS("AIProcessBridge") << "launchPythonProcess is disabled in client mode: " << script_path << LL_ENDL;
    return false;
}

void AIProcessBridge::killPythonProcess() {
    python_process_ = 0;
}

bool AIProcessBridge::waitForConnection(int timeout_seconds) {
    LL_INFOS("AIProcessBridge") << "⏳ Connecting to AI server..." << LL_ENDL;
    return connectToServer(timeout_seconds);
}

// ============================================================================
// Communication
// ============================================================================

bool AIProcessBridge::sendMessage(const AIMessage& message) {
    if (!connection_active_.load()) {
        LL_WARNS("AIProcessBridge") << "❌ Cannot send message - no active connection (type: '" 
                 << message.type << "', id: '" << message.id << "')" << LL_ENDL;
        return false;
    }
    
    LL_INFOS("AIProcessBridge") << "📨 Sending message (type: '" << message.type 
             << "', id: '" << message.id << "')" << LL_ENDL;
    
    queueOutgoingMessage(message);
    return true;
}

bool AIProcessBridge::sendChatMessage(const std::string& text, const std::string& from_name, 
                                    const std::string& from_id, const std::string& spatial_context,
                                    const std::string& logged_in_agent) {
    
    if (current_persona_.empty() && !logged_in_agent.empty()) {
        current_persona_ = logged_in_agent;
    }

    json chat_data;
    chat_data["text"] = text;
    chat_data["from_name"] = from_name;
    chat_data["from_id"] = from_id;
    chat_data["spatial_context"] = spatial_context;
    chat_data["logged_in_agent"] = logged_in_agent;
    chat_data["persona"] = current_persona_;
    
    AIMessage message("process_chat", chat_data.dump());
    message.id = generateMessageId();
    
    LL_INFOS("AIProcessBridge") << "📤 Sending chat message to AI: '" << text << "'" << LL_ENDL;
    return sendMessage(message);
}

void AIProcessBridge::queueOutgoingMessage(const AIMessage& message) {
    std::lock_guard<std::mutex> lock(outgoing_mutex_);
    
    LL_INFOS("AIProcessBridge") << "📥 Queuing outgoing message (type: '" << message.type 
             << "', id: '" << message.id << "'). Queue was: " << outgoing_messages_.size() << LL_ENDL;
    
    outgoing_messages_.push(message);
    
    LL_INFOS("AIProcessBridge") << "📊 Outgoing queue now has " << outgoing_messages_.size() 
             << " messages. Notifying writer thread..." << LL_ENDL;
    
    outgoing_cv_.notify_one();
    
    LL_DEBUGS("AIProcessBridge") << "🔍 Writer thread notification sent" << LL_ENDL;
}

std::string AIProcessBridge::generateMessageId() {
    return "msg_" + std::to_string(message_counter_.fetch_add(1));
}

// ============================================================================
// Message Processing (Called from Game Loop!)
// ============================================================================

void AIProcessBridge::processIncomingMessages() {
    std::lock_guard<std::mutex> lock(incoming_mutex_);
    
    size_t initial_queue_size = incoming_messages_.size();
    
    if (initial_queue_size > 0) {
        LL_INFOS("AIProcessBridge") << "🔄 Processing " << initial_queue_size 
                 << " incoming messages from game loop" << LL_ENDL;
    }
    
    int processed_count = 0;
    while (!incoming_messages_.empty()) {
        AIMessage message = incoming_messages_.front();
        incoming_messages_.pop();
        processed_count++;
        
        LL_INFOS("AIProcessBridge") << "🎯 Processing message " << processed_count << "/" 
                 << initial_queue_size << " (type: '" << message.type 
                 << "', id: '" << message.id << "')" << LL_ENDL;
        
        handleIncomingMessage(message);
    }
    
    if (processed_count > 0) {
        LL_INFOS("AIProcessBridge") << "✅ Completed processing " << processed_count 
                 << " messages from incoming queue" << LL_ENDL;
    }
}

bool AIProcessBridge::hasIncomingMessages() const {
    std::lock_guard<std::mutex> lock(incoming_mutex_);
    return !incoming_messages_.empty();
}

void AIProcessBridge::handleIncomingMessage(const AIMessage& message) {
    LL_INFOS("AIProcessBridge") << "📥 Processing AI message: " << message.type << LL_ENDL;
    
    if (message.type == "chat_response") {
        // Parse AI response and execute commands
        try {
            json response = json::parse(message.data);
            
            // Execute any commands the AI wants to perform
            if (response.contains("commands") && response["commands"].is_array()) {
                for (const auto& cmd : response["commands"]) {
                    AICommandData command_data;
                    command_data.command_type = cmd["type"].get<std::string>();
                    if (cmd.contains("content")) {
                        command_data.content = cmd["content"].get<std::string>();
                    }
                    if (cmd.contains("target_uuid")) {
                        command_data.target_uuid = cmd["target_uuid"].get<std::string>();
                    }
                    if (cmd.contains("x")) command_data.x = cmd["x"].get<float>();
                    if (cmd.contains("y")) command_data.y = cmd["y"].get<float>();
                    if (cmd.contains("z")) command_data.z = cmd["z"].get<float>();
                    if (cmd.contains("parameters") && cmd["parameters"].is_object()) {
                        for (const auto& [key, value] : cmd["parameters"].items()) {
                            if (value.is_string()) {
                                command_data.parameters[key] = value.get<std::string>();
                            } else {
                                command_data.parameters[key] = value.dump();
                            }
                        }
                    }
                    
                    executeCommand(command_data);
                }
            }
        } catch (const std::exception& e) {
            LL_WARNS("AIProcessBridge") << "Failed to parse chat response: " << e.what() << LL_ENDL;
        }
        
    } else if (message.type == "autonomous_action") {
        // Handle autonomous AI action
        try {
            json action = json::parse(message.data);
            
            AICommandData command_data;
            command_data.command_type = action["command"].get<std::string>();
            if (action.contains("parameters")) {
                for (const auto& [key, value] : action["parameters"].items()) {
                    command_data.parameters[key] = value.get<std::string>();
                }
            }
            
            executeCommand(command_data);
            
        } catch (const std::exception& e) {
            LL_WARNS("AIProcessBridge") << "Failed to parse autonomous action: " << e.what() << LL_ENDL;
        }
        
    } else if (message.type == "status") {
        // Handle status updates from AI
        LL_INFOS("AIProcessBridge") << "AI Status: " << message.data << LL_ENDL;
        
    } else if (message.type == "environment_request") {
        // Handle environment update request from Python AI
        LL_INFOS("AIProcessBridge") << "📍 Environment update requested by Python AI" << LL_ENDL;
        
        try {
            // Call FSAIIntegration to gather and send environment data
            LL_INFOS("AIProcessBridge") << "📍 Environment update request received - calling FSAIIntegration::requestEnvironmentUpdate()" << LL_ENDL;
            
            // Get FSAIIntegration instance and call the environment update method
            FSAIIntegration* ai_integration = FSAIIntegration::getInstance();
            if (ai_integration) {
                ai_integration->requestEnvironmentUpdate();
                LL_INFOS("AIProcessBridge") << "✅ FSAIIntegration::requestEnvironmentUpdate() called successfully" << LL_ENDL;
            } else {
                LL_WARNS("AIProcessBridge") << "❌ FSAIIntegration instance not available" << LL_ENDL;
            }
            
        } catch (const std::exception& e) {
            LL_WARNS("AIProcessBridge") << "💥 Error handling environment request: " << e.what() << LL_ENDL;
        }
        
    } else {
        LL_WARNS("AIProcessBridge") << "Unknown message type: " << message.type << LL_ENDL;
    }
}

// ============================================================================
// Communication Threads
// ============================================================================

void AIProcessBridge::messageReaderThread() {
    LL_INFOS("AIProcessBridge") << "📡 Message reader thread started" << LL_ENDL;
    
    char buffer[8192];
    std::string message_buffer;
    int total_bytes_received = 0;
    int message_count = 0;
    
    while (connection_active_.load() && !shutdown_requested_.load()) {
        LL_DEBUGS("AIProcessBridge") << "🔍 Reader thread: Waiting for data on socket " << client_socket_ << LL_ENDL;
        
        int bytes_received = recv(client_socket_, buffer, sizeof(buffer) - 1, 0);
        
        LL_DEBUGS("AIProcessBridge") << "🔍 Reader thread: recv() returned " << bytes_received << " bytes" << LL_ENDL;
        
        if (bytes_received <= 0) {
            if (connection_active_.load()) {
                if (bytes_received == 0) {
                    LL_WARNS("AIProcessBridge") << "❌ Connection closed by peer (recv returned 0) - " 
                             << "(total received: " << total_bytes_received << " bytes, " 
                             << message_count << " messages)" << LL_ENDL;
                    // Peer closed connection - this is a real disconnect, should exit
                    connection_active_.store(false);
                    break;
                } else {
                    // Check if it's a timeout or recoverable error
#ifdef _WIN32
                    int error = WSAGetLastError();
                    if (error == WSAEWOULDBLOCK || error == WSAEINTR || error == WSAECONNRESET) {
                        LL_DEBUGS("AIProcessBridge") << "⚠️ Recoverable recv error: " << error << " - retrying..." << LL_ENDL;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        continue; // Retry instead of exiting
                    } else {
                        LL_WARNS("AIProcessBridge") << "❌ Non-recoverable connection error in recv(): " << error 
                                 << " (total received: " << total_bytes_received << " bytes, " 
                                 << message_count << " messages)" << LL_ENDL;
                        connection_active_.store(false);
                        break;
                    }
#else
                    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                        LL_INFOS("AIProcessBridge") << "⚠️ Recoverable recv error: " << errno << " - retrying..." << LL_ENDL;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        continue; // Retry instead of exiting
                    } else {
                        LL_WARNS("AIProcessBridge") << "❌ Non-recoverable connection error in recv(): " << errno 
                                 << " (total received: " << total_bytes_received << " bytes, " 
                                 << message_count << " messages)" << LL_ENDL;
                        connection_active_.store(false);
                        break;
                    }
#endif
                }
            } else {
                LL_INFOS("AIProcessBridge") << "📡 Reader thread exiting normally (connection_active=false)" << LL_ENDL;
                break;
            }
        }
        
        total_bytes_received += bytes_received;
        buffer[bytes_received] = '\0';
        
        LL_INFOS("AIProcessBridge") << "📥 Received " << bytes_received << " bytes (total: " 
                 << total_bytes_received << "): '" << std::string(buffer, bytes_received) << "'" << LL_ENDL;
        
        message_buffer += buffer;
        
        LL_DEBUGS("AIProcessBridge") << "🔍 Message buffer now contains " << message_buffer.length() 
                 << " chars: '" << message_buffer << "'" << LL_ENDL;
        
        // Process complete JSON messages (newline-delimited)
        size_t pos = 0;
        int messages_in_batch = 0;
        while ((pos = message_buffer.find('\n')) != std::string::npos) {
            std::string json_msg = message_buffer.substr(0, pos);
            message_buffer.erase(0, pos + 1);
            messages_in_batch++;
            
            LL_INFOS("AIProcessBridge") << "🔗 Found message boundary, extracted message " 
                     << messages_in_batch << " (length " << json_msg.length() << "): '" 
                     << json_msg << "'" << LL_ENDL;
            
            if (!json_msg.empty()) {
                // Check for potential message corruption
                if (json_msg.find('{') == std::string::npos || json_msg.find('}') == std::string::npos) {
                    LL_WARNS("AIProcessBridge") << "⚠️ Potentially corrupted message (no braces): '" << json_msg << "'" << LL_ENDL;
                    continue;
                }
                
                LL_DEBUGS("AIProcessBridge") << "🔍 Attempting to parse JSON message..." << LL_ENDL;
                try {
                    AIMessage message = AIMessage::fromJSON(json_msg);
                    LL_DEBUGS("AIProcessBridge") << "🔍 JSON parsing completed, message type: '" << message.type << "'" << LL_ENDL;
                    
                    // Only queue valid messages
                    if (!message.type.empty() && message.type != "unknown") {
                        LL_INFOS("AIProcessBridge") << "✅ Queuing valid message (type: '" << message.type 
                                 << "', id: '" << message.id << "')" << LL_ENDL;
                        
                        std::lock_guard<std::mutex> lock(incoming_mutex_);
                        incoming_messages_.push(message);
                        message_count++;
                        
                        LL_INFOS("AIProcessBridge") << "📊 Incoming queue now has " << incoming_messages_.size() 
                                 << " messages" << LL_ENDL;
                    } else {
                        LL_WARNS("AIProcessBridge") << "❌ Discarding invalid message with type: '" << message.type << "'" << LL_ENDL;
                    }
                } catch (const std::exception& e) {
                    LL_WARNS("AIProcessBridge") << "❌ JSON parsing failed for message: '" << json_msg 
                             << "' - Error: " << e.what() << LL_ENDL;
                } catch (...) {
                    LL_WARNS("AIProcessBridge") << "❌ Unknown error parsing JSON message: '" << json_msg << "'" << LL_ENDL;
                }
            } else {
                LL_WARNS("AIProcessBridge") << "❌ Received empty message between newlines" << LL_ENDL;
            }
        }
        
        if (messages_in_batch > 0) {
            LL_INFOS("AIProcessBridge") << "📦 Processed " << messages_in_batch 
                     << " messages in this batch. Buffer remainder: " << message_buffer.length() << " chars" << LL_ENDL;
        }
        
        LL_DEBUGS("AIProcessBridge") << "🔍 Reader thread: End of recv() loop iteration" << LL_ENDL;
    }
    
    process_running_.store(false);
    LL_INFOS("AIProcessBridge") << "📡 Message reader thread stopped (total: " << total_bytes_received 
             << " bytes, " << message_count << " messages processed)" << LL_ENDL;
}

void AIProcessBridge::messageWriterThread() {
    LL_INFOS("AIProcessBridge") << "📤 Message writer thread started" << LL_ENDL;
    
    int total_bytes_sent = 0;
    int message_count = 0;
    
    while (connection_active_.load() && !shutdown_requested_.load()) {
        LL_DEBUGS("AIProcessBridge") << "🔍 Writer thread: Waiting for messages..." << LL_ENDL;
        
        std::unique_lock<std::mutex> lock(outgoing_mutex_);
        outgoing_cv_.wait(lock, [this] { 
            bool should_wake = !outgoing_messages_.empty() || !connection_active_.load() || shutdown_requested_.load();
            LL_DEBUGS("AIProcessBridge") << "🔍 Writer thread: Condition variable triggered, should_wake=" 
                     << should_wake << " (queue_size=" << outgoing_messages_.size() 
                     << ", active=" << connection_active_.load() 
                     << ", shutdown=" << shutdown_requested_.load() << ")" << LL_ENDL;
            return should_wake;
        });
        
        LL_DEBUGS("AIProcessBridge") << "🔍 Writer thread: Woke up, checking queue..." << LL_ENDL;
        
        while (!outgoing_messages_.empty() && connection_active_.load()) {
            AIMessage message = outgoing_messages_.front();
            outgoing_messages_.pop();
            
            LL_INFOS("AIProcessBridge") << "📤 Dequeued message for sending (type: '" << message.type 
                     << "', id: '" << message.id << "'). Queue remaining: " 
                     << outgoing_messages_.size() << LL_ENDL;
            
            lock.unlock();
            
            std::string json_msg = message.toJSON() + "\n";
            
            LL_INFOS("AIProcessBridge") << "📡 Sending " << json_msg.length() 
                     << " bytes to socket " << client_socket_ << ": '" << json_msg.substr(0, json_msg.length()-1) << "'" << LL_ENDL;
            
            int bytes_sent = send(client_socket_, json_msg.c_str(), static_cast<int>(json_msg.length()), 0);
            
            LL_DEBUGS("AIProcessBridge") << "🔍 Writer thread: send() returned " << bytes_sent << LL_ENDL;
            
            if (bytes_sent <= 0) {
                // Check if it's a recoverable error
#ifdef _WIN32
                int error = WSAGetLastError();
                if (error == WSAEWOULDBLOCK || error == WSAEINTR) {
                    LL_INFOS("AIProcessBridge") << "⚠️ Recoverable send error: " << error << " - retrying..." << LL_ENDL;
                    // Re-queue the message at front for retry (we already have the lock)
                    std::queue<AIMessage> temp_queue;
                    temp_queue.push(message);
                    while (!outgoing_messages_.empty()) {
                        temp_queue.push(outgoing_messages_.front());
                        outgoing_messages_.pop();
                    }
                    outgoing_messages_ = temp_queue;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    break; // Exit inner loop to recheck connection
                } else {
                    LL_WARNS("AIProcessBridge") << "❌ Non-recoverable send error: " << error 
                             << " (total sent: " << total_bytes_sent << " bytes, " 
                             << message_count << " messages)" << LL_ENDL;
                    connection_active_.store(false);
                    break;
                }
#else
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                    LL_INFOS("AIProcessBridge") << "⚠️ Recoverable send error: " << errno << " - retrying..." << LL_ENDL;
                    // Re-queue the message at front for retry (we already have the lock)
                    std::queue<AIMessage> temp_queue;
                    temp_queue.push(message);
                    while (!outgoing_messages_.empty()) {
                        temp_queue.push(outgoing_messages_.front());
                        outgoing_messages_.pop();
                    }
                    outgoing_messages_ = temp_queue;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    break; // Exit inner loop to recheck connection
                } else {
                    LL_WARNS("AIProcessBridge") << "❌ Non-recoverable send error: " << errno 
                             << " (total sent: " << total_bytes_sent << " bytes, " 
                             << message_count << " messages)" << LL_ENDL;
                    connection_active_.store(false);
                    break;
                }
#endif
            } else if (bytes_sent != static_cast<int>(json_msg.length())) {
                LL_WARNS("AIProcessBridge") << "⚠️ Partial send: " << bytes_sent << "/" << json_msg.length() 
                         << " bytes sent" << LL_ENDL;
                // TODO: Handle partial sends properly
            } else {
                total_bytes_sent += bytes_sent;
                message_count++;
                LL_INFOS("AIProcessBridge") << "✅ Message sent successfully (" << bytes_sent 
                         << " bytes, total: " << total_bytes_sent << " bytes, " 
                         << message_count << " messages)" << LL_ENDL;
            }
            
            lock.lock();
        }
        
        LL_DEBUGS("AIProcessBridge") << "🔍 Writer thread: End of message processing loop" << LL_ENDL;
    }
    
    LL_INFOS("AIProcessBridge") << "📤 Message writer thread stopped (total: " << total_bytes_sent 
             << " bytes, " << message_count << " messages sent)" << LL_ENDL;
}

void AIProcessBridge::processMonitorThread() {
    LL_INFOS("AIProcessBridge") << "🔍 Process monitor thread started" << LL_ENDL;
    
    while (process_running_.load() && !shutdown_requested_.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        if (!process_running_.load() || shutdown_requested_.load()) {
            break;
        }
        
        // Check if process is still alive
        bool process_alive = false;
        
#ifdef _WIN32
        if (python_process_ != INVALID_HANDLE_VALUE) {
            DWORD exit_code;
            if (GetExitCodeProcess(python_process_, &exit_code)) {
                process_alive = (exit_code == STILL_ACTIVE);
            }
        }
#else
        if (python_process_ > 0) {
            int status;
            pid_t result = waitpid(python_process_, &status, WNOHANG);
            process_alive = (result == 0);  // Still running
        }
#endif
        
        if (!process_alive && auto_restart_enabled_) {
            LL_WARNS("AIProcessBridge") << "🚨 AI process crashed! Auto-restarting..." << LL_ENDL;
            
            // Reset connection state
            connection_active_.store(false);
            
            // Attempt restart
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (!restartAIProcess()) {
                LL_ERRS("AIProcessBridge") << "💥 Auto-restart failed! AI system offline." << LL_ENDL;
                break;
            }
        }
    }
    
    LL_INFOS("AIProcessBridge") << "🔍 Process monitor thread stopped" << LL_ENDL;
}

// ============================================================================
// Command Handler System (Same as before!)
// ============================================================================

void AIProcessBridge::registerCommandHandler(const std::string& command_type, CommandCallback callback) {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    command_handlers_[command_type] = callback;
    LL_INFOS("AIProcessBridge") << "Registered handler for command: " << command_type << LL_ENDL;
}

void AIProcessBridge::executeCommand(const AICommandData& command_data) {
    CommandCallback handler;
    
    // Thread-safe handler lookup
    {
        std::lock_guard<std::mutex> lock(handlers_mutex_);
        auto handler_it = command_handlers_.find(command_data.command_type);
        if (handler_it != command_handlers_.end()) {
            handler = handler_it->second;
        }
    }
    
    if (handler) {
        try {
            LL_INFOS("AIProcessBridge") << "⚡ Executing AI command: " << command_data.command_type << LL_ENDL;
            handler(command_data);
        } catch (const std::exception& e) {
            LL_ERRS("AIProcessBridge") << "Error executing command " << command_data.command_type << ": " << e.what() << LL_ENDL;
        }
    } else {
        LL_WARNS("AIProcessBridge") << "No handler registered for command: " << command_data.command_type << LL_ENDL;
    }
}

// ============================================================================
// Duplicate methods removed - using implementations from lines 478-493
// ============================================================================

// ============================================================================
// Persona Management (Same Interface!)
// ============================================================================

bool AIProcessBridge::switchPersona(const std::string& persona_name) {
    current_persona_ = persona_name;
    LL_INFOS("AIProcessBridge") << "🎭 Persona stored for session: " << persona_name << LL_ENDL;
    return true;
}

void AIProcessBridge::setUserIdAsync(const std::string& user_id) {
    if (!connection_active_.load()) {
        LL_WARNS("AIProcessBridge") << "Cannot set user ID - no active connection" << LL_ENDL;
        return;
    }
    
    current_user_id_ = user_id;
    
    json user_data;
    user_data["user_id"] = user_id;
    
    AIMessage message("set_user_id", user_data.dump());
    message.id = generateMessageId();
    
    LL_INFOS("AIProcessBridge") << "👤 Setting user ID: " << user_id << LL_ENDL;
    sendMessage(message);
}

// ============================================================================
// Status and Utilities
// ============================================================================

std::string AIProcessBridge::getProcessStatus() const {
    std::ostringstream status;
    status << "AI Process Bridge Status:\n";
    status << "  Initialized: " << (initialized_.load() ? "YES" : "NO") << "\n";
    status << "  Process Running: " << (process_running_.load() ? "YES" : "NO") << "\n";
    status << "  Connection Active: " << (connection_active_.load() ? "YES" : "NO") << "\n";
    status << "  TCP Port: " << tcp_port_ << "\n";
    status << "  Current Persona: " << current_persona_ << "\n";
    status << "  Auto Restart: " << (auto_restart_enabled_ ? "ENABLED" : "DISABLED") << "\n";
    status << "  Server: " << server_host_ << ":" << tcp_port_ << "\n";
    
    {
        std::lock_guard<std::mutex> lock(handlers_mutex_);
        status << "  Registered Commands: " << command_handlers_.size() << "\n";
    }
    
    {
        std::lock_guard<std::mutex> lock(incoming_mutex_);
        status << "  Incoming Messages: " << incoming_messages_.size() << "\n";
    }
    
    {
        std::lock_guard<std::mutex> lock(outgoing_mutex_);
        status << "  Outgoing Messages: " << outgoing_messages_.size() << "\n";
    }
    
    return status.str();
}

bool AIProcessBridge::waitForReconnection(int timeout_seconds) {
    LL_INFOS("AIProcessBridge") << "⏳ Preparing to reconnect to AI server..." << LL_ENDL;
    
    if (!initialized_.load()) {
        LL_ERRS("AIProcessBridge") << "Bridge not initialized" << LL_ENDL;
        return false;
    }

    connection_active_.store(false);
    disconnectFromServer();

    if (timeout_seconds <= 0) {
        return false;
    }

    if (!connectToServer(timeout_seconds)) {
        return false;
    }

    // Signal old threads to exit and wait for them to complete
    LL_INFOS("AIProcessBridge") << "🔄 Waiting for old communication threads to exit..." << LL_ENDL;

    {
        std::lock_guard<std::mutex> lock(outgoing_mutex_);
        outgoing_cv_.notify_all();
    }

    if (message_reader_thread_.joinable()) {
        LL_DEBUGS("AIProcessBridge") << "🔍 Joining old reader thread..." << LL_ENDL;
        message_reader_thread_.join();
        LL_DEBUGS("AIProcessBridge") << "✅ Old reader thread joined" << LL_ENDL;
    }
    if (message_writer_thread_.joinable()) {
        LL_DEBUGS("AIProcessBridge") << "🔍 Joining old writer thread..." << LL_ENDL;
        message_writer_thread_.join();
        LL_DEBUGS("AIProcessBridge") << "✅ Old writer thread joined" << LL_ENDL;
    }

    connection_active_.store(true);
    process_running_.store(true);

    LL_INFOS("AIProcessBridge") << "🚀 Starting new communication threads for reconnection..." << LL_ENDL;
    message_reader_thread_ = std::thread(&AIProcessBridge::messageReaderThread, this);
    message_writer_thread_ = std::thread(&AIProcessBridge::messageWriterThread, this);

    LL_INFOS("AIProcessBridge") << "✅ Communication threads restarted for reconnection" << LL_ENDL;
    return true;
}

bool AIProcessBridge::sendTestMessage() {
    if (!connection_active_.load()) {
        LL_WARNS("AIProcessBridge") << "❌ Cannot send test message - no active connection" << LL_ENDL;
        return false;
    }
    
    AIMessage test_msg("test", "Connection test message from C++");
    test_msg.id = generateMessageId();
    
    LL_INFOS("AIProcessBridge") << "🧪 Sending test message to verify connection..." << LL_ENDL;
    return sendMessage(test_msg);
}

bool AIProcessBridge::checkForReconnection() {
    if (connection_active_.load()) {
        return true; // Already connected
    }

    if (!initialized_.load()) {
        return false;
    }

    if (!connectToServer(0)) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(outgoing_mutex_);
        outgoing_cv_.notify_all();
    }

    if (message_reader_thread_.joinable()) {
        message_reader_thread_.join();
    }
    if (message_writer_thread_.joinable()) {
        message_writer_thread_.join();
    }

    connection_active_.store(true);
    process_running_.store(true);

    message_reader_thread_ = std::thread(&AIProcessBridge::messageReaderThread, this);
    message_writer_thread_ = std::thread(&AIProcessBridge::messageWriterThread, this);

    LL_INFOS("AIProcessBridge") << "🔗 AI server reconnected successfully!" << LL_ENDL;
    return true;
}

// ============================================================================
// C Interface (Same as before!)
// ============================================================================

extern "C" {
    bool ai_process_bridge_initialize(int port) {
        if (!g_ai_bridge) {
            g_ai_bridge = std::make_unique<AIProcessBridge>();
        }
        return g_ai_bridge->initialize(port);
    }
    
    void ai_process_bridge_shutdown() {
        if (g_ai_bridge) {
            g_ai_bridge->shutdown();
            g_ai_bridge.reset();
        }
    }
    
    bool ai_process_bridge_start_ai(const char* script_path) {
        if (!g_ai_bridge) return false;
        return g_ai_bridge->startAIProcess(script_path ? script_path : "ai_process.py");
    }
    
    void ai_process_bridge_stop_ai() {
        if (g_ai_bridge) {
            g_ai_bridge->stopAIProcess();
        }
    }
    
    bool ai_process_bridge_restart_ai() {
        if (!g_ai_bridge) return false;
        return g_ai_bridge->restartAIProcess();  // 🔥 HOT RESTART! 🔥
    }
    
    bool ai_process_bridge_send_chat(const char* text, const char* from_name, 
                                   const char* from_id, const char* spatial_context, 
                                   const char* logged_in_agent) {
        if (!g_ai_bridge) return false;
        return g_ai_bridge->sendChatMessage(
            text ? text : "",
            from_name ? from_name : "",
            from_id ? from_id : "",
            spatial_context ? spatial_context : "",
            logged_in_agent ? logged_in_agent : ""
        );
    }
    
    void ai_process_bridge_process_messages() {
        if (g_ai_bridge) {
            g_ai_bridge->processIncomingMessages();
        }
    }
    
    const char* ai_process_bridge_get_status() {
        if (!g_ai_bridge) return "Bridge not initialized";
        
        static std::string status;
        status = g_ai_bridge->getProcessStatus();
        return status.c_str();
    }
    
    bool ai_process_bridge_wait_for_reconnection() {
        if (!g_ai_bridge) return false;
        return g_ai_bridge->waitForReconnection();
    }
    
    bool ai_process_bridge_is_connection_active() {
        if (!g_ai_bridge) return false;
        return g_ai_bridge->isConnectionActive();
    }
    
    bool ai_process_bridge_send_test_message() {
        if (!g_ai_bridge) return false;
        return g_ai_bridge->sendTestMessage();
    }
}
