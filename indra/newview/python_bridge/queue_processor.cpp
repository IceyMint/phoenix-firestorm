#include "queue_processor.h"
#include <nlohmann/json.hpp>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <sstream>

using json = nlohmann::json;

namespace python_bridge {

// Static instance for singleton pattern
QueueProcessor QueueProcessorIntegration::instance_;
std::mutex QueueProcessorIntegration::instance_mutex_;

//==============================================================================
// QueueProcessor Implementation
//==============================================================================

QueueProcessor::QueueProcessor()
    : message_queue_(nullptr)
    , max_commands_per_frame_(10)
    , detailed_logging_(false)
{
}

QueueProcessor::~QueueProcessor() {
    shutdown();
}

bool QueueProcessor::initialize(MessageQueueInterface* queue) {
    if (!queue) {
        std::cerr << "[QueueProcessor] Cannot initialize with null queue" << std::endl;
        return false;
    }
    
    if (initialized_.load()) {
        std::cerr << "[QueueProcessor] Already initialized" << std::endl;
        return true;
    }
    
    message_queue_ = queue;
    initialized_.store(true);
    
    // Reset statistics
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_ = ProcessorStats();
        processing_times_.clear();
        processing_times_.reserve(1000); // Pre-allocate for performance
    }
    
    std::cout << "[QueueProcessor] Initialized successfully" << std::endl;
    return true;
}

void QueueProcessor::shutdown() {
    if (!initialized_.load()) {
        return;
    }
    
    initialized_.store(false);
    message_queue_ = nullptr;
    
    // Clear handlers
    {
        std::lock_guard<std::mutex> lock(handlers_mutex_);
        chat_handler_ = nullptr;
        emote_handler_ = nullptr;
        move_handler_ = nullptr;
        touch_handler_ = nullptr;
        sit_handler_ = nullptr;
        stand_handler_ = nullptr;
        look_at_handler_ = nullptr;
    }
    
    std::cout << "[QueueProcessor] Shut down" << std::endl;
}

void QueueProcessor::registerChatHandler(ChatHandler handler) {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    chat_handler_ = handler;
}

void QueueProcessor::registerEmoteHandler(EmoteHandler handler) {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    emote_handler_ = handler;
}

void QueueProcessor::registerMoveHandler(MoveHandler handler) {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    move_handler_ = handler;
}

void QueueProcessor::registerTouchHandler(TouchHandler handler) {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    touch_handler_ = handler;
}

void QueueProcessor::registerSitHandler(SitHandler handler) {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    sit_handler_ = handler;
}

void QueueProcessor::registerStandHandler(StandHandler handler) {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    stand_handler_ = handler;
}

void QueueProcessor::registerLookAtHandler(LookAtHandler handler) {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    look_at_handler_ = handler;
}

void QueueProcessor::processCommands(int max_commands_per_frame) {
    if (!initialized_.load() || !message_queue_) {
        return;
    }
    
    int commands_processed = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    while (commands_processed < max_commands_per_frame) {
        // Get next command from queue
        auto command_opt = message_queue_->dequeue();
        if (!command_opt.has_value()) {
            break; // No more commands
        }
        
        // Process the command
        auto cmd_start = std::chrono::high_resolution_clock::now();
        bool success = processCommand(command_opt.value());
        auto cmd_end = std::chrono::high_resolution_clock::now();
        
        // Record processing time
        auto cmd_duration = std::chrono::duration<double, std::milli>(cmd_end - cmd_start);
        recordProcessingTime(cmd_duration.count());
        
        // Update statistics
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            if (success) {
                stats_.commands_processed++;
            } else {
                stats_.commands_failed++;
            }
        }
        
        commands_processed++;
    }
    
    // Log performance if we processed commands
    if (commands_processed > 0) {
        auto total_time = std::chrono::high_resolution_clock::now() - start_time;
        auto total_ms = std::chrono::duration<double, std::milli>(total_time).count();
        
        if (detailed_logging_) {
            std::cout << "[QueueProcessor] Processed " << commands_processed 
                     << " commands in " << total_ms << "ms" << std::endl;
        }
    }
}

bool QueueProcessor::processCommand(const std::string& json_command) {
    try {
        return parseAndExecuteCommand(json_command);
    } catch (const std::exception& e) {
        std::cerr << "[QueueProcessor] Exception processing command: " << e.what() << std::endl;
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.commands_failed++;
        return false;
    }
}

bool QueueProcessor::parseAndExecuteCommand(const std::string& json_command) {
    // Parse JSON
    json command_json;
    try {
        command_json = json::parse(json_command);
    } catch (const json::exception& e) {
        std::cerr << "[QueueProcessor] JSON parse error: " << e.what() << std::endl;
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.json_parse_errors++;
        return false;
    }
    
    // Extract command type
    if (!command_json.contains("command_type") || !command_json["command_type"].is_string()) {
        std::cerr << "[QueueProcessor] Invalid command: missing or invalid command_type" << std::endl;
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.validation_errors++;
        return false;
    }
    
    std::string command_type = command_json["command_type"];
    
    // Extract parameters
    std::map<std::string, std::string> parameters;
    if (command_json.contains("parameters") && command_json["parameters"].is_object()) {
        for (auto& [key, value] : command_json["parameters"].items()) {
            if (value.is_string()) {
                parameters[key] = value.get<std::string>();
            } else {
                // Convert non-string values to strings
                parameters[key] = value.dump();
            }
        }
    }
    
    // Execute based on command type
    bool success = false;
    if (command_type == "CHAT") {
        success = executeChatCommand(parameters);
    } else if (command_type == "EMOTE") {
        success = executeEmoteCommand(parameters);
    } else if (command_type == "MOVE") {
        success = executeMoveCommand(parameters);
    } else if (command_type == "TOUCH") {
        success = executeTouchCommand(parameters);
    } else if (command_type == "SIT") {
        success = executeSitCommand(parameters);
    } else if (command_type == "STAND") {
        success = executeStandCommand(parameters);
    } else if (command_type == "LOOK_AT") {
        success = executeLookAtCommand(parameters);
    } else {
        std::cerr << "[QueueProcessor] Unknown command type: " << command_type << std::endl;
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.validation_errors++;
        return false;
    }
    
    logCommandExecution(command_type, success);
    return success;
}

bool QueueProcessor::executeChatCommand(const std::map<std::string, std::string>& parameters) {
    if (!validateChatParameters(parameters)) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    if (!chat_handler_) {
        std::cerr << "[QueueProcessor] No chat handler registered" << std::endl;
        return false;
    }
    
    try {
        chat_handler_(parameters.at("content"));
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[QueueProcessor] Chat handler error: " << e.what() << std::endl;
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.handler_errors++;
        return false;
    }
}

bool QueueProcessor::executeEmoteCommand(const std::map<std::string, std::string>& parameters) {
    if (!validateEmoteParameters(parameters)) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    if (!emote_handler_) {
        std::cerr << "[QueueProcessor] No emote handler registered" << std::endl;
        return false;
    }
    
    try {
        emote_handler_(parameters.at("content"));
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[QueueProcessor] Emote handler error: " << e.what() << std::endl;
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.handler_errors++;
        return false;
    }
}

bool QueueProcessor::executeMoveCommand(const std::map<std::string, std::string>& parameters) {
    if (!validateMoveParameters(parameters)) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    if (!move_handler_) {
        std::cerr << "[QueueProcessor] No move handler registered" << std::endl;
        return false;
    }
    
    try {
        float x, y, z;
        parseFloat(parameters.at("x"), x);
        parseFloat(parameters.at("y"), y);
        parseFloat(parameters.at("z"), z);
        
        move_handler_(x, y, z);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[QueueProcessor] Move handler error: " << e.what() << std::endl;
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.handler_errors++;
        return false;
    }
}

bool QueueProcessor::executeTouchCommand(const std::map<std::string, std::string>& parameters) {
    if (!validateTouchParameters(parameters)) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    if (!touch_handler_) {
        std::cerr << "[QueueProcessor] No touch handler registered" << std::endl;
        return false;
    }
    
    try {
        touch_handler_(parameters.at("target_uuid"));
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[QueueProcessor] Touch handler error: " << e.what() << std::endl;
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.handler_errors++;
        return false;
    }
}

bool QueueProcessor::executeSitCommand(const std::map<std::string, std::string>& parameters) {
    if (!validateSitParameters(parameters)) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    if (!sit_handler_) {
        std::cerr << "[QueueProcessor] No sit handler registered" << std::endl;
        return false;
    }
    
    try {
        // Sit command may or may not have target_uuid
        auto it = parameters.find("target_uuid");
        std::string target_uuid = (it != parameters.end()) ? it->second : "";
        sit_handler_(target_uuid);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[QueueProcessor] Sit handler error: " << e.what() << std::endl;
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.handler_errors++;
        return false;
    }
}

bool QueueProcessor::executeStandCommand(const std::map<std::string, std::string>& parameters) {
    // Stand command doesn't need parameters validation
    
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    if (!stand_handler_) {
        std::cerr << "[QueueProcessor] No stand handler registered" << std::endl;
        return false;
    }
    
    try {
        stand_handler_();
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[QueueProcessor] Stand handler error: " << e.what() << std::endl;
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.handler_errors++;
        return false;
    }
}

bool QueueProcessor::executeLookAtCommand(const std::map<std::string, std::string>& parameters) {
    if (!validateLookAtParameters(parameters)) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    if (!look_at_handler_) {
        std::cerr << "[QueueProcessor] No look_at handler registered" << std::endl;
        return false;
    }
    
    try {
        float x, y, z;
        parseFloat(parameters.at("x"), x);
        parseFloat(parameters.at("y"), y);
        parseFloat(parameters.at("z"), z);
        
        look_at_handler_(x, y, z);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[QueueProcessor] Look_at handler error: " << e.what() << std::endl;
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.handler_errors++;
        return false;
    }
}

// Validation methods
bool QueueProcessor::validateChatParameters(const std::map<std::string, std::string>& params) {
    if (params.find("content") == params.end()) {
        std::cerr << "[QueueProcessor] CHAT command missing 'content' parameter" << std::endl;
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.validation_errors++;
        return false;
    }
    return true;
}

bool QueueProcessor::validateEmoteParameters(const std::map<std::string, std::string>& params) {
    if (params.find("content") == params.end()) {
        std::cerr << "[QueueProcessor] EMOTE command missing 'content' parameter" << std::endl;
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.validation_errors++;
        return false;
    }
    return true;
}

bool QueueProcessor::validateMoveParameters(const std::map<std::string, std::string>& params) {
    for (const std::string& coord : {"x", "y", "z"}) {
        auto it = params.find(coord);
        if (it == params.end()) {
            std::cerr << "[QueueProcessor] MOVE command missing '" << coord << "' parameter" << std::endl;
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.validation_errors++;
            return false;
        }
        
        float value;
        if (!parseFloat(it->second, value)) {
            std::cerr << "[QueueProcessor] MOVE command invalid '" << coord << "' value: " << it->second << std::endl;
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.validation_errors++;
            return false;
        }
    }
    return true;
}

bool QueueProcessor::validateTouchParameters(const std::map<std::string, std::string>& params) {
    if (params.find("target_uuid") == params.end()) {
        std::cerr << "[QueueProcessor] TOUCH command missing 'target_uuid' parameter" << std::endl;
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.validation_errors++;
        return false;
    }
    return true;
}

bool QueueProcessor::validateSitParameters(const std::map<std::string, std::string>& params) {
    // SIT command can work with or without target_uuid
    return true;
}

bool QueueProcessor::validateLookAtParameters(const std::map<std::string, std::string>& params) {
    for (const std::string& coord : {"x", "y", "z"}) {
        auto it = params.find(coord);
        if (it == params.end()) {
            std::cerr << "[QueueProcessor] LOOK_AT command missing '" << coord << "' parameter" << std::endl;
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.validation_errors++;
            return false;
        }
        
        float value;
        if (!parseFloat(it->second, value)) {
            std::cerr << "[QueueProcessor] LOOK_AT command invalid '" << coord << "' value: " << it->second << std::endl;
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.validation_errors++;
            return false;
        }
    }
    return true;
}

bool QueueProcessor::parseFloat(const std::string& str, float& result) {
    try {
        result = std::stof(str);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

void QueueProcessor::recordProcessingTime(double time_ms) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    processing_times_.push_back(time_ms);
    
    // Keep only recent times for moving average
    if (processing_times_.size() > 1000) {
        processing_times_.erase(processing_times_.begin(), 
                              processing_times_.begin() + 500);
    }
    
    // Update max time
    if (time_ms > stats_.max_processing_time_ms) {
        stats_.max_processing_time_ms = time_ms;
    }
    
    // Calculate average
    double sum = 0.0;
    for (double time : processing_times_) {
        sum += time;
    }
    stats_.avg_processing_time_ms = sum / processing_times_.size();
}

void QueueProcessor::logCommandExecution(const std::string& command_type, bool success, 
                                       const std::string& details) {
    if (detailed_logging_) {
        std::cout << "[QueueProcessor] " << command_type << " command " 
                 << (success ? "succeeded" : "failed");
        if (!details.empty()) {
            std::cout << ": " << details;
        }
        std::cout << std::endl;
    }
}

QueueProcessor::ProcessorStats QueueProcessor::getStatistics() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

void QueueProcessor::resetStatistics() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_ = ProcessorStats();
    processing_times_.clear();
}

//==============================================================================
// QueueProcessorIntegration Implementation  
//==============================================================================

QueueProcessor& QueueProcessorIntegration::getInstance() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    return instance_;
}

void QueueProcessorIntegration::initializeWithHandlers(
    MessageQueueInterface* queue,
    std::function<void(const char*)> chat_handler,
    std::function<void(const char*)> emote_handler,
    std::function<void(float, float, float)> move_handler,
    std::function<void(const char*)> touch_handler,
    std::function<void(const char*)> sit_handler,
    std::function<void()> stand_handler,
    std::function<void(float, float, float)> look_at_handler
) {
    auto& processor = getInstance();
    
    // Initialize the processor
    if (!processor.initialize(queue)) {
        std::cerr << "[QueueProcessorIntegration] Failed to initialize processor" << std::endl;
        return;
    }
    
    // Register handlers (with proper conversion)
    if (chat_handler) {
        processor.registerChatHandler([chat_handler](const std::string& content) {
            chat_handler(content.c_str());
        });
    }
    
    if (emote_handler) {
        processor.registerEmoteHandler([emote_handler](const std::string& content) {
            emote_handler(content.c_str());
        });
    }
    
    if (move_handler) {
        processor.registerMoveHandler(move_handler);
    }
    
    if (touch_handler) {
        processor.registerTouchHandler([touch_handler](const std::string& uuid) {
            touch_handler(uuid.c_str());
        });
    }
    
    if (sit_handler) {
        processor.registerSitHandler([sit_handler](const std::string& uuid) {
            sit_handler(uuid.c_str());
        });
    }
    
    if (stand_handler) {
        processor.registerStandHandler(stand_handler);
    }
    
    if (look_at_handler) {
        processor.registerLookAtHandler(look_at_handler);
    }
    
    std::cout << "[QueueProcessorIntegration] Initialized with handlers" << std::endl;
}

void QueueProcessorIntegration::processCommandsInGameLoop(int max_commands_per_frame) {
    auto& processor = getInstance();
    processor.processCommands(max_commands_per_frame);
}

QueueProcessor::ProcessorStats QueueProcessorIntegration::getProcessingStatistics() {
    auto& processor = getInstance();
    return processor.getStatistics();
}

} // namespace python_bridge
