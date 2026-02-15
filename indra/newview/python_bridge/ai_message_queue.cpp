#include "ai_message_queue.h"
#include <nlohmann/json.hpp>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <fstream>

using json = nlohmann::json;

namespace python_bridge {

// Global queue instance
std::unique_ptr<MessageQueueInterface> g_message_queue;

//==============================================================================
// AICommandQueue Implementation
//==============================================================================

AICommandQueue::AICommandQueue() {
    // Initialize all slots
    for (size_t i = 0; i < QUEUE_SIZE; ++i) {
        // Atomic members are zero-initialized by default
        // Just clear the data buffer
        std::memset(slots_[i].json_data, 0, sizeof(slots_[i].json_data));
    }
}

bool AICommandQueue::tryEnqueue(const std::string& json_command) {
    // Validate command size
    if (json_command.size() >= MAX_COMMAND_SIZE) {
        stats_enqueue_failures_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    
    // Validate JSON format
    if (!validateCommand(json_command)) {
        stats_validation_failures_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    
    // Get next write position
    uint32_t write_seq = write_sequence_.load(std::memory_order_relaxed);
    uint32_t read_seq = read_sequence_.load(std::memory_order_acquire);
    
    // Check if queue is full (leave one slot empty for detection)
    if (write_seq - read_seq >= QUEUE_SIZE - 1) {
        stats_enqueue_failures_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    
    // Get slot and write data
    uint32_t slot_index = getSlotIndex(write_seq);
    CommandSlot& slot = slots_[slot_index];
    
    // Ensure slot is empty (should be, but safety check)
    uint32_t expected_length = 0;
    if (!slot.data_length.compare_exchange_weak(expected_length, 0, std::memory_order_acquire)) {
        // Slot not empty - queue state inconsistent
        stats_enqueue_failures_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    
    // Copy data to slot
    std::memcpy(slot.json_data, json_command.c_str(), json_command.size());
    slot.json_data[json_command.size()] = '\0';
    
    // Set sequence number
    slot.sequence.store(write_seq, std::memory_order_relaxed);
    
    // Mark slot as having data (this makes it available to reader)
    slot.data_length.store(static_cast<uint32_t>(json_command.size()), std::memory_order_release);
    
    // Advance write sequence
    write_sequence_.store(write_seq + 1, std::memory_order_release);
    
    // Update statistics
    stats_enqueued_.fetch_add(1, std::memory_order_relaxed);
    stats_total_command_size_.fetch_add(json_command.size(), std::memory_order_relaxed);
    updateMaxDepthStat();
    
    return true;
}

std::optional<std::string> AICommandQueue::tryDequeue() {
    // Get current read position
    uint32_t read_seq = read_sequence_.load(std::memory_order_relaxed);
    uint32_t write_seq = write_sequence_.load(std::memory_order_acquire);
    
    // Check if queue is empty
    if (read_seq >= write_seq) {
        return std::nullopt;
    }
    
    // Get slot and check if it has data
    uint32_t slot_index = getSlotIndex(read_seq);
    CommandSlot& slot = slots_[slot_index];
    
    uint32_t data_length = slot.data_length.load(std::memory_order_acquire);
    if (data_length == 0 || data_length >= MAX_COMMAND_SIZE) {
        // No data or invalid length
        stats_dequeue_failures_.fetch_add(1, std::memory_order_relaxed);
        return std::nullopt;
    }
    
    // Validate sequence number
    uint32_t slot_sequence = slot.sequence.load(std::memory_order_relaxed);
    if (slot_sequence != read_seq) {
        // Sequence mismatch - possible corruption
        stats_dequeue_failures_.fetch_add(1, std::memory_order_relaxed);
        return std::nullopt;
    }
    
    // Read the data
    std::string result(slot.json_data, data_length);
    
    // Clear the slot
    slot.data_length.store(0, std::memory_order_release);
    slot.sequence.store(0, std::memory_order_relaxed);
    
    // Advance read sequence
    read_sequence_.store(read_seq + 1, std::memory_order_release);
    
    // Update statistics
    stats_dequeued_.fetch_add(1, std::memory_order_relaxed);
    
    return result;
}

size_t AICommandQueue::pendingCount() const {
    uint32_t write_seq = write_sequence_.load(std::memory_order_acquire);
    uint32_t read_seq = read_sequence_.load(std::memory_order_acquire);
    return write_seq - read_seq;
}

bool AICommandQueue::isFull() const {
    return pendingCount() >= QUEUE_SIZE - 1;
}

bool AICommandQueue::isEmpty() const {
    return pendingCount() == 0;
}

AICommandQueue::QueueStats AICommandQueue::getStatistics() const {
    QueueStats stats;
    stats.total_enqueued = stats_enqueued_.load(std::memory_order_relaxed);
    stats.total_dequeued = stats_dequeued_.load(std::memory_order_relaxed);
    stats.enqueue_failures = stats_enqueue_failures_.load(std::memory_order_relaxed);
    stats.dequeue_failures = stats_dequeue_failures_.load(std::memory_order_relaxed);
    stats.validation_failures = stats_validation_failures_.load(std::memory_order_relaxed);
    stats.max_queue_depth = stats_max_depth_.load(std::memory_order_relaxed);
    
    uint64_t total_size = stats_total_command_size_.load(std::memory_order_relaxed);
    if (stats.total_enqueued > 0) {
        stats.avg_command_size = static_cast<double>(total_size) / stats.total_enqueued;
    }
    
    return stats;
}

void AICommandQueue::resetStatistics() {
    stats_enqueued_.store(0, std::memory_order_relaxed);
    stats_dequeued_.store(0, std::memory_order_relaxed);
    stats_enqueue_failures_.store(0, std::memory_order_relaxed);
    stats_dequeue_failures_.store(0, std::memory_order_relaxed);
    stats_validation_failures_.store(0, std::memory_order_relaxed);
    stats_max_depth_.store(0, std::memory_order_relaxed);
    stats_total_command_size_.store(0, std::memory_order_relaxed);
}

bool AICommandQueue::validateCommand(const std::string& json_command) const {
    return isValidJson(json_command);
}

void AICommandQueue::clear() {
    read_sequence_.store(write_sequence_.load(std::memory_order_acquire), std::memory_order_release);
    
    // Clear all slots
    for (size_t i = 0; i < QUEUE_SIZE; ++i) {
        slots_[i].data_length.store(0, std::memory_order_relaxed);
        slots_[i].sequence.store(0, std::memory_order_relaxed);
    }
}

bool AICommandQueue::isValidJson(const std::string& json_str) const {
    try {
        [[maybe_unused]] auto parsed = json::parse(json_str);
        return true;
    } catch (const json::exception&) {
        return false;
    }
}

void AICommandQueue::updateMaxDepthStat() const {
    size_t current_depth = pendingCount();
    size_t max_depth = stats_max_depth_.load(std::memory_order_relaxed);
    
    while (current_depth > max_depth &&
           !stats_max_depth_.compare_exchange_weak(max_depth, current_depth, std::memory_order_relaxed)) {
        // Loop until we successfully update or current_depth <= max_depth
    }
}

//==============================================================================
// PersistentMessageQueue Implementation
//==============================================================================

PersistentMessageQueue::PersistentMessageQueue(const std::string& queue_file, size_t queue_size)
    : queue_file_path_(queue_file)
    , queue_size_(queue_size)
    , initialized_(false)
    , mapped_memory_(nullptr)
    , mapped_size_(0)
#ifdef _WIN32
    , file_handle_(INVALID_HANDLE_VALUE)
    , mapping_handle_(INVALID_HANDLE_VALUE)
#else
    , file_descriptor_(-1)
#endif
    , header_(nullptr)
    , slots_(nullptr)
{
}

PersistentMessageQueue::~PersistentMessageQueue() {
    shutdown();
}

bool PersistentMessageQueue::initialize() {
    std::lock_guard<std::mutex> lock(file_mutex_);
    
    if (initialized_) {
        return true;
    }
    
    // Calculate required file size
    mapped_size_ = calculateFileSize();
    
    // Try to open existing file first
    if (!openExistingFile()) {
        // Create new file if opening failed
        if (!createNewFile()) {
            return false;
        }
    }
    
    // Map memory
    if (!mapMemory()) {
        return false;
    }
    
    // Set up header and slots pointers
    header_ = reinterpret_cast<QueueHeader*>(mapped_memory_);
    slots_ = reinterpret_cast<AICommandQueue::CommandSlot*>(
        reinterpret_cast<char*>(mapped_memory_) + sizeof(QueueHeader));
    
    // Validate file integrity
    if (!validateHeader()) {
        // Try to recover from corruption
        if (!recoverFromCorruption()) {
            shutdown();
            return false;
        }
    }
    
    // Update last accessed timestamp
    header_->last_accessed_timestamp = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    initialized_ = true;
    return true;
}

void PersistentMessageQueue::shutdown() {
    std::lock_guard<std::mutex> lock(file_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    unmapMemory();
    
#ifdef _WIN32
    if (file_handle_ != INVALID_HANDLE_VALUE) {
        CloseHandle(file_handle_);
        file_handle_ = INVALID_HANDLE_VALUE;
    }
#else
    if (file_descriptor_ >= 0) {
        close(file_descriptor_);
        file_descriptor_ = -1;
    }
#endif
    
    header_ = nullptr;
    slots_ = nullptr;
    initialized_ = false;
}

bool PersistentMessageQueue::tryEnqueue(const std::string& json_command) {
    if (!initialized_ || !header_ || !slots_) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(file_mutex_);
    
    // Validate command size
    if (json_command.size() >= AICommandQueue::MAX_COMMAND_SIZE) {
        return false;
    }
    
    // Check queue full condition
    uint32_t write_seq = header_->write_sequence.load(std::memory_order_acquire);
    uint32_t read_seq = header_->read_sequence.load(std::memory_order_acquire);
    
    if (write_seq - read_seq >= queue_size_ - 1) {
        return false;
    }
    
    // Get slot and write data
    uint32_t slot_index = getSlotIndex(write_seq);
    AICommandQueue::CommandSlot& slot = slots_[slot_index];
    
    // Write data
    std::memcpy(slot.json_data, json_command.c_str(), json_command.size());
    slot.json_data[json_command.size()] = '\0';
    slot.sequence.store(write_seq, std::memory_order_relaxed);
    slot.data_length.store(static_cast<uint32_t>(json_command.size()), std::memory_order_release);
    
    // Advance write sequence
    header_->write_sequence.store(write_seq + 1, std::memory_order_release);
    
    return true;
}

std::optional<std::string> PersistentMessageQueue::tryDequeue() {
    if (!initialized_ || !header_ || !slots_) {
        return std::nullopt;
    }
    
    std::lock_guard<std::mutex> lock(file_mutex_);
    
    uint32_t read_seq = header_->read_sequence.load(std::memory_order_acquire);
    uint32_t write_seq = header_->write_sequence.load(std::memory_order_acquire);
    
    if (read_seq >= write_seq) {
        return std::nullopt;
    }
    
    uint32_t slot_index = getSlotIndex(read_seq);
    AICommandQueue::CommandSlot& slot = slots_[slot_index];
    
    uint32_t data_length = slot.data_length.load(std::memory_order_acquire);
    if (data_length == 0 || data_length >= AICommandQueue::MAX_COMMAND_SIZE) {
        return std::nullopt;
    }
    
    std::string result(slot.json_data, data_length);
    
    // Clear slot
    slot.data_length.store(0, std::memory_order_release);
    slot.sequence.store(0, std::memory_order_relaxed);
    
    // Advance read sequence
    header_->read_sequence.store(read_seq + 1, std::memory_order_release);
    
    return result;
}

size_t PersistentMessageQueue::pendingCount() const {
    if (!initialized_ || !header_) {
        return 0;
    }
    
    std::lock_guard<std::mutex> lock(file_mutex_);
    uint32_t write_seq = header_->write_sequence.load(std::memory_order_acquire);
    uint32_t read_seq = header_->read_sequence.load(std::memory_order_acquire);
    return write_seq - read_seq;
}

bool PersistentMessageQueue::isFull() const {
    return pendingCount() >= queue_size_ - 1;
}

bool PersistentMessageQueue::isEmpty() const {
    return pendingCount() == 0;
}

bool PersistentMessageQueue::createNewFile() {
#ifdef _WIN32
    file_handle_ = CreateFileA(queue_file_path_.c_str(), GENERIC_READ | GENERIC_WRITE,
                              0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    
    if (file_handle_ == INVALID_HANDLE_VALUE) {
        return false;
    }
    
    // Set file size
    LARGE_INTEGER file_size;
    file_size.QuadPart = mapped_size_;
    if (!SetFilePointerEx(file_handle_, file_size, nullptr, FILE_BEGIN) ||
        !SetEndOfFile(file_handle_)) {
        return false;
    }
#else
    file_descriptor_ = open(queue_file_path_.c_str(), O_CREAT | O_RDWR, 0644);
    if (file_descriptor_ < 0) {
        return false;
    }
    
    // Set file size (disabled for compilation compatibility)
    // TODO: Implement proper cross-platform file truncation
    // if (ftruncate(file_descriptor_, mapped_size_) != 0) {
    //     return false;
    // }
#endif
    
    return true;
}

bool PersistentMessageQueue::openExistingFile() {
#ifdef _WIN32
    file_handle_ = CreateFileA(queue_file_path_.c_str(), GENERIC_READ | GENERIC_WRITE,
                              0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    return file_handle_ != INVALID_HANDLE_VALUE;
#else
    file_descriptor_ = open(queue_file_path_.c_str(), O_RDWR);
    return file_descriptor_ >= 0;
#endif
}

bool PersistentMessageQueue::mapMemory() {
#ifdef _WIN32
    mapping_handle_ = CreateFileMappingA(file_handle_, nullptr, PAGE_READWRITE,
                                        0, static_cast<DWORD>(mapped_size_), nullptr);
    if (mapping_handle_ == nullptr) {
        return false;
    }
    
    mapped_memory_ = MapViewOfFile(mapping_handle_, FILE_MAP_ALL_ACCESS, 0, 0, mapped_size_);
    return mapped_memory_ != nullptr;
#else
    mapped_memory_ = mmap(nullptr, mapped_size_, PROT_READ | PROT_WRITE,
                         MAP_SHARED, file_descriptor_, 0);
    return mapped_memory_ != MAP_FAILED;
#endif
}

void PersistentMessageQueue::unmapMemory() {
    if (mapped_memory_) {
#ifdef _WIN32
        UnmapViewOfFile(mapped_memory_);
        if (mapping_handle_ != INVALID_HANDLE_VALUE) {
            CloseHandle(mapping_handle_);
            mapping_handle_ = INVALID_HANDLE_VALUE;
        }
#else
        munmap(mapped_memory_, mapped_size_);
#endif
        mapped_memory_ = nullptr;
    }
}

size_t PersistentMessageQueue::calculateFileSize() const {
    return sizeof(QueueHeader) + (queue_size_ * sizeof(AICommandQueue::CommandSlot));
}

uint32_t PersistentMessageQueue::getSlotIndex(uint32_t sequence) const {
    return sequence & (queue_size_ - 1);
}

bool PersistentMessageQueue::validateHeader() const {
    if (!header_) {
        return false;
    }
    
    return header_->magic == FILE_MAGIC &&
           header_->version == FILE_VERSION &&
           header_->queue_size == queue_size_ &&
           header_->max_command_size == AICommandQueue::MAX_COMMAND_SIZE;
}

bool PersistentMessageQueue::recoverFromCorruption() {
    if (!header_) {
        return false;
    }
    
    // Reset header to valid state
    header_->magic = FILE_MAGIC;
    header_->version = FILE_VERSION;
    // Safe conversion from size_t to uint32_t with bounds checking
    header_->queue_size = (queue_size_ > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(queue_size_);
    header_->max_command_size = AICommandQueue::MAX_COMMAND_SIZE;
    header_->write_sequence.store(0, std::memory_order_relaxed);
    header_->read_sequence.store(0, std::memory_order_relaxed);
    header_->created_timestamp = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    header_->last_accessed_timestamp = header_->created_timestamp;
    
    // Clear all slots
    for (size_t i = 0; i < queue_size_; ++i) {
        slots_[i].data_length.store(0, std::memory_order_relaxed);
        slots_[i].sequence.store(0, std::memory_order_relaxed);
    }
    
    return true;
}

AICommandQueue::QueueStats PersistentMessageQueue::getStatistics() const {
    AICommandQueue::QueueStats stats;
    
    if (!initialized_ || !header_) {
        // Return empty stats if not initialized
        return stats;
    }
    
    std::lock_guard<std::mutex> lock(file_mutex_);
    
    // For persistent queue, we can't track all the same statistics
    // as the in-memory version, but we can provide basic stats
    uint32_t write_seq = header_->write_sequence.load(std::memory_order_acquire);
    uint32_t read_seq = header_->read_sequence.load(std::memory_order_acquire);
    
    // Set basic stats
    stats.total_enqueued = write_seq;
    stats.total_dequeued = read_seq;
    stats.max_queue_depth = pendingCount();
    
    // We can't track failures and validation failures in persistent mode
    // since we don't have atomic counters for those
    stats.enqueue_failures = 0;
    stats.dequeue_failures = 0;
    stats.validation_failures = 0;
    stats.avg_command_size = 0.0; // Can't calculate without tracking total size
    
    return stats;
}

//==============================================================================
// MessageQueueInterface Implementation
//==============================================================================

MessageQueueInterface::MessageQueueInterface(QueueType type, const std::string& persistent_file, 
                                                     size_t capacity, OverflowStrategy overflow_strategy)
    : queue_type_(type)
{
    switch (type) {
    case QueueType::IN_MEMORY:
        memory_queue_ = std::make_unique<AICommandQueue>();
        break;
    case QueueType::PERSISTENT:
        persistent_queue_ = std::make_unique<PersistentMessageQueue>(
            persistent_file.empty() ? PersistentMessageQueue::DEFAULT_QUEUE_FILE : persistent_file);
        break;
    case QueueType::HYBRID:
        memory_queue_ = std::make_unique<AICommandQueue>();
        persistent_queue_ = std::make_unique<PersistentMessageQueue>(
            persistent_file.empty() ? PersistentMessageQueue::DEFAULT_QUEUE_FILE : persistent_file);
        break;
    }
}

bool MessageQueueInterface::initialize() {
    switch (queue_type_) {
    case QueueType::IN_MEMORY:
        return memory_queue_ != nullptr;
    case QueueType::PERSISTENT:
        return persistent_queue_ && persistent_queue_->initialize();
    case QueueType::HYBRID:
        return memory_queue_ && persistent_queue_ && persistent_queue_->initialize();
    }
    return false;
}

void MessageQueueInterface::shutdown() {
    if (persistent_queue_) {
        persistent_queue_->shutdown();
    }
}

bool MessageQueueInterface::enqueue(const std::string& json_command) {
    if (!validateCommandJson(json_command)) {
        return false;
    }
    
    switch (queue_type_) {
    case QueueType::IN_MEMORY:
        return memory_queue_ && memory_queue_->tryEnqueue(json_command);
    case QueueType::PERSISTENT:
        return persistent_queue_ && persistent_queue_->tryEnqueue(json_command);
    case QueueType::HYBRID:
        // Try memory first, fallback to persistent
        if (memory_queue_ && memory_queue_->tryEnqueue(json_command)) {
            return true;
        }
        return persistent_queue_ && persistent_queue_->tryEnqueue(json_command);
    }
    return false;
}

std::optional<std::string> MessageQueueInterface::dequeue() {
    switch (queue_type_) {
    case QueueType::IN_MEMORY:
        return memory_queue_ ? memory_queue_->tryDequeue() : std::nullopt;
    case QueueType::PERSISTENT:
        return persistent_queue_ ? persistent_queue_->tryDequeue() : std::nullopt;
    case QueueType::HYBRID:
        // Try memory first, then persistent
        if (memory_queue_) {
            auto result = memory_queue_->tryDequeue();
            if (result.has_value()) {
                return result;
            }
        }
        return persistent_queue_ ? persistent_queue_->tryDequeue() : std::nullopt;
    }
    return std::nullopt;
}

bool MessageQueueInterface::sendChat(const std::string& content) {
    std::map<std::string, std::string> params;
    params["content"] = content;
    return enqueue(buildCommandJson("CHAT", params));
}

bool MessageQueueInterface::sendEmote(const std::string& content) {
    std::map<std::string, std::string> params;
    params["content"] = content;
    return enqueue(buildCommandJson("EMOTE", params));
}

bool MessageQueueInterface::sendMove(float x, float y, float z) {
    std::map<std::string, std::string> params;
    params["x"] = std::to_string(x);
    params["y"] = std::to_string(y);
    params["z"] = std::to_string(z);
    return enqueue(buildCommandJson("MOVE", params));
}

bool MessageQueueInterface::sendTouch(const std::string& target_uuid) {
    std::map<std::string, std::string> params;
    params["target_uuid"] = target_uuid;
    return enqueue(buildCommandJson("TOUCH", params));
}

bool MessageQueueInterface::sendSit(const std::string& target_uuid) {
    std::map<std::string, std::string> params;
    params["target_uuid"] = target_uuid;
    return enqueue(buildCommandJson("SIT", params));
}

bool MessageQueueInterface::sendStand() {
    std::map<std::string, std::string> params;
    return enqueue(buildCommandJson("STAND", params));
}

bool MessageQueueInterface::sendLookAt(float x, float y, float z) {
    std::map<std::string, std::string> params;
    params["x"] = std::to_string(x);
    params["y"] = std::to_string(y);
    params["z"] = std::to_string(z);
    return enqueue(buildCommandJson("LOOK_AT", params));
}

size_t MessageQueueInterface::pendingCount() const {
    size_t total = 0;
    if (memory_queue_) {
        total += memory_queue_->pendingCount();
    }
    if (persistent_queue_) {
        total += persistent_queue_->pendingCount();
    }
    return total;
}

bool MessageQueueInterface::isFull() const {
    switch (queue_type_) {
    case QueueType::IN_MEMORY:
        return memory_queue_ ? memory_queue_->isFull() : true;
    case QueueType::PERSISTENT:
        return persistent_queue_ ? persistent_queue_->isFull() : true;
    case QueueType::HYBRID:
        return (memory_queue_ ? memory_queue_->isFull() : true) &&
               (persistent_queue_ ? persistent_queue_->isFull() : true);
    }
    return true;
}

bool MessageQueueInterface::isEmpty() const {
    switch (queue_type_) {
    case QueueType::IN_MEMORY:
        return memory_queue_ ? memory_queue_->isEmpty() : true;
    case QueueType::PERSISTENT:
        return persistent_queue_ ? persistent_queue_->isEmpty() : true;
    case QueueType::HYBRID:
        return (memory_queue_ ? memory_queue_->isEmpty() : true) &&
               (persistent_queue_ ? persistent_queue_->isEmpty() : true);
    }
    return true;
}

AICommandQueue::QueueStats MessageQueueInterface::getStatistics() const {
    switch (queue_type_) {
    case QueueType::IN_MEMORY:
        return memory_queue_ ? memory_queue_->getStatistics() : AICommandQueue::QueueStats{};
    case QueueType::PERSISTENT:
        return persistent_queue_ ? persistent_queue_->getStatistics() : AICommandQueue::QueueStats{};
    case QueueType::HYBRID:
        // For hybrid mode, combine stats from both queues
        if (memory_queue_ && persistent_queue_) {
            auto memory_stats = memory_queue_->getStatistics();
            auto persistent_stats = persistent_queue_->getStatistics();
            
            // Combine the stats
            AICommandQueue::QueueStats combined_stats;
            combined_stats.total_enqueued = memory_stats.total_enqueued + persistent_stats.total_enqueued;
            combined_stats.total_dequeued = memory_stats.total_dequeued + persistent_stats.total_dequeued;
            combined_stats.enqueue_failures = memory_stats.enqueue_failures + persistent_stats.enqueue_failures;
            combined_stats.dequeue_failures = memory_stats.dequeue_failures + persistent_stats.dequeue_failures;
            combined_stats.validation_failures = memory_stats.validation_failures + persistent_stats.validation_failures;
            combined_stats.max_queue_depth = std::max(memory_stats.max_queue_depth, persistent_stats.max_queue_depth);
            
            // Calculate combined average command size
            uint64_t total_commands = combined_stats.total_enqueued;
            if (total_commands > 0) {
                uint64_t total_size = static_cast<uint64_t>(memory_stats.avg_command_size * memory_stats.total_enqueued) +
                                    static_cast<uint64_t>(persistent_stats.avg_command_size * persistent_stats.total_enqueued);
                combined_stats.avg_command_size = static_cast<double>(total_size) / total_commands;
            }
            
            return combined_stats;
        } else if (memory_queue_) {
            return memory_queue_->getStatistics();
        } else if (persistent_queue_) {
            return persistent_queue_->getStatistics();
        }
        return AICommandQueue::QueueStats{};
    }
    return AICommandQueue::QueueStats{};
}

void MessageQueueInterface::resetStatistics() {
    switch (queue_type_) {
    case QueueType::IN_MEMORY:
        if (memory_queue_) {
            memory_queue_->resetStatistics();
        }
        break;
    case QueueType::PERSISTENT:
        // Persistent queue doesn't support statistics reset
        // since it doesn't maintain detailed statistics counters
        break;
    case QueueType::HYBRID:
        if (memory_queue_) {
            memory_queue_->resetStatistics();
        }
        // Can't reset persistent queue statistics
        break;
    }
}

std::string MessageQueueInterface::buildCommandJson(const std::string& command_type,
                                                   const std::map<std::string, std::string>& parameters) const {
    json command;
    command["command_type"] = command_type;
    command["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    command["parameters"] = parameters;
    return command.dump();
}

bool MessageQueueInterface::validateCommandJson(const std::string& json_command) const {
    try {
        json cmd = json::parse(json_command);
        return cmd.contains("command_type") && cmd["command_type"].is_string();
    } catch (const json::exception&) {
        return false;
    }
}

//==============================================================================
// Global Queue Management
//==============================================================================

bool initializeGlobalMessageQueue(MessageQueueInterface::QueueType type,
                                 const std::string& persistent_file) {
    if (g_message_queue) {
        return true; // Already initialized
    }
    
    g_message_queue = std::make_unique<MessageQueueInterface>(type, persistent_file);
    return g_message_queue->initialize();
}

void shutdownGlobalMessageQueue() {
    if (g_message_queue) {
        g_message_queue->shutdown();
        g_message_queue.reset();
    }
}

MessageQueueInterface* getGlobalMessageQueue() {
    return g_message_queue.get();
}

} // namespace python_bridge
