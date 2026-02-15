#ifndef AI_MESSAGE_QUEUE_H
#define AI_MESSAGE_QUEUE_H

#include <atomic>
#include <string>
#include <optional>
#include <memory>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <map>
#include <functional>

#ifdef _WIN32
#include <windows.h>
#include <io.h>
#else
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#endif

namespace python_bridge {

/**
 * Overflow handling strategy for message queues
 */
enum class OverflowStrategy {
    DROP_OLDEST,    // Remove oldest messages when full
    DROP_NEWEST,    // Drop new messages when full  
    BLOCK,          // Block until space available
    EXPAND          // Dynamically expand queue size
};

/**
 * Lock-free ring buffer for AI command messages.
 * 
 * PERFORMANCE CHARACTERISTICS:
 * - Lock-free single producer, single consumer (SPSC) design
 * - Sub-microsecond enqueue/dequeue operations
 * - Cache-line aligned slots to minimize false sharing
 * - JSON serialization for type safety
 * - Graceful overflow handling with command validation
 * 
 * THREAD SAFETY:
 * - Safe for concurrent access from one writer (Python) and one reader (C++)
 * - Uses memory ordering for proper synchronization
 * - No mutexes or locks in the critical path
 * 
 * RELIABILITY:
 * - Atomic sequence numbers prevent ABA problems
 * - Data length validation prevents buffer overruns
 * - Graceful handling of queue full scenarios
 * - Command validation and error reporting
 */
class AICommandQueue {
public:
    static constexpr size_t QUEUE_SIZE = 1024;           // Must be power of 2
    static constexpr size_t MAX_COMMAND_SIZE = 4096;     // Max JSON command size
    static constexpr uint32_t SEQUENCE_MASK = QUEUE_SIZE - 1;
    
    // Command slot structure (cache-line aligned)
    struct alignas(64) CommandSlot {
        char json_data[MAX_COMMAND_SIZE];
        std::atomic<uint32_t> data_length{0};    // 0 = empty, >0 = has data
        std::atomic<uint32_t> sequence{0};       // Sequence number for validation
        
        CommandSlot() {
            std::memset(json_data, 0, MAX_COMMAND_SIZE);
        }
    };
    
    // Statistics for monitoring
    struct QueueStats {
        uint64_t total_enqueued = 0;
        uint64_t total_dequeued = 0;
        uint64_t enqueue_failures = 0;
        uint64_t dequeue_failures = 0;
        uint64_t validation_failures = 0;
        size_t max_queue_depth = 0;
        double avg_command_size = 0.0;
    };

public:
    AICommandQueue();
    ~AICommandQueue() = default;
    
    // Core queue operations
    bool tryEnqueue(const std::string& json_command);
    std::optional<std::string> tryDequeue();
    
    // Queue state queries
    size_t pendingCount() const;
    bool isFull() const;
    bool isEmpty() const;
    size_t capacity() const { return QUEUE_SIZE; }
    
    // Statistics and monitoring
    QueueStats getStatistics() const;
    void resetStatistics();
    
    // Validation and testing
    bool validateCommand(const std::string& json_command) const;
    void clear();

private:
    // Cache-line aligned ring buffer
    alignas(64) CommandSlot slots_[QUEUE_SIZE];
    
    // Producer/consumer sequence counters (cache-line aligned)
    alignas(64) std::atomic<uint32_t> write_sequence_{0};
    alignas(64) std::atomic<uint32_t> read_sequence_{0};
    
    // Statistics (mutable for const methods)
    mutable std::atomic<uint64_t> stats_enqueued_{0};
    mutable std::atomic<uint64_t> stats_dequeued_{0};
    mutable std::atomic<uint64_t> stats_enqueue_failures_{0};
    mutable std::atomic<uint64_t> stats_dequeue_failures_{0};
    mutable std::atomic<uint64_t> stats_validation_failures_{0};
    mutable std::atomic<size_t> stats_max_depth_{0};
    mutable std::atomic<uint64_t> stats_total_command_size_{0};
    
    // Helper methods
    uint32_t getSlotIndex(uint32_t sequence) const {
        return sequence & SEQUENCE_MASK;
    }
    
    bool isValidJson(const std::string& json_str) const;
    void updateMaxDepthStat() const;
};

/**
 * Persistent message queue using memory-mapped file.
 * 
 * PERSISTENCE FEATURES:
 * - Commands survive process restarts
 * - Memory-mapped file for efficient I/O
 * - Atomic writes prevent corruption
 * - Cross-platform implementation
 * 
 * RELIABILITY:
 * - File locking prevents concurrent access
 * - Header validation ensures file integrity
 * - Recovery from corrupted files
 * - Graceful handling of disk full scenarios
 */
class PersistentMessageQueue {
public:
    static constexpr const char* DEFAULT_QUEUE_FILE = "ai_command_queue.bin";
    static constexpr size_t DEFAULT_QUEUE_SIZE = 1024;
    static constexpr uint32_t FILE_MAGIC = 0x41495145; // "AIQE"
    static constexpr uint32_t FILE_VERSION = 1;
    
    // File header structure
    struct QueueHeader {
        uint32_t magic;
        uint32_t version;
        uint32_t queue_size;
        uint32_t max_command_size;
        std::atomic<uint32_t> write_sequence;
        std::atomic<uint32_t> read_sequence;
        uint64_t created_timestamp;
        uint64_t last_accessed_timestamp;
        uint8_t reserved[32]; // For future expansion
    };

public:
    explicit PersistentMessageQueue(const std::string& queue_file = DEFAULT_QUEUE_FILE,
                                  size_t queue_size = DEFAULT_QUEUE_SIZE);
    ~PersistentMessageQueue();
    
    // Lifecycle
    bool initialize();
    void shutdown();
    bool isInitialized() const { return initialized_; }
    
    // Queue operations (same interface as in-memory queue)
    bool tryEnqueue(const std::string& json_command);
    std::optional<std::string> tryDequeue();
    
    // Queue state
    size_t pendingCount() const;
    bool isFull() const;
    bool isEmpty() const;
    size_t capacity() const { return queue_size_; }
    
    // File management
    bool validateFile() const;
    bool recoverFromCorruption();
    void clearQueue();
    
    // Statistics
    AICommandQueue::QueueStats getStatistics() const;

private:
    std::string queue_file_path_;
    size_t queue_size_;
    bool initialized_;
    
    // Memory mapping
    void* mapped_memory_;
    size_t mapped_size_;
    
#ifdef _WIN32
    HANDLE file_handle_;
    HANDLE mapping_handle_;
#else
    int file_descriptor_;
#endif
    
    // Queue components in mapped memory
    QueueHeader* header_;
    AICommandQueue::CommandSlot* slots_;
    
    // Thread safety
    mutable std::mutex file_mutex_;
    
    // Helper methods
    bool createNewFile();
    bool openExistingFile();
    bool mapMemory();
    void unmapMemory();
    size_t calculateFileSize() const;
    uint32_t getSlotIndex(uint32_t sequence) const;
    bool validateHeader() const;
};

/**
 * Unified message queue interface.
 * 
 * This class provides a single interface that can use either in-memory
 * or persistent queues based on configuration.
 */
class MessageQueueInterface {
public:
    enum class QueueType {
        IN_MEMORY,      // Fast, non-persistent
        PERSISTENT,     // Slower, survives restarts
        HYBRID         // In-memory with periodic persistence
    };

public:
    explicit MessageQueueInterface(QueueType type = QueueType::IN_MEMORY,
                                 const std::string& persistent_file = "",
                                 size_t capacity = 1000,
                                 OverflowStrategy overflow_strategy = OverflowStrategy::DROP_OLDEST);
    ~MessageQueueInterface() = default;
    
    // Lifecycle
    bool initialize();
    void shutdown();
    
    // Queue operations
    bool enqueue(const std::string& json_command);
    std::optional<std::string> dequeue();
    
    // Convenience methods for specific command types
    bool sendChat(const std::string& content);
    bool sendEmote(const std::string& content);
    bool sendMove(float x, float y, float z);
    bool sendTouch(const std::string& target_uuid);
    bool sendSit(const std::string& target_uuid);
    bool sendStand();
    bool sendLookAt(float x, float y, float z);
    
    // Queue state
    size_t pendingCount() const;
    size_t getPendingCount() const { return pendingCount(); } // Alias for tests
    bool isFull() const;
    bool isEmpty() const;
    
    // Queue access (for testing)
    AICommandQueue* getQueue() const { return memory_queue_.get(); }
    
    // Statistics and monitoring
    AICommandQueue::QueueStats getStatistics() const;
    void resetStatistics();

private:
    QueueType queue_type_;
    std::unique_ptr<AICommandQueue> memory_queue_;
    std::unique_ptr<PersistentMessageQueue> persistent_queue_;
    
    // Helper methods
    std::string buildCommandJson(const std::string& command_type, 
                               const std::map<std::string, std::string>& parameters) const;
    bool validateCommandJson(const std::string& json_command) const;
};

// Global queue instance for easy access
extern std::unique_ptr<MessageQueueInterface> g_message_queue;

// Helper functions for queue management
bool initializeGlobalMessageQueue(MessageQueueInterface::QueueType type = MessageQueueInterface::QueueType::IN_MEMORY,
                                const std::string& persistent_file = "");
void shutdownGlobalMessageQueue();
MessageQueueInterface* getGlobalMessageQueue();

} // namespace python_bridge

#endif // AI_MESSAGE_QUEUE_H