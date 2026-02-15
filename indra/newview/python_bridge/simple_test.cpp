/*
 * Simple test for AI Message Queue System (without Google Test dependency)
 * 
 * This provides basic validation of the message queue components
 * for environments where Google Test is not available.
 */

#include "ai_message_queue.h"
#include "queue_processor.h"
#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include <cassert>
#include <atomic>

using namespace python_bridge;

// Simple test framework macros
#define TEST_ASSERT(condition, message) \
    if (!(condition)) { \
        std::cerr << "❌ ASSERTION FAILED: " << message << std::endl; \
        std::cerr << "   at " << __FILE__ << ":" << __LINE__ << std::endl; \
        return false; \
    }

#define TEST_EXPECT_EQ(actual, expected, message) \
    if ((actual) != (expected)) { \
        std::cerr << "❌ EXPECTATION FAILED: " << message << std::endl; \
        std::cerr << "   Expected: " << (expected) << ", Got: " << (actual) << std::endl; \
        std::cerr << "   at " << __FILE__ << ":" << __LINE__ << std::endl; \
        return false; \
    }

#define TEST_EXPECT_TRUE(condition, message) \
    if (!(condition)) { \
        std::cerr << "❌ EXPECTATION FAILED: " << message << std::endl; \
        std::cerr << "   Expected: true, Got: false" << std::endl; \
        std::cerr << "   at " << __FILE__ << ":" << __LINE__ << std::endl; \
        return false; \
    }

class SimpleTestRunner {
private:
    int tests_run = 0;
    int tests_passed = 0;
    
public:
    bool runTest(const std::string& test_name, std::function<bool()> test_func) {
        std::cout << "Running: " << test_name << "... ";
        tests_run++;
        
        bool result = test_func();
        if (result) {
            tests_passed++;
            std::cout << "✅ PASSED" << std::endl;
        } else {
            std::cout << "❌ FAILED" << std::endl;
        }
        
        return result;
    }
    
    void printResults() {
        std::cout << std::endl;
        std::cout << "===============================================" << std::endl;
        std::cout << "Test Results: " << tests_passed << "/" << tests_run << " passed" << std::endl;
        
        if (tests_passed == tests_run) {
            std::cout << "✅ ALL TESTS PASSED!" << std::endl;
            std::cout << "Message queue system is ready for deployment." << std::endl;
        } else {
            std::cout << "❌ SOME TESTS FAILED!" << std::endl;
            std::cout << "System is NOT ready for deployment." << std::endl;
        }
        std::cout << "===============================================" << std::endl;
    }
    
    bool allTestsPassed() const {
        return tests_passed == tests_run;
    }
};

// Test Functions
bool testBasicQueueOperations() {
    AICommandQueue queue;
    
    std::string test_command = R"({"command_type": "CHAT", "parameters": {"content": "test"}})";
    
    // Test enqueue
    TEST_EXPECT_TRUE(queue.tryEnqueue(test_command), "Basic enqueue should succeed");
    TEST_EXPECT_EQ(queue.pendingCount(), 1, "Queue should have 1 pending command");
    TEST_EXPECT_TRUE(!queue.isEmpty(), "Queue should not be empty");
    
    // Test dequeue
    auto result = queue.tryDequeue();
    TEST_EXPECT_TRUE(result.has_value(), "Dequeue should return a value");
    TEST_EXPECT_EQ(result.value(), test_command, "Dequeued command should match");
    TEST_EXPECT_EQ(queue.pendingCount(), 0, "Queue should be empty after dequeue");
    TEST_EXPECT_TRUE(queue.isEmpty(), "Queue should be empty");
    
    return true;
}

bool testQueueCapacity() {
    AICommandQueue queue;
    std::string test_command = R"({"command_type": "CHAT", "parameters": {"content": "test"}})";
    
    // Fill queue to capacity
    size_t capacity = queue.capacity();
    for (size_t i = 0; i < capacity - 1; ++i) {
        TEST_EXPECT_TRUE(queue.tryEnqueue(test_command), "Should be able to enqueue within capacity");
    }
    
    // Queue should be full now
    TEST_EXPECT_TRUE(queue.isFull(), "Queue should be full");
    TEST_EXPECT_TRUE(!queue.tryEnqueue(test_command), "Should not be able to enqueue when full");
    
    return true;
}

bool testInvalidJson() {
    AICommandQueue queue;
    
    // Test invalid JSON
    TEST_EXPECT_TRUE(!queue.tryEnqueue("invalid json"), "Invalid JSON should be rejected");
    TEST_EXPECT_TRUE(!queue.tryEnqueue("{incomplete"), "Incomplete JSON should be rejected");
    TEST_EXPECT_TRUE(!queue.tryEnqueue(""), "Empty string should be rejected");
    
    // Queue should remain empty
    TEST_EXPECT_TRUE(queue.isEmpty(), "Queue should remain empty after invalid commands");
    TEST_EXPECT_EQ(queue.pendingCount(), 0, "Pending count should be 0");
    
    return true;
}

bool testThreadSafety() {
    AICommandQueue queue;
    const int num_commands = 1000;
    const int num_threads = 4;
    
    std::atomic<int> commands_produced{0};
    std::atomic<int> commands_consumed{0};
    std::vector<std::string> consumed_commands;
    std::mutex consumed_mutex;
    
    // Producer function
    auto producer = [&](int thread_id) {
        for (int i = 0; i < num_commands / num_threads; ++i) {
            std::string command = R"({"command_type": "CHAT", "parameters": {"content": "msg_)" + 
                                std::to_string(thread_id) + "_" + std::to_string(i) + R"("}})";
            
            // Retry logic for full queue
            int retries = 0;
            while (retries < 10) {
                if (queue.tryEnqueue(command)) {
                    commands_produced.fetch_add(1);
                    break;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                retries++;
            }
        }
    };
    
    // Consumer function
    auto consumer = [&]() {
        while (commands_consumed.load() < num_commands || !queue.isEmpty()) {
            auto result = queue.tryDequeue();
            if (result.has_value()) {
                {
                    std::lock_guard<std::mutex> lock(consumed_mutex);
                    consumed_commands.push_back(result.value());
                }
                commands_consumed.fetch_add(1);
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        }
    };
    
    // Start threads
    std::vector<std::thread> producer_threads;
    std::vector<std::thread> consumer_threads;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Start producers
    for (int i = 0; i < num_threads / 2; ++i) {
        producer_threads.emplace_back(producer, i);
    }
    
    // Start consumers
    for (int i = 0; i < num_threads / 2; ++i) {
        consumer_threads.emplace_back(consumer);
    }
    
    // Wait for completion
    for (auto& t : producer_threads) {
        t.join();
    }
    for (auto& t : consumer_threads) {
        t.join();
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    // Verify results
    TEST_EXPECT_EQ(commands_consumed.load(), num_commands, "All commands should be consumed");
    TEST_EXPECT_EQ(consumed_commands.size(), num_commands, "Consumed command count should match");
    TEST_EXPECT_TRUE(queue.isEmpty(), "Queue should be empty at end");
    
    std::cout << std::endl << "  Thread safety test: " << num_commands << " commands in " 
              << duration.count() << "ms (" 
              << (num_commands * 1000.0 / duration.count()) << " ops/sec)" << std::endl;
    
    return true;
}

bool testMessageQueueInterface() {
    auto queue_interface = std::make_unique<MessageQueueInterface>(
        MessageQueueInterface::QueueType::IN_MEMORY, "", 1000,
        OverflowStrategy::DROP_OLDEST
    );
    
    TEST_EXPECT_TRUE(queue_interface->initialize(), "Queue interface should initialize");
    
    // Test sending commands
    TEST_EXPECT_TRUE(queue_interface->sendChat("Hello"), "Should be able to send chat command");
    TEST_EXPECT_TRUE(queue_interface->sendEmote("waves"), "Should be able to send emote command");
    TEST_EXPECT_TRUE(queue_interface->sendMove(1.0f, 2.0f, 3.0f), "Should be able to send move command");
    TEST_EXPECT_TRUE(queue_interface->sendStand(), "Should be able to send stand command");
    
    // Check queue state
    TEST_EXPECT_EQ(queue_interface->getPendingCount(), 4, "Should have 4 pending commands");
    TEST_EXPECT_TRUE(!queue_interface->isEmpty(), "Queue should not be empty");
    
    // Test dequeue
    auto result = queue_interface->getQueue()->tryDequeue();
    TEST_EXPECT_TRUE(result.has_value(), "Should be able to dequeue command");
    
    queue_interface->shutdown();
    return true;
}

bool testQueueProcessor() {
    auto queue_interface = std::make_unique<MessageQueueInterface>(
        MessageQueueInterface::QueueType::IN_MEMORY, "", 100,
        OverflowStrategy::DROP_OLDEST
    );
    TEST_EXPECT_TRUE(queue_interface->initialize(), "Queue interface should initialize");
    
    auto processor = std::make_unique<QueueProcessor>();
    TEST_EXPECT_TRUE(processor->initialize(queue_interface.get()), "Processor should initialize");
    
    bool chat_called = false;
    std::string received_content;
    
    // Register handler
    processor->registerChatHandler([&](const std::string& content) {
        chat_called = true;
        received_content = content;
    });
    
    // Send command
    TEST_EXPECT_TRUE(queue_interface->sendChat("Test message"), "Should send chat command");
    
    // Process commands
    processor->processCommands(10);
    
    // Verify handler was called
    TEST_EXPECT_TRUE(chat_called, "Chat handler should be called");
    TEST_EXPECT_EQ(received_content, "Test message", "Received content should match");
    
    // Cleanup
    processor->shutdown();
    queue_interface->shutdown();
    
    return true;
}

int main() {
    std::cout << "===============================================" << std::endl;
    std::cout << "Genesis AI Message Queue System - Simple Tests" << std::endl;
    std::cout << "===============================================" << std::endl;
    std::cout << std::endl;
    
    SimpleTestRunner runner;
    
    // Run all tests
    runner.runTest("Basic Queue Operations", testBasicQueueOperations);
    runner.runTest("Queue Capacity Limits", testQueueCapacity);
    runner.runTest("Invalid JSON Rejection", testInvalidJson);
    runner.runTest("Thread Safety", testThreadSafety);
    runner.runTest("Message Queue Interface", testMessageQueueInterface);
    runner.runTest("Queue Processor", testQueueProcessor);
    
    // Print final results
    runner.printResults();
    
    return runner.allTestsPassed() ? 0 : 1;
}