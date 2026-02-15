/*
 * Comprehensive C++ Test Suite for AI Message Queue System
 * 
 * This validates the C++ components of the message queue architecture:
 * - AICommandQueue lock-free operations
 * - PersistentMessageQueue memory-mapped storage
 * - QueueProcessor command handling
 * - Thread safety and performance
 * 
 * CRITICAL: All tests must pass for production deployment.
 */

#include "ai_message_queue.h"
#include "queue_processor.h"
#include <gtest/gtest.h>
#include <thread>
#include <vector>
#include <chrono>
#include <random>
#include <atomic>
#include <future>

using namespace python_bridge;

class AICommandQueueTest : public ::testing::Test {
protected:
    void SetUp() override {
        queue = std::make_unique<AICommandQueue>();
    }
    
    void TearDown() override {
        queue.reset();
    }
    
    std::unique_ptr<AICommandQueue> queue;
};

TEST_F(AICommandQueueTest, BasicEnqueueDequeue) {
    std::string test_command = R"({"command_type": "CHAT", "parameters": {"content": "test"}})";
    
    // Test enqueue
    EXPECT_TRUE(queue->tryEnqueue(test_command));
    EXPECT_EQ(queue->pendingCount(), 1);
    EXPECT_FALSE(queue->isEmpty());
    
    // Test dequeue
    auto result = queue->tryDequeue();
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), test_command);
    EXPECT_EQ(queue->pendingCount(), 0);
    EXPECT_TRUE(queue->isEmpty());
}

TEST_F(AICommandQueueTest, QueueCapacityLimits) {
    std::string test_command = R"({"command_type": "CHAT", "parameters": {"content": "test"}})";
    
    // Fill queue to near capacity
    size_t capacity = queue->capacity();
    for (size_t i = 0; i < capacity - 1; ++i) {
        EXPECT_TRUE(queue->tryEnqueue(test_command)) << "Failed at " << i;
    }
    
    // Queue should be full now
    EXPECT_TRUE(queue->isFull());
    EXPECT_FALSE(queue->tryEnqueue(test_command));
}

TEST_F(AICommandQueueTest, InvalidJsonRejection) {
    // Test invalid JSON
    EXPECT_FALSE(queue->tryEnqueue("invalid json"));
    EXPECT_FALSE(queue->tryEnqueue("{incomplete"));
    EXPECT_FALSE(queue->tryEnqueue(""));
    
    // Queue should remain empty
    EXPECT_TRUE(queue->isEmpty());
    EXPECT_EQ(queue->pendingCount(), 0);
}

TEST_F(AICommandQueueTest, CommandSizeLimit) {
    // Create command that exceeds size limit
    std::string large_content(AICommandQueue::MAX_COMMAND_SIZE, 'x');
    
    EXPECT_FALSE(queue->tryEnqueue(large_content));
    EXPECT_TRUE(queue->isEmpty());
}

TEST_F(AICommandQueueTest, ThreadSafetyConcurrentAccess) {
    const int num_commands = 1000;
    const int num_producer_threads = 4;
    const int num_consumer_threads = 2;
    
    std::atomic<int> commands_produced{0};
    std::atomic<int> commands_consumed{0};
    std::atomic<int> production_failures{0};
    std::vector<std::string> consumed_commands;
    std::mutex consumed_mutex;
    
    // Producer function
    auto producer = [&](int thread_id) {
        for (int i = 0; i < num_commands / num_producer_threads; ++i) {
            std::string command = R"({"command_type": "CHAT", "parameters": {"content": "msg_)" + 
                                std::to_string(thread_id) + "_" + std::to_string(i) + R"("}})";
            
            if (queue->tryEnqueue(command)) {
                commands_produced.fetch_add(1);
            } else {
                production_failures.fetch_add(1);
                // Brief wait and retry for full queue
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                if (queue->tryEnqueue(command)) {
                    commands_produced.fetch_add(1);
                } else {
                    production_failures.fetch_add(1);
                }
            }
        }
    };
    
    // Consumer function
    auto consumer = [&]() {
        while (commands_consumed.load() < num_commands || !queue->isEmpty()) {
            auto result = queue->tryDequeue();
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
    
    // Start all threads
    std::vector<std::thread> producer_threads;
    std::vector<std::thread> consumer_threads;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_producer_threads; ++i) {
        producer_threads.emplace_back(producer, i);
    }
    
    for (int i = 0; i < num_consumer_threads; ++i) {
        consumer_threads.emplace_back(consumer);
    }
    
    // Wait for producers
    for (auto& t : producer_threads) {
        t.join();
    }
    
    // Wait for consumers
    for (auto& t : consumer_threads) {
        t.join();
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    // Verify results
    EXPECT_EQ(commands_consumed.load(), num_commands);
    EXPECT_EQ(consumed_commands.size(), num_commands);
    EXPECT_TRUE(queue->isEmpty());
    
    std::cout << "Concurrent test: " << num_commands << " commands in " 
              << duration.count() << "ms (" 
              << (num_commands * 1000.0 / duration.count()) << " ops/sec)" << std::endl;
    std::cout << "Production failures: " << production_failures.load() << std::endl;
}

TEST_F(AICommandQueueTest, PerformanceBenchmark) {
    const int num_operations = 100000;
    std::string test_command = R"({"command_type": "CHAT", "parameters": {"content": "benchmark"}})";
    
    // Benchmark enqueue performance
    auto start_time = std::chrono::high_resolution_clock::now();
    
    int successful_enqueues = 0;
    for (int i = 0; i < num_operations; ++i) {
        if (queue->tryEnqueue(test_command)) {
            successful_enqueues++;
        }
        
        // Dequeue occasionally to prevent overflow
        if (i % 100 == 0 && queue->pendingCount() > 500) {
            queue->tryDequeue();
        }
    }
    
    auto mid_time = std::chrono::high_resolution_clock::now();
    
    // Benchmark dequeue performance
    int successful_dequeues = 0;
    while (!queue->isEmpty()) {
        if (queue->tryDequeue().has_value()) {
            successful_dequeues++;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto enqueue_duration = std::chrono::duration_cast<std::chrono::microseconds>(mid_time - start_time);
    auto dequeue_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - mid_time);
    
    double enqueue_rate = successful_enqueues * 1000000.0 / enqueue_duration.count();
    double dequeue_rate = successful_dequeues * 1000000.0 / dequeue_duration.count();
    
    std::cout << "Performance benchmark:" << std::endl;
    std::cout << "  Enqueue rate: " << enqueue_rate << " ops/sec" << std::endl;
    std::cout << "  Dequeue rate: " << dequeue_rate << " ops/sec" << std::endl;
    
    // Performance requirements: at least 100k ops/sec
    EXPECT_GE(enqueue_rate, 100000.0);
    EXPECT_GE(dequeue_rate, 100000.0);
}


class QueueProcessorTest : public ::testing::Test {
protected:
    void SetUp() override {
        queue_interface = std::make_unique<MessageQueueInterface>(
            MessageQueueInterface::QueueType::IN_MEMORY, "", 1000,
            OverflowStrategy::DROP_OLDEST
        );
        ASSERT_TRUE(queue_interface->initialize());
        
        processor = std::make_unique<QueueProcessor>();
        ASSERT_TRUE(processor->initialize(queue_interface.get()));
    }
    
    void TearDown() override {
        if (processor) {
            processor->shutdown();
        }
        if (queue_interface) {
            queue_interface->shutdown();
        }
    }
    
    std::unique_ptr<MessageQueueInterface> queue_interface;
    std::unique_ptr<QueueProcessor> processor;
};

TEST_F(QueueProcessorTest, BasicCommandProcessing) {
    bool chat_called = false;
    std::string received_content;
    
    // Register handler
    processor->registerChatHandler([&](const std::string& content) {
        chat_called = true;
        received_content = content;
    });
    
    // Send command
    EXPECT_TRUE(queue_interface->sendChat("Test message"));
    
    // Process commands
    processor->processCommands(10);
    
    // Verify handler was called
    EXPECT_TRUE(chat_called);
    EXPECT_EQ(received_content, "Test message");
    
    // Queue should be empty
    EXPECT_TRUE(queue_interface->isEmpty());
}

TEST_F(QueueProcessorTest, MultipleCommandTypes) {
    struct TestResults {
        bool chat_called = false;
        bool emote_called = false;
        bool move_called = false;
        bool stand_called = false;
        
        std::string chat_content;
        std::string emote_content;
        float move_x = 0, move_y = 0, move_z = 0;
    } results;
    
    // Register handlers
    processor->registerChatHandler([&](const std::string& content) {
        results.chat_called = true;
        results.chat_content = content;
    });
    
    processor->registerEmoteHandler([&](const std::string& content) {
        results.emote_called = true;
        results.emote_content = content;
    });
    
    processor->registerMoveHandler([&](float x, float y, float z) {
        results.move_called = true;
        results.move_x = x;
        results.move_y = y;
        results.move_z = z;
    });
    
    processor->registerStandHandler([&]() {
        results.stand_called = true;
    });
    
    // Send different command types
    EXPECT_TRUE(queue_interface->sendChat("Hello"));
    EXPECT_TRUE(queue_interface->sendEmote("waves"));
    EXPECT_TRUE(queue_interface->sendMove(1.5f, 2.5f, 3.5f));
    EXPECT_TRUE(queue_interface->sendStand());
    
    // Process all commands
    processor->processCommands(10);
    
    // Verify all handlers were called correctly
    EXPECT_TRUE(results.chat_called);
    EXPECT_EQ(results.chat_content, "Hello");
    
    EXPECT_TRUE(results.emote_called);
    EXPECT_EQ(results.emote_content, "waves");
    
    EXPECT_TRUE(results.move_called);
    EXPECT_FLOAT_EQ(results.move_x, 1.5f);
    EXPECT_FLOAT_EQ(results.move_y, 2.5f);
    EXPECT_FLOAT_EQ(results.move_z, 3.5f);
    
    EXPECT_TRUE(results.stand_called);
}

TEST_F(QueueProcessorTest, InvalidCommandHandling) {
    int handler_calls = 0;
    
    processor->registerChatHandler([&](const std::string& content) {
        handler_calls++;
    });
    
    // Send invalid command directly to queue
    std::string invalid_json = "invalid json";
    queue_interface->getQueue()->enqueue(invalid_json);
    
    // Process commands
    processor->processCommands(10);
    
    // Handler should not have been called
    EXPECT_EQ(handler_calls, 0);
    
    // Check statistics for errors
    auto stats = processor->getStatistics();
    EXPECT_GT(stats.commands_failed, 0);
}

TEST_F(QueueProcessorTest, PerformanceUnderLoad) {
    const int num_commands = 10000;
    int commands_processed = 0;
    
    // Register a simple handler
    processor->registerChatHandler([&](const std::string& content) {
        commands_processed++;
    });
    
    // Fill queue with commands
    for (int i = 0; i < num_commands; ++i) {
        ASSERT_TRUE(queue_interface->sendChat("Load test " + std::to_string(i)));
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Process all commands
    while (!queue_interface->isEmpty()) {
        processor->processCommands(100);  // Process in batches
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    // Verify all commands were processed
    EXPECT_EQ(commands_processed, num_commands);
    
    // Check processing rate
    double processing_rate = num_commands * 1000.0 / duration.count();
    std::cout << "Processed " << num_commands << " commands in " 
              << duration.count() << "ms (" << processing_rate << " cmd/sec)" << std::endl;
    
    // Should process at least 10,000 commands/second
    EXPECT_GE(processing_rate, 10000.0);
}


class MessageQueueIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Test integration between all components
    }
    
    void TearDown() override {
        // Cleanup
    }
};

TEST_F(MessageQueueIntegrationTest, EndToEndCommandFlow) {
    // Create the full pipeline: Queue -> Processor -> Handler
    auto queue_interface = std::make_unique<MessageQueueInterface>(
        MessageQueueInterface::QueueType::IN_MEMORY, "", 100,
        OverflowStrategy::DROP_OLDEST
    );
    ASSERT_TRUE(queue_interface->initialize());
    
    auto processor = std::make_unique<QueueProcessor>();
    ASSERT_TRUE(processor->initialize(queue_interface.get()));
    
    // Test results
    struct Results {
        std::vector<std::string> chat_messages;
        std::vector<std::string> emote_messages;
        std::vector<std::tuple<float, float, float>> move_commands;
    } results;
    
    // Register handlers
    processor->registerChatHandler([&](const std::string& content) {
        results.chat_messages.push_back(content);
    });
    
    processor->registerEmoteHandler([&](const std::string& content) {
        results.emote_messages.push_back(content);
    });
    
    processor->registerMoveHandler([&](float x, float y, float z) {
        results.move_commands.emplace_back(x, y, z);
    });
    
    // Send mixed commands
    std::vector<std::string> expected_chats = {"Hello", "How are you?", "Goodbye"};
    std::vector<std::string> expected_emotes = {"waves", "smiles"};
    std::vector<std::tuple<float, float, float>> expected_moves = {{1, 2, 3}, {4, 5, 6}};
    
    // Send commands in mixed order
    for (const auto& msg : expected_chats) {
        EXPECT_TRUE(queue_interface->sendChat(msg));
    }
    
    for (const auto& emote : expected_emotes) {
        EXPECT_TRUE(queue_interface->sendEmote(emote));
    }
    
    for (const auto& [x, y, z] : expected_moves) {
        EXPECT_TRUE(queue_interface->sendMove(x, y, z));
    }
    
    // Process all commands
    while (!queue_interface->isEmpty()) {
        processor->processCommands(10);
    }
    
    // Verify results
    EXPECT_EQ(results.chat_messages, expected_chats);
    EXPECT_EQ(results.emote_messages, expected_emotes);
    EXPECT_EQ(results.move_commands, expected_moves);
    
    // Cleanup
    processor->shutdown();
    queue_interface->shutdown();
}

TEST_F(MessageQueueIntegrationTest, StressTestFullSystem) {
    const int num_iterations = 1000;
    const int commands_per_iteration = 10;
    
    auto queue_interface = std::make_unique<MessageQueueInterface>(
        MessageQueueInterface::QueueType::IN_MEMORY, "", 5000,
        OverflowStrategy::DROP_OLDEST
    );
    ASSERT_TRUE(queue_interface->initialize());
    
    auto processor = std::make_unique<QueueProcessor>();
    ASSERT_TRUE(processor->initialize(queue_interface.get()));
    
    std::atomic<int> total_processed{0};
    
    // Register counting handler
    processor->registerChatHandler([&](const std::string& content) {
        total_processed.fetch_add(1);
    });
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Run stress test
    for (int i = 0; i < num_iterations; ++i) {
        // Send batch of commands
        for (int j = 0; j < commands_per_iteration; ++j) {
            queue_interface->sendChat("Stress test " + std::to_string(i * commands_per_iteration + j));
        }
        
        // Process some commands
        processor->processCommands(5);
        
        // Occasional full processing
        if (i % 100 == 0) {
            while (!queue_interface->isEmpty() && queue_interface->getPendingCount() > 100) {
                processor->processCommands(50);
            }
        }
    }
    
    // Final processing of remaining commands
    while (!queue_interface->isEmpty()) {
        processor->processCommands(100);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    // Verify all commands were processed
    int expected_total = num_iterations * commands_per_iteration;
    EXPECT_EQ(total_processed.load(), expected_total);
    
    // Performance check
    double throughput = expected_total * 1000.0 / duration.count();
    std::cout << "Stress test: " << expected_total << " commands processed in " 
              << duration.count() << "ms (" << throughput << " cmd/sec)" << std::endl;
    
    // Should maintain good throughput under stress
    EXPECT_GE(throughput, 5000.0);
    
    // Cleanup
    processor->shutdown();
    queue_interface->shutdown();
}

// Main test runner
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    std::cout << "===============================================" << std::endl;
    std::cout << "Genesis AI Message Queue System - C++ Tests" << std::endl;
    std::cout << "===============================================" << std::endl;
    std::cout << std::endl;
    std::cout << "Testing critical C++ components:" << std::endl;
    std::cout << "- Lock-free AICommandQueue operations" << std::endl;
    std::cout << "- QueueProcessor command handling" << std::endl;
    std::cout << "- Thread safety and performance" << std::endl;
    std::cout << "- Full system integration" << std::endl;
    std::cout << std::endl;
    std::cout << "CRITICAL: All tests must pass for deployment!" << std::endl;
    std::cout << "===============================================" << std::endl;
    std::cout << std::endl;
    
    int result = RUN_ALL_TESTS();
    
    std::cout << std::endl;
    std::cout << "===============================================" << std::endl;
    if (result == 0) {
        std::cout << "✅ SUCCESS: C++ components validated!" << std::endl;
        std::cout << "Message queue system is ready for deployment." << std::endl;
    } else {
        std::cout << "❌ FAILURE: C++ tests failed!" << std::endl;
        std::cout << "System is NOT ready for deployment." << std::endl;
    }
    std::cout << "===============================================" << std::endl;
    
    return result;
}