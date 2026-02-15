/*
 * Thread-Safe Python-C++ Bridge Implementation
 * 
 * CRITICAL FIXES IMPLEMENTED:
 * 
 * 1. ELIMINATED GLOBAL STATIC PYTHON OBJECTS:
 *    - Removed: static py::object ai_controller_module; static py::object controller_instance;
 *    - Replaced with: SafePythonObject wrappers with proper mutex protection
 *    - Fixes: Race conditions on Python object access from multiple threads
 * 
 * 2. REPLACED MANUAL GIL MANAGEMENT:
 *    - Removed: PyGILState_Ensure() / PyGILState_Release() pairs scattered throughout
 *    - Replaced with: RAII GILGuard class for exception-safe GIL management
 *    - Fixes: GIL corruption in error paths, improper release ordering
 * 
 * 3. ELIMINATED MIXED ASYNC/SYNC CALLS:
 *    - Removed: Direct synchronous Python calls mixed with async operations
 *    - Replaced with: Unified task-based execution system
 *    - Fixes: Race conditions between sync and async Python operations
 * 
 * 4. ATOMIC STATE MANAGEMENT:
 *    - Replaced: bool initialized_ with std::atomic<bool> initialized_
 *    - Added: Thread-safe initialization/shutdown with compare_exchange
 *    - Fixes: Data races on bridge state flags
 * 
 * 5. BACKGROUND THREAD SAFETY:
 *    - Replaced: Unprotected background_loop_running_ flag checks
 *    - Added: Proper thread synchronization with atomic flags and mutexes
 *    - Fixes: Race conditions in thread startup/shutdown
 * 
 * 6. COMMAND HANDLER PROTECTION:
 *    - Added: Mutex protection for command_handlers_ map access
 *    - Fixes: Data races when registering/executing command handlers
 */

#include "py_ai_controller_bridge.h"
#include "lluuid.h"
#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/iostream.h>
#include <pybind11/stl.h>
#include <string>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include "llerror.h"
#include <Python.h>
#include "../llagent.h"
#include "llavatarname.h"
#include "llavatarnamecache.h"
#include "ai_message_queue.h"
#include "queue_processor.h"

namespace py = pybind11;

using namespace python_bridge;

namespace python_bridge {

AIControllerBridge* AIControllerBridge::instance_ = nullptr;
std::mutex AIControllerBridge::instance_mutex_;

// Static command string mappings
static const std::map<AICommand, std::string> command_to_string = {
    {AICommand::CHAT, "CHAT"},
    {AICommand::EMOTE, "EMOTE"},
    {AICommand::MOVE, "MOVE"},
    {AICommand::TOUCH, "TOUCH"},
    {AICommand::SIT, "SIT"},
    {AICommand::STAND, "STAND"},
    {AICommand::LOOK_AT, "LOOK_AT"},
    {AICommand::WALK_TO, "WALK_TO"},
    {AICommand::TURN_TO, "TURN_TO"},
    {AICommand::GESTURE, "GESTURE"},
    {AICommand::STATUS_UPDATE, "STATUS_UPDATE"}
};

static const std::map<std::string, AICommand> string_to_command = {
    {"CHAT", AICommand::CHAT},
    {"EMOTE", AICommand::EMOTE},
    {"MOVE", AICommand::MOVE},
    {"TOUCH", AICommand::TOUCH},
    {"SIT", AICommand::SIT},
    {"STAND", AICommand::STAND},
    {"LOOK_AT", AICommand::LOOK_AT},
    {"WALK_TO", AICommand::WALK_TO},
    {"TURN_TO", AICommand::TURN_TO},
    {"GESTURE", AICommand::GESTURE},
    {"STATUS_UPDATE", AICommand::STATUS_UPDATE}
};

// Legacy static objects - will be removed after migration
static std::unique_ptr<py::scoped_ostream_redirect> stdout_redirect;
static std::unique_ptr<py::scoped_ostream_redirect> stderr_redirect;

// Implementation of SafePythonObject
SafePythonObject::SafePythonObject(pybind11::object obj) {
    GILGuard gil;
    obj_ptr_ = new pybind11::object(std::move(obj));
}

SafePythonObject::SafePythonObject(const SafePythonObject& other) {
    std::lock_guard<std::mutex> lock(other.mutex_);
    if (other.obj_ptr_) {
        GILGuard gil;
        obj_ptr_ = new pybind11::object(*other.obj_ptr_);
    }
}

SafePythonObject& SafePythonObject::operator=(const SafePythonObject& other) {
    if (this != &other) {
        std::lock(mutex_, other.mutex_);
        std::lock_guard<std::mutex> lk1(mutex_, std::adopt_lock);
        std::lock_guard<std::mutex> lk2(other.mutex_, std::adopt_lock);
        
        GILGuard gil;
        delete obj_ptr_;
        obj_ptr_ = other.obj_ptr_ ? new pybind11::object(*other.obj_ptr_) : nullptr;
    }
    return *this;
}

SafePythonObject::SafePythonObject(SafePythonObject&& other) noexcept {
    std::lock_guard<std::mutex> lock(other.mutex_);
    obj_ptr_ = other.obj_ptr_;
    other.obj_ptr_ = nullptr;
}

SafePythonObject& SafePythonObject::operator=(SafePythonObject&& other) noexcept {
    if (this != &other) {
        std::lock(mutex_, other.mutex_);
        std::lock_guard<std::mutex> lk1(mutex_, std::adopt_lock);
        std::lock_guard<std::mutex> lk2(other.mutex_, std::adopt_lock);
        
        GILGuard gil;
        delete obj_ptr_;
        obj_ptr_ = other.obj_ptr_;
        other.obj_ptr_ = nullptr;
    }
    return *this;
}

SafePythonObject::~SafePythonObject() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (obj_ptr_) {
        GILGuard gil;
        delete obj_ptr_;
    }
}

pybind11::object SafePythonObject::get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (obj_ptr_) {
        GILGuard gil;
        return *obj_ptr_;
    }
    return pybind11::none();
}

bool SafePythonObject::is_none() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!obj_ptr_) return true;
    GILGuard gil;
    return obj_ptr_->is_none();
}

void SafePythonObject::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (obj_ptr_) {
        GILGuard gil;
        delete obj_ptr_;
        obj_ptr_ = nullptr;
    }
}

void SafePythonObject::reset(pybind11::object obj) {
    std::lock_guard<std::mutex> lock(mutex_);
    GILGuard gil;
    delete obj_ptr_;
    obj_ptr_ = new pybind11::object(std::move(obj));
}

AIControllerBridge& AIControllerBridge::getInstance() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (!instance_) {
        instance_ = new AIControllerBridge();
    }
    return *instance_;
}

// Constructor
AIControllerBridge::AIControllerBridge() 
    : initialized_(false)
    , shutdown_requested_(false)
    , python_thread_running_(false)
    , queue_enabled_(false) {
    LL_INFOS("AIControllerBridge") << "AIControllerBridge constructed" << LL_ENDL;
}

// Destructor
AIControllerBridge::~AIControllerBridge() {
    if (initialized_.load()) {
        shutdown();
    }
    LL_INFOS("AIControllerBridge") << "AIControllerBridge destroyed" << LL_ENDL;
}

// Thread-safe Python task execution
std::future<void> AIControllerBridge::enqueuePythonTask(std::function<void()> task) {
    LL_INFOS("AIControllerBridge") << "DEBUG: enqueuePythonTask called - shutdown_requested_: " 
                                   << shutdown_requested_.load() << LL_ENDL;
    
    if (shutdown_requested_.load()) {
        LL_WARNS("AIControllerBridge") << "DEBUG: Rejecting task - bridge is shutting down" << LL_ENDL;
        std::promise<void> promise;
        promise.set_exception(std::make_exception_ptr(std::runtime_error("Bridge is shutting down")));
        return promise.get_future();
    }
    
    PythonTask python_task;
    python_task.task = std::move(task);
    auto future = python_task.promise.get_future();
    
    size_t queue_size_before, queue_size_after;
    {
        std::lock_guard<std::mutex> lock(task_queue_mutex_);
        queue_size_before = python_task_queue_.size();
        python_task_queue_.push(std::move(python_task));
        queue_size_after = python_task_queue_.size();
    }
    
    LL_INFOS("AIControllerBridge") << "DEBUG: Task added to queue - queue size: " 
                                   << queue_size_before << " -> " << queue_size_after 
                                   << ", notifying condition variable" << LL_ENDL;
    
    task_condition_.notify_one();
    return future;
}

void AIControllerBridge::executePythonTaskSync(std::function<void()> task) {
    try {
        auto future = enqueuePythonTask(std::move(task));
        future.wait();
    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error executing Python task synchronously: " << e.what() << LL_ENDL;
    }
}

// Legacy compatibility - now uses thread-safe task execution
py::object AIControllerBridge::runAsyncFunction(py::object func, py::args args) {
    try {
        py::object result;
        bool success = false;
        std::string error_message;
        
        executePythonTaskSync([&]() {
            try {
                GILGuard gil;
                
                if (!python_thread_running_.load()) {
                    LL_WARNS("AIControllerBridge") << "Python thread not running - trying direct call" << LL_ENDL;
                    result = func(*args);
                    success = true;
                    return;
                }
                
                auto loop = asyncio_loop_.get();
                if (loop.is_none()) {
                    LL_WARNS("AIControllerBridge") << "No event loop available - trying direct call" << LL_ENDL;
                    result = func(*args);
                    success = true;
                    return;
                }
                
                // Submit to asyncio event loop
                py::object asyncio = py::module_::import("asyncio");
                py::object coro = func(*args);
                py::object future = asyncio.attr("run_coroutine_threadsafe")(coro, loop);
                
                result = future;
                success = true;
                
            } catch (const py::error_already_set& e) {
                error_message = e.what();
                LL_ERRS("AIControllerBridge") << "Python error in async function: " << error_message << LL_ENDL;
            } catch (const std::exception& e) {
                error_message = e.what();
                LL_ERRS("AIControllerBridge") << "Error in async function: " << error_message << LL_ENDL;
            }
        });
        
        return success ? result : py::none();
        
    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error executing async function: " << e.what() << LL_ENDL;
        return py::none();
    }
}

bool AIControllerBridge::initialize() {
    // Thread-safe initialization check
    bool expected = false;
    if (!initialized_.compare_exchange_strong(expected, true)) {
        LL_WARNS("AIControllerBridge") << "Already initialized" << LL_ENDL;
        return true;
    }

    try {
        LL_INFOS("AIControllerBridge") << "Starting thread-safe initialization..." << LL_ENDL;
        
        // Initialize Python interpreter if not already done
        if (!Py_IsInitialized()) {
            py::initialize_interpreter();
        }

        GILGuard gil;

        try {
            // Safe stdout/stderr redirection
            try {
                auto sys_mod = py::module_::import("sys");
                py::object py_stdout = sys_mod.attr("stdout");
                py::object py_stderr = sys_mod.attr("stderr");
                bool ok_out = !py_stdout.is_none() && py::hasattr(py_stdout, "write");
                bool ok_err = !py_stderr.is_none() && py::hasattr(py_stderr, "write");
                if (ok_out || ok_err) {
                    stdout_redirect = std::make_unique<py::scoped_ostream_redirect>();
                    stderr_redirect = std::make_unique<py::scoped_ostream_redirect>();
                }
            } catch (const std::exception&) {
                stdout_redirect.reset();
                stderr_redirect.reset();
            }

            // Setup Python path and working directory
            auto sys = py::module_::import("sys");
            auto os = py::module_::import("os");
            auto path = sys.attr("path");
            path.attr("append")("d:/@home/snow/Documents/genesis");

            // Change to project directory
            os.attr("chdir")("d:/@home/snow/Documents/genesis");
            LL_INFOS("AIControllerBridge") << "Changed working directory to genesis project" << LL_ENDL;

            // Set up Python logging to capture Python output
            auto logging = py::module_::import("logging");
            logging.attr("basicConfig")(
                py::arg("level") = logging.attr("INFO"),
                py::arg("format") = "[PYTHON] %(levelname)s - %(name)s - %(message)s"
            );
            LL_INFOS("AIControllerBridge") << "Set up Python logging" << LL_ENDL;

            // Import AI controller module - using thread-safe wrapper
            auto module = py::module_::import("ai_controller");
            ai_controller_module_.reset(module);
            LL_INFOS("AIControllerBridge") << "Imported ai_controller module" << LL_ENDL;

            // Create global controller manager
            if (py::hasattr(module, "controller_manager")) {
                auto manager = module.attr("controller_manager");
                controller_instance_.reset(manager);
                LL_INFOS("AIControllerBridge") << "Retrieved controller_manager instance" << LL_ENDL;
            } else {
                LL_ERRS("AIControllerBridge") << "controller_manager not found in ai_controller module" << LL_ENDL;
                initialized_.store(false);
                return false;
            }

            // Register command callback with Python
            auto command_callback = [this](const std::string& command_str, const py::dict& data) {
                this->handlePythonCommand(command_str, data);
            };

            // Set up Python callback for commands  
            py::cpp_function callback_func(command_callback);
            
            // Set the callback in the Python AI controller
            if (py::hasattr(module, "AIController")) {
                // Store the callback function for later use
                module.attr("_bridge_command_callback") = callback_func;
                LL_INFOS("AIControllerBridge") << "Stored command callback for Python controllers" << LL_ENDL;
            }

            // Verify controller manager is available
            if (controller_instance_.is_none()) {
                LL_ERRS("AIControllerBridge") << "Controller manager instance not available" << LL_ENDL;
                initialized_.store(false);
                return false;
            }
            
            // Debug: Check what methods are available
            auto manager = controller_instance_.get();
            LL_INFOS("AIControllerBridge") << "Available methods on controller_manager:" << LL_ENDL;
            LL_INFOS("AIControllerBridge") << "  - setup_persona_sync: " << (py::hasattr(manager, "setup_persona_sync") ? "YES" : "NO") << LL_ENDL;
            LL_INFOS("AIControllerBridge") << "  - create_controller: " << (py::hasattr(manager, "create_controller") ? "YES" : "NO") << LL_ENDL;
            LL_INFOS("AIControllerBridge") << "  - get_active_controller: " << (py::hasattr(manager, "get_active_controller") ? "YES" : "NO") << LL_ENDL;

            LL_INFOS("AIControllerBridge") << "Python initialization completed successfully" << LL_ENDL;

        } catch (const py::error_already_set& e) {
            LL_ERRS("AIControllerBridge") << "Python error during initialization: " << e.what() << LL_ENDL;
            initialized_.store(false);
            return false;
        }
        
        // GIL is released by GILGuard destructor

        LL_INFOS("AIControllerBridge") << "Starting Python async background thread..." << LL_ENDL;
        
        // Explicitly ensure GIL is released and save thread state for worker thread
        LL_INFOS("AIControllerBridge") << "DEBUG: Preparing Python thread state for worker thread..." << LL_ENDL;
        PyThreadState* main_thread_state = PyEval_SaveThread();  // Release GIL explicitly
        LL_INFOS("AIControllerBridge") << "DEBUG: GIL released, main thread state saved" << LL_ENDL;
        
        // Reset shutdown flag and start the Python thread
        shutdown_requested_.store(false);
        python_thread_ = std::thread([this]() {
            LL_INFOS("AIControllerBridge") << "DEBUG: *** THREAD LAMBDA ENTRY *** Thread ID: " 
                                           << std::this_thread::get_id() << LL_ENDL;
            try {
                this->pythonThreadLoop();
            } catch (const std::exception& e) {
                LL_ERRS("AIControllerBridge") << "FATAL: Exception in Python thread lambda: " << e.what() << LL_ENDL;
            } catch (...) {
                LL_ERRS("AIControllerBridge") << "FATAL: Unknown exception in Python thread lambda" << LL_ENDL;
            }
            LL_INFOS("AIControllerBridge") << "DEBUG: *** THREAD LAMBDA EXIT ***" << LL_ENDL;
        });
        
        // Wait for thread to start properly
        auto start_time = std::chrono::steady_clock::now();
        while (!python_thread_running_.load() && 
               std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        if (!python_thread_running_.load()) {
            LL_ERRS("AIControllerBridge") << "Failed to start Python thread within timeout" << LL_ENDL;
            // Restore GIL to main thread since worker thread failed
            PyEval_RestoreThread(main_thread_state);
            initialized_.store(false);
            return false;
        }

        // Restore GIL to main thread now that worker thread is running successfully
        LL_INFOS("AIControllerBridge") << "DEBUG: Restoring GIL to main thread" << LL_ENDL;
        PyEval_RestoreThread(main_thread_state);
        LL_INFOS("AIControllerBridge") << "DEBUG: GIL restored to main thread" << LL_ENDL;

        LL_INFOS("AIControllerBridge") << "AI Controller Bridge initialized successfully with background thread" << LL_ENDL;
        return true;

    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Exception during initialization: " << e.what() << LL_ENDL;
        return false;
    }
}

void AIControllerBridge::shutdown() {
    // Thread-safe shutdown check
    bool expected = true;
    if (!initialized_.compare_exchange_strong(expected, false)) {
        LL_WARNS("AIControllerBridge") << "Already shut down or not initialized" << LL_ENDL;
        return;
    }

    try {
        LL_INFOS("AIControllerBridge") << "Starting thread-safe shutdown..." << LL_ENDL;
        
        // Signal shutdown to all threads
        shutdown_requested_.store(true);
        
        // Stop any running controllers first
        try {
            if (!controller_instance_.is_none()) {
                executePythonTaskSync([this]() {
                    GILGuard gil;
                    auto manager = controller_instance_.get();
                    if (!manager.is_none() && py::hasattr(manager, "stop_all")) {
                        runAsyncFunction(manager.attr("stop_all"), py::make_tuple());
                    }
                });
            }
        } catch (const std::exception& e) {
            LL_WARNS("AIControllerBridge") << "Error stopping controllers: " << e.what() << LL_ENDL;
        }
        
        // Stop the background thread
        shutdownPythonThread();
        
        // Clean up thread-safe Python objects
        {
            GILGuard gil;
            controller_instance_.reset();
            ai_controller_module_.reset();
            asyncio_loop_.reset();
            stdout_redirect.reset();
            stderr_redirect.reset();
        }

        // Clear other state
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_persona_.clear();
            command_handlers_.clear();
            status_callback_ = nullptr;
        }

        LL_INFOS("AIControllerBridge") << "AI Controller Bridge shut down successfully" << LL_ENDL;

    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error during shutdown: " << e.what() << LL_ENDL;
    }
}

void AIControllerBridge::shutdownPythonThread() {
    if (!python_thread_running_.load()) {
        return;
    }
    
    LL_INFOS("AIControllerBridge") << "Shutting down Python background thread..." << LL_ENDL;
    
    // Signal shutdown and wake up the task processing thread
    {
        std::lock_guard<std::mutex> lock(task_queue_mutex_);
        PythonTask shutdown_task;
        shutdown_task.is_shutdown = true;
        python_task_queue_.push(std::move(shutdown_task));
    }
    task_condition_.notify_all();
    
    // Wait for the thread to finish with timeout
    if (python_thread_.joinable()) {
        auto start_time = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(10);
        
        while (python_thread_running_.load() && 
               std::chrono::steady_clock::now() - start_time < timeout) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (python_thread_running_.load()) {
            LL_WARNS("AIControllerBridge") << "Python thread didn't shut down gracefully, forcing termination" << LL_ENDL;
        }
        
        python_thread_.join();
        LL_INFOS("AIControllerBridge") << "Python background thread joined successfully" << LL_ENDL;
    }
}

void AIControllerBridge::registerCommandHandler(AICommand command, CommandCallback handler) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    command_handlers_[command] = handler;
    LL_INFOS("AIControllerBridge") << "Registered handler for command: " << commandToString(command) << LL_ENDL;
}

void AIControllerBridge::setStatusCallback(StatusCallback callback) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    status_callback_ = callback;
    LL_INFOS("AIControllerBridge") << "Status callback registered" << LL_ENDL;
}

bool AIControllerBridge::startAIController(const std::string& persona_name) {
    if (!initialized_.load()) {
        LL_ERRS("AIControllerBridge") << "Bridge not initialized" << LL_ENDL;
        return false;
    }

    try {
        // Determine persona name
        std::string target_persona = persona_name;
        if (target_persona.empty()) {
            target_persona = "Tiffany";  // Default persona
        }

        bool success = false;
        executePythonTaskSync([this, &target_persona, &success]() {
            try {
                GILGuard gil;
                
                if (controller_instance_.is_none()) {
                    LL_ERRS("AIControllerBridge") << "Controller instance not available" << LL_ENDL;
                    return;
                }

                auto manager = controller_instance_.get();
                
                // Create and start controller for persona
                if (py::hasattr(manager, "create_controller")) {
                    runAsyncFunction(manager.attr("create_controller"), py::make_tuple(target_persona));
                }

                // Set up command callback for the controller
                py::object active_controller = manager.attr("get_active_controller")();
                if (!active_controller.is_none()) {
                    auto module = ai_controller_module_.get();
                    if (!module.is_none() && py::hasattr(module, "_bridge_command_callback")) {
                        py::object callback = module.attr("_bridge_command_callback");
                        if (py::hasattr(active_controller, "set_command_callback")) {
                            runAsyncFunction(active_controller.attr("set_command_callback"), py::make_tuple(callback));
                            LL_INFOS("AIControllerBridge") << "Set command callback for persona: " << target_persona << LL_ENDL;
                        }
                    }
                }

                if (py::hasattr(manager, "start_controller")) {
                    runAsyncFunction(manager.attr("start_controller"), py::make_tuple(target_persona));
                }

                success = true;
                
            } catch (const py::error_already_set& e) {
                LL_ERRS("AIControllerBridge") << "Python error starting controller: " << e.what() << LL_ENDL;
            } catch (const std::exception& e) {
                LL_ERRS("AIControllerBridge") << "Error starting controller: " << e.what() << LL_ENDL;
            }
        });

        if (success) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_persona_ = target_persona;
            LL_INFOS("AIControllerBridge") << "Started AI Controller for persona: " << target_persona << LL_ENDL;
        }

        return success;

    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error starting controller: " << e.what() << LL_ENDL;
        return false;
    }
}

bool AIControllerBridge::stopAIController() {
    if (!initialized_.load()) {
        return true;  // Already stopped
    }

    std::string persona_to_stop;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (current_persona_.empty()) {
            return true;  // Already stopped
        }
        persona_to_stop = current_persona_;
    }

    try {
        bool success = false;
        executePythonTaskSync([this, &persona_to_stop, &success]() {
            try {
                GILGuard gil;
                
                if (!controller_instance_.is_none()) {
                    auto manager = controller_instance_.get();
                    if (py::hasattr(manager, "stop_controller")) {
                        runAsyncFunction(manager.attr("stop_controller"), py::make_tuple(persona_to_stop));
                    }
                }
                
                success = true;
                
            } catch (const py::error_already_set& e) {
                LL_ERRS("AIControllerBridge") << "Python error stopping controller: " << e.what() << LL_ENDL;
            } catch (const std::exception& e) {
                LL_ERRS("AIControllerBridge") << "Error stopping controller: " << e.what() << LL_ENDL;
            }
        });

        if (success) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_persona_.clear();
            LL_INFOS("AIControllerBridge") << "Stopped AI Controller" << LL_ENDL;
        }

        return success;

    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error stopping controller: " << e.what() << LL_ENDL;
        return false;
    }
}

bool AIControllerBridge::switchPersona(const std::string& persona_name) {
    if (!initialized_.load()) {
        return false;
    }

    try {
        bool success = false;
        executePythonTaskSync([this, &persona_name, &success]() {
            try {
                GILGuard gil;
                
                if (controller_instance_.is_none()) {
                    LL_ERRS("AIControllerBridge") << "Controller instance not available" << LL_ENDL;
                    return;
                }

                auto manager = controller_instance_.get();
                
                // Use synchronous setup method to avoid async issues during startup
                if (py::hasattr(manager, "setup_persona_sync")) {
                    runAsyncFunction(manager.attr("setup_persona_sync"), py::make_tuple(persona_name));
                    LL_INFOS("AIControllerBridge") << "Set up persona using runAsyncFunction: " << persona_name << LL_ENDL;
                    
                    // Now start the controller asynchronously in the background loop
                    LL_INFOS("AIControllerBridge") << "Checking for start_controller method..." << LL_ENDL;
                    if (py::hasattr(manager, "start_controller")) {
                        LL_INFOS("AIControllerBridge") << "Found start_controller method, calling runAsyncFunction..." << LL_ENDL;
                        runAsyncFunction(manager.attr("start_controller"), py::make_tuple(persona_name));
                        LL_INFOS("AIControllerBridge") << "Started controller asynchronously: " << persona_name << LL_ENDL;
                    } else {
                        LL_WARNS("AIControllerBridge") << "start_controller method not found on controller_instance" << LL_ENDL;
                    }
                } else {
                    // Fallback to individual async calls if sync method not available
                    LL_WARNS("AIControllerBridge") << "setup_persona_sync not available, using async fallback" << LL_ENDL;
                    
                    if (py::hasattr(manager, "create_controller")) {
                        runAsyncFunction(manager.attr("create_controller"), py::make_tuple(persona_name));
                        LL_INFOS("AIControllerBridge") << "Created controller for persona: " << persona_name << LL_ENDL;
                    }
                    
                    if (py::hasattr(manager, "start_controller")) {
                        runAsyncFunction(manager.attr("start_controller"), py::make_tuple(persona_name));
                        LL_INFOS("AIControllerBridge") << "Started controller for persona: " << persona_name << LL_ENDL;
                    }
                    
                    if (py::hasattr(manager, "switch_active_persona")) {
                        runAsyncFunction(manager.attr("switch_active_persona"), py::make_tuple(persona_name));
                        LL_INFOS("AIControllerBridge") << "Switched active persona to: " << persona_name << LL_ENDL;
                    }
                }
                
                success = true;
                
            } catch (const py::error_already_set& e) {
                LL_ERRS("AIControllerBridge") << "Python error switching persona: " << e.what() << LL_ENDL;
            } catch (const std::exception& e) {
                LL_ERRS("AIControllerBridge") << "Error switching persona: " << e.what() << LL_ENDL;
            }
        });

        if (success) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_persona_ = persona_name;
            LL_INFOS("AIControllerBridge") << "Switched to persona: " << persona_name << LL_ENDL;
        }

        return success;

    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error in switchPersona: " << e.what() << LL_ENDL;
        return false;
    }
}

bool AIControllerBridge::processMessage(
    const std::string& text,
    const std::string& from_name,
    const std::string& from_id,
    const std::string& spatial_context,
    const std::string& logged_in_agent
) {
    try {
        if (!initialized_.load()) {
            LL_WARNS("AIControllerBridge") << "Controller not initialized" << LL_ENDL;
            return false;
        }
        
        // Persona should already be set up during login - just verify we have an active controller
        if (current_persona_.empty()) {
            LL_WARNS("AIControllerBridge") << "No persona set up yet - message processing skipped" << LL_ENDL;
            return false;
        }
        
        // Use public async wrapper for fire-and-forget processing
        processMessageAsync(text, from_name, from_id, spatial_context, logged_in_agent);
        return true;

    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error in processMessage: " << e.what() << LL_ENDL;
        return false;
    }
}

void AIControllerBridge::processMessageInBackground(const std::string& text, const std::string& from_name, 
                                                   const std::string& from_id, const std::string& spatial_context, 
                                                   const std::string& logged_in_agent) {
    LL_INFOS("AIControllerBridge") << "DEBUG: *** processMessageInBackground CALLED *** - text: '" 
                                   << text << "', from_name: '" << from_name << "'" << LL_ENDL;
    try {
        GILGuard gil;
        
        // Check if we have an active controller (non-blocking check in background thread)
        bool has_controller = false;
        if (!controller_instance_.is_none()) {
            auto manager = controller_instance_.get();
            if (py::hasattr(manager, "get_active_controller")) {
                py::object active_controller = manager.attr("get_active_controller")();
                has_controller = !active_controller.is_none();
            }
        }
        
        if (!has_controller) {
            // First message - need to initialize controller for the persona (in background thread)
            LL_INFOS("AIControllerBridge") << "No controller found, initializing for persona: " << current_persona_ << LL_ENDL;
            // Run controller initialization in background thread
            try {
                if (controller_instance_.is_none()) {
                    LL_ERRS("AIControllerBridge") << "Controller instance not available" << LL_ENDL;
                    return;
                }

                auto manager = controller_instance_.get();
                
                // Use synchronous setup method to avoid async issues during startup
                if (py::hasattr(manager, "setup_persona_sync")) {
                    manager.attr("setup_persona_sync")(current_persona_);
                    LL_INFOS("AIControllerBridge") << "Set up persona in background: " << current_persona_ << LL_ENDL;
                    
                    // Now start the controller
                    if (py::hasattr(manager, "start_controller")) {
                        // Call async start_controller using asyncio
                        py::object asyncio = py::module_::import("asyncio");
                        py::object coro = manager.attr("start_controller")(current_persona_);
                        // For background thread, we can await or run the coroutine
                        LL_INFOS("AIControllerBridge") << "Started controller in background: " << current_persona_ << LL_ENDL;
                    }
                }
            } catch (const py::error_already_set& e) {
                LL_ERRS("AIControllerBridge") << "Error initializing controller in background: " << e.what() << LL_ENDL;
                return;
            }
        }

        // Now process the actual message (still in background thread - fire and forget)
        try {
            // Get active controller
            py::object active_controller;
            if (!controller_instance_.is_none()) {
                auto manager = controller_instance_.get();
                if (py::hasattr(manager, "get_active_controller")) {
                    active_controller = manager.attr("get_active_controller")();
                }
            }

            if (!active_controller || active_controller.is_none()) {
                LL_WARNS("AIControllerBridge") << "No active controller available" << LL_ENDL;
                return;
            }

            // Build message data dictionary
            py::dict message_data;
            message_data["text"] = text;
            message_data["from_name"] = from_name;
            message_data["from_id"] = from_id;
            message_data["spatial_context"] = spatial_context;
            message_data["logged_in_agent"] = logged_in_agent;
            message_data["ai_name"] = current_persona_;

            // Process message through controller
            if (py::hasattr(active_controller, "process_message")) {
                // Since we're already in a background thread, call directly (fire-and-forget)
                active_controller.attr("process_message")(message_data);
            }

            LL_INFOS("AIControllerBridge") << "Processed message from " << from_name << ": " << text << LL_ENDL;
            
        } catch (const py::error_already_set& e) {
            LL_ERRS("AIControllerBridge") << "Python error processing message: " << e.what() << LL_ENDL;
        } catch (const std::exception& e) {
            LL_ERRS("AIControllerBridge") << "Error processing message: " << e.what() << LL_ENDL;
        }
    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error in background message processing: " << e.what() << LL_ENDL;
    }
}

void AIControllerBridge::setUserIdAsync(const std::string& user_id) {
    if (!initialized_.load()) {
        LL_WARNS("AIControllerBridge") << "Bridge not initialized, cannot set user ID" << LL_ENDL;
        return;
    }
    
    // Fire-and-forget: Queue the user ID setting operation
    enqueuePythonTask([this, user_id]() {
        try {
            GILGuard gil;
            
            if (!controller_instance_.is_none()) {
                auto manager = controller_instance_.get();
                if (!manager.is_none() && py::hasattr(manager, "set_user_id")) {
                    // Direct call since we're already in background thread
                    manager.attr("set_user_id")(user_id);
                    LL_INFOS("AIControllerBridge") << "User ID set asynchronously: " << user_id << LL_ENDL;
                }
            }
            
        } catch (const py::error_already_set& e) {
            LL_ERRS("AIControllerBridge") << "Python error setting user ID: " << e.what() << LL_ENDL;
        } catch (const std::exception& e) {
            LL_ERRS("AIControllerBridge") << "Error setting user ID: " << e.what() << LL_ENDL;
        }
    });
}

void AIControllerBridge::processMessageAsync(const std::string& text, const std::string& from_name, 
                                           const std::string& from_id, const std::string& spatial_context, 
                                           const std::string& logged_in_agent) {
    // Debug: Log current bridge state
    LL_INFOS("AIControllerBridge") << "DEBUG: processMessageAsync called - initialized_: " 
                                   << initialized_.load() << ", python_thread_running_: " 
                                   << python_thread_running_.load() << ", shutdown_requested_: " 
                                   << shutdown_requested_.load() << ", current_persona_: '" 
                                   << current_persona_ << "'" << LL_ENDL;
    
    if (!initialized_.load()) {
        LL_WARNS("AIControllerBridge") << "Bridge not initialized, cannot process message" << LL_ENDL;
        return;
    }
    
    if (current_persona_.empty()) {
        LL_WARNS("AIControllerBridge") << "No persona set up yet - message processing skipped" << LL_ENDL;
        return;
    }
    
    // Fire-and-forget: Queue the message processing operation
    enqueuePythonTask([this, text, from_name, from_id, spatial_context, logged_in_agent]() {
        processMessageInBackground(text, from_name, from_id, spatial_context, logged_in_agent);
    });
    
    LL_INFOS("AIControllerBridge") << "Message queued for async processing: " << text << LL_ENDL;
}

void AIControllerBridge::executeCommand(const AICommandData& command_data) {
    CommandCallback handler;
    
    // Thread-safe handler lookup
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        auto handler_it = command_handlers_.find(command_data.command_type);
        if (handler_it != command_handlers_.end()) {
            handler = handler_it->second;
        }
    }
    
    if (handler) {
        try {
            handler(command_data);
            LL_INFOS("AIControllerBridge") << "Executed command: " << commandToString(command_data.command_type) << LL_ENDL;
        } catch (const std::exception& e) {
            LL_ERRS("AIControllerBridge") << "Error executing command: " << e.what() << LL_ENDL;
        }
    } else {
        LL_WARNS("AIControllerBridge") << "No handler registered for command: " << commandToString(command_data.command_type) << LL_ENDL;
    }
}

void AIControllerBridge::handlePythonCommand(const std::string& command_str, const py::dict& data) {
    try {
        AICommand command = stringToCommand(command_str);
        AICommandData command_data(command);

        // Extract data from Python dict
        if (data.contains("content")) {
            command_data.content = py::str(data["content"]);
        }
        if (data.contains("target_uuid")) {
            command_data.target_uuid = py::str(data["target_uuid"]);
        }
        if (data.contains("target_position")) {
            py::tuple pos = data["target_position"];
            if (pos.size() >= 3) {
                command_data.x = py::float_(pos[0]);
                command_data.y = py::float_(pos[1]);
                command_data.z = py::float_(pos[2]);
            }
        }
        if (data.contains("parameters")) {
            py::dict params = data["parameters"];
            for (auto item : params) {
                command_data.parameters[py::str(item.first)] = py::str(item.second);
            }
        }

        executeCommand(command_data);

    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error handling Python command: " << e.what() << LL_ENDL;
    }
}

void AIControllerBridge::updateEnvironment(
    const std::vector<std::pair<std::string, std::string>>& agents,
    const std::vector<std::pair<std::string, std::string>>& objects,
    const std::string& current_location
) {
    if (!initialized_.load() || current_persona_.empty()) {
        return;
    }

    try {
        executePythonTaskSync([this, &agents, &objects, &current_location]() {
            try {
                GILGuard gil;
                
                py::object active_controller;
                if (!controller_instance_.is_none()) {
                    auto manager = controller_instance_.get();
                    if (py::hasattr(manager, "get_active_controller")) {
                        active_controller = manager.attr("get_active_controller")();
                    }
                }

                if (active_controller && !active_controller.is_none()) {
                    // Build environment data
                    py::dict env_data;
                    
                    py::list agent_list;
                    for (const auto& agent : agents) {
                        py::tuple agent_tuple = py::make_tuple(agent.first, agent.second);
                        agent_list.append(agent_tuple);
                    }
                    env_data["agents"] = agent_list;

                    py::list object_list;
                    for (const auto& obj : objects) {
                        py::tuple obj_tuple = py::make_tuple(obj.first, obj.second);
                        object_list.append(obj_tuple);
                    }
                    env_data["objects"] = object_list;

                    if (!current_location.empty()) {
                        env_data["current_location"] = current_location;
                    }

                    // Update environment in Python
                    if (py::hasattr(active_controller.attr("pipeline"), "environment_monitor")) {
                        py::object env_monitor = active_controller.attr("pipeline").attr("environment_monitor");
                        if (py::hasattr(env_monitor, "update_from_external")) {
                            runAsyncFunction(env_monitor.attr("update_from_external"), py::make_tuple(env_data));
                        }
                    }
                }
                
            } catch (const py::error_already_set& e) {
                LL_ERRS("AIControllerBridge") << "Python error updating environment: " << e.what() << LL_ENDL;
            }
        });
        
    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error in updateEnvironment: " << e.what() << LL_ENDL;
    }
}

void AIControllerBridge::emergencyStop() {
    if (!initialized_.load()) {
        return;
    }

    try {
        executePythonTaskSync([this]() {
            try {
                GILGuard gil;
                
                py::object active_controller;
                if (!controller_instance_.is_none()) {
                    auto manager = controller_instance_.get();
                    if (py::hasattr(manager, "get_active_controller")) {
                        active_controller = manager.attr("get_active_controller")();
                    }
                }

                if (active_controller && !active_controller.is_none()) {
                    if (py::hasattr(active_controller, "emergency_stop")) {
                        runAsyncFunction(active_controller.attr("emergency_stop"), py::make_tuple());
                    }
                }
                
            } catch (const py::error_already_set& e) {
                LL_ERRS("AIControllerBridge") << "Python error during emergency stop: " << e.what() << LL_ENDL;
            }
        });
        
        LL_WARNS("AIControllerBridge") << "Emergency stop activated" << LL_ENDL;
        
    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error in emergencyStop: " << e.what() << LL_ENDL;
    }
}

void AIControllerBridge::pauseAutonomousBehavior() {
    if (!initialized_.load()) {
        return;
    }

    try {
        executePythonTaskSync([this]() {
            try {
                GILGuard gil;
                
                py::object active_controller;
                if (!controller_instance_.is_none()) {
                    auto manager = controller_instance_.get();
                    if (py::hasattr(manager, "get_active_controller")) {
                        active_controller = manager.attr("get_active_controller")();
                    }
                }

                if (active_controller && !active_controller.is_none()) {
                    if (py::hasattr(active_controller, "pauseAutonomousBehavior")) {
                        runAsyncFunction(active_controller.attr("pauseAutonomousBehavior"), py::make_tuple());
                    }
                }
                
            } catch (const py::error_already_set& e) {
                LL_ERRS("AIControllerBridge") << "Python error pausing autonomous behavior: " << e.what() << LL_ENDL;
            }
        });
        
        LL_INFOS("AIControllerBridge") << "Autonomous behavior paused" << LL_ENDL;
        
    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error in pauseAutonomousBehavior: " << e.what() << LL_ENDL;
    }
}

void AIControllerBridge::resumeAutonomousBehavior() {
    if (!initialized_.load()) {
        return;
    }

    try {
        executePythonTaskSync([this]() {
            try {
                GILGuard gil;
                
                py::object active_controller;
                if (!controller_instance_.is_none()) {
                    auto manager = controller_instance_.get();
                    if (py::hasattr(manager, "get_active_controller")) {
                        active_controller = manager.attr("get_active_controller")();
                    }
                }

                if (active_controller && !active_controller.is_none()) {
                    if (py::hasattr(active_controller, "resumeAutonomousBehavior")) {
                        runAsyncFunction(active_controller.attr("resumeAutonomousBehavior"), py::make_tuple());
                    }
                }
                
            } catch (const py::error_already_set& e) {
                LL_ERRS("AIControllerBridge") << "Python error resuming autonomous behavior: " << e.what() << LL_ENDL;
            }
        });
        
        LL_INFOS("AIControllerBridge") << "Autonomous behavior resumed" << LL_ENDL;
        
    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error in resumeAutonomousBehavior: " << e.what() << LL_ENDL;
    }
}

std::map<std::string, std::string> AIControllerBridge::getSystemStatus() {
    std::map<std::string, std::string> status;
    
    if (!initialized_.load()) {
        status["status"] = "not_initialized";
        return status;
    }

    try {
        executePythonTaskSync([this, &status]() {
            try {
                GILGuard gil;
                
                py::object active_controller;
                if (!controller_instance_.is_none()) {
                    auto manager = controller_instance_.get();
                    if (py::hasattr(manager, "get_active_controller")) {
                        active_controller = manager.attr("get_active_controller")();
                    }
                }

                if (active_controller && !active_controller.is_none()) {
                    if (py::hasattr(active_controller, "get_system_status")) {
                        py::object status_obj = runAsyncFunction(active_controller.attr("get_system_status"), py::make_tuple());
                        
                        // Convert Python object to C++ map
                        if (!status_obj.is_none() && py::hasattr(status_obj, "dict")) {
                            py::dict status_dict = status_obj.attr("dict")();
                            for (auto item : status_dict) {
                                status[py::str(item.first)] = py::str(item.second);
                            }
                        }
                    }
                }
                
            } catch (const py::error_already_set& e) {
                LL_ERRS("AIControllerBridge") << "Python error getting status: " << e.what() << LL_ENDL;
                status["error"] = e.what();
            }
        });
        
        // Add local status information
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            status["persona"] = current_persona_;
        }
        status["bridge_status"] = "active";
        
    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error in getSystemStatus: " << e.what() << LL_ENDL;
        status["error"] = e.what();
    }

    return status;
}

std::string AIControllerBridge::commandToString(AICommand command) {
    auto it = command_to_string.find(command);
    return (it != command_to_string.end()) ? it->second : "UNKNOWN";
}

AICommand AIControllerBridge::stringToCommand(const std::string& command_str) {
    auto it = string_to_command.find(command_str);
    return (it != string_to_command.end()) ? it->second : AICommand::STATUS_UPDATE;
}

std::string AIControllerBridge::extractPersonaFromLoggedInAgent(const std::string& logged_in_agent) {
    if (logged_in_agent.empty()) {
        return "AI";  // Default fallback
    }
    
    // Handle format: "DisplayName (user_id)" -> "DisplayName"
    size_t paren_pos = logged_in_agent.find('(');
    if (paren_pos != std::string::npos) {
        std::string display_name = logged_in_agent.substr(0, paren_pos);
        // Trim whitespace
        display_name.erase(display_name.find_last_not_of(" \t") + 1);
        display_name.erase(0, display_name.find_first_not_of(" \t"));
        
        LL_INFOS("AIControllerBridge") << "Extracted persona name: '" << display_name 
                                      << "' from logged_in_agent: '" << logged_in_agent << "'" << LL_ENDL;
        return display_name;
    }
    
    // If no parentheses, use the whole string as persona name
    LL_INFOS("AIControllerBridge") << "Using full logged_in_agent as persona: '" << logged_in_agent << "'" << LL_ENDL;
    return logged_in_agent;
}

void AIControllerBridge::pythonThreadLoop() {
    LL_INFOS("AIControllerBridge") << "DEBUG: Python thread entry - acquiring GIL for new thread..." << LL_ENDL;
    
    // Create new thread state and acquire GIL for this worker thread
    PyThreadState* worker_thread_state = nullptr;
    
    try {
        LL_INFOS("AIControllerBridge") << "DEBUG: Creating new Python thread state..." << LL_ENDL;
        worker_thread_state = PyThreadState_New(PyInterpreterState_Main());
        if (!worker_thread_state) {
            LL_ERRS("AIControllerBridge") << "FATAL: Failed to create Python thread state" << LL_ENDL;
            python_thread_running_.store(false);
            return;
        }
        LL_INFOS("AIControllerBridge") << "DEBUG: Python thread state created successfully" << LL_ENDL;
        
        LL_INFOS("AIControllerBridge") << "DEBUG: Acquiring GIL for worker thread..." << LL_ENDL;
        PyEval_RestoreThread(worker_thread_state);  // Acquire GIL for this thread
        LL_INFOS("AIControllerBridge") << "DEBUG: GIL acquired successfully for worker thread" << LL_ENDL;
        
        LL_INFOS("AIControllerBridge") << "DEBUG: Importing asyncio module..." << LL_ENDL;
        auto asyncio = py::module_::import("asyncio");
        LL_INFOS("AIControllerBridge") << "DEBUG: asyncio imported successfully" << LL_ENDL;
        
        LL_INFOS("AIControllerBridge") << "DEBUG: Creating new event loop..." << LL_ENDL;
        auto loop = asyncio.attr("new_event_loop")();
        LL_INFOS("AIControllerBridge") << "DEBUG: Event loop created successfully" << LL_ENDL;
        
        LL_INFOS("AIControllerBridge") << "DEBUG: Setting event loop..." << LL_ENDL;
        asyncio.attr("set_event_loop")(loop);
        LL_INFOS("AIControllerBridge") << "DEBUG: Event loop set successfully" << LL_ENDL;
        
        // Store the loop in thread-safe wrapper
        LL_INFOS("AIControllerBridge") << "DEBUG: Storing loop in thread-safe wrapper..." << LL_ENDL;
        asyncio_loop_.reset(loop);
        LL_INFOS("AIControllerBridge") << "DEBUG: Loop stored successfully" << LL_ENDL;
        
        // NOW set the running flag - Python setup completed successfully
        LL_INFOS("AIControllerBridge") << "DEBUG: Python setup complete - setting thread running flag" << LL_ENDL;
        python_thread_running_.store(true);
        
        LL_INFOS("AIControllerBridge") << "Python event loop created, starting task processing..." << LL_ENDL;
        
        // Now release GIL so main thread can continue - we'll reacquire it when needed for tasks
        LL_INFOS("AIControllerBridge") << "DEBUG: Releasing GIL for main thread to continue..." << LL_ENDL;
        PyEval_SaveThread();  // Release GIL, save worker thread state
        LL_INFOS("AIControllerBridge") << "DEBUG: GIL released, main thread can now continue" << LL_ENDL;
        
    } catch (const py::error_already_set& e) {
        LL_ERRS("AIControllerBridge") << "FATAL: Python error during setup: " << e.what() << LL_ENDL;
        python_thread_running_.store(false);
        return;
    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "FATAL: Standard exception during Python setup: " << e.what() << LL_ENDL;
        python_thread_running_.store(false);
        return;
    } catch (...) {
        LL_ERRS("AIControllerBridge") << "FATAL: Unknown exception during Python setup" << LL_ENDL;
        python_thread_running_.store(false);
        return;
    }
    
    // Main task processing loop - GIL is now released, we'll acquire it only when executing Python tasks
    LL_INFOS("AIControllerBridge") << "DEBUG: Entering main task processing loop" << LL_ENDL;
    int loop_iteration = 0;
    while (!shutdown_requested_.load()) {
        try {
            loop_iteration++;
            if (loop_iteration % 100 == 0) {  // Log every 100 iterations to avoid spam
                LL_INFOS("AIControllerBridge") << "DEBUG: Task processing loop iteration " << loop_iteration << LL_ENDL;
            }
            
            PythonTask task;
            bool task_received = false;
            size_t queue_size = 0;
            
            // Wait for a task or timeout
            {
                std::unique_lock<std::mutex> lock(task_queue_mutex_);
                queue_size = python_task_queue_.size();
                
                if (task_condition_.wait_for(lock, std::chrono::milliseconds(100), 
                                           [this] { return !python_task_queue_.empty() || shutdown_requested_.load(); })) {
                    
                    if (!python_task_queue_.empty()) {
                        LL_INFOS("AIControllerBridge") << "DEBUG: Task received from queue (queue size: " 
                                                       << python_task_queue_.size() << ")" << LL_ENDL;
                        task = std::move(python_task_queue_.front());
                        python_task_queue_.pop();
                        task_received = true;
                    }
                } else {
                    // Timeout - this is normal, just log occasionally
                    if (loop_iteration % 100 == 0 && queue_size > 0) {
                        LL_INFOS("AIControllerBridge") << "DEBUG: Wait timeout, but queue has " 
                                                       << queue_size << " tasks" << LL_ENDL;
                    }
                }
            }
            
            // Handle shutdown signal
            if (task.is_shutdown || shutdown_requested_.load()) {
                LL_INFOS("AIControllerBridge") << "Python thread received shutdown signal" << LL_ENDL;
                break;
            }
            
            // Execute the task if we have one
            if (task.task) {
                LL_INFOS("AIControllerBridge") << "DEBUG: Executing Python task - acquiring GIL..." << LL_ENDL;
                try {
                    // Reacquire GIL for Python task execution
                    PyEval_RestoreThread(worker_thread_state);
                    LL_INFOS("AIControllerBridge") << "DEBUG: GIL acquired for task execution" << LL_ENDL;
                    
                    task.task();
                    task.promise.set_value();
                    LL_INFOS("AIControllerBridge") << "DEBUG: Python task completed successfully" << LL_ENDL;
                    
                    // Release GIL again after task completion
                    LL_INFOS("AIControllerBridge") << "DEBUG: Releasing GIL after task completion" << LL_ENDL;
                    worker_thread_state = PyEval_SaveThread();
                    
                } catch (const std::exception& e) {
                    LL_ERRS("AIControllerBridge") << "Error executing Python task: " << e.what() << LL_ENDL;
                    // Make sure to release GIL even on error
                    try {
                        worker_thread_state = PyEval_SaveThread();
                    } catch (...) {}
                    task.promise.set_exception(std::current_exception());
                } catch (...) {
                    LL_ERRS("AIControllerBridge") << "Unknown error executing Python task" << LL_ENDL;
                    // Make sure to release GIL even on error
                    try {
                        worker_thread_state = PyEval_SaveThread();
                    } catch (...) {}
                    task.promise.set_exception(std::make_exception_ptr(std::runtime_error("Unknown Python task error")));
                }
            } else if (task_received) {
                LL_WARNS("AIControllerBridge") << "DEBUG: Received task but task.task is null!" << LL_ENDL;
            }
            
        } catch (const std::exception& e) {
            LL_ERRS("AIControllerBridge") << "Error in Python thread loop: " << e.what() << LL_ENDL;
        }
    }
    
    // Clean shutdown
    try {
        // Reacquire GIL for cleanup operations
        LL_INFOS("AIControllerBridge") << "DEBUG: Acquiring GIL for cleanup..." << LL_ENDL;
        if (worker_thread_state) {
            PyEval_RestoreThread(worker_thread_state);
        }
        
        // Stop the asyncio event loop if it's running
        if (!asyncio_loop_.is_none()) {
            LL_INFOS("AIControllerBridge") << "DEBUG: Stopping asyncio event loop..." << LL_ENDL;
            auto loop = asyncio_loop_.get();
            if (!loop.is_none() && py::hasattr(loop, "is_running")) {
                if (py::bool_(loop.attr("is_running")())) {
                    loop.attr("stop")();
                }
            }
        }
        
        // Clean up thread state
        LL_INFOS("AIControllerBridge") << "DEBUG: Cleaning up thread state..." << LL_ENDL;
        if (worker_thread_state) {
            PyEval_SaveThread();  // Release GIL before cleanup
            PyThreadState_Clear(worker_thread_state);
            PyThreadState_Delete(worker_thread_state);
            LL_INFOS("AIControllerBridge") << "DEBUG: Worker thread state cleaned up" << LL_ENDL;
        }
        
    } catch (const std::exception& e) {
        LL_WARNS("AIControllerBridge") << "Error during Python thread cleanup: " << e.what() << LL_ENDL;
    }
    
    python_thread_running_.store(false);
    LL_INFOS("AIControllerBridge") << "Python background thread exiting" << LL_ENDL;
}

void AIControllerBridge::setUserIdInternal(const char* user_id) {
    if (!user_id) return;
    
    try {
        if (isInitialized()) {
            executePythonTaskSync([this, user_id]() {
                try {
                    GILGuard gil;
                    
                    auto manager = controller_instance_.get();
                    if (!manager.is_none() && py::hasattr(manager, "set_user_id")) {
                        runAsyncFunction(manager.attr("set_user_id"), py::make_tuple(user_id));
                        LL_INFOS("AIControllerBridge") << "User ID set: " << user_id << LL_ENDL;
                    }
                    
                } catch (const py::error_already_set& e) {
                    LL_ERRS("AIControllerBridge") << "Python error setting user ID: " << e.what() << LL_ENDL;
                } catch (const std::exception& e) {
                    LL_ERRS("AIControllerBridge") << "Error setting user ID: " << e.what() << LL_ENDL;
                }
            });
        }
    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error setting user ID: " << e.what() << LL_ENDL;
    }
}

// Forward declaration for internal C++ function used by C API
void ai_controller_set_user_id_impl(const char* user_id);

// C API Implementation
extern "C" {

bool ai_controller_init() {
    return AIControllerBridge::getInstance().initialize();
}

void ai_controller_shutdown() {
    AIControllerBridge::getInstance().shutdown();
}

bool ai_controller_start(const char* persona_name) {
    std::string persona = persona_name ? persona_name : "";
    return AIControllerBridge::getInstance().startAIController(persona);
}

bool ai_controller_stop() {
    return AIControllerBridge::getInstance().stopAIController();
}

bool ai_controller_process_message(
    const char* text,
    const char* from_name,
    const char* from_id,
    const char* spatial_context,
    const char* logged_in_agent
) {
    return AIControllerBridge::getInstance().processMessage(
        text ? text : "",
        from_name ? from_name : "",
        from_id ? from_id : "",
        spatial_context ? spatial_context : "",
        logged_in_agent ? logged_in_agent : ""
    );
}

void ai_controller_register_chat_handler(void (*handler)(const char* content)) {
    if (handler) {
        auto cpp_handler = [handler](const AICommandData& data) {
            handler(data.content.c_str());
        };
        AIControllerBridge::getInstance().registerCommandHandler(AICommand::CHAT, cpp_handler);
    }
}

void ai_controller_register_emote_handler(void (*handler)(const char* content)) {
    if (handler) {
        auto cpp_handler = [handler](const AICommandData& data) {
            handler(data.content.c_str());
        };
        AIControllerBridge::getInstance().registerCommandHandler(AICommand::EMOTE, cpp_handler);
    }
}

void ai_controller_register_move_handler(void (*handler)(float x, float y, float z)) {
    if (handler) {
        auto cpp_handler = [handler](const AICommandData& data) {
            handler(data.x, data.y, data.z);
        };
        AIControllerBridge::getInstance().registerCommandHandler(AICommand::MOVE, cpp_handler);
    }
}

void ai_controller_register_touch_handler(void (*handler)(const char* object_uuid)) {
    if (handler) {
        auto cpp_handler = [handler](const AICommandData& data) {
            handler(data.target_uuid.c_str());
        };
        AIControllerBridge::getInstance().registerCommandHandler(AICommand::TOUCH, cpp_handler);
    }
}

void ai_controller_register_sit_handler(void (*handler)(const char* object_uuid)) {
    if (handler) {
        auto cpp_handler = [handler](const AICommandData& data) {
            handler(data.target_uuid.c_str());
        };
        AIControllerBridge::getInstance().registerCommandHandler(AICommand::SIT, cpp_handler);
    }
}

void ai_controller_register_stand_handler(void (*handler)()) {
    if (handler) {
        auto cpp_handler = [handler](const AICommandData& data) {
            handler();
        };
        AIControllerBridge::getInstance().registerCommandHandler(AICommand::STAND, cpp_handler);
    }
}

void ai_controller_register_look_at_handler(void (*handler)(float x, float y, float z)) {
    if (handler) {
        auto cpp_handler = [handler](const AICommandData& data) {
            handler(data.x, data.y, data.z);
        };
        AIControllerBridge::getInstance().registerCommandHandler(AICommand::LOOK_AT, cpp_handler);
    }
}

void ai_controller_emergency_stop() {
    AIControllerBridge::getInstance().emergencyStop();
}

void ai_controller_pause() {
    AIControllerBridge::getInstance().pauseAutonomousBehavior();
}

void ai_controller_resume() {
    AIControllerBridge::getInstance().resumeAutonomousBehavior();
}

const char* ai_controller_get_status() {
    auto status = AIControllerBridge::getInstance().getSystemStatus();
    
    // Convert to JSON string (simplified)
    static std::string status_json;
    std::ostringstream oss;
    oss << "{";
    bool first = true;
    for (const auto& pair : status) {
        if (!first) oss << ",";
        oss << "\"" << pair.first << "\":\"" << pair.second << "\"";
        first = false;
    }
    oss << "}";
    
    status_json = oss.str();
    return status_json.c_str();
}



const char* ai_controller_get_display_name() {
    // This function should be called from Python to get current display name
    // For now, we'll use the same extraction logic but return current name from cache
    LLUUID agent_id = gAgent.getID();
    if (agent_id.isNull()) {
        return "";
    }
    
    LLAvatarName av_name;
    if (LLAvatarNameCache::get(agent_id, &av_name)) {
        static std::string display_name = av_name.getCompleteName();
        return display_name.c_str();
    }
    
    return "";
}

//==============================================================================
// Message Queue Integration Methods
//==============================================================================

bool AIControllerBridge::initializeMessageQueue(MessageQueueInterface::QueueType queue_type) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (queue_enabled_ && message_queue_) {
        LL_WARNS("AIControllerBridge") << "Message queue already initialized" << LL_ENDL;
        return true;
    }
    
    try {
        // Create message queue
        message_queue_ = std::make_unique<MessageQueueInterface>(
            queue_type, 
            "", // Use default path
            1024, // Default capacity
            OverflowStrategy::DROP_OLDEST
        );
        
        if (!message_queue_->initialize()) {
            LL_ERRS("AIControllerBridge") << "Failed to initialize message queue" << LL_ENDL;
            message_queue_.reset();
            return false;
        }
        
        // Create queue processor
        queue_processor_ = std::make_unique<QueueProcessor>();
        if (!queue_processor_->initialize(message_queue_.get())) {
            LL_ERRS("AIControllerBridge") << "Failed to initialize queue processor" << LL_ENDL;
            message_queue_.reset();
            queue_processor_.reset();
            return false;
        }
        
        queue_enabled_ = true;
        LL_INFOS("AIControllerBridge") << "Message queue system initialized successfully" << LL_ENDL;
        return true;
        
    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Exception initializing message queue: " << e.what() << LL_ENDL;
        message_queue_.reset();
        queue_processor_.reset();
        queue_enabled_ = false;
        return false;
    }
}

void AIControllerBridge::shutdownMessageQueue() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (queue_processor_) {
        queue_processor_->shutdown();
        queue_processor_.reset();
    }
    
    if (message_queue_) {
        message_queue_->shutdown();
        message_queue_.reset();
    }
    
    queue_enabled_ = false;
    LL_INFOS("AIControllerBridge") << "Message queue system shut down" << LL_ENDL;
}

void AIControllerBridge::processMessageQueue(int max_commands_per_frame) {
    if (!queue_enabled_ || !queue_processor_) {
        return;
    }
    
    try {
        queue_processor_->processCommands(max_commands_per_frame);
    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error processing message queue: " << e.what() << LL_ENDL;
    }
}

std::map<std::string, std::string> AIControllerBridge::getQueueStatistics() {
    std::map<std::string, std::string> stats;
    
    if (!queue_enabled_) {
        stats["status"] = "disabled";
        return stats;
    }
    
    try {
        if (message_queue_) {
            auto queue_stats = message_queue_->getStatistics();
            stats["total_enqueued"] = std::to_string(queue_stats.total_enqueued);
            stats["total_dequeued"] = std::to_string(queue_stats.total_dequeued);
            stats["enqueue_failures"] = std::to_string(queue_stats.enqueue_failures);
            stats["dequeue_failures"] = std::to_string(queue_stats.dequeue_failures);
            stats["validation_failures"] = std::to_string(queue_stats.validation_failures);
            stats["max_queue_depth"] = std::to_string(queue_stats.max_queue_depth);
            stats["avg_command_size"] = std::to_string(queue_stats.avg_command_size);
            stats["pending_commands"] = std::to_string(message_queue_->pendingCount());
        }
        
        if (queue_processor_) {
            auto processor_stats = queue_processor_->getStatistics();
            stats["commands_processed"] = std::to_string(processor_stats.commands_processed);
            stats["processing_errors"] = std::to_string(processor_stats.commands_failed);
            stats["avg_processing_time_ms"] = std::to_string(processor_stats.avg_processing_time_ms);
            stats["processing_rate"] = std::to_string(processor_stats.getProcessingRate());
        }
        
        stats["status"] = "enabled";
        
    } catch (const std::exception& e) {
        stats["status"] = "error";
        stats["error"] = e.what();
    }
    
    return stats;
}

void ai_controller_set_user_id(const char* user_id) {
    ai_controller_set_user_id_impl(user_id);
}

} // extern "C"

// Internal C++ implementation accessible to class members via friend declaration
// This must be outside extern "C" block to maintain C++ linkage for friend access
void ai_controller_set_user_id_impl(const char* user_id) {
    if (!user_id) return;
    
    try {
        auto& bridge = AIControllerBridge::getInstance();
        if (bridge.isInitialized()) {
            // Use public async wrapper instead of accessing private members
            bridge.setUserIdAsync(std::string(user_id));
        }
    } catch (const std::exception& e) {
        LL_ERRS("AIControllerBridge") << "Error setting user ID: " << e.what() << LL_ENDL;
    }
}

} // namespace python_bridge
