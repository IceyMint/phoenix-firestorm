# AI Integration Documentation for Firestorm Viewer

## Overview
This document provides a complete reference for AI integration points in the Firestorm viewer codebase. It's designed to help future developers understand where AI-related code is located and how to maintain/extend the system.

## Key Files and Locations

### 1. AI Integration Core Files
- **`fsaiintegration.cpp`** - Main AI integration logic
- **`fsaiintegration.h`** - AI integration class definition
- **Location**: `viewer_code/newview/`
- **Purpose**: Handles all ongoing AI operations, environment monitoring, command callbacks

### 2. Python Bridge Files
- **`py_ai_controller_bridge.cpp`** - C++ bridge to Python AI system
- **`py_ai_controller_bridge.h`** - Bridge interface definitions
- **`example_integration.cpp`** - Example integration patterns
- **Location**: `viewer_code/newview/python_bridge/`
- **Purpose**: Provides C++ interface to Python AI controller

### 3. Chat Integration Point
- **`fsfloaternearbychat.cpp`** - Nearby chat floater (MINIMAL AI CODE)
- **Location**: `viewer_code/newview/`
- **AI Integration**: Only calls `FSAIIntegration::processMessage()` in `addMessage()` function
- **Lines to modify**: Around line 295-325 in `addMessage()` function

### 4. Viewer Startup/Shutdown Points
- **`llappviewer.cpp`** - Main application lifecycle
- **Location**: `viewer_code/newview/`
- **AI Integration Points**:
  - `init()` or `initializeViewerComponents()` - Call `ai_controller_init()`
  - `cleanup()` or similar - Call `ai_controller_shutdown()`

## Viewer Architecture Key Points

### Startup Sequence
```cpp
// In llappviewer.cpp or similar initialization function
bool LLAppViewer::init() {
    // ... existing initialization code ...
    
    // Initialize AI controller bridge
    if (gSavedSettings.getBOOL("EnablePythonAI")) {
        if (!ai_controller_init()) {
            LL_WARNS("AI") << "Failed to initialize AI controller bridge" << LL_ENDL;
        } else {
            // Start AI controller with default persona
            ai_controller_start("Tiffany");
            LL_INFOS("AI") << "AI controller bridge initialized successfully" << LL_ENDL;
        }
    }
    
    // ... continue with other initialization ...
}
```

### Shutdown Sequence
```cpp
// In llappviewer.cpp cleanup function
void LLAppViewer::cleanup() {
    // ... existing cleanup code ...
    
    // Shutdown AI controller bridge
    ai_controller_shutdown();
    
    // ... continue with other cleanup ...
}
```

### Update Loop Integration
```cpp
// In main update loop (likely llappviewer.cpp)
void LLAppViewer::mainLoop() {
    // ... existing update logic ...
    
    // Update AI environment monitoring
    FSAIIntegration::getInstance().updateEnvironmentMonitoring();
    
    // ... continue with other updates ...
}
```

## Chat Integration Pattern

### Current Integration (Minimal)
```cpp
// In fsfloaternearbychat.cpp, addMessage() function
void FSFloaterNearbyChat::addMessage(const LLChat& chat, bool archive, const LLSD& args)
{
    // ... existing chat processing ...
    
    // AI integration (ONLY THIS CODE ADDED)
    if (gSavedSettings.getBOOL("EnablePythonAI") && 
        chat.mText.find("is online.") == std::string::npos &&
        chat.mText.find("is offline.") == std::string::npos && 
        chat.mText.find("is typing") == std::string::npos &&
        chat.mText.find(" has stopped typing") == std::string::npos)
    {
        FSAIIntegration::getInstance().processMessage(chat);
    }
    
    // ... continue with existing chat processing ...
}
```

## Environment Monitoring System

### Key Components
1. **Environment State Tracking**: Monitors avatars, objects, and world state
2. **Change Detection**: Identifies significant environmental changes
3. **Periodic Updates**: Sends environment data to Python AI system
4. **Real-time Context**: Provides spatial context on-demand

### Integration Points
- **Avatar Tracking**: Uses `LLWorld::instance().getAvatars()`
- **Object Tracking**: Integrates with existing area search (`FSAreaSearch`)
- **Region Info**: Uses `gAgent.getRegion()` for location data

## Command System Integration

### Command Handlers
Commands from Python AI are processed through registered handlers:

```cpp
// Registered during AI initialization
ai_controller_register_chat_handler(handleAIChatOutput);
ai_controller_register_emote_handler(handleAIEmoteOutput);
ai_controller_register_move_handler(handleAIMovementOutput);
// ... other handlers ...
```

### Chat Output Handler
```cpp
void handleAIChatOutput(const char* content) {
    if (FSFloaterNearbyChat* floater = FSFloaterNearbyChat::getInstance()) {
        floater->sendChatFromViewer(content, CHAT_TYPE_NORMAL, true);
    }
}
```

## Important Viewer Systems

### 1. Settings System
- **Location**: Global `gSavedSettings`
- **AI Setting**: `"EnablePythonAI"` - Boolean toggle for AI system
- **Usage**: Check this setting before any AI operations

### 2. Agent System
- **Location**: Global `gAgent`
- **Key Functions**:
  - `gAgent.getPositionGlobal()` - Avatar position
  - `gAgent.getRegion()` - Current region
  - `gAgent.getID()` - Avatar UUID

### 3. Avatar Name Cache
- **Location**: `LLAvatarNameCache`
- **Usage**: `LLAvatarNameCache::get(uuid, &name)` - Get display names

### 4. Object System
- **Location**: Global `gObjectList`
- **Usage**: `gObjectList.findObject(uuid)` - Find objects by UUID

### 5. World System
- **Location**: `LLWorld::instance()`
- **Usage**: `getAvatars()` - Get nearby avatars and positions

### 6. Area Search Integration
- **Location**: `FSAreaSearch* areaSearch = LLFloaterReg::findTypedInstance<FSAreaSearch>("area_search")`
- **Usage**: Access `areaSearch->mObjectDetails` for nearby object information

## Build System Integration

### CMakeLists.txt Updates
Add new AI files to the build system:

```cmake
# In viewer_code/newview/CMakeLists.txt
set(newview_SOURCE_FILES
    # ... existing files ...
    fsaiintegration.cpp
    # ... other files ...
)

set(newview_HEADER_FILES
    # ... existing files ...
    fsaiintegration.h
    # ... other files ...
)
```

## Error Handling and Logging

### Logging Categories
- **"AI"** - General AI system messages
- **"AIBridge"** - Python bridge specific messages
- **"AIEnvironment"** - Environment monitoring messages

### Logging Examples
```cpp
LL_INFOS("AI") << "AI system initialized" << LL_ENDL;
LL_WARNS("AI") << "AI system error: " << error_message << LL_ENDL;
LL_DEBUGS("AI") << "Environment update: " << agent_count << " agents" << LL_ENDL;
```

### Error Handling Pattern
```cpp
try {
    // AI operation
} catch (const std::exception& e) {
    LL_WARNS("AI") << "AI operation failed: " << e.what() << LL_ENDL;
    // Graceful degradation - continue without AI
}
```

## Settings and Configuration

### Viewer Settings
- **EnablePythonAI**: Boolean - Master toggle for AI system
- **Location**: Controlled via chat options menu
- **Storage**: `gSavedSettings.getBOOL("EnablePythonAI")`

### AI Configuration Files
- **Location**: Genesis project root (`D:\@home\snow\Documents\genesis\`)
- **config.yaml**: Main AI configuration
- **Various Python modules**: AI behavior definitions

## Threading and Performance

### Threading Model
- **Main Thread**: UI updates, command execution
- **AI Bridge**: Handles Python communication internally
- **Environment Monitoring**: Lightweight, runs in main update loop

### Performance Considerations
- Environment updates limited to every 2 seconds
- Minimal impact on viewer frame rate
- Asynchronous Python communication
- Bounded memory usage for state caching

## Common Integration Patterns

### 1. Adding New AI Commands
```cpp
// 1. Add command to AICommand enum in py_ai_controller_bridge.h
// 2. Register handler during initialization
ai_controller_register_new_handler(handleNewCommand);
// 3. Implement handler function
void handleNewCommand(const char* data) {
    // Implementation
}
```

### 2. Adding Environment Data
```cpp
// 1. Modify gatherAndSendEnvironmentUpdate() in fsaiintegration.cpp
// 2. Add new data to agents_str or objects_str vectors
// 3. Ensure format is (name, position_string) pairs
```

### 3. Adding New Settings
```cpp
// 1. Add to viewer settings definitions
// 2. Check setting in AI code: gSavedSettings.getBOOL("NewAISetting")
// 3. Add UI toggle in chat options menu
```

## Debugging and Development

### Debug Build Flags
- Enable AI logging: Set log level to DEBUG for "AI" category
- Python bridge debugging: Enable logging in bridge code
- Environment monitoring: Watch LL_DEBUGS("AI") messages

### Common Debug Points
- AI initialization success/failure
- Message processing flow
- Environment update frequency
- Command execution results

### Testing Integration Points
- Chat message processing: Send test messages
- Environment monitoring: Move avatar, observe updates
- Command execution: Verify AI responses appear in chat
- Bridge communication: Check Python side receives data

## Future Development Notes

### Extensibility Points
- **New Commands**: Easy to add through bridge command system
- **Enhanced Environment**: Can add more data types to monitoring
- **Multiple Personas**: Bridge supports persona switching
- **Physical Actions**: Framework ready for movement commands

### Upgrade Considerations
- Keep AI code isolated in dedicated files
- Maintain minimal changes to core viewer files
- Use proper error handling for graceful degradation
- Follow existing viewer patterns and conventions

---

This documentation should be updated whenever significant changes are made to the AI integration system.