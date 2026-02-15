# CMakeLists.txt for AI Controller Bridge

set(python_bridge_SOURCE_FILES
    py_bridge.cpp
    py_bridge.h
    py_ai_controller_bridge.cpp
    py_ai_controller_bridge.h
    example_integration.cpp
)

set(python_bridge_HEADER_FILES
    py_bridge.h
    py_ai_controller_bridge.h
)

# Find Python
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Find pybind11
find_package(pybind11 REQUIRED)

# Create the library
add_library(python_bridge STATIC ${python_bridge_SOURCE_FILES})

# Include directories
target_include_directories(python_bridge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${Python3_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/llcommon
    ${CMAKE_SOURCE_DIR}/llmath
    ${CMAKE_SOURCE_DIR}/llmessage
    ${CMAKE_SOURCE_DIR}/newview
)

# Link libraries
target_link_libraries(python_bridge
    ${Python3_LIBRARIES}
    pybind11::module
    pybind11::embed
    llcommon
)

# Compiler definitions
target_compile_definitions(python_bridge PRIVATE
    LL_COMMON=1
    LL_WINDOWS=1  # Adjust for your platform
)

# Set C++ standard
set_property(TARGET python_bridge PROPERTY CXX_STANDARD 17)

# Platform-specific settings
if(WIN32)
    target_compile_definitions(python_bridge PRIVATE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
    )
endif()

# Installation (if needed)
install(TARGETS python_bridge
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${python_bridge_HEADER_FILES}
    DESTINATION include/python_bridge
)