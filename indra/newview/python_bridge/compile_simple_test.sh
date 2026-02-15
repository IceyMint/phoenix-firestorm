#!/bin/bash

echo "==============================================="
echo "Compiling Simple Message Queue Tests (No Google Test)"
echo "==============================================="

# Set compiler and flags
CXX="g++"
CXXFLAGS="-std=c++17 -pthread -O2 -DNDEBUG -Wall -Wextra"

# Set include directories (include nlohmann directory)
INCLUDES="-I. -I./nlohmann"

# Source files (no Google Test dependency)
SOURCES="simple_test.cpp ai_message_queue.cpp queue_processor.cpp"

# Output executable
OUTPUT="simple_test"

echo
echo "Compiling with:"
echo "  Compiler: $CXX"
echo "  Flags: $CXXFLAGS"
echo "  Includes: $INCLUDES"
echo "  Sources: $SOURCES"
echo "  Output: $OUTPUT"
echo

# Check if source files exist
echo "Checking source files..."
for file in $SOURCES; do
    if [ ! -f "$file" ]; then
        echo "ERROR: Source file not found: $file"
        echo "Available files:"
        ls -la *.cpp *.h 2>/dev/null || echo "No C++ files found"
        exit 1
    fi
    echo "  ✓ $file"
done

# Compile
echo
echo "Compiling..."
$CXX $CXXFLAGS $INCLUDES $SOURCES -o $OUTPUT

if [ $? -ne 0 ]; then
    echo
    echo "❌ COMPILATION FAILED!"
    echo "Check the error messages above."
    exit 1
fi

echo
echo "✅ COMPILATION SUCCESSFUL!"
echo
echo "Executable created: $OUTPUT"
echo "File size: $(du -h $OUTPUT | cut -f1)"
echo
echo "To run the tests:"
echo "  ./$OUTPUT"
echo