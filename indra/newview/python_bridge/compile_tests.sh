#!/bin/bash

echo "==============================================="
echo "Compiling Genesis AI Message Queue Tests"
echo "==============================================="

# Set compiler and flags
CXX="g++"
CXXFLAGS="-std=c++17 -pthread -O2 -DNDEBUG -Wall -Wextra"

# Set include directories
INCLUDES="-I. -I.. -I../.. -I../../.."

# Set library directories and libraries
LIBS="-lgtest -lgtest_main -pthread"

# Source files
SOURCES="test_queue_system.cpp ai_message_queue.cpp queue_processor.cpp"

# Output executable
OUTPUT="test_queue_system"

echo
echo "Compiling with:"
echo "  Compiler: $CXX"
echo "  Flags: $CXXFLAGS"
echo "  Includes: $INCLUDES"
echo "  Libraries: $LIBS"
echo "  Sources: $SOURCES"
echo "  Output: $OUTPUT"
echo

# Check if Google Test is available
echo "Checking for Google Test..."
if ! pkg-config --exists gtest; then
    echo "Trying to find gtest manually..."
    if ! echo '#include <gtest/gtest.h>' | $CXX -xc++ -c - -o /dev/null 2>/dev/null; then
        echo "ERROR: Google Test not found!"
        echo
        echo "Please install Google Test:"
        echo "  On Ubuntu/Debian: sudo apt-get install libgtest-dev"
        echo "  On CentOS/RHEL: sudo yum install gtest-devel"
        echo "  On macOS: brew install googletest"
        echo "  On Windows (MSYS2): pacman -S mingw-w64-x86_64-gtest"
        echo
        exit 1
    fi
else
    echo "Google Test found via pkg-config!"
    # Use pkg-config for better library detection
    LIBS="$(pkg-config --libs gtest) -pthread"
    INCLUDES="$INCLUDES $(pkg-config --cflags gtest)"
fi

# Check if source files exist
echo
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
$CXX $CXXFLAGS $INCLUDES $SOURCES $LIBS -o $OUTPUT

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
echo "To run with verbose output:"
echo "  ./$OUTPUT --gtest_verbose"
echo
echo "To run specific tests:"
echo "  ./$OUTPUT --gtest_filter=AICommandQueueTest.*"
echo