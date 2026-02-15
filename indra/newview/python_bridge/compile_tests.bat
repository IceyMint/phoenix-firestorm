@echo off
echo ===============================================
echo Compiling Genesis AI Message Queue Tests
echo ===============================================

:: Set compiler and flags
set CXX=g++
set CXXFLAGS=-std=c++17 -pthread -O2 -DNDEBUG -Wall -Wextra

:: Set include directories
set INCLUDES=-I. -I.. -I../.. -I../../.. 

:: Set library directories and libraries
set LIBS=-lgtest -lgtest_main -pthread

:: Source files
set SOURCES=test_queue_system.cpp ai_message_queue.cpp queue_processor.cpp

:: Output executable
set OUTPUT=test_queue_system.exe

echo.
echo Compiling with:
echo   Compiler: %CXX%
echo   Flags: %CXXFLAGS%
echo   Includes: %INCLUDES%
echo   Libraries: %LIBS%
echo   Sources: %SOURCES%
echo   Output: %OUTPUT%
echo.

:: Check if Google Test is available
echo Checking for Google Test...
%CXX% -lgtest -lgtest_main -x c++ - -o nul 2>nul < nul
if errorlevel 1 (
    echo ERROR: Google Test not found!
    echo.
    echo Please install Google Test. On Windows with MSYS2/MinGW:
    echo   pacman -S mingw-w64-x86_64-gtest
    echo.
    echo Or on Ubuntu/Debian:
    echo   sudo apt-get install libgtest-dev
    echo.
    pause
    exit /b 1
)
echo Google Test found!

:: Compile
echo.
echo Compiling...
%CXX% %CXXFLAGS% %INCLUDES% %SOURCES% %LIBS% -o %OUTPUT%

if errorlevel 1 (
    echo.
    echo ❌ COMPILATION FAILED!
    echo Check the error messages above.
    pause
    exit /b 1
)

echo.
echo ✅ COMPILATION SUCCESSFUL!
echo.
echo Executable created: %OUTPUT%
echo.
echo To run the tests:
echo   %OUTPUT%
echo.
pause