@echo off
echo ===============================================
echo Compiling Genesis AI Message Queue Tests (Windows)
echo ===============================================

:: Try different compilers in order of preference
set COMPILER_FOUND=0

:: Check for MSVC (Visual Studio)
where cl >nul 2>&1
if %ERRORLEVEL% == 0 (
    echo Found Visual Studio C++ compiler
    set CXX=cl
    set COMPILE_CMD=cl /std:c++17 /EHsc /O2 /I. /I./nlohmann simple_test.cpp ai_message_queue.cpp queue_processor.cpp /Fe:simple_test.exe
    set COMPILER_FOUND=1
    goto :compile
)

:: Check for MinGW
where g++.exe >nul 2>&1
if %ERRORLEVEL% == 0 (
    echo Found MinGW g++ compiler
    set CXX=g++
    set COMPILE_CMD=g++ -std=c++17 -pthread -O2 -DNDEBUG -Wall -I. -I./nlohmann simple_test.cpp ai_message_queue.cpp queue_processor.cpp -o simple_test.exe
    set COMPILER_FOUND=1
    goto :compile
)

:: Check for clang
where clang++.exe >nul 2>&1
if %ERRORLEVEL% == 0 (
    echo Found Clang++ compiler
    set CXX=clang++
    set COMPILE_CMD=clang++ -std=c++17 -pthread -O2 -I. -I./nlohmann simple_test.cpp ai_message_queue.cpp queue_processor.cpp -o simple_test.exe
    set COMPILER_FOUND=1
    goto :compile
)

if %COMPILER_FOUND% == 0 (
    echo ERROR: No C++ compiler found!
    echo.
    echo Please install one of the following:
    echo - Visual Studio Build Tools
    echo - MinGW-w64
    echo - Clang
    echo.
    pause
    exit /b 1
)

:compile
echo.
echo Compiling with: %CXX%
echo Command: %COMPILE_CMD%
echo.

:: Check if source files exist
if not exist "simple_test.cpp" (
    echo ERROR: simple_test.cpp not found!
    pause
    exit /b 1
)

if not exist "ai_message_queue.cpp" (
    echo ERROR: ai_message_queue.cpp not found!
    pause
    exit /b 1
)

if not exist "queue_processor.cpp" (
    echo ERROR: queue_processor.cpp not found!
    pause
    exit /b 1
)

if not exist "nlohmann" (
    echo ERROR: nlohmann directory not found!
    echo Please ensure nlohmann JSON library is in ./nlohmann/
    pause
    exit /b 1
)

echo Compiling...
%COMPILE_CMD%

if %ERRORLEVEL% neq 0 (
    echo.
    echo ❌ COMPILATION FAILED!
    echo Check the error messages above.
    pause
    exit /b 1
)

echo.
echo ✅ COMPILATION SUCCESSFUL!
echo.
echo Executable created: simple_test.exe
echo.
echo To run the tests:
echo   simple_test.exe
echo.
pause