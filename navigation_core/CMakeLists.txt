# CMakeLists.txt for Navigation Core Tests
# Build with: mkdir build && cd build && cmake .. && make

cmake_minimum_required(VERSION 3.10)
project(navigation_core_tests)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Unity or use bundled
find_path(UNITY_DIR unity.h PATHS ../unity src .)

if(UNITY_DIR)
    include_directories(${UNITY_DIR})
else()
    # Bundled minimal Unity implementation
    add_definitions(-DUNITY_INCLUDE_CONFIG_H)
endif()

# Source files
set(SOURCES
    test_navigation.cpp
    NavigationCore.h
)

# Executable
add_executable(nav_test ${SOURCES})

# Use M_PI from math.h
add_definitions(-D_ISOC99_SOURCE)
