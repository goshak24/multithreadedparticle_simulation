cmake_minimum_required(VERSION 3.16)
project(PhysicsSimulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find threads package for multithreading support
find_package(Threads REQUIRED)

# Add executable
add_executable(simulation main.cpp)

# Link with threads
target_link_libraries(simulation Threads::Threads)

# Optional: Use lld linker if available for faster linking
find_program(LLD_LINKER lld)
if(LLD_LINKER)
    set(CMAKE_LINKER ${LLD_LINKER})
    message(STATUS "Using lld linker")
endif()